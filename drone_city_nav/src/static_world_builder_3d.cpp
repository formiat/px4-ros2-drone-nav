#include "static_world_builder_3d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool configValid(const StaticWorldBuilderConfig3D& config) noexcept {
  const std::shared_ptr<const OccupancyGrid3D>& occupancy = config.resources.occupancy;
  return occupancy != nullptr && occupancy->fingerprint() != 0U &&
         occupancy->contentFingerprint() != 0U &&
         std::isfinite(config.route_lookahead_m) && config.route_lookahead_m > 0.0 &&
         std::isfinite(config.roi_halo_m) && config.roi_halo_m >= 0.0 &&
         std::isfinite(config.maximum_distance_m) && config.maximum_distance_m > 0.0 &&
         (!config.resources.topology ||
          config.resources.topology->compatibleWith(*occupancy)) &&
         (!config.resources.esdf_cache.has_value() ||
          config.resources.esdf_cache->compatibleWith(*occupancy,
                                                      config.maximum_distance_m));
}

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] GridBounds3D localStaticEsdfBounds(const OccupancyGrid3D& occupancy,
                                                 const Point3& start,
                                                 const Point3& goal,
                                                 const double planning_distance_m,
                                                 const double halo_m) {
  const GridBounds3D& world = occupancy.bounds();
  const double horizontal_distance = std::hypot(goal.x - start.x, goal.y - start.y);
  const double ratio = horizontal_distance > planning_distance_m
                           ? planning_distance_m / horizontal_distance
                           : 1.0;
  const Point2 endpoint{std::lerp(start.x, goal.x, ratio),
                        std::lerp(start.y, goal.y, ratio)};
  const auto clamp_cell = [](const int value, const int maximum) {
    return std::clamp(value, 0, maximum - 1);
  };
  const int min_x =
      clamp_cell(static_cast<int>(std::floor(
                     (std::min(start.x, endpoint.x) - halo_m - world.origin_x) /
                     world.resolution_m)),
                 world.width_cells);
  const int max_x =
      clamp_cell(static_cast<int>(std::floor(
                     (std::max(start.x, endpoint.x) + halo_m - world.origin_x) /
                     world.resolution_m)),
                 world.width_cells);
  const int min_y =
      clamp_cell(static_cast<int>(std::floor(
                     (std::min(start.y, endpoint.y) - halo_m - world.origin_y) /
                     world.resolution_m)),
                 world.height_cells);
  const int max_y =
      clamp_cell(static_cast<int>(std::floor(
                     (std::max(start.y, endpoint.y) + halo_m - world.origin_y) /
                     world.resolution_m)),
                 world.height_cells);
  return GridBounds3D{
      .origin_x = world.origin_x + static_cast<double>(min_x) * world.resolution_m,
      .origin_y = world.origin_y + static_cast<double>(min_y) * world.resolution_m,
      .origin_z = world.origin_z,
      .resolution_m = world.resolution_m,
      .width_cells = max_x - min_x + 1,
      .height_cells = max_y - min_y + 1,
      .depth_cells = world.depth_cells,
  };
}

[[nodiscard]] bool sameStaticEsdfGrid(const mppi::EsdfGrid& grid,
                                      const GridBounds3D& bounds) noexcept {
  constexpr double kTolerance{1.0e-6};
  return grid.width == bounds.width_cells && grid.height == bounds.height_cells &&
         grid.depth == bounds.depth_cells &&
         std::abs(static_cast<double>(grid.resolution_m) - bounds.resolution_m) <=
             kTolerance &&
         std::abs(static_cast<double>(grid.origin_x_m) - bounds.origin_x) <=
             kTolerance &&
         std::abs(static_cast<double>(grid.origin_y_m) - bounds.origin_y) <=
             kTolerance &&
         std::abs(static_cast<double>(grid.origin_z_m) - bounds.origin_z) <= kTolerance;
}

[[nodiscard]] mppi::EsdfGrid esdfGrid(const GridBounds3D& bounds) noexcept {
  return mppi::EsdfGrid{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .origin_x_m = static_cast<float>(bounds.origin_x),
      .origin_y_m = static_cast<float>(bounds.origin_y),
      .depth = bounds.depth_cells,
      .origin_z_m = static_cast<float>(bounds.origin_z),
  };
}

} // namespace

std::string_view
staticWorldUpdateStatus3DName(const StaticWorldUpdateStatus3D status) noexcept {
  switch (status) {
    case StaticWorldUpdateStatus3D::kPrepared:
      return "prepared";
    case StaticWorldUpdateStatus3D::kPublished:
      return "published";
    case StaticWorldUpdateStatus3D::kAlreadyCurrent:
      return "already_current";
    case StaticWorldUpdateStatus3D::kUnavailableStaticOccupancy:
      return "unavailable_static_occupancy";
    case StaticWorldUpdateStatus3D::kUnavailableNavigation:
      return "unavailable_navigation";
    case StaticWorldUpdateStatus3D::kUnavailableObjective:
      return "unavailable_objective";
    case StaticWorldUpdateStatus3D::kRefreshSuperseded:
      return "refresh_superseded";
    case StaticWorldUpdateStatus3D::kInvalidCommitContext:
      return "invalid_commit_context";
    case StaticWorldUpdateStatus3D::kConstructionFailed:
      return "construction_failed";
    case StaticWorldUpdateStatus3D::kUploadRejected:
      return "upload_rejected";
    case StaticWorldUpdateStatus3D::kUploadFailed:
      return "upload_failed";
    case StaticWorldUpdateStatus3D::kMixedLocalWorldGeneration:
      return "mixed_local_world_generation";
  }
  return "unknown";
}

std::string_view
staticWorldEsdfSource3DName(const StaticWorldEsdfSource3D source) noexcept {
  switch (source) {
    case StaticWorldEsdfSource3D::kPrecomputedCache:
      return "precomputed_cache";
    case StaticWorldEsdfSource3D::kRuntimeEdt:
      return "runtime_edt";
  }
  return "unknown";
}

StaticWorldBuilder3D::StaticWorldBuilder3D(StaticWorldBuilderConfig3D&& config)
    : config_{std::move(config)} {
  if (!configValid(config_)) {
    throw std::invalid_argument{"invalid static world builder configuration"};
  }
  const std::span<const PassageTraversalEdge> traversals =
      config_.resources.topology
          ? std::span<const PassageTraversalEdge>{config_.resources.topology
                                                      ->traversalEdges()}
          : std::span<const PassageTraversalEdge>{};
  topology_traversals_ = std::make_shared<const std::vector<PassageTraversalEdge>>(
      traversals.begin(), traversals.end());
}

const std::shared_ptr<const OccupancyGrid3D>&
StaticWorldBuilder3D::occupancy() const noexcept {
  return config_.resources.occupancy;
}

PreparedStaticWorldBuild3D StaticWorldBuilder3D::prepare(
    const StaticWorldBuildRequest3D& request,
    const std::shared_ptr<const WorldSnapshot3D>& active_world,
    const std::shared_ptr<const StaticWorldEsdfArtifact3D>& uploaded_artifact) {
  PreparedStaticWorldBuild3D result;
  result.request = request;
  const std::shared_ptr<const OccupancyGrid3D>& static_occupancy =
      config_.resources.occupancy;
  if (static_occupancy == nullptr) {
    result.status = StaticWorldUpdateStatus3D::kUnavailableStaticOccupancy;
    return result;
  }
  if (!result.request.world_state_authoritative ||
      !finitePoint(result.request.position)) {
    result.status = StaticWorldUpdateStatus3D::kUnavailableNavigation;
    return result;
  }
  if (!result.request.objective.available ||
      !finitePoint(result.request.objective.goal)) {
    result.status = StaticWorldUpdateStatus3D::kUnavailableObjective;
    return result;
  }
  result.proactive_refresh = result.request.refresh.valid();
  if (result.proactive_refresh && result.request.resident_route_generation !=
                                      result.request.refresh.base_route_generation) {
    result.status = StaticWorldUpdateStatus3D::kRefreshSuperseded;
    return result;
  }
  const bool current_artifact_uploaded =
      current_artifact_ != nullptr && uploaded_artifact == current_artifact_;
  const bool active_world_current =
      active_world != nullptr && productionWorldGenerationCoherent(*active_world) &&
      current_artifact_uploaded &&
      active_world->distances_m == current_artifact_->distances_m &&
      active_world->static_occupancy == static_occupancy;
  if (!result.proactive_refresh && active_world_current) {
    result.status = StaticWorldUpdateStatus3D::kAlreadyCurrent;
    result.artifact = current_artifact_;
    result.telemetry = current_artifact_->telemetry;
    result.diagnostics = current_artifact_->diagnostics;
    result.diagnostics.cpu_resource_reused = true;
    result.diagnostics.gpu_resource_reused = true;
    return result;
  }

  const GridBounds3D requested_bounds = localStaticEsdfBounds(
      *static_occupancy, result.request.position, result.request.objective.goal,
      config_.route_lookahead_m, config_.roi_halo_m);
  const GridBounds3D local_bounds = StaticEsdfCache::alignRegionToChunks(
      static_occupancy->bounds(), requested_bounds);
  const bool reuse_cpu_resource =
      current_artifact_ != nullptr &&
      sameStaticEsdfGrid(current_artifact_->grid, local_bounds);
  std::string cache_fallback_error;
  if (!reuse_cpu_resource) {
    bool precomputed_cache_used{false};
    StaticEsdfCacheExtractionStats cache_stats;
    std::optional<DistanceField3D> cached_field;
    if (config_.resources.esdf_cache.has_value()) {
      try {
        StaticEsdfCacheExtraction extraction = config_.resources.esdf_cache->extract(
            local_bounds, config_.maximum_distance_m);
        cache_stats = extraction.stats;
        cached_field.emplace(std::move(extraction.field));
        precomputed_cache_used = true;
      } catch (const std::exception& error) {
        cache_fallback_error = error.what();
        config_.resources.esdf_cache.reset();
      }
    }
    const DistanceField3D field =
        cached_field.has_value()
            ? std::move(*cached_field)
            : DistanceField3D::buildLocal(*static_occupancy, local_bounds,
                                          config_.maximum_distance_m,
                                          config_.worker_pool);
    const DistanceField3DBuildStats field_stats = field.stats();
    const ProductionWorldBuildTelemetry3D telemetry{
        .build_ms = field_stats.duration_ms,
        .esdf_x_pass_ms = field_stats.x_pass_ms,
        .esdf_y_pass_ms = field_stats.y_pass_ms,
        .esdf_z_pass_ms = field_stats.z_pass_ms,
        .esdf_finalize_ms = field_stats.finalize_ms,
    };
    current_artifact_ =
        std::make_shared<const StaticWorldEsdfArtifact3D>(StaticWorldEsdfArtifact3D{
            .grid = esdfGrid(field.bounds()),
            .distances_m = std::make_shared<const std::vector<float>>(
                field.distancesM().begin(), field.distancesM().end()),
            .telemetry = telemetry,
            .diagnostics =
                StaticWorldBuildDiagnostics3D{
                    .field = field_stats,
                    .cache = cache_stats,
                    .source = precomputed_cache_used
                                  ? StaticWorldEsdfSource3D::kPrecomputedCache
                                  : StaticWorldEsdfSource3D::kRuntimeEdt,
                    .cache_fallback_error = {},
                    .maximum_distance_m = config_.maximum_distance_m,
                    .cpu_resource_reused = false,
                    .gpu_resource_reused = false,
                },
        });
  }

  result.artifact = current_artifact_;
  result.telemetry = current_artifact_->telemetry;
  result.diagnostics = current_artifact_->diagnostics;
  result.diagnostics.cache_fallback_error = std::move(cache_fallback_error);
  result.diagnostics.cpu_resource_reused = reuse_cpu_resource;
  result.diagnostics.gpu_resource_reused = uploaded_artifact == current_artifact_;
  result.upload_required = !result.diagnostics.gpu_resource_reused;
  result.world.producer_instance_id = 0U;
  result.world.revision = static_occupancy->fingerprint();
  result.world.source_occupied_fingerprint = static_occupancy->contentFingerprint();
  result.world.raw_occupied_fingerprint = static_occupancy->contentFingerprint();
  result.world.source_stamp_ns = result.request.source_stamp_ns;
  result.world.grid = current_artifact_->grid;
  result.world.distances_m = current_artifact_->distances_m;
  result.world.static_occupancy = static_occupancy;
  result.world.topology_passage_traversals = topology_traversals_;
  result.status = StaticWorldUpdateStatus3D::kPrepared;
  return result;
}

} // namespace drone_city_nav
