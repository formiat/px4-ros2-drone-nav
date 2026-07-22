#include "drone_city_nav/planning_grid_builder.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] double sanitizedNonNegative(const double value) noexcept {
  return std::isfinite(value) && value > 0.0 ? value : 0.0;
}

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point started_at) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - started_at)
                                 .count()) /
         1000.0;
}

void populateSourceStats(PlanningGridBuildResult& result,
                         const PlanningGridBuilderConfig& config,
                         const PlanningGridSources& sources) {
  result.static_source.enabled = config.use_static_map;
  result.static_source.loaded = sources.static_grid != nullptr;
  result.static_source.rectangles = sources.static_rectangles;
  result.static_source.occupied_cells = sources.static_occupied_cells;
  result.static_source.path = sources.static_map_path;

  result.memory.enabled = true;
  result.memory.seen = sources.memory_grid != nullptr;

  result.current_lidar = sources.current_lidar;
  result.current_lidar.enabled = true;
}

void overlayMemorySource(OccupancyGrid2D& raw_grid, PlanningGridBuildResult& result,
                         const PlanningGridSources& sources) {
  if (sources.memory_grid == nullptr) {
    return;
  }
  result.memory.source_counts = collectGridStats(*sources.memory_grid);
  result.memory.geometry_matches = haveSameGridGeometry(raw_grid, *sources.memory_grid);
  if (result.memory.geometry_matches) {
    result.memory.overlay = overlayKnownMemoryCells(raw_grid, *sources.memory_grid);
    result.memory.used = true;
  }
}

[[nodiscard]] bool overlayCurrentLidarSource(OccupancyGrid2D& raw_grid,
                                             PlanningGridBuildResult& result,
                                             const PlanningGridSources& sources) {
  if (sources.current_lidar_grid == nullptr || !result.current_lidar.fresh ||
      !result.current_lidar.used ||
      !haveSameGridGeometry(raw_grid, *sources.current_lidar_grid)) {
    return false;
  }
  const GridOverlayStats current_overlay =
      overlayCurrentLidarCells(raw_grid, *sources.current_lidar_grid);
  result.current_lidar.occupied_cells =
      current_overlay.occupied_cells_applied + current_overlay.occupied_cells_preserved;
  result.current_lidar.overlay_occupied_cells_applied =
      current_overlay.occupied_cells_applied;
  result.current_lidar.overlay_occupied_cells_preserved =
      current_overlay.occupied_cells_preserved;
  return true;
}

void overlayDynamicSources(OccupancyGrid2D& dynamic_grid,
                           const PlanningGridBuildResult& result,
                           const PlanningGridSources& sources) {
  if (sources.memory_grid != nullptr && result.memory.geometry_matches) {
    (void)overlayKnownMemoryCells(dynamic_grid, *sources.memory_grid);
  }
  if (sources.current_lidar_grid != nullptr && result.current_lidar.fresh &&
      result.current_lidar.used &&
      haveSameGridGeometry(dynamic_grid, *sources.current_lidar_grid)) {
    (void)overlayCurrentLidarCells(dynamic_grid, *sources.current_lidar_grid);
  }
}

[[nodiscard]] bool sameFingerprint(const OccupancyGridFingerprint& lhs,
                                   const OccupancyGridFingerprint& rhs) noexcept {
  return lhs.bounds.origin_x == rhs.bounds.origin_x &&
         lhs.bounds.origin_y == rhs.bounds.origin_y &&
         lhs.bounds.resolution_m == rhs.bounds.resolution_m &&
         lhs.bounds.width_cells == rhs.bounds.width_cells &&
         lhs.bounds.height_cells == rhs.bounds.height_cells &&
         lhs.cells_hash == rhs.cells_hash && lhs.inflated_hash == rhs.inflated_hash;
}

[[nodiscard]] std::size_t countOccupiedCells(const OccupancyGrid2D& grid) {
  std::size_t occupied_cells = 0U;
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      if (grid.isOccupied(GridIndex{x, y})) {
        ++occupied_cells;
      }
    }
  }
  return occupied_cells;
}

[[nodiscard]] PlanningGridBuildResult
buildPlanningGridUncached(const PlanningGridBuilderConfig& config,
                          const PlanningGridSources& sources) {
  PlanningGridBuildResult result{};
  populateSourceStats(result, config, sources);

  if (config.use_static_map && sources.static_grid == nullptr) {
    result.status = PlanningGridStatus::kStaticMapEnabledButMissing;
    return result;
  }

  const std::optional<GridBounds> bounds = selectPlanningGridBounds(config, sources);
  if (!bounds.has_value()) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }

  OccupancyGrid2D raw_grid{*bounds};
  if (config.use_static_map && sources.static_grid != nullptr) {
    const GridOverlayStats static_overlay =
        overlayOccupiedCells(raw_grid, *sources.static_grid);
    result.static_source.occupied_cells = static_overlay.source_occupied_cells;
    result.static_source.used = true;
  }

  overlayMemorySource(raw_grid, result, sources);
  const bool current_lidar_applied =
      overlayCurrentLidarSource(raw_grid, result, sources);

  const bool current_lidar_ready =
      result.current_lidar.used && result.current_lidar.fresh && current_lidar_applied;
  const bool any_source_ready =
      result.static_source.used || result.memory.used || current_lidar_ready;
  if (!any_source_ready) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }

  const double inflation_radius_m = sanitizedNonNegative(config.inflation_radius_m);
  const double planning_clearance_m = sanitizedNonNegative(config.planning_clearance_m);

  const DistanceField2D occupied_distance_field = DistanceField2D::build(
      raw_grid,
      inflation_radius_m + planning_clearance_m + (0.5 * raw_grid.resolution()),
      DistanceFieldSource::kOccupied);

  OccupancyGrid2D prohibited_grid = raw_grid;
  prohibited_grid.applyInflationFromDistanceField(occupied_distance_field,
                                                  inflation_radius_m);

  OccupancyGrid2D planning_grid = raw_grid;
  planning_grid.applyInflationFromDistanceField(
      occupied_distance_field, inflation_radius_m + planning_clearance_m);

  result.status = PlanningGridStatus::kReady;
  result.raw_grid = std::move(raw_grid);
  result.grid = std::move(prohibited_grid);
  result.planning_grid = std::move(planning_grid);
  return result;
}

} // namespace

const char* planningGridStatusName(const PlanningGridStatus status) noexcept {
  switch (status) {
    case PlanningGridStatus::kReady:
      return "ready";
    case PlanningGridStatus::kStaticMapEnabledButMissing:
      return "static_map_enabled_but_missing";
    case PlanningGridStatus::kNoReadySourceData:
      return "no_ready_source_data";
  }
  return "unknown";
}

std::optional<GridBounds>
selectPlanningGridBounds(const PlanningGridBuilderConfig& config,
                         const PlanningGridSources& sources) {
  if (config.use_static_map && sources.static_grid != nullptr) {
    return sources.static_grid->bounds();
  }
  if (sources.memory_grid != nullptr) {
    return sources.memory_grid->bounds();
  }
  if (sources.current_lidar_grid != nullptr && sources.current_lidar.used &&
      sources.current_lidar.fresh) {
    return sources.current_lidar_grid->bounds();
  }
  return config.fallback_bounds;
}

PlanningGridBuilder::StaticGridCache::StaticGridCache(const GridBounds& bounds)
    : raw_grid{bounds},
      prohibited_grid{bounds},
      planning_grid{bounds} {
}

PlanningGridBuildResult
PlanningGridBuilder::build(const PlanningGridBuilderConfig& config,
                           const PlanningGridSources& sources) {
  PlanningGridBuildResult result{};
  populateSourceStats(result, config, sources);

  if (config.use_static_map && sources.static_grid == nullptr) {
    result.status = PlanningGridStatus::kStaticMapEnabledButMissing;
    return result;
  }

  const std::optional<GridBounds> bounds = selectPlanningGridBounds(config, sources);
  if (!bounds.has_value()) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }

  if (!config.use_static_map || sources.static_grid == nullptr) {
    return buildPlanningGridUncached(config, sources);
  }

  const double inflation_radius_m = sanitizedNonNegative(config.inflation_radius_m);
  const double planning_clearance_m = sanitizedNonNegative(config.planning_clearance_m);
  const OccupancyGridFingerprint static_fingerprint =
      sources.static_grid->prohibitedFingerprint();
  const bool cache_hit =
      static_cache_.has_value() &&
      sameFingerprint(static_cache_->fingerprint, static_fingerprint) &&
      static_cache_->inflation_radius_m == inflation_radius_m &&
      static_cache_->planning_clearance_m == planning_clearance_m;
  result.cache.static_cache_eligible = true;
  result.cache.static_cache_hit = cache_hit;
  result.cache.static_cache_rebuilt = !cache_hit;

  if (!cache_hit) {
    static_cache_.emplace(*bounds);
    static_cache_->fingerprint = static_fingerprint;
    static_cache_->inflation_radius_m = inflation_radius_m;
    static_cache_->planning_clearance_m = planning_clearance_m;
    static_cache_->overlay =
        overlayOccupiedCells(static_cache_->raw_grid, *sources.static_grid);
    const auto static_distance_started_at = std::chrono::steady_clock::now();
    static_cache_->occupied_distance_field =
        DistanceField2D::build(static_cache_->raw_grid,
                               inflation_radius_m + planning_clearance_m +
                                   (0.5 * static_cache_->raw_grid.resolution()),
                               DistanceFieldSource::kOccupied);
    result.cache.static_distance_field_duration_ms =
        elapsedMilliseconds(static_distance_started_at);
    result.cache.static_distance_source_cells =
        static_cache_->occupied_distance_field.stats().source_cells;
    const auto static_inflation_started_at = std::chrono::steady_clock::now();
    static_cache_->prohibited_grid = static_cache_->raw_grid;
    static_cache_->prohibited_grid.applyInflationFromDistanceField(
        static_cache_->occupied_distance_field, inflation_radius_m);
    static_cache_->planning_grid = static_cache_->raw_grid;
    static_cache_->planning_grid.applyInflationFromDistanceField(
        static_cache_->occupied_distance_field,
        inflation_radius_m + planning_clearance_m);
    result.cache.static_inflation_mask_duration_ms =
        elapsedMilliseconds(static_inflation_started_at);
  } else {
    result.cache.static_distance_source_cells =
        static_cache_->occupied_distance_field.stats().source_cells;
  }

  OccupancyGrid2D raw_grid = static_cache_->raw_grid;
  result.static_source.occupied_cells = static_cache_->overlay.source_occupied_cells;
  result.static_source.used = true;
  overlayMemorySource(raw_grid, result, sources);
  const bool current_lidar_applied =
      overlayCurrentLidarSource(raw_grid, result, sources);

  const bool current_lidar_ready =
      result.current_lidar.used && result.current_lidar.fresh && current_lidar_applied;
  const bool any_source_ready =
      result.static_source.used || result.memory.used || current_lidar_ready;
  if (!any_source_ready) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }

  OccupancyGrid2D dynamic_raw{*bounds};
  overlayDynamicSources(dynamic_raw, result, sources);
  result.cache.dynamic_distance_source_cells = countOccupiedCells(dynamic_raw);

  OccupancyGrid2D prohibited_grid = raw_grid;
  prohibited_grid.mergeInflationFrom(static_cache_->prohibited_grid);
  OccupancyGrid2D planning_grid = raw_grid;
  planning_grid.mergeInflationFrom(static_cache_->planning_grid);

  if (result.cache.dynamic_distance_source_cells > 0U) {
    const auto dynamic_distance_started_at = std::chrono::steady_clock::now();
    const DistanceField2D dynamic_distance_field = DistanceField2D::build(
        dynamic_raw,
        inflation_radius_m + planning_clearance_m + (0.5 * dynamic_raw.resolution()),
        DistanceFieldSource::kOccupied);
    result.cache.dynamic_distance_field_duration_ms =
        elapsedMilliseconds(dynamic_distance_started_at);
    result.cache.dynamic_distance_source_cells =
        dynamic_distance_field.stats().source_cells;

    const auto dynamic_inflation_started_at = std::chrono::steady_clock::now();
    OccupancyGrid2D dynamic_prohibited_grid = dynamic_raw;
    dynamic_prohibited_grid.applyInflationFromDistanceField(dynamic_distance_field,
                                                            inflation_radius_m);
    prohibited_grid.mergeInflationFrom(dynamic_prohibited_grid);

    OccupancyGrid2D dynamic_planning_grid = dynamic_raw;
    dynamic_planning_grid.applyInflationFromDistanceField(
        dynamic_distance_field, inflation_radius_m + planning_clearance_m);
    planning_grid.mergeInflationFrom(dynamic_planning_grid);
    result.cache.dynamic_inflation_mask_duration_ms =
        elapsedMilliseconds(dynamic_inflation_started_at);
  }

  result.status = PlanningGridStatus::kReady;
  result.raw_grid = std::move(raw_grid);
  result.grid = std::move(prohibited_grid);
  result.planning_grid = std::move(planning_grid);
  return result;
}

void PlanningGridBuilder::clearCache() noexcept {
  static_cache_.reset();
}

PlanningGridBuildResult buildPlanningGrid(const PlanningGridBuilderConfig& config,
                                          const PlanningGridSources& sources) {
  return buildPlanningGridUncached(config, sources);
}

} // namespace drone_city_nav
