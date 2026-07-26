#include "drone_city_nav/planning_grid_builder.hpp"

#include "drone_city_nav/grid_config.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] double sanitizedNonNegative(const double value) noexcept {
  return std::isfinite(value) && value > 0.0 ? value : 0.0;
}

void populateSourceStats(ObstacleFieldBuildResult& result,
                         const ObstacleFieldBuilderConfig& config,
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

void overlayMemorySource(OccupancyGrid2D& raw_grid, ObstacleFieldBuildResult& result,
                         const PlanningGridSources& sources) {
  if (sources.memory_grid == nullptr) {
    return;
  }
  result.memory.source_counts = collectGridStats(*sources.memory_grid);
  result.memory.geometry_matches = haveSameGridGeometry(raw_grid, *sources.memory_grid);
  if (!result.memory.geometry_matches) {
    return;
  }
  result.memory.overlay = overlayKnownMemoryCells(raw_grid, *sources.memory_grid);
  result.memory.used = true;
  result.applied_memory_producer_instance_id = sources.memory_producer_instance_id;
  result.applied_memory_sequence = sources.memory_sequence;
}

[[nodiscard]] bool overlayCurrentLidarSource(OccupancyGrid2D& raw_grid,
                                             ObstacleFieldBuildResult& result,
                                             const PlanningGridSources& sources) {
  if (sources.current_lidar_grid == nullptr || !result.current_lidar.fresh ||
      !result.current_lidar.used ||
      !haveSameGridGeometry(raw_grid, *sources.current_lidar_grid)) {
    return false;
  }
  const GridOverlayStats overlay =
      overlayCurrentLidarCells(raw_grid, *sources.current_lidar_grid);
  result.current_lidar.occupied_cells =
      overlay.occupied_cells_applied + overlay.occupied_cells_preserved;
  result.current_lidar.overlay_occupied_cells_applied = overlay.occupied_cells_applied;
  result.current_lidar.overlay_occupied_cells_preserved =
      overlay.occupied_cells_preserved;
  result.applied_lidar_update_ns = sources.lidar_update_ns;
  return true;
}

[[nodiscard]] bool sameFingerprint(const OccupancyGridFingerprint& lhs,
                                   const OccupancyGridFingerprint& rhs) noexcept {
  return lhs.bounds.origin_x == rhs.bounds.origin_x &&
         lhs.bounds.origin_y == rhs.bounds.origin_y &&
         lhs.bounds.resolution_m == rhs.bounds.resolution_m &&
         lhs.bounds.width_cells == rhs.bounds.width_cells &&
         lhs.bounds.height_cells == rhs.bounds.height_cells &&
         lhs.cells_hash == rhs.cells_hash;
}

[[nodiscard]] std::optional<GridBounds>
clippedWindowBounds(const GridBounds& source, const GridBounds& requested) {
  if (!gridBoundsUsable(source) || !gridBoundsUsable(requested) ||
      source.resolution_m != requested.resolution_m) {
    return std::nullopt;
  }
  const double source_max_x =
      source.origin_x + source.resolution_m * static_cast<double>(source.width_cells);
  const double source_max_y =
      source.origin_y + source.resolution_m * static_cast<double>(source.height_cells);
  const double requested_max_x =
      requested.origin_x +
      requested.resolution_m * static_cast<double>(requested.width_cells);
  const double requested_max_y =
      requested.origin_y +
      requested.resolution_m * static_cast<double>(requested.height_cells);
  const double min_x = std::max(source.origin_x, requested.origin_x);
  const double min_y = std::max(source.origin_y, requested.origin_y);
  const double max_x = std::min(source_max_x, requested_max_x);
  const double max_y = std::min(source_max_y, requested_max_y);
  if (max_x <= min_x || max_y <= min_y) {
    return std::nullopt;
  }
  const int start_x = std::max(
      0, static_cast<int>(std::floor((min_x - source.origin_x) / source.resolution_m)));
  const int start_y = std::max(
      0, static_cast<int>(std::floor((min_y - source.origin_y) / source.resolution_m)));
  const int end_x = std::min(
      source.width_cells,
      static_cast<int>(std::ceil((max_x - source.origin_x) / source.resolution_m)));
  const int end_y = std::min(
      source.height_cells,
      static_cast<int>(std::ceil((max_y - source.origin_y) / source.resolution_m)));
  if (end_x <= start_x || end_y <= start_y) {
    return std::nullopt;
  }
  return GridBounds{
      .origin_x = source.origin_x + source.resolution_m * static_cast<double>(start_x),
      .origin_y = source.origin_y + source.resolution_m * static_cast<double>(start_y),
      .resolution_m = source.resolution_m,
      .width_cells = end_x - start_x,
      .height_cells = end_y - start_y,
  };
}

[[nodiscard]] GridBounds expandedBounds(const GridBounds& bounds, const double halo_m) {
  const int halo_cells =
      std::max(0, static_cast<int>(std::ceil(halo_m / bounds.resolution_m)));
  return GridBounds{
      .origin_x =
          bounds.origin_x - static_cast<double>(halo_cells) * bounds.resolution_m,
      .origin_y =
          bounds.origin_y - static_cast<double>(halo_cells) * bounds.resolution_m,
      .resolution_m = bounds.resolution_m,
      .width_cells = bounds.width_cells + (2 * halo_cells),
      .height_cells = bounds.height_cells + (2 * halo_cells),
  };
}

[[nodiscard]] OccupancyGrid2D extractWindow(const OccupancyGrid2D& source,
                                            const GridBounds& bounds) {
  OccupancyGrid2D window{bounds};
  for (int y = 0; y < window.height(); ++y) {
    for (int x = 0; x < window.width(); ++x) {
      const GridIndex destination{x, y};
      const std::optional<GridIndex> source_cell =
          source.worldToCell(window.cellCenter(destination));
      if (!source_cell.has_value()) {
        continue;
      }
      switch (source.state(*source_cell)) {
        case CellState::kUnknown:
          window.setUnknown(destination);
          break;
        case CellState::kFree:
          window.setFree(destination);
          break;
        case CellState::kOccupied:
          window.setOccupied(destination);
          break;
      }
    }
  }
  return window;
}

[[nodiscard]] ObstacleFieldBuildResult
buildObstacleFieldUncached(const ObstacleFieldBuilderConfig& config,
                           const PlanningGridSources& sources) {
  ObstacleFieldBuildResult result{};
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
    const GridOverlayStats overlay =
        overlayOccupiedCells(raw_grid, *sources.static_grid);
    result.static_source.occupied_cells = overlay.source_occupied_cells;
    result.static_source.used = true;
  }
  overlayMemorySource(raw_grid, result, sources);
  const bool lidar_applied = overlayCurrentLidarSource(raw_grid, result, sources);
  if (!result.static_source.used && !result.memory.used &&
      !(result.current_lidar.used && result.current_lidar.fresh && lidar_applied)) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }

  const double critical_m = sanitizedNonNegative(config.inflation_radius_m);
  result.risk_policy = {
      .critical_distance_m = critical_m,
      .preferred_distance_m =
          critical_m + sanitizedNonNegative(config.planning_clearance_m),
  };
  result.cache.global_cells = raw_grid.cellCount();
  result.evaluation_bounds = raw_grid.bounds();

  if (config.local_planning_bounds.has_value()) {
    const std::optional<GridBounds> evaluation =
        clippedWindowBounds(raw_grid.bounds(), *config.local_planning_bounds);
    if (evaluation.has_value()) {
      const GridBounds requested_source =
          expandedBounds(*evaluation, result.risk_policy.preferred_distance_m +
                                          (0.5 * raw_grid.resolution()));
      const std::optional<GridBounds> source =
          clippedWindowBounds(raw_grid.bounds(), requested_source);
      if (source.has_value()) {
        raw_grid = extractWindow(raw_grid, *source);
        result.evaluation_bounds = evaluation;
        result.cache.local_planning_window_applied = true;
      }
    }
  }
  result.cache.planning_cells = raw_grid.cellCount();
  result.status = PlanningGridStatus::kReady;
  result.raw_occupancy = std::move(raw_grid);
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
selectPlanningGridBounds(const ObstacleFieldBuilderConfig& config,
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

ObstacleFieldBuilder::StaticGridCache::StaticGridCache(const GridBounds& bounds)
    : raw_grid{bounds} {
}

ObstacleFieldBuildResult
ObstacleFieldBuilder::build(const ObstacleFieldBuilderConfig& config,
                            const PlanningGridSources& sources) {
  if (!config.use_static_map || sources.static_grid == nullptr) {
    return buildObstacleFieldUncached(config, sources);
  }

  ObstacleFieldBuildResult result{};
  populateSourceStats(result, config, sources);
  const std::optional<GridBounds> bounds = selectPlanningGridBounds(config, sources);
  if (!bounds.has_value()) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }
  const OccupancyGridFingerprint fingerprint = sources.static_grid->rawFingerprint();
  const bool cache_hit = static_cache_.has_value() &&
                         sameFingerprint(static_cache_->fingerprint, fingerprint);
  result.cache.static_cache_eligible = true;
  result.cache.static_cache_hit = cache_hit;
  result.cache.static_cache_rebuilt = !cache_hit;
  if (!cache_hit) {
    static_cache_.emplace(*bounds);
    static_cache_->fingerprint = fingerprint;
    static_cache_->overlay =
        overlayOccupiedCells(static_cache_->raw_grid, *sources.static_grid);
  }

  OccupancyGrid2D raw_grid = static_cache_->raw_grid;
  result.static_source.occupied_cells = static_cache_->overlay.source_occupied_cells;
  result.static_source.used = true;
  overlayMemorySource(raw_grid, result, sources);
  (void)overlayCurrentLidarSource(raw_grid, result, sources);

  const double critical_m = sanitizedNonNegative(config.inflation_radius_m);
  result.risk_policy = {
      .critical_distance_m = critical_m,
      .preferred_distance_m =
          critical_m + sanitizedNonNegative(config.planning_clearance_m),
  };
  result.cache.global_cells = raw_grid.cellCount();
  result.evaluation_bounds = raw_grid.bounds();
  if (config.local_planning_bounds.has_value()) {
    const std::optional<GridBounds> evaluation =
        clippedWindowBounds(raw_grid.bounds(), *config.local_planning_bounds);
    if (evaluation.has_value()) {
      const GridBounds requested_source =
          expandedBounds(*evaluation, result.risk_policy.preferred_distance_m +
                                          (0.5 * raw_grid.resolution()));
      const std::optional<GridBounds> source =
          clippedWindowBounds(raw_grid.bounds(), requested_source);
      if (source.has_value()) {
        raw_grid = extractWindow(raw_grid, *source);
        result.evaluation_bounds = evaluation;
        result.cache.local_planning_window_applied = true;
      }
    }
  }
  result.cache.planning_cells = raw_grid.cellCount();
  result.status = PlanningGridStatus::kReady;
  result.raw_occupancy = std::move(raw_grid);
  return result;
}

void ObstacleFieldBuilder::clearCache() noexcept {
  static_cache_.reset();
}

ObstacleFieldBuildResult buildObstacleField(const ObstacleFieldBuilderConfig& config,
                                            const PlanningGridSources& sources) {
  return buildObstacleFieldUncached(config, sources);
}

} // namespace drone_city_nav
