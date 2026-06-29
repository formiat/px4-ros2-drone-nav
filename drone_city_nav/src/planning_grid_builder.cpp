#include "drone_city_nav/planning_grid_builder.hpp"

#include <utility>

namespace drone_city_nav {

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

PlanningGridBuildResult buildPlanningGrid(const PlanningGridBuilderConfig& config,
                                          const PlanningGridSources& sources) {
  PlanningGridBuildResult result{};
  result.static_source.enabled = config.use_static_map;
  result.static_source.loaded = sources.static_grid != nullptr;
  result.static_source.rectangles = sources.static_rectangles;
  result.static_source.occupied_cells = sources.static_occupied_cells;
  result.static_source.path = sources.static_map_path;

  result.memory.enabled = true;
  result.memory.seen = sources.memory_grid != nullptr;

  result.current_lidar = sources.current_lidar;
  result.current_lidar.enabled = true;

  if (config.use_static_map && sources.static_grid == nullptr) {
    result.status = PlanningGridStatus::kStaticMapEnabledButMissing;
    return result;
  }

  const std::optional<GridBounds> bounds = selectPlanningGridBounds(config, sources);
  if (!bounds.has_value()) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }

  OccupancyGrid2D planning_grid{*bounds};
  if (config.use_static_map && sources.static_grid != nullptr) {
    const GridOverlayStats static_overlay =
        overlayOccupiedCells(planning_grid, *sources.static_grid);
    result.static_source.occupied_cells = static_overlay.source_occupied_cells;
    result.static_source.used = true;
  }

  if (sources.memory_grid != nullptr) {
    result.memory.source_counts = collectGridStats(*sources.memory_grid);
    result.memory.geometry_matches =
        haveSameGridGeometry(planning_grid, *sources.memory_grid);
    if (result.memory.geometry_matches) {
      result.memory.overlay =
          overlayKnownMemoryCells(planning_grid, *sources.memory_grid);
      result.memory.used = true;
    }
  }

  bool current_lidar_applied = false;
  if (sources.current_lidar_grid != nullptr && result.current_lidar.fresh &&
      result.current_lidar.used &&
      haveSameGridGeometry(planning_grid, *sources.current_lidar_grid)) {
    const GridOverlayStats current_overlay =
        overlayCurrentLidarCells(planning_grid, *sources.current_lidar_grid);
    result.current_lidar.occupied_cells = current_overlay.occupied_cells_applied +
                                          current_overlay.occupied_cells_preserved;
    result.current_lidar.overlay_occupied_cells_applied =
        current_overlay.occupied_cells_applied;
    result.current_lidar.overlay_occupied_cells_preserved =
        current_overlay.occupied_cells_preserved;
    current_lidar_applied = true;
  }

  const bool current_lidar_ready =
      result.current_lidar.used && result.current_lidar.fresh && current_lidar_applied;
  const bool any_source_ready =
      result.static_source.used || result.memory.used || current_lidar_ready;
  if (!any_source_ready) {
    result.status = PlanningGridStatus::kNoReadySourceData;
    return result;
  }

  planning_grid.rebuildInflation(config.inflation_radius_m);
  result.status = PlanningGridStatus::kReady;
  result.grid = std::move(planning_grid);
  return result;
}

} // namespace drone_city_nav
