#pragma once

#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/planner_core.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace drone_city_nav {

enum class PlanningGridStatus {
  kReady,
  kStaticMapEnabledButMissing,
  kNoReadySourceData,
};

struct StaticSourceStats {
  bool enabled{false};
  bool loaded{false};
  bool used{false};
  std::size_t rectangles{0U};
  std::size_t occupied_cells{0U};
  std::string path;
};

struct MemorySourceStats {
  bool enabled{false};
  bool seen{false};
  bool used{false};
  bool geometry_matches{false};
  GridStats source_counts{};
  GridOverlayStats overlay{};
};

struct PlanningGridBuilderConfig {
  bool use_static_map{true};
  GridBounds fallback_bounds{};
  double inflation_radius_m{2.0};
  double planning_clearance_m{2.0};
};

struct PlanningGridSources {
  const OccupancyGrid2D* static_grid{nullptr};
  std::size_t static_rectangles{0U};
  std::size_t static_occupied_cells{0U};
  std::string static_map_path;
  const OccupancyGrid2D* memory_grid{nullptr};
  const OccupancyGrid2D* current_lidar_grid{nullptr};
  CurrentLidarOverlayStats current_lidar{};
};

struct PlanningGridBuildResult {
  PlanningGridStatus status{PlanningGridStatus::kNoReadySourceData};
  std::optional<OccupancyGrid2D> grid;
  std::optional<OccupancyGrid2D> planning_grid;
  std::optional<OccupancyGrid2D> current_lidar_grid;
  StaticSourceStats static_source{};
  MemorySourceStats memory{};
  CurrentLidarOverlayStats current_lidar{};
};

[[nodiscard]] const char* planningGridStatusName(PlanningGridStatus status) noexcept;

[[nodiscard]] std::optional<GridBounds>
selectPlanningGridBounds(const PlanningGridBuilderConfig& config,
                         const PlanningGridSources& sources);

[[nodiscard]] PlanningGridBuildResult
buildPlanningGrid(const PlanningGridBuilderConfig& config,
                  const PlanningGridSources& sources);

} // namespace drone_city_nav
