#pragma once

#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/path_smoothing.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

struct GridStats {
  std::size_t unknown_cells{0U};
  std::size_t free_cells{0U};
  std::size_t occupied_cells{0U};
  std::size_t inflated_cells{0U};
};

struct PathMetrics {
  std::size_t points{0U};
  std::size_t segments{0U};
  std::size_t straight_segments{0U};
  std::size_t turns{0U};
  std::size_t segments_shorter_than_2m{0U};
  std::size_t segments_shorter_than_5m{0U};
  std::size_t segments_shorter_than_10m{0U};
  double length_m{0.0};
  double min_segment_length_m{std::numeric_limits<double>::quiet_NaN()};
  double mean_segment_length_m{std::numeric_limits<double>::quiet_NaN()};
  double max_segment_length_m{std::numeric_limits<double>::quiet_NaN()};
};

struct PlannerCoreConfig {
  AStarConfig astar{};
  int nearest_free_radius_cells{10};
  double clearance_diagnostic_radius_m{10.0};
  double stable_path_goal_tolerance_m{3.0};
  double stable_path_reuse_max_deviation_m{1.0};
};

struct PathComputationResult {
  AStarResult astar{};
  std::vector<GridIndex> smoothed_cells;
  PathSmoothingStats smoothing_stats{};
  GridStats grid_stats{};
  PathMetrics raw_path_metrics{};
  PathMetrics smoothed_path_metrics{};
  double raw_path_clearance_m{std::numeric_limits<double>::infinity()};
  double smoothed_path_clearance_m{std::numeric_limits<double>::infinity()};
  std::optional<GridIndex> start_cell;
  std::optional<GridIndex> goal_cell;
  std::optional<GridIndex> allowed_start_cell;
  std::optional<GridIndex> allowed_goal_cell;
  bool smoothing_returned_empty_path{false};
};

struct PathProjection2D {
  std::size_t segment_start_index{0U};
  double segment_t{0.0};
  double distance_sq{std::numeric_limits<double>::infinity()};
  Point2 point{};
};

enum class StablePathDecisionReason {
  kDisabled,
  kNoPreviousPath,
  kGoalMismatch,
  kProjectionUnavailable,
  kDeviationTooLarge,
  kClear,
  kProhibitedConfirmed,
};

struct StablePathDecision {
  bool keep_path{false};
  StablePathDecisionReason reason{StablePathDecisionReason::kDisabled};
  std::vector<Point2> remaining_path;
  double deviation_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t prohibited_segment_index{0U};
};

[[nodiscard]] const char*
stablePathDecisionReasonName(StablePathDecisionReason reason) noexcept;

[[nodiscard]] GridStats collectGridStats(const OccupancyGrid2D& grid);

[[nodiscard]] PathMetrics gridPathMetrics(const OccupancyGrid2D& grid,
                                          std::span<const GridIndex> path);

[[nodiscard]] PathMetrics pointPathMetrics(std::span<const Point2> path_points);

[[nodiscard]] double pathMinimumProhibitedClearanceM(const OccupancyGrid2D& grid,
                                                     std::span<const GridIndex> path,
                                                     double max_distance_m);

[[nodiscard]] bool pathSegmentIsAllowed(const OccupancyGrid2D& grid, Point2 start,
                                        Point2 end);

[[nodiscard]] bool pathSegmentIsTraversable(const OccupancyGrid2D& grid, Point2 start,
                                            Point2 end);

[[nodiscard]] std::optional<PathProjection2D>
closestPathProjection(std::span<const Point2> path_points, Point2 current_position);

[[nodiscard]] std::optional<std::vector<Point2>> remainingPathFromCurrentPose(
    std::span<const Point2> path_points, Point2 current_position, Point2 goal,
    double stable_path_goal_tolerance_m, double stable_path_reuse_max_deviation_m,
    double& deviation_m);

[[nodiscard]] bool
pathIsTraversable(const OccupancyGrid2D& grid, std::span<const Point2> path_points,
                  std::size_t* non_traversable_segment_index = nullptr);

class PlannerCore {
public:
  explicit PlannerCore(const PlannerCoreConfig& config = PlannerCoreConfig{});

  void setConfig(const PlannerCoreConfig& config);
  [[nodiscard]] const PlannerCoreConfig& config() const noexcept;

  [[nodiscard]] std::optional<PathComputationResult>
  computePath(const OccupancyGrid2D& grid, Point2 current_position, Point2 goal) const;
  [[nodiscard]] std::optional<PathComputationResult>
  computePath(const OccupancyGrid2D& grid, Point2 current_position, Point2 goal,
              const AStarConfig& astar_config) const;

  [[nodiscard]] StablePathDecision
  evaluateStablePath(const OccupancyGrid2D& grid, std::span<const Point2> previous_path,
                     Point2 current_position, Point2 goal) const;

private:
  PlannerCoreConfig config_{};
  AStarPlanner planner_{};
};

} // namespace drone_city_nav
