#include "drone_city_nav/passage_traversal_sensor_policy.hpp"

#include "drone_city_nav/known_passage_matching.hpp"
#include "drone_city_nav/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-9;

struct TrajectoryStationProjection {
  bool valid{false};
  double s_m{std::numeric_limits<double>::quiet_NaN()};
  double distance_sq_m2{std::numeric_limits<double>::infinity()};
};

struct OpeningLocalPoint2D {
  double u{0.0};
  double v{0.0};
};

[[nodiscard]] bool finitePoint(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] TrajectoryStationProjection
projectPointToTrajectorySamples(const std::span<const TrajectoryPointSample> samples,
                                const Point2 point) noexcept {
  TrajectoryStationProjection best{};
  if (!finitePoint(point) || samples.size() < 2U) {
    return best;
  }

  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const Point2 start = samples[i].point;
    const Point2 end = samples[i + 1U].point;
    if (!finitePoint(start) || !finitePoint(end) || !std::isfinite(samples[i].s_m) ||
        !std::isfinite(samples[i + 1U].s_m)) {
      continue;
    }
    const Point2 segment{end.x - start.x, end.y - start.y};
    const double length_sq = segment.x * segment.x + segment.y * segment.y;
    if (!(length_sq > kTinyDistanceM)) {
      continue;
    }

    const Point2 relative{point.x - start.x, point.y - start.y};
    const double t = std::clamp(
        (relative.x * segment.x + relative.y * segment.y) / length_sq, 0.0, 1.0);
    const Point2 projected{start.x + segment.x * t, start.y + segment.y * t};
    const double distance_sq = squaredDistance(point, projected);
    if (distance_sq >= best.distance_sq_m2) {
      continue;
    }

    const double station_delta = samples[i + 1U].s_m - samples[i].s_m;
    best.valid = true;
    best.s_m = samples[i].s_m + station_delta * t;
    best.distance_sq_m2 = distance_sq;
  }

  return best;
}

[[nodiscard]] const PassageStructure*
findStructureById(const KnownPassageMap& map,
                  const std::string& structure_id) noexcept {
  const auto match = std::ranges::find_if(
      map.structures, [&structure_id](const PassageStructure& item) {
        return item.id == structure_id;
      });
  return match == map.structures.end() ? nullptr : &*match;
}

[[nodiscard]] double stationDistanceToInterval(const double s_m, const double start_s_m,
                                               const double end_s_m) noexcept {
  if (s_m < start_s_m) {
    return start_s_m - s_m;
  }
  if (s_m > end_s_m) {
    return s_m - end_s_m;
  }
  return 0.0;
}

[[nodiscard]] OpeningLocalPoint2D
toOpeningLocalPoint(const Point2 point, const PassageOpening& opening) noexcept {
  const double dx = point.x - opening.center.x;
  const double dy = point.y - opening.center.y;
  const Point2 normal = opening.normal_xy;
  const Point2 lateral{-normal.y, normal.x};
  return OpeningLocalPoint2D{
      .u = dx * normal.x + dy * normal.y,
      .v = dx * lateral.x + dy * lateral.y,
  };
}

[[nodiscard]] bool insideStructureFootprint(const Point2 point,
                                            const PassageStructure& structure,
                                            const double margin_m) noexcept {
  const double half_x = (structure.size_x_m / 2.0) + margin_m;
  const double half_y = (structure.size_y_m / 2.0) + margin_m;
  return std::abs(point.x - structure.center.x) <= half_x &&
         std::abs(point.y - structure.center.y) <= half_y;
}

[[nodiscard]] bool insideOpeningCorridor(const Point2 point,
                                         const PassageOpening& opening,
                                         const double lateral_margin_m,
                                         const double depth_margin_m) noexcept {
  const OpeningLocalPoint2D local = toOpeningLocalPoint(point, opening);
  const double half_width = (opening.width_m / 2.0) + lateral_margin_m;
  const double half_depth = (opening.depth_m / 2.0) + depth_margin_m;
  return std::abs(local.v) <= half_width && std::abs(local.u) <= half_depth;
}

void updatePolicyAfterClassification(PassageTraversalSensorPolicyStats& stats,
                                     const PassageObstacleClassification classification,
                                     const bool current_lidar_source) noexcept {
  switch (classification) {
    case PassageObstacleClassification::kNormalObstacle:
      return;
    case PassageObstacleClassification::kExpectedPassageWall:
      ++stats.ignored_expected_obstacle_count;
      if (current_lidar_source) {
        ++stats.current_lidar_expected_wall_cells;
      } else {
        ++stats.memory_expected_wall_cells;
      }
      if (stats.lidar_policy != PassageLidarPolicy::kEmergencyBlocker) {
        stats.lidar_policy = PassageLidarPolicy::kIgnoreExpectedWalls;
      }
      return;
    case PassageObstacleClassification::kOpeningCorridorBlocker:
      ++stats.emergency_blocker_count;
      stats.lidar_policy = PassageLidarPolicy::kEmergencyBlocker;
      return;
  }
}

void filterOccupiedSourceGrid(OccupancyGrid2D& grid,
                              PassageTraversalSensorPolicyStats& stats,
                              const PassageTraversalSensorPolicyConfig& config,
                              const bool current_lidar_source) {
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (!grid.isOccupied(cell)) {
        continue;
      }
      if (current_lidar_source) {
        ++stats.current_lidar_cells_checked;
      } else {
        ++stats.memory_cells_checked;
      }

      const Point2 center = grid.cellCenter(cell);
      const PassageObstacleClassification classification =
          classifyPassageObstaclePoint(stats.active_passage, config, center);
      updatePolicyAfterClassification(stats, classification, current_lidar_source);
      if (classification == PassageObstacleClassification::kExpectedPassageWall) {
        grid.setUnknown(cell);
      }
    }
  }
}

} // namespace

const char* passageLidarPolicyName(const PassageLidarPolicy policy) noexcept {
  switch (policy) {
    case PassageLidarPolicy::kNormal:
      return "normal";
    case PassageLidarPolicy::kIgnoreExpectedWalls:
      return "ignore_expected_walls";
    case PassageLidarPolicy::kEmergencyBlocker:
      return "emergency_blocker";
  }
  return "unknown";
}

const char* passageObstacleClassificationName(
    const PassageObstacleClassification classification) noexcept {
  switch (classification) {
    case PassageObstacleClassification::kNormalObstacle:
      return "normal_obstacle";
    case PassageObstacleClassification::kExpectedPassageWall:
      return "expected_passage_wall";
    case PassageObstacleClassification::kOpeningCorridorBlocker:
      return "opening_corridor_blocker";
  }
  return "unknown";
}

const char* passageAwareReuseActionName(const PassageAwareReuseAction action) noexcept {
  switch (action) {
    case PassageAwareReuseAction::kRunAStar:
      return "run_astar";
    case PassageAwareReuseAction::kEmergencyBlocker:
      return "emergency_blocker";
  }
  return "unknown";
}

ActivePassageTraversal findActivePassageTraversal(
    const PassageTraversalSensorPolicyConfig& config,
    const KnownPassageValidationConfig& validation_config,
    const KnownPassageMap* const known_passage_map,
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const Point2 current_position) {
  ActivePassageTraversal active{};
  if (!config.enabled || known_passage_map == nullptr ||
      !trajectorySamplesAreUsable(trajectory_samples)) {
    return active;
  }

  const TrajectoryStationProjection projection =
      projectPointToTrajectorySamples(trajectory_samples, current_position);
  if (!projection.valid || !std::isfinite(projection.s_m)) {
    return active;
  }

  const std::vector<KnownPassageTraversalMatch> matches =
      findKnownPassageTraversalMatches(trajectory_samples, *known_passage_map,
                                       validation_config, false);
  double best_station_distance_m = std::numeric_limits<double>::infinity();
  for (const KnownPassageTraversalMatch& match : matches) {
    if (!match.valid || match.opening_id.empty()) {
      continue;
    }
    const double activation_start_s_m =
        std::max(0.0, match.entry_s_m - config.activation_margin_m);
    const double activation_end_s_m = match.exit_s_m + config.activation_margin_m;
    if (projection.s_m < activation_start_s_m || projection.s_m > activation_end_s_m) {
      continue;
    }

    const PassageStructure* const structure =
        findStructureById(*known_passage_map, match.structure_id);
    if (structure == nullptr) {
      continue;
    }

    const double station_distance_m =
        stationDistanceToInterval(projection.s_m, match.entry_s_m, match.exit_s_m);
    if (station_distance_m >= best_station_distance_m) {
      continue;
    }

    best_station_distance_m = station_distance_m;
    active.active = true;
    active.structure_id = match.structure_id;
    active.opening_id = match.opening_id;
    active.structure = *structure;
    active.opening = match.opening;
    active.entry_s_m = match.entry_s_m;
    active.exit_s_m = match.exit_s_m;
    active.activation_start_s_m = activation_start_s_m;
    active.activation_end_s_m = activation_end_s_m;
    active.active_s_m = projection.s_m;
    active.distance_sq_m2 = projection.distance_sq_m2;
  }

  return active;
}

PassageObstacleClassification
classifyPassageObstaclePoint(const ActivePassageTraversal& active_passage,
                             const PassageTraversalSensorPolicyConfig& config,
                             const Point2 point) noexcept {
  if (!active_passage.active || !finitePoint(point)) {
    return PassageObstacleClassification::kNormalObstacle;
  }
  if (insideOpeningCorridor(point, active_passage.opening,
                            std::max(0.0, config.opening_corridor_lateral_margin_m),
                            std::max(0.0, config.opening_corridor_depth_margin_m))) {
    return PassageObstacleClassification::kOpeningCorridorBlocker;
  }
  if (insideStructureFootprint(point, active_passage.structure,
                               std::max(0.0, config.expected_wall_margin_m))) {
    return PassageObstacleClassification::kExpectedPassageWall;
  }
  return PassageObstacleClassification::kNormalObstacle;
}

PassageTraversalSensorPolicyResult
applyPassageTraversalSensorPolicy(const PassageTraversalSensorPolicyInput& input) {
  PassageTraversalSensorPolicyResult result{};
  result.stats.active_passage = findActivePassageTraversal(
      input.config, input.validation_config, input.known_passage_map,
      input.trajectory_samples, input.current_position);
  result.stats.passage_traversal_active = result.stats.active_passage.active;
  result.stats.active_structure_id = result.stats.active_passage.structure_id;
  result.stats.active_opening_id = result.stats.active_passage.opening_id;
  result.stats.active_s_m = result.stats.active_passage.active_s_m;
  if (!result.stats.passage_traversal_active) {
    return result;
  }

  if (input.current_lidar_grid != nullptr) {
    filterOccupiedSourceGrid(*input.current_lidar_grid, result.stats, input.config,
                             true);
  }
  if (input.memory_grid != nullptr) {
    result.filtered_memory_grid = *input.memory_grid;
    filterOccupiedSourceGrid(*result.filtered_memory_grid, result.stats, input.config,
                             false);
  }

  return result;
}

PassageAwareReuseAction evaluatePassageAwareProhibitedIntersectionAction(
    const PassageTraversalSensorPolicyStats& stats) noexcept {
  if (stats.emergency_blocker_count > 0U) {
    return PassageAwareReuseAction::kEmergencyBlocker;
  }
  return PassageAwareReuseAction::kRunAStar;
}

} // namespace drone_city_nav
