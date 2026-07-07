#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_passage_validation.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>

namespace drone_city_nav {

enum class PassageLidarPolicy {
  kNormal,
  kIgnoreExpectedWalls,
  kEmergencyBlocker,
};

enum class PassageObstacleClassification {
  kNormalObstacle,
  kExpectedPassageWall,
  kOpeningCorridorBlocker,
};

enum class PassageAwareReuseAction {
  kRunAStar,
  kEmergencyBlocker,
};

struct PassageTraversalSensorPolicyConfig {
  bool enabled{true};
  double activation_margin_m{3.0};
  double opening_corridor_lateral_margin_m{0.75};
  double opening_corridor_depth_margin_m{1.0};
  double expected_wall_margin_m{0.5};
};

struct ActivePassageTraversal {
  bool active{false};
  std::string structure_id;
  std::string opening_id;
  PassageStructure structure{};
  PassageOpening opening{};
  double entry_s_m{0.0};
  double exit_s_m{0.0};
  double activation_start_s_m{0.0};
  double activation_end_s_m{0.0};
  double active_s_m{std::numeric_limits<double>::quiet_NaN()};
  double distance_sq_m2{std::numeric_limits<double>::infinity()};
};

struct PassageTraversalSensorPolicyStats {
  bool passage_traversal_active{false};
  PassageLidarPolicy lidar_policy{PassageLidarPolicy::kNormal};
  std::size_t ignored_expected_obstacle_count{0U};
  std::size_t emergency_blocker_count{0U};
  std::size_t current_lidar_cells_checked{0U};
  std::size_t memory_cells_checked{0U};
  std::size_t current_lidar_expected_wall_cells{0U};
  std::size_t memory_expected_wall_cells{0U};
  std::string active_structure_id;
  std::string active_opening_id;
  double active_s_m{std::numeric_limits<double>::quiet_NaN()};
  ActivePassageTraversal active_passage{};
};

struct PassageTraversalSensorPolicyInput {
  PassageTraversalSensorPolicyConfig config{};
  KnownPassageValidationConfig validation_config{};
  const KnownPassageMap* known_passage_map{nullptr};
  std::span<const TrajectoryPointSample> trajectory_samples;
  Point2 current_position{};
  const OccupancyGrid2D* memory_grid{nullptr};
  OccupancyGrid2D* current_lidar_grid{nullptr};
};

struct PassageTraversalSensorPolicyResult {
  PassageTraversalSensorPolicyStats stats{};
  std::optional<OccupancyGrid2D> filtered_memory_grid;
};

[[nodiscard]] const char* passageLidarPolicyName(PassageLidarPolicy policy) noexcept;

[[nodiscard]] const char* passageObstacleClassificationName(
    PassageObstacleClassification classification) noexcept;

[[nodiscard]] const char*
passageAwareReuseActionName(PassageAwareReuseAction action) noexcept;

[[nodiscard]] ActivePassageTraversal
findActivePassageTraversal(const PassageTraversalSensorPolicyConfig& config,
                           const KnownPassageValidationConfig& validation_config,
                           const KnownPassageMap* known_passage_map,
                           std::span<const TrajectoryPointSample> trajectory_samples,
                           Point2 current_position);

[[nodiscard]] PassageObstacleClassification
classifyPassageObstaclePoint(const ActivePassageTraversal& active_passage,
                             const PassageTraversalSensorPolicyConfig& config,
                             Point2 point) noexcept;

[[nodiscard]] PassageTraversalSensorPolicyResult
applyPassageTraversalSensorPolicy(const PassageTraversalSensorPolicyInput& input);

[[nodiscard]] PassageAwareReuseAction evaluatePassageAwareProhibitedIntersectionAction(
    const PassageTraversalSensorPolicyStats& stats) noexcept;

} // namespace drone_city_nav
