#include "drone_city_nav/passage_traversal_sensor_policy.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  return OccupancyGrid2D{GridBounds{0.0, -10.0, 1.0, 30, 20}};
}

void setOccupiedAt(OccupancyGrid2D& grid, const Point2 point) {
  const std::optional<GridIndex> cell = grid.worldToCell(point);
  if (!cell.has_value()) {
    ADD_FAILURE() << "test point is outside grid";
    return;
  }
  grid.setOccupied(cell.value());
}

[[nodiscard]] bool occupiedAt(const OccupancyGrid2D& grid, const Point2 point) {
  const std::optional<GridIndex> cell = grid.worldToCell(point);
  EXPECT_TRUE(cell.has_value());
  return cell.has_value() && grid.isOccupied(*cell);
}

[[nodiscard]] KnownPassageMap makePassageMap() {
  PassageOpening opening{};
  opening.id = "opening_a";
  opening.structure_id = "structure_a";
  opening.center = Point3{10.0, 0.0, 5.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 4.0;
  opening.height_m = 2.0;
  opening.depth_m = 4.0;
  opening.min_z_m = 4.0;
  opening.max_z_m = 6.0;

  PassageStructure structure{};
  structure.id = "structure_a";
  structure.center = Point2{10.0, 0.0};
  structure.size_x_m = 4.0;
  structure.size_y_m = 10.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 10.0;
  structure.openings.push_back(opening);

  KnownPassageMap map{};
  map.frame_id = "map";
  map.structures.push_back(structure);
  return map;
}

[[nodiscard]] KnownPassageMap makeTwoPassageMap() {
  KnownPassageMap map = makePassageMap();
  PassageOpening opening{};
  opening.id = "opening_b";
  opening.structure_id = "structure_b";
  opening.center = Point3{20.0, 0.0, 5.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 4.0;
  opening.height_m = 2.0;
  opening.depth_m = 4.0;
  opening.min_z_m = 4.0;
  opening.max_z_m = 6.0;

  PassageStructure structure{};
  structure.id = "structure_b";
  structure.center = Point2{20.0, 0.0};
  structure.size_x_m = 4.0;
  structure.size_y_m = 10.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 10.0;
  structure.openings.push_back(opening);
  map.structures.push_back(structure);
  return map;
}

[[nodiscard]] std::vector<TrajectoryPointSample> makeTrajectorySamples() {
  TrajectoryPointSample start{};
  start.s_m = 0.0;
  start.point = Point2{0.0, 0.0};
  start.tangent = Point2{1.0, 0.0};
  start.z_m = 5.0;
  start.vertical_profile_passage_id = "opening_a";

  TrajectoryPointSample end{};
  end.s_m = 30.0;
  end.point = Point2{30.0, 0.0};
  end.tangent = Point2{1.0, 0.0};
  end.z_m = 5.0;
  end.vertical_profile_passage_id = "opening_a";
  return {start, end};
}

[[nodiscard]] PassageTraversalSensorPolicyInput
makeInput(const KnownPassageMap& map, std::span<const TrajectoryPointSample> samples,
          Point2 current_position, const OccupancyGrid2D* memory_grid,
          OccupancyGrid2D* current_lidar_grid) {
  PassageTraversalSensorPolicyInput input{};
  input.known_passage_map = &map;
  input.trajectory_samples = samples;
  input.current_position = current_position;
  input.memory_grid = memory_grid;
  input.current_lidar_grid = current_lidar_grid;
  input.validation_config.enabled = true;
  input.validation_config.min_opening_overlap_m = 0.5;
  input.validation_config.clearance_margin_m = 0.0;
  input.config.enabled = true;
  input.config.activation_margin_m = 3.0;
  input.config.opening_corridor_lateral_margin_m = 0.75;
  input.config.opening_corridor_depth_margin_m = 1.0;
  input.config.expected_wall_margin_m = 0.5;
  return input;
}

} // namespace

TEST(PassageTraversalSensorPolicy, InactivePolicyDoesNotModifySources) {
  const KnownPassageMap map = makePassageMap();
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();
  OccupancyGrid2D current_lidar = makeGrid();
  setOccupiedAt(current_lidar, Point2{10.2, 4.2});

  PassageTraversalSensorPolicyInput input =
      makeInput(map, samples, Point2{10.0, 0.0}, nullptr, &current_lidar);
  input.config.enabled = false;

  const PassageTraversalSensorPolicyResult result =
      applyPassageTraversalSensorPolicy(input);

  EXPECT_FALSE(result.stats.passage_traversal_active);
  EXPECT_EQ(result.stats.lidar_policy, PassageLidarPolicy::kNormal);
  EXPECT_FALSE(result.filtered_memory_grid.has_value());
  EXPECT_TRUE(occupiedAt(current_lidar, Point2{10.2, 4.2}));
}

TEST(PassageTraversalSensorPolicy, MissingKnownPassageMapKeepsNormalPolicy) {
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();
  OccupancyGrid2D current_lidar = makeGrid();
  setOccupiedAt(current_lidar, Point2{10.2, 4.2});

  PassageTraversalSensorPolicyInput input{};
  input.trajectory_samples = samples;
  input.current_position = Point2{10.0, 0.0};
  input.current_lidar_grid = &current_lidar;

  const PassageTraversalSensorPolicyResult result =
      applyPassageTraversalSensorPolicy(input);

  EXPECT_FALSE(result.stats.passage_traversal_active);
  EXPECT_EQ(result.stats.lidar_policy, PassageLidarPolicy::kNormal);
  EXPECT_TRUE(occupiedAt(current_lidar, Point2{10.2, 4.2}));
}

TEST(PassageTraversalSensorPolicy, ExpectedWallCellsAreIgnoredDuringActiveTraversal) {
  const KnownPassageMap map = makePassageMap();
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();
  OccupancyGrid2D current_lidar = makeGrid();
  setOccupiedAt(current_lidar, Point2{10.2, 4.2});

  const PassageTraversalSensorPolicyResult result = applyPassageTraversalSensorPolicy(
      makeInput(map, samples, Point2{10.0, 0.0}, nullptr, &current_lidar));

  EXPECT_TRUE(result.stats.passage_traversal_active);
  EXPECT_EQ(result.stats.lidar_policy, PassageLidarPolicy::kIgnoreExpectedWalls);
  EXPECT_EQ(result.stats.ignored_expected_obstacle_count, 1U);
  EXPECT_EQ(result.stats.emergency_blocker_count, 0U);
  EXPECT_EQ(result.stats.current_lidar_expected_wall_cells, 1U);
  EXPECT_FALSE(occupiedAt(current_lidar, Point2{10.2, 4.2}));
}

TEST(PassageTraversalSensorPolicy, ExpectedWallCellsAreIgnoredBeforeOpeningEntry) {
  const KnownPassageMap map = makePassageMap();
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();
  OccupancyGrid2D current_lidar = makeGrid();
  setOccupiedAt(current_lidar, Point2{10.2, 4.2});

  PassageTraversalSensorPolicyInput input =
      makeInput(map, samples, Point2{4.0, 0.0}, nullptr, &current_lidar);
  input.config.activation_margin_m = 3.0;
  input.config.lookahead_margin_m = 8.0;

  const PassageTraversalSensorPolicyResult result =
      applyPassageTraversalSensorPolicy(input);

  EXPECT_TRUE(result.stats.passage_traversal_active);
  EXPECT_EQ(result.stats.lidar_policy, PassageLidarPolicy::kIgnoreExpectedWalls);
  EXPECT_EQ(result.stats.current_lidar_expected_wall_cells, 1U);
  EXPECT_FALSE(occupiedAt(current_lidar, Point2{10.2, 4.2}));
}

TEST(PassageTraversalSensorPolicy, OpeningCorridorBlockerRemainsOccupiedAndEmergency) {
  const KnownPassageMap map = makePassageMap();
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();
  OccupancyGrid2D current_lidar = makeGrid();
  setOccupiedAt(current_lidar, Point2{10.2, 0.2});

  const PassageTraversalSensorPolicyResult result = applyPassageTraversalSensorPolicy(
      makeInput(map, samples, Point2{10.0, 0.0}, nullptr, &current_lidar));

  EXPECT_TRUE(result.stats.passage_traversal_active);
  EXPECT_EQ(result.stats.lidar_policy, PassageLidarPolicy::kEmergencyBlocker);
  EXPECT_EQ(result.stats.ignored_expected_obstacle_count, 0U);
  EXPECT_EQ(result.stats.emergency_blocker_count, 1U);
  EXPECT_TRUE(occupiedAt(current_lidar, Point2{10.2, 0.2}));
}

TEST(PassageTraversalSensorPolicy, MemoryFilteringUsesCopyAndLeavesOriginalUnchanged) {
  const KnownPassageMap map = makePassageMap();
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();
  OccupancyGrid2D memory = makeGrid();
  setOccupiedAt(memory, Point2{10.2, 4.2});

  const PassageTraversalSensorPolicyResult result = applyPassageTraversalSensorPolicy(
      makeInput(map, samples, Point2{10.0, 0.0}, &memory, nullptr));

  if (!result.filtered_memory_grid.has_value()) {
    ADD_FAILURE() << "expected filtered memory copy";
    return;
  }
  EXPECT_EQ(result.stats.memory_cells_checked, 1U);
  EXPECT_EQ(result.stats.memory_expected_wall_cells, 1U);
  EXPECT_TRUE(occupiedAt(memory, Point2{10.2, 4.2}));
  EXPECT_FALSE(occupiedAt(result.filtered_memory_grid.value(), Point2{10.2, 4.2}));
}

TEST(PassageTraversalSensorPolicy, NormalObstacleOutsideStructureIsNotFiltered) {
  const KnownPassageMap map = makePassageMap();
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();
  OccupancyGrid2D current_lidar = makeGrid();
  setOccupiedAt(current_lidar, Point2{15.2, 4.2});

  const PassageTraversalSensorPolicyResult result = applyPassageTraversalSensorPolicy(
      makeInput(map, samples, Point2{10.0, 0.0}, nullptr, &current_lidar));

  EXPECT_TRUE(result.stats.passage_traversal_active);
  EXPECT_EQ(result.stats.lidar_policy, PassageLidarPolicy::kNormal);
  EXPECT_EQ(result.stats.ignored_expected_obstacle_count, 0U);
  EXPECT_TRUE(occupiedAt(current_lidar, Point2{15.2, 4.2}));
}

TEST(PassageTraversalSensorPolicy, ChoosesActiveSpanClosestToCurrentStation) {
  const KnownPassageMap map = makeTwoPassageMap();
  const std::vector<TrajectoryPointSample> samples = makeTrajectorySamples();

  PassageTraversalSensorPolicyConfig config{};
  config.activation_margin_m = 15.0;
  KnownPassageValidationConfig validation_config{};
  validation_config.enabled = true;
  validation_config.min_opening_overlap_m = 0.5;

  const ActivePassageTraversal active = findActivePassageTraversal(
      config, validation_config, &map, samples, Point2{12.0, 0.0});

  EXPECT_TRUE(active.active);
  EXPECT_EQ(active.structure_id, "structure_a");
  EXPECT_EQ(active.opening_id, "opening_a");
}

TEST(PassageTraversalSensorPolicy, ReuseDecisionNeverSuppressesHardProhibited) {
  PassageTraversalSensorPolicyStats stats{};
  stats.passage_traversal_active = true;
  stats.active_passage.active = true;
  stats.active_passage.structure = makePassageMap().structures.front();
  stats.active_passage.opening = stats.active_passage.structure.openings.front();

  EXPECT_EQ(evaluatePassageAwareProhibitedIntersectionAction(stats),
            PassageAwareReuseAction::kRunAStar);

  stats.emergency_blocker_count = 1U;
  EXPECT_EQ(evaluatePassageAwareProhibitedIntersectionAction(stats),
            PassageAwareReuseAction::kEmergencyBlocker);
}

} // namespace drone_city_nav
