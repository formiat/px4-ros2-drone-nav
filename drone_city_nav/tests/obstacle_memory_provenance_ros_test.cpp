#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] nav_msgs::msg::OccupancyGrid makeGrid(const int occupied_x = 2) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp.sec = 12;
  grid.header.stamp.nanosec = 34U;
  grid.header.frame_id = "map";
  grid.info.map_load_time = grid.header.stamp;
  grid.info.resolution = 1.0F;
  grid.info.width = 4U;
  grid.info.height = 3U;
  grid.info.origin.position.x = 10.0;
  grid.info.origin.position.y = 20.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(12U, static_cast<std::int8_t>(-1));
  if (occupied_x >= 0) {
    grid.data[static_cast<std::size_t>(occupied_x)] = 100;
  }
  return grid;
}

[[nodiscard]] AcceptedObstacleMemoryHit makeHit(const double z,
                                                const std::int64_t stamp_ns) {
  AcceptedObstacleMemoryHit hit;
  hit.beam.beam_index = 17U;
  hit.beam.acquisition_stamp_ns = stamp_ns;
  hit.beam.acquisition_stamp_valid = true;
  hit.beam.receive_stamp_ns = stamp_ns + 1000;
  hit.beam.receive_stamp_valid = true;
  hit.beam.projection.ray_origin_map_m = Point3{10.5, 20.5, 18.0};
  hit.beam.projection.ray_direction_map = Point3{1.0, 0.0, 0.0};
  hit.beam.projection.endpoint = Point2{12.5, 20.5};
  hit.beam.projection.endpoint_map_m = Point3{12.5, 20.5, z};
  hit.beam.projection.endpoint_xyz_valid = true;
  hit.beam.projection.attitude_compensation_applied = true;
  hit.beam.projection.applied_roll_rad = 0.1;
  hit.beam.projection.applied_pitch_rad = 0.2;
  hit.beam.projection.applied_tilt_rad = std::hypot(0.1, 0.2);
  hit.beam.measured_range_m = 2.0;
  hit.beam.source_attitude_valid = true;
  hit.beam.source_roll_rad = 0.1;
  hit.beam.source_pitch_rad = 0.2;
  hit.beam.source_tilt_rad = std::hypot(0.1, 0.2);
  hit.known_static.classifier_applied = true;
  hit.known_static.classification = KnownStaticLidarHitClassification::kUnexpected;
  hit.known_static.volume_matched = true;
  hit.known_static.confident_face_interior = true;
  hit.known_static.part_kind_valid = true;
  hit.known_static.part_kind = KnownPassageSolidPartKind::kUpper;
  hit.known_static.structure_id = "building";
  hit.known_static.opening_id = "opening";
  hit.known_static.part_id = "upper_mass";
  hit.known_static.expected_range_m = 3.0;
  hit.known_static.range_delta_m = -1.0;
  hit.ingestion_decision = LidarIngestionDecisionSnapshot{
      .action = LidarIngestionAction::kIntegrateFreeAndHit,
      .reason = LidarIngestionReason::kObstacleBeforeExpectedSurface,
      .expected_surface = LidarExpectedSurfaceKind::kGround,
      .expected_range_m = 4.0,
      .range_delta_m = -2.0,
  };
  return hit;
}

[[nodiscard]] std::unordered_map<std::size_t, MemoryCellProvenance> makeProvenance() {
  MemoryCellProvenance record;
  record.cell = GridIndex{2, 0};
  record.occupancy_trigger = makeHit(17.0, 1'000'000'000LL);
  record.last_hit = makeHit(19.0, 2'000'000'000LL);
  record.min_endpoint_z_m = 17.0;
  record.max_endpoint_z_m = 19.0;
  record.accepted_hit_count = std::numeric_limits<std::uint64_t>::max();
  return {{2U, record}};
}

} // namespace

TEST(ObstacleMemoryProvenanceRos, RoundTripsPersistentRecordWithoutNarrowing) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const msg::ObstacleMemoryProvenance message =
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance());

  const MemoryProvenanceParseResult parsed =
      parseObstacleMemoryProvenanceMessage(message);

  ASSERT_TRUE(parsed.snapshot.has_value());
  const MemoryProvenanceSnapshot& snapshot =
      parsed.snapshot.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_EQ(snapshot.cells.size(), 1U);
  const MemoryCellProvenance& record = snapshot.cells.at(2U);
  EXPECT_EQ(record.accepted_hit_count, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(record.occupancy_trigger.beam.acquisition_stamp_ns, 1'000'000'000LL);
  EXPECT_EQ(record.last_hit.known_static.part_id, "upper_mass");
  EXPECT_EQ(record.occupancy_trigger.ingestion_decision.action,
            LidarIngestionAction::kIntegrateFreeAndHit);
  EXPECT_EQ(record.occupancy_trigger.ingestion_decision.reason,
            LidarIngestionReason::kObstacleBeforeExpectedSurface);
  EXPECT_EQ(record.occupancy_trigger.ingestion_decision.expected_surface,
            LidarExpectedSurfaceKind::kGround);
  EXPECT_DOUBLE_EQ(record.occupancy_trigger.ingestion_decision.expected_range_m, 4.0);
  EXPECT_DOUBLE_EQ(record.occupancy_trigger.ingestion_decision.range_delta_m, -2.0);
  ASSERT_TRUE(record.min_endpoint_z_m.has_value());
  ASSERT_TRUE(record.max_endpoint_z_m.has_value());
  EXPECT_DOUBLE_EQ(record.min_endpoint_z_m.value_or(0.0), 17.0);
  EXPECT_DOUBLE_EQ(record.max_endpoint_z_m.value_or(0.0), 19.0);
}

TEST(ObstacleMemoryProvenanceRos, SupportsEmptySnapshotWithGridIdentity) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid(-1);
  const msg::ObstacleMemoryProvenance message =
      makeObstacleMemoryProvenanceMessage(grid, {});
  const MemoryProvenanceParseResult parsed =
      parseObstacleMemoryProvenanceMessage(message);
  ASSERT_TRUE(parsed.snapshot.has_value());
  const MemoryProvenanceSnapshot& snapshot =
      parsed.snapshot.value(); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(snapshot.cells.empty());

  MemoryProvenanceCache cache;
  cache.insert(snapshot);
  EXPECT_EQ(cache.match(grid).reason, MemoryProvenanceUnavailableReason::kNone);
}

TEST(ObstacleMemoryProvenanceRos, PreservesInvalidEndpointZExplicitly) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  auto provenance = makeProvenance();
  MemoryCellProvenance& record = provenance.at(2U);
  record.occupancy_trigger.beam.projection.endpoint_xyz_valid = false;
  record.last_hit.beam.projection.endpoint_xyz_valid = false;
  record.min_endpoint_z_m.reset();
  record.max_endpoint_z_m.reset();

  const auto parsed = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(grid, provenance));

  ASSERT_TRUE(parsed.snapshot.has_value());
  const MemoryProvenanceSnapshot& snapshot =
      parsed.snapshot.value(); // NOLINT(bugprone-unchecked-optional-access)
  const MemoryCellProvenance& parsed_record = snapshot.cells.at(2U);
  EXPECT_FALSE(parsed_record.occupancy_trigger.beam.projection.endpoint_xyz_valid);
  EXPECT_FALSE(parsed_record.min_endpoint_z_m.has_value());
  EXPECT_DOUBLE_EQ(parsed_record.occupancy_trigger.beam.projection.endpoint.x, 12.5);
}

TEST(ObstacleMemoryProvenanceRos, RejectsSchemaDuplicateAndExpectedStaticRecords) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  msg::ObstacleMemoryProvenance message =
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.schema_version = 999U;
  EXPECT_EQ(parseObstacleMemoryProvenanceMessage(message).reason,
            MemoryProvenanceUnavailableReason::kSchemaInvalid);

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.push_back(message.cells.front());
  message.occupied_cell_count = 2U;
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().occupancy_trigger.classification =
      msg::ObstacleMemoryHitObservation::CLASSIFICATION_EXPECTED_STATIC;
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().last_hit.endpoint_map_m.x += 1.0;
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().last_hit.endpoint_map_m.x = std::numeric_limits<double>::max();
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().occupancy_trigger.ingestion_action = 255U;
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().occupancy_trigger.ingestion_action =
      msg::ObstacleMemoryHitObservation::INGESTION_ACTION_SUPPRESS_ALL;
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().occupancy_trigger.ingestion_expected_range_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  constexpr std::array<std::uint8_t, 4U> impossible_accepted_reasons{
      msg::ObstacleMemoryHitObservation::INGESTION_REASON_EXPECTED_KNOWN_STATIC,
      msg::ObstacleMemoryHitObservation::INGESTION_REASON_EXPECTED_GROUND,
      msg::ObstacleMemoryHitObservation::INGESTION_REASON_AMBIGUOUS_GROUND,
      msg::ObstacleMemoryHitObservation::INGESTION_REASON_TIED_EXPECTED_SURFACES,
  };
  for (const std::uint8_t reason : impossible_accepted_reasons) {
    message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
    message.cells.front().occupancy_trigger.ingestion_reason = reason;
    EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());
  }

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().occupancy_trigger.ingestion_expected_surface =
      msg::ObstacleMemoryHitObservation::EXPECTED_SURFACE_NONE;
  message.cells.front().occupancy_trigger.ingestion_expected_range_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().occupancy_trigger.ingestion_range_delta_m = 1.0;
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());

  message = makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  message.cells.front().occupancy_trigger.ingestion_reason =
      msg::ObstacleMemoryHitObservation::INGESTION_REASON_NO_EXPECTED_SURFACE;
  EXPECT_FALSE(parseObstacleMemoryProvenanceMessage(message).snapshot.has_value());
}

TEST(ObstacleMemoryProvenanceRos, SerializedSizeIncludesStringsAndCells) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const msg::ObstacleMemoryProvenance base =
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  msg::ObstacleMemoryProvenance longer_strings = base;
  longer_strings.cells.front().occupancy_trigger.structure_id.append(2048U, 'x');
  const msg::ObstacleMemoryProvenance additional_cell = [&longer_strings]() {
    msg::ObstacleMemoryProvenance message = longer_strings;
    message.cells.push_back(message.cells.front());
    return message;
  }();

  const std::size_t base_size = serializedObstacleMemoryProvenanceSize(base);
  const std::size_t longer_strings_size =
      serializedObstacleMemoryProvenanceSize(longer_strings);
  const std::size_t additional_cell_size =
      serializedObstacleMemoryProvenanceSize(additional_cell);

  EXPECT_GE(longer_strings_size, base_size + 2048U);
  EXPECT_GT(additional_cell_size, longer_strings_size);
}

TEST(ObstacleMemoryProvenanceRos, CacheRequiresExactSnapshotIdentityAndContent) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const auto parsed = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance()));
  ASSERT_TRUE(parsed.snapshot.has_value());
  MemoryProvenanceCache cache{2U};
  cache.insert(parsed.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_EQ(cache.match(grid).reason, MemoryProvenanceUnavailableReason::kNone);

  nav_msgs::msg::OccupancyGrid stale = grid;
  ++stale.header.stamp.nanosec;
  EXPECT_EQ(cache.match(stale).reason,
            MemoryProvenanceUnavailableReason::kStampMismatch);
  nav_msgs::msg::OccupancyGrid wrong_frame = grid;
  wrong_frame.header.frame_id = "other";
  EXPECT_EQ(cache.match(wrong_frame).reason,
            MemoryProvenanceUnavailableReason::kFrameMismatch);
  nav_msgs::msg::OccupancyGrid wrong_geometry = grid;
  wrong_geometry.info.origin.position.x += 1.0;
  EXPECT_EQ(cache.match(wrong_geometry).reason,
            MemoryProvenanceUnavailableReason::kGeometryMismatch);
  nav_msgs::msg::OccupancyGrid wrong_content = grid;
  wrong_content.data[2U] = 0;
  EXPECT_EQ(cache.match(wrong_content).reason,
            MemoryProvenanceUnavailableReason::kContentMismatch);
}

TEST(ObstacleMemoryProvenanceRos, SameStampMismatchDoesNotHideExactCandidate) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const auto exact = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance()));
  ASSERT_TRUE(exact.snapshot.has_value());
  nav_msgs::msg::OccupancyGrid different_geometry = grid;
  different_geometry.info.origin.position.x += 1.0;
  auto different = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(different_geometry, makeProvenance()));
  EXPECT_FALSE(different.snapshot.has_value());

  MemoryProvenanceCache cache;
  cache.insert(exact.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)
  MemoryProvenanceSnapshot mismatched =
      exact.snapshot.value(); // NOLINT(bugprone-unchecked-optional-access)
  mismatched.identity.grid_info.origin.position.x += 1.0;
  cache.insert(std::move(mismatched));

  EXPECT_EQ(cache.match(grid).reason, MemoryProvenanceUnavailableReason::kNone);
}

TEST(ObstacleMemoryProvenanceRos, FormatterDistinguishesMatchedAndUnavailable) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const auto parsed = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance()));
  ASSERT_TRUE(parsed.snapshot.has_value());
  MemoryProvenanceCache cache;
  cache.insert(parsed.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)

  const std::string matched =
      formatMemoryProvenanceDiagnostic(cache.match(grid), GridIndex{2, 0});
  EXPECT_NE(matched.find("status=matched"), std::string::npos);
  EXPECT_NE(matched.find("accepted_hits=18446744073709551615"), std::string::npos);
  EXPECT_NE(matched.find("trigger_ingestion_reason=obstacle_before_expected_surface"),
            std::string::npos);
  EXPECT_NE(matched.find("trigger_ingestion_surface=ground"), std::string::npos);
  EXPECT_EQ(formatMemoryProvenanceDiagnostic({}, std::nullopt),
            "memory_provenance[status=not_applicable]");
  EXPECT_EQ(formatMemoryProvenanceDiagnostic({}, GridIndex{2, 0}),
            "memory_provenance[status=unavailable reason=not_received]");
}

TEST(ObstacleMemoryProvenanceRos, CacheEvictsOldestDistinctIdentity) {
  MemoryProvenanceCache cache{2U};
  nav_msgs::msg::OccupancyGrid first = makeGrid();
  nav_msgs::msg::OccupancyGrid second = first;
  nav_msgs::msg::OccupancyGrid third = first;
  second.header.stamp.sec = 13;
  second.info.map_load_time = second.header.stamp;
  third.header.stamp.sec = 14;
  third.info.map_load_time = third.header.stamp;
  for (const auto* grid : {&first, &second, &third}) {
    const auto parsed = parseObstacleMemoryProvenanceMessage(
        makeObstacleMemoryProvenanceMessage(*grid, makeProvenance()));
    ASSERT_TRUE(parsed.snapshot.has_value());
    cache.insert(parsed.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)
  }

  EXPECT_EQ(cache.size(), 2U);
  EXPECT_EQ(cache.match(first).reason,
            MemoryProvenanceUnavailableReason::kStampMismatch);
  EXPECT_EQ(cache.match(second).reason, MemoryProvenanceUnavailableReason::kNone);
  EXPECT_EQ(cache.match(third).reason, MemoryProvenanceUnavailableReason::kNone);
}

} // namespace drone_city_nav
