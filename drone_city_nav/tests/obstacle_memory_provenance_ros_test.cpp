#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <deque>
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

TEST(ObstacleMemoryProvenanceRos, AtomicSnapshotRequiresExactGridProvenancePair) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  msg::ObstacleMemorySnapshot message =
      makeObstacleMemorySnapshotMessage(grid, makeProvenance(), 17U, 23U);

  EXPECT_EQ(message.producer_instance_id, 23U);
  EXPECT_EQ(message.sequence, 17U);

  MemoryProvenanceParseResult parsed = parseObstacleMemorySnapshotMessage(message);
  ASSERT_TRUE(parsed.snapshot.has_value());
  EXPECT_EQ(parsed.reason, MemoryProvenanceUnavailableReason::kNone);

  ++message.grid.header.stamp.nanosec;
  parsed = parseObstacleMemorySnapshotMessage(message);
  EXPECT_FALSE(parsed.snapshot.has_value());
  EXPECT_EQ(parsed.reason, MemoryProvenanceUnavailableReason::kStampMismatch);

  message = makeObstacleMemorySnapshotMessage(grid, makeProvenance());
  message.grid.data.at(2U) = 0;
  parsed = parseObstacleMemorySnapshotMessage(message);
  EXPECT_FALSE(parsed.snapshot.has_value());
  EXPECT_EQ(parsed.reason, MemoryProvenanceUnavailableReason::kContentMismatch);
}

TEST(ObstacleMemoryProvenanceRos,
     SerializerRepairsMalformedDecisionWithoutDroppingOccupiedCell) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  auto provenance = makeProvenance();
  MemoryCellProvenance& record = provenance.at(2U);
  for (AcceptedObstacleMemoryHit* hit : {&record.occupancy_trigger, &record.last_hit}) {
    hit->ingestion_decision.reason = LidarIngestionReason::kUnexpectedKnownStatic;
    hit->ingestion_decision.expected_surface = LidarExpectedSurfaceKind::kKnownStatic;
    hit->ingestion_decision.expected_range_m = std::numeric_limits<double>::quiet_NaN();
    hit->ingestion_decision.range_delta_m = -183.0;
  }

  const msg::ObstacleMemorySnapshot message =
      makeObstacleMemorySnapshotMessage(grid, provenance, 119U, 23U);
  ASSERT_EQ(message.provenance.cells.size(), 1U);
  const msg::ObstacleMemoryHitObservation& serialized =
      message.provenance.cells.front().occupancy_trigger;
  EXPECT_FALSE(serialized.classifier_applied);
  EXPECT_EQ(serialized.ingestion_reason,
            msg::ObstacleMemoryHitObservation::INGESTION_REASON_NO_EXPECTED_SURFACE);
  EXPECT_EQ(serialized.ingestion_expected_surface,
            msg::ObstacleMemoryHitObservation::EXPECTED_SURFACE_NONE);
  EXPECT_TRUE(std::isnan(serialized.ingestion_expected_range_m));
  EXPECT_TRUE(std::isnan(serialized.ingestion_range_delta_m));

  const MemoryProvenanceParseResult parsed =
      parseObstacleMemorySnapshotMessage(message);
  ASSERT_TRUE(parsed.snapshot.has_value());
  EXPECT_EQ(parsed.reason, MemoryProvenanceUnavailableReason::kNone);
  const MemoryProvenanceSnapshot& snapshot =
      parsed.snapshot.value(); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_EQ(snapshot.cells.size(), 1U);
  const AcceptedObstacleMemoryHit& parsed_hit = snapshot.cells.at(2U).occupancy_trigger;
  EXPECT_EQ(parsed_hit.ingestion_decision.reason,
            LidarIngestionReason::kNoExpectedSurface);
  EXPECT_FALSE(parsed_hit.known_static.classifier_applied);
}

TEST(ObstacleMemoryProvenanceRos,
     AtomicSnapshotKeepsCurrentAuditExactWhileCallbackBacklogIsDelayed) {
  nav_msgs::msg::OccupancyGrid current_grid = makeGrid();
  MemoryProvenanceParseResult current = parseObstacleMemorySnapshotMessage(
      makeObstacleMemorySnapshotMessage(current_grid, makeProvenance()));
  ASSERT_TRUE(current.snapshot.has_value());
  MemoryProvenanceSnapshot current_provenance =
      std::move(current.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)

  // Model a reliable KeepLast(1) subscription while the single-threaded planner
  // is busy: future publications replace one another, but cannot separate the
  // grid currently used by planning from its provenance.
  std::deque<msg::ObstacleMemorySnapshot> callback_backlog;
  for (std::int32_t sec = 13; sec < 77; ++sec) {
    nav_msgs::msg::OccupancyGrid newer_grid = makeGrid();
    newer_grid.header.stamp.sec = sec;
    newer_grid.info.map_load_time = newer_grid.header.stamp;
    callback_backlog.clear();
    callback_backlog.push_back(
        makeObstacleMemorySnapshotMessage(newer_grid, makeProvenance()));
  }

  const std::string before_callback = formatMemoryProvenanceDiagnostic(
      MemoryProvenanceMatchResult{&current_provenance,
                                  MemoryProvenanceUnavailableReason::kNone},
      GridIndex{2, 0});
  EXPECT_NE(before_callback.find("status=matched"), std::string::npos);
  EXPECT_NE(before_callback.find("trigger_endpoint=(12.5,20.5,17)"), std::string::npos);
  EXPECT_NE(before_callback.find("trigger_ingestion_surface=ground"),
            std::string::npos);

  ASSERT_EQ(callback_backlog.size(), 1U);
  const msg::ObstacleMemorySnapshot& delivered = callback_backlog.front();
  MemoryProvenanceParseResult latest = parseObstacleMemorySnapshotMessage(delivered);
  ASSERT_TRUE(latest.snapshot.has_value());
  current_grid = delivered.grid;
  current_provenance =
      std::move(latest.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)

  EXPECT_EQ(current_provenance.identity.stamp_ns,
            memoryGridSnapshotIdentity(current_grid).stamp_ns);
  const std::string after_callback = formatMemoryProvenanceDiagnostic(
      MemoryProvenanceMatchResult{&current_provenance,
                                  MemoryProvenanceUnavailableReason::kNone},
      GridIndex{2, 0});
  EXPECT_NE(after_callback.find("status=matched"), std::string::npos);
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

  const msg::ObstacleMemorySnapshot snapshot =
      makeObstacleMemorySnapshotMessage(grid, makeProvenance(), 1U);
  EXPECT_GT(serializedObstacleMemorySnapshotSize(snapshot), base_size);
  EXPECT_GT(serializedObstacleMemorySnapshotSize(snapshot), grid.data.size());
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

TEST(ObstacleMemoryProvenanceRos,
     AuditTrackerMatchesImmediatelyWhenProvenanceArrivesBeforeGridAudit) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const MemoryProvenanceParseResult parsed = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance()));
  ASSERT_TRUE(parsed.snapshot.has_value());

  MemoryProvenanceAuditTracker tracker;
  const std::vector<MemoryProvenanceAuditOutcome> outcomes = tracker.insert(
      parsed.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)
  EXPECT_TRUE(outcomes.empty());

  const MemoryProvenanceAuditResult audit = tracker.audit(grid, GridIndex{2, 0});
  EXPECT_FALSE(audit.pending_audit_id.has_value());
  EXPECT_NE(audit.diagnostic.find("memory_provenance[status=matched"),
            std::string::npos);
  EXPECT_NE(audit.diagnostic.find("trigger_endpoint=(12.5,20.5,17)"),
            std::string::npos);
  EXPECT_EQ(tracker.cachedSnapshotCount(), 1U);
  EXPECT_EQ(tracker.pendingAuditCount(), 0U);
}

TEST(ObstacleMemoryProvenanceRos,
     AuditTrackerEnrichesGridFirstAuditAfterMatchingProvenanceArrives) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const MemoryProvenanceParseResult parsed = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance()));
  ASSERT_TRUE(parsed.snapshot.has_value());

  MemoryProvenanceAuditTracker tracker;
  const MemoryProvenanceAuditResult first = tracker.audit(grid, GridIndex{2, 0});
  ASSERT_TRUE(first.pending_audit_id.has_value());
  EXPECT_NE(first.diagnostic.find("status=pending"), std::string::npos);
  EXPECT_NE(first.diagnostic.find("reason=not_received"), std::string::npos);

  const MemoryProvenanceAuditResult repeated = tracker.audit(grid, GridIndex{2, 0});
  EXPECT_EQ(repeated.pending_audit_id, first.pending_audit_id);
  EXPECT_EQ(tracker.pendingAuditCount(), 1U);

  const std::vector<MemoryProvenanceAuditOutcome> outcomes = tracker.insert(
      parsed.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_EQ(outcomes.size(), 1U);
  EXPECT_EQ(outcomes.front().audit_id, first.pending_audit_id.value_or(0U));
  EXPECT_EQ(outcomes.front().reason, MemoryProvenanceUnavailableReason::kNone);
  EXPECT_EQ(outcomes.front().cell, (GridIndex{2, 0}));
  EXPECT_NE(outcomes.front().diagnostic.find("status=matched"), std::string::npos);
  EXPECT_NE(outcomes.front().diagnostic.find("trigger_ingestion_surface=ground"),
            std::string::npos);
  EXPECT_EQ(tracker.pendingAuditCount(), 0U);
}

TEST(ObstacleMemoryProvenanceRos,
     AuditTrackerResolvesStampMismatchWithExactLaterSnapshot) {
  const nav_msgs::msg::OccupancyGrid old_grid = makeGrid();
  nav_msgs::msg::OccupancyGrid current_grid = old_grid;
  current_grid.header.stamp.sec = 13;
  current_grid.info.map_load_time = current_grid.header.stamp;

  const MemoryProvenanceParseResult old_snapshot = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(old_grid, makeProvenance()));
  const MemoryProvenanceParseResult current_snapshot =
      parseObstacleMemoryProvenanceMessage(
          makeObstacleMemoryProvenanceMessage(current_grid, makeProvenance()));
  ASSERT_TRUE(old_snapshot.snapshot.has_value());
  ASSERT_TRUE(current_snapshot.snapshot.has_value());

  MemoryProvenanceAuditTracker tracker;
  EXPECT_TRUE(tracker
                  .insert(old_snapshot.snapshot.value()) // NOLINT
                  .empty());
  const MemoryProvenanceAuditResult pending =
      tracker.audit(current_grid, GridIndex{2, 0});
  ASSERT_TRUE(pending.pending_audit_id.has_value());
  EXPECT_NE(pending.diagnostic.find("reason=stamp_mismatch"), std::string::npos);

  const std::vector<MemoryProvenanceAuditOutcome> outcomes = tracker.insert(
      current_snapshot.snapshot.value()); // NOLINT(bugprone-unchecked-optional-access)
  ASSERT_EQ(outcomes.size(), 1U);
  EXPECT_EQ(outcomes.front().audit_id, pending.pending_audit_id.value_or(0U));
  EXPECT_EQ(outcomes.front().identity.stamp_ns, 13'000'000'034LL);
  EXPECT_EQ(outcomes.front().reason, MemoryProvenanceUnavailableReason::kNone);
  EXPECT_NE(outcomes.front().diagnostic.find("status=matched"), std::string::npos);

  const MemoryProvenanceAuditResult immediate =
      tracker.audit(current_grid, GridIndex{2, 0});
  EXPECT_FALSE(immediate.pending_audit_id.has_value());
  EXPECT_NE(immediate.diagnostic.find("status=matched"), std::string::npos);
}

TEST(ObstacleMemoryProvenanceRos, AuditTrackerTerminatesExactIdentityContentMismatch) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  const MemoryProvenanceParseResult parsed = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance()));
  ASSERT_TRUE(parsed.snapshot.has_value());

  MemoryProvenanceSnapshot mismatched =
      parsed.snapshot.value(); // NOLINT(bugprone-unchecked-optional-access)
  MemoryCellProvenance record = std::move(mismatched.cells.at(2U));
  mismatched.cells.erase(2U);
  mismatched.cells.emplace(3U, std::move(record));

  MemoryProvenanceAuditTracker tracker;
  const MemoryProvenanceAuditResult pending = tracker.audit(grid, GridIndex{2, 0});
  ASSERT_TRUE(pending.pending_audit_id.has_value());
  const std::vector<MemoryProvenanceAuditOutcome> outcomes =
      tracker.insert(std::move(mismatched));
  ASSERT_EQ(outcomes.size(), 1U);
  EXPECT_EQ(outcomes.front().audit_id, pending.pending_audit_id.value_or(0U));
  EXPECT_EQ(outcomes.front().reason,
            MemoryProvenanceUnavailableReason::kContentMismatch);
  EXPECT_EQ(outcomes.front().diagnostic,
            "memory_provenance[status=unavailable reason=content_mismatch]");
  EXPECT_EQ(tracker.pendingAuditCount(), 0U);
}

TEST(ObstacleMemoryProvenanceRos,
     AuditTrackerExpiresLostExactSnapshotAfterRetentionHorizon) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  MemoryProvenanceAuditTracker tracker{4U, 256U, 2U};
  const MemoryProvenanceAuditResult pending = tracker.audit(grid, GridIndex{2, 0});
  ASSERT_TRUE(pending.pending_audit_id.has_value());

  nav_msgs::msg::OccupancyGrid newer_grid = grid;
  newer_grid.header.stamp.sec = 13;
  newer_grid.info.map_load_time = newer_grid.header.stamp;
  MemoryProvenanceParseResult newer = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(newer_grid, makeProvenance()));
  ASSERT_TRUE(newer.snapshot.has_value());
  EXPECT_TRUE(tracker
                  .insert(std::move(newer.snapshot.value())) // NOLINT
                  .empty());
  EXPECT_EQ(tracker.pendingAuditCount(), 1U);

  newer_grid.header.stamp.sec = 14;
  newer_grid.info.map_load_time = newer_grid.header.stamp;
  newer = parseObstacleMemoryProvenanceMessage(
      makeObstacleMemoryProvenanceMessage(newer_grid, makeProvenance()));
  ASSERT_TRUE(newer.snapshot.has_value());
  const std::vector<MemoryProvenanceAuditOutcome> outcomes =
      tracker.insert(std::move(newer.snapshot.value())); // NOLINT
  ASSERT_EQ(outcomes.size(), 1U);
  EXPECT_EQ(outcomes.front().audit_id, pending.pending_audit_id.value_or(0U));
  EXPECT_EQ(outcomes.front().reason,
            MemoryProvenanceUnavailableReason::kHistoryExpired);
  EXPECT_EQ(outcomes.front().diagnostic,
            "memory_provenance[status=unavailable reason=history_expired]");
  EXPECT_EQ(tracker.pendingAuditCount(), 0U);
}

TEST(ObstacleMemoryProvenanceRos,
     AuditTrackerTerminatesMalformedSnapshotWithExactIdentity) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  MemoryProvenanceAuditTracker tracker;
  const MemoryProvenanceAuditResult pending = tracker.audit(grid, GridIndex{2, 0});
  ASSERT_TRUE(pending.pending_audit_id.has_value());

  msg::ObstacleMemoryProvenance message =
      makeObstacleMemoryProvenanceMessage(grid, makeProvenance());
  ASSERT_FALSE(message.cells.empty());
  message.cells.front().accepted_hit_count = 0U;
  const MemoryProvenanceParseResult parsed =
      parseObstacleMemoryProvenanceMessage(message);
  EXPECT_FALSE(parsed.snapshot.has_value());
  EXPECT_EQ(parsed.reason, MemoryProvenanceUnavailableReason::kMalformed);
  const std::optional<MemoryGridSnapshotIdentity> identity =
      memoryProvenanceMessageIdentity(message);
  ASSERT_TRUE(identity.has_value());

  const std::vector<MemoryProvenanceAuditOutcome> outcomes =
      tracker.observeUnavailable(identity.value(), parsed.reason); // NOLINT
  ASSERT_EQ(outcomes.size(), 1U);
  EXPECT_EQ(outcomes.front().audit_id, pending.pending_audit_id.value_or(0U));
  EXPECT_EQ(outcomes.front().reason, MemoryProvenanceUnavailableReason::kMalformed);
  EXPECT_EQ(outcomes.front().diagnostic,
            "memory_provenance[status=unavailable reason=malformed]");
  EXPECT_EQ(tracker.pendingAuditCount(), 0U);
}

TEST(ObstacleMemoryProvenanceRos,
     MalformedNewerIdentitiesAdvanceHorizonWithoutDuplicateOrOutOfOrderCredit) {
  const nav_msgs::msg::OccupancyGrid grid = makeGrid();
  MemoryProvenanceAuditTracker tracker{4U, 256U, 2U};
  const MemoryProvenanceAuditResult pending = tracker.audit(grid, GridIndex{2, 0});
  ASSERT_TRUE(pending.pending_audit_id.has_value());

  const auto malformedIdentityAt = [](const std::int32_t sec,
                                      const std::uint32_t nanosec) {
    nav_msgs::msg::OccupancyGrid newer_grid = makeGrid();
    newer_grid.header.stamp.sec = sec;
    newer_grid.header.stamp.nanosec = nanosec;
    newer_grid.info.map_load_time = newer_grid.header.stamp;
    msg::ObstacleMemoryProvenance message =
        makeObstacleMemoryProvenanceMessage(newer_grid, makeProvenance());
    message.cells.front().accepted_hit_count = 0U;
    const MemoryProvenanceParseResult parsed =
        parseObstacleMemoryProvenanceMessage(message);
    EXPECT_FALSE(parsed.snapshot.has_value());
    EXPECT_EQ(parsed.reason, MemoryProvenanceUnavailableReason::kMalformed);
    return memoryProvenanceMessageIdentity(message);
  };

  const std::optional<MemoryGridSnapshotIdentity> newer = malformedIdentityAt(13, 0U);
  ASSERT_TRUE(newer.has_value());
  EXPECT_TRUE(tracker
                  .observeUnavailable(newer.value(), // NOLINT
                                      MemoryProvenanceUnavailableReason::kMalformed)
                  .empty());
  EXPECT_TRUE(tracker
                  .observeUnavailable(newer.value(), // NOLINT
                                      MemoryProvenanceUnavailableReason::kMalformed)
                  .empty());

  const std::optional<MemoryGridSnapshotIdentity> out_of_order =
      malformedIdentityAt(12, 500U);
  ASSERT_TRUE(out_of_order.has_value());
  EXPECT_TRUE(tracker
                  .observeUnavailable(out_of_order.value(), // NOLINT
                                      MemoryProvenanceUnavailableReason::kMalformed)
                  .empty());
  EXPECT_EQ(tracker.pendingAuditCount(), 1U);

  const std::optional<MemoryGridSnapshotIdentity> horizon = malformedIdentityAt(14, 0U);
  ASSERT_TRUE(horizon.has_value());
  const std::vector<MemoryProvenanceAuditOutcome> outcomes =
      tracker.observeUnavailable(horizon.value(), // NOLINT
                                 MemoryProvenanceUnavailableReason::kMalformed);
  ASSERT_EQ(outcomes.size(), 1U);
  EXPECT_EQ(outcomes.front().audit_id, pending.pending_audit_id.value_or(0U));
  EXPECT_EQ(outcomes.front().reason,
            MemoryProvenanceUnavailableReason::kHistoryExpired);
  EXPECT_EQ(tracker.pendingAuditCount(), 0U);
}

} // namespace drone_city_nav
