#include "drone_city_nav/route_3d.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

TEST(PassageTraversalEvidenceTest, RecordsSelectedRouteEntryAndCompletion) {
  PassageTraversalEvidenceTracker tracker;
  ConstrainedRouteObservation observation{
      .phase = ConstrainedRoutePhase::kApproach,
      .route_generation = 7U,
      .span_index = 2U,
      .span_count = 3U,
      .span_available = true,
      .passage_traversal_id = "test_passage",
      .within_vertical_window = true,
      .station_m = 9.0,
      .begin_station_m = 10.0,
      .end_station_m = 20.0,
      .segment_spans = {},
  };

  EXPECT_TRUE(
      tracker.update(observation, Point3{9.0, 0.0, 5.0}, 1'000'000'000LL).empty());

  observation.phase = ConstrainedRoutePhase::kTraversal;
  observation.station_m = 10.2;
  observation.cross_track_error_m = 0.4;
  observation.vertical_error_m = -0.2;
  const std::vector<PassageTraversalEvidenceEvent> entered =
      tracker.update(observation, Point3{10.2, 0.4, 4.8}, 1'020'000'000LL);
  ASSERT_EQ(entered.size(), 1U);
  EXPECT_EQ(entered.front().status, PassageTraversalEvidenceStatus::kEntered);
  EXPECT_EQ(entered.front().reason,
            PassageTraversalEvidenceReason::kEntryBoundaryCrossed);
  EXPECT_EQ(entered.front().passage_traversal_id, "test_passage");
  EXPECT_EQ(entered.front().sequence, 1U);

  observation.station_m = 18.0;
  observation.cross_track_error_m = 0.8;
  observation.vertical_error_m = 0.3;
  EXPECT_TRUE(
      tracker.update(observation, Point3{18.0, 0.8, 5.3}, 1'800'000'000LL).empty());

  observation.phase = ConstrainedRoutePhase::kDeparture;
  observation.station_m = 20.1;
  const std::vector<PassageTraversalEvidenceEvent> completed =
      tracker.update(observation, Point3{20.1, 0.2, 5.0}, 2'020'000'000LL);
  ASSERT_EQ(completed.size(), 1U);
  EXPECT_EQ(completed.front().status, PassageTraversalEvidenceStatus::kCompleted);
  EXPECT_EQ(completed.front().reason,
            PassageTraversalEvidenceReason::kExitBoundaryCrossed);
  EXPECT_EQ(completed.front().sequence, 2U);
  EXPECT_EQ(completed.front().traversal_observation_count, 2U);
  EXPECT_NEAR(completed.front().duration_s, 1.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(completed.front().maximum_cross_track_error_m, 0.8);
  EXPECT_DOUBLE_EQ(completed.front().maximum_absolute_vertical_error_m, 0.3);
  EXPECT_TRUE(completed.front().vertical_window_preserved);
}

TEST(PassageTraversalEvidenceTest, AbortsWhenSelectedRouteIsReplaced) {
  PassageTraversalEvidenceTracker tracker;
  ConstrainedRouteObservation observation{
      .phase = ConstrainedRoutePhase::kTraversal,
      .route_generation = 7U,
      .span_index = 0U,
      .span_count = 1U,
      .span_available = true,
      .passage_traversal_id = "old_passage",
      .within_vertical_window = true,
      .station_m = 12.0,
      .begin_station_m = 10.0,
      .end_station_m = 20.0,
      .segment_spans = {},
  };
  ASSERT_EQ(tracker.update(observation, Point3{12.0, 0.0, 5.0}, 1'000'000'000LL).size(),
            1U);

  observation.route_generation = 8U;
  observation.passage_traversal_id = "new_passage";
  observation.begin_station_m = 11.0;
  observation.end_station_m = 25.0;
  const std::vector<PassageTraversalEvidenceEvent> changed =
      tracker.update(observation, Point3{12.0, 0.0, 5.0}, 1'020'000'000LL);

  ASSERT_EQ(changed.size(), 2U);
  EXPECT_EQ(changed[0].status, PassageTraversalEvidenceStatus::kAborted);
  EXPECT_EQ(changed[0].reason, PassageTraversalEvidenceReason::kRouteChanged);
  EXPECT_EQ(changed[0].passage_traversal_id, "old_passage");
  EXPECT_EQ(changed[1].status, PassageTraversalEvidenceStatus::kEntered);
  EXPECT_EQ(changed[1].passage_traversal_id, "new_passage");
}

TEST(PassageTraversalEvidenceTest, RecordsGeometryEvidenceWithoutSelectedRouteSpan) {
  PassageGeometryEvidenceTracker tracker;
  PassageGeometryEvidenceConfig config;
  std::vector<PassageGeometryObservation> observations{
      PassageGeometryObservation{
          .passage_traversal_id = "long_passage",
          .within_corridor = true,
          .station_m = 0.5,
          .traversal_length_m = 20.0,
          .cross_track_error_m = 0.8,
      },
      PassageGeometryObservation{
          .passage_traversal_id = "short_passage",
          .within_corridor = true,
          .station_m = 0.4,
          .traversal_length_m = 16.0,
          .cross_track_error_m = 0.9,
      },
  };

  const std::vector<PassageGeometryEvidenceEvent> entered =
      tracker.update(observations, Point3{0.0, 0.0, 5.0}, 1'000'000'000LL, config);
  ASSERT_EQ(entered.size(), 1U);
  EXPECT_EQ(entered.front().status, PassageTraversalEvidenceStatus::kEntered);
  EXPECT_EQ(entered.front().passage_traversal_id, "short_passage");

  observations = {PassageGeometryObservation{
      .passage_traversal_id = "short_passage",
      .within_corridor = true,
      .station_m = 12.0,
      .traversal_length_m = 16.0,
      .cross_track_error_m = 1.2,
  }};
  EXPECT_TRUE(
      tracker.update(observations, Point3{12.0, 0.0, 5.0}, 2'000'000'000LL, config)
          .empty());

  observations.front().station_m = 14.5;
  observations.front().cross_track_error_m = 0.6;
  const std::vector<PassageGeometryEvidenceEvent> completed =
      tracker.update(observations, Point3{16.0, 0.0, 5.0}, 3'000'000'000LL, config);
  ASSERT_EQ(completed.size(), 1U);
  EXPECT_EQ(completed.front().status, PassageTraversalEvidenceStatus::kCompleted);
  EXPECT_EQ(completed.front().reason,
            PassageTraversalEvidenceReason::kExitBoundaryCrossed);
  EXPECT_EQ(completed.front().observation_count, 3U);
  EXPECT_DOUBLE_EQ(completed.front().maximum_station_m, 14.5);
  EXPECT_DOUBLE_EQ(completed.front().maximum_cross_track_error_m, 1.2);
  EXPECT_NEAR(completed.front().duration_s, 2.0, 1.0e-9);
}

TEST(PassageTraversalEvidenceTest, AbortsGeometryEvidenceAfterObservationTimeout) {
  PassageGeometryEvidenceTracker tracker;
  const PassageGeometryEvidenceConfig config;
  std::vector<PassageGeometryObservation> observations{
      PassageGeometryObservation{
          .passage_traversal_id = "passage",
          .within_corridor = true,
          .station_m = 1.0,
          .traversal_length_m = 20.0,
          .cross_track_error_m = 0.5,
      },
  };
  ASSERT_EQ(tracker.update(observations, Point3{}, 1'000'000'000LL, config).size(), 1U);
  observations.front().within_corridor = false;
  EXPECT_TRUE(tracker.update(observations, Point3{}, 2'000'000'000LL, config).empty());
  const std::vector<PassageGeometryEvidenceEvent> aborted =
      tracker.update(observations, Point3{}, 3'100'000'000LL, config);
  ASSERT_EQ(aborted.size(), 1U);
  EXPECT_EQ(aborted.front().status, PassageTraversalEvidenceStatus::kAborted);
  EXPECT_EQ(aborted.front().reason, PassageTraversalEvidenceReason::kObservationLost);
}

} // namespace
} // namespace drone_city_nav
