#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] AmbiguousStaticHitObservation
observation(const std::int64_t stamp_ns, const double origin_x,
            const KnownStaticEndpointRelation relation =
                KnownStaticEndpointRelation::kNearSurface) {
  return AmbiguousStaticHitObservation{
      .structure_id = "building",
      .part_id = "upper_mass",
      .endpoint_map_m = Point3{5.1, 2.1, 3.1},
      .ray_origin_map_m = Point3{origin_x, 0.0, 3.0},
      .ray_direction_map = Point3{1.0, 0.0, 0.0},
      .endpoint_relation = relation,
      .endpoint_solid_distance_m =
          relation == KnownStaticEndpointRelation::kOutside ? 0.8 : 0.1,
      .distance_before_solid_m = 0.7,
      .range_residual_m = -0.7,
      .scan_stamp_ns = stamp_ns,
  };
}

TEST(AmbiguousLidarHitTrackerTest,
     ConfirmsStaticAttachedOnlyFromIndependentScansAndViewpoints) {
  AmbiguousLidarHitTracker tracker;

  EXPECT_EQ(tracker.observe(observation(100'000'000, 0.0)).independent_scans, 1U);
  EXPECT_FALSE(tracker.observe(observation(100'000'000, 1.0)).new_scan_vote);
  EXPECT_FALSE(tracker.observe(observation(200'000'000, 0.1)).new_scan_vote);
  EXPECT_TRUE(tracker.observe(observation(200'000'000, 0.6)).new_scan_vote);
  const AmbiguousLidarHitConfirmation confirmation =
      tracker.observe(observation(300'000'000, 1.2));

  EXPECT_EQ(confirmation.resolution,
            AmbiguousLidarHitResolution::kConfirmedStaticAttached);
  EXPECT_EQ(confirmation.independent_scans, 3U);
  EXPECT_EQ(confirmation.static_attached_observations, 3U);
}

TEST(AmbiguousLidarHitTrackerTest, OpeningBoundaryIsStaticAttachedEvidence) {
  AmbiguousLidarHitTracker tracker;

  EXPECT_TRUE(
      tracker
          .observe(observation(100'000'000, 0.0,
                               KnownStaticEndpointRelation::kInsideOpeningBoundary))
          .new_scan_vote);
  EXPECT_TRUE(
      tracker
          .observe(observation(200'000'000, 0.6,
                               KnownStaticEndpointRelation::kInsideOpeningBoundary))
          .new_scan_vote);
  const AmbiguousLidarHitConfirmation confirmation = tracker.observe(observation(
      300'000'000, 1.2, KnownStaticEndpointRelation::kInsideOpeningBoundary));

  EXPECT_EQ(confirmation.resolution,
            AmbiguousLidarHitResolution::kConfirmedStaticAttached);
  EXPECT_EQ(confirmation.static_attached_observations, 3U);
}

TEST(AmbiguousLidarHitTrackerTest, ConfirmsDetachedClusterAsObstacle) {
  AmbiguousLidarHitTracker tracker;

  EXPECT_TRUE(
      tracker
          .observe(observation(100'000'000, 0.0, KnownStaticEndpointRelation::kOutside))
          .new_scan_vote);
  EXPECT_TRUE(
      tracker
          .observe(observation(200'000'000, 0.6, KnownStaticEndpointRelation::kOutside))
          .new_scan_vote);
  const AmbiguousLidarHitConfirmation confirmation = tracker.observe(
      observation(300'000'000, 1.2, KnownStaticEndpointRelation::kOutside));

  EXPECT_EQ(confirmation.resolution,
            AmbiguousLidarHitResolution::kConfirmedDetachedObstacle);
  EXPECT_EQ(confirmation.detached_obstacle_observations, 3U);
}

TEST(AmbiguousLidarHitTrackerTest,
     OpeningBoundaryCandidateCanResolveAsDetachedObstacle) {
  AmbiguousLidarHitTracker tracker;

  EXPECT_TRUE(
      tracker
          .observe(observation(100'000'000, 0.0,
                               KnownStaticEndpointRelation::kInsideOpeningBoundary))
          .new_scan_vote);
  EXPECT_TRUE(
      tracker
          .observe(observation(200'000'000, 0.6, KnownStaticEndpointRelation::kOutside))
          .new_scan_vote);
  EXPECT_TRUE(
      tracker
          .observe(observation(300'000'000, 1.2, KnownStaticEndpointRelation::kOutside))
          .new_scan_vote);
  const AmbiguousLidarHitConfirmation confirmation = tracker.observe(
      observation(400'000'000, 1.8, KnownStaticEndpointRelation::kOutside));

  EXPECT_EQ(confirmation.resolution,
            AmbiguousLidarHitResolution::kConfirmedDetachedObstacle);
  EXPECT_TRUE(confirmation.opening_boundary_observed);
  EXPECT_EQ(confirmation.detached_obstacle_observations, 3U);
}

TEST(AmbiguousLidarHitTrackerTest, ConflictingGeometryRemainsPending) {
  AmbiguousLidarHitTracker tracker;

  EXPECT_TRUE(tracker.observe(observation(100'000'000, 0.0)).new_scan_vote);
  EXPECT_TRUE(
      tracker
          .observe(observation(200'000'000, 0.6, KnownStaticEndpointRelation::kOutside))
          .new_scan_vote);
  const AmbiguousLidarHitConfirmation confirmation =
      tracker.observe(observation(300'000'000, 1.2));

  EXPECT_EQ(confirmation.resolution, AmbiguousLidarHitResolution::kPending);
}

TEST(AmbiguousLidarHitTrackerTest, GapRestartsEvidenceAndReportsExpiry) {
  AmbiguousLidarHitTracker tracker{AmbiguousLidarHitTrackerConfig{
      .required_independent_scans = 3U,
      .max_scan_gap_ns = 100'000'000,
      .retention_ns = 200'000'000,
  }};
  EXPECT_TRUE(tracker.observe(observation(100'000'000, 0.0)).new_scan_vote);
  EXPECT_TRUE(tracker.observe(observation(150'000'000, 0.6)).new_scan_vote);

  const AmbiguousLidarHitConfirmation restarted =
      tracker.observe(observation(400'000'000, 1.2));

  EXPECT_EQ(restarted.independent_scans, 1U);
  EXPECT_EQ(restarted.expired_candidates, 1U);
  EXPECT_EQ(restarted.resolution, AmbiguousLidarHitResolution::kPending);
}

} // namespace
} // namespace drone_city_nav
