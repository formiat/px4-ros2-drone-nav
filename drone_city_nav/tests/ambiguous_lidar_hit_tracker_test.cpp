#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] UncertainLidarHitObservation observation(
    const std::int64_t stamp_ns, const double origin_x,
    const UncertainLidarHitEvidence evidence =
        UncertainLidarHitEvidence::kExpectedSurfaceAttached,
    const UncertainLidarHitKind kind = UncertainLidarHitKind::kKnownStaticBoundary,
    const Point3 endpoint = Point3{5.1, 2.1, 3.1}) {
  return UncertainLidarHitObservation{
      .kind = kind,
      .evidence = evidence,
      .association_id =
          kind == UncertainLidarHitKind::kKnownStaticBoundary ? "building" : "ground",
      .part_id = kind == UncertainLidarHitKind::kKnownStaticBoundary ? "upper_mass"
                                                                     : "ground_plane",
      .endpoint_map_m = endpoint,
      .ray_origin_map_m = Point3{origin_x, 0.0, 3.0},
      .ray_direction_map = Point3{1.0, 0.0, 0.0},
      .endpoint_surface_distance_m =
          evidence == UncertainLidarHitEvidence::kExpectedSurfaceAttached ? 0.1 : 0.8,
      .distance_before_surface_m = 0.7,
      .range_residual_m = -0.7,
      .scan_stamp_ns = stamp_ns,
  };
}

TEST(UncertainLidarHitTrackerTest,
     ConfirmsExpectedSurfaceOnlyFromIndependentScansAndViewpoints) {
  UncertainLidarHitTracker tracker;

  EXPECT_EQ(tracker.observe(observation(100'000'000, 0.0)).independent_scans, 1U);
  EXPECT_FALSE(tracker.observe(observation(100'000'000, 1.0)).new_scan_vote);
  EXPECT_FALSE(tracker.observe(observation(200'000'000, 0.1)).new_scan_vote);
  EXPECT_TRUE(tracker.observe(observation(200'000'000, 0.6)).new_scan_vote);
  const UncertainLidarHitConfirmation confirmation =
      tracker.observe(observation(300'000'000, 1.2));

  EXPECT_EQ(confirmation.resolution,
            UncertainLidarHitResolution::kConfirmedExpectedSurface);
  EXPECT_EQ(confirmation.independent_scans, 3U);
  EXPECT_EQ(confirmation.expected_surface_observations, 3U);
}

TEST(UncertainLidarHitTrackerTest, ConfirmsDetachedClusterAsObstacle) {
  UncertainLidarHitTracker tracker;
  constexpr auto kDetached = UncertainLidarHitEvidence::kDetachedObstacle;

  EXPECT_TRUE(tracker.observe(observation(100'000'000, 0.0, kDetached)).new_scan_vote);
  EXPECT_TRUE(tracker.observe(observation(200'000'000, 0.6, kDetached)).new_scan_vote);
  const UncertainLidarHitConfirmation confirmation =
      tracker.observe(observation(300'000'000, 1.2, kDetached));

  EXPECT_EQ(confirmation.resolution, UncertainLidarHitResolution::kConfirmedObstacle);
  EXPECT_EQ(confirmation.detached_obstacle_observations, 3U);
}

TEST(UncertainLidarHitTrackerTest, ConflictingGeometryRemainsPending) {
  UncertainLidarHitTracker tracker;

  EXPECT_TRUE(tracker.observe(observation(100'000'000, 0.0)).new_scan_vote);
  EXPECT_TRUE(tracker
                  .observe(observation(200'000'000, 0.6,
                                       UncertainLidarHitEvidence::kDetachedObstacle))
                  .new_scan_vote);
  const UncertainLidarHitConfirmation confirmation =
      tracker.observe(observation(300'000'000, 1.2));

  EXPECT_EQ(confirmation.resolution, UncertainLidarHitResolution::kPending);
}

TEST(UncertainLidarHitTrackerTest, NeighboringVoxelsTrackOneEndpointCluster) {
  UncertainLidarHitTracker tracker;
  constexpr auto kGround = UncertainLidarHitKind::kGroundCandidate;

  EXPECT_TRUE(tracker
                  .observe(observation(100'000'000, 0.0,
                                       UncertainLidarHitEvidence::kDetachedObstacle,
                                       kGround, Point3{0.49, 1.0, 0.4}))
                  .new_scan_vote);
  EXPECT_TRUE(tracker
                  .observe(observation(200'000'000, 0.6,
                                       UncertainLidarHitEvidence::kDetachedObstacle,
                                       kGround, Point3{0.51, 1.0, 0.4}))
                  .new_scan_vote);
  const UncertainLidarHitConfirmation confirmation = tracker.observe(
      observation(300'000'000, 1.2, UncertainLidarHitEvidence::kDetachedObstacle,
                  kGround, Point3{0.53, 1.0, 0.4}));

  EXPECT_EQ(confirmation.independent_scans, 3U);
  EXPECT_EQ(confirmation.resolution, UncertainLidarHitResolution::kConfirmedObstacle);
  EXPECT_EQ(tracker.candidateCount(), 1U);
}

TEST(UncertainLidarHitTrackerTest, CandidateKindsDoNotShareEvidence) {
  UncertainLidarHitTracker tracker;
  constexpr auto kDetached = UncertainLidarHitEvidence::kDetachedObstacle;

  EXPECT_EQ(tracker.observe(observation(100'000'000, 0.0, kDetached)).independent_scans,
            1U);
  EXPECT_EQ(tracker
                .observe(observation(200'000'000, 0.6, kDetached,
                                     UncertainLidarHitKind::kGroundCandidate))
                .independent_scans,
            1U);
  EXPECT_EQ(tracker.candidateCount(), 2U);
}

TEST(UncertainLidarHitTrackerTest, GapRestartsEvidenceAndReportsExpiry) {
  UncertainLidarHitTracker tracker{UncertainLidarHitTrackerConfig{
      .required_independent_scans = 3U,
      .max_scan_gap_ns = 100'000'000,
      .retention_ns = 200'000'000,
  }};
  EXPECT_TRUE(tracker.observe(observation(100'000'000, 0.0)).new_scan_vote);
  EXPECT_TRUE(tracker.observe(observation(150'000'000, 0.6)).new_scan_vote);

  EXPECT_EQ(tracker.expire(400'000'001), 1U);
  EXPECT_EQ(tracker.candidateCount(), 0U);
  const UncertainLidarHitConfirmation restarted =
      tracker.observe(observation(500'000'000, 1.2));
  EXPECT_EQ(restarted.independent_scans, 1U);
  EXPECT_EQ(restarted.resolution, UncertainLidarHitResolution::kPending);
}

} // namespace
} // namespace drone_city_nav
