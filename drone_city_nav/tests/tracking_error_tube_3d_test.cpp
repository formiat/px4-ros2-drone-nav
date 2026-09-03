#include "drone_city_nav/tracking_error_tube_3d.hpp"
#include "drone_city_nav/tracking_error_tube_handoff_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] GridBounds3D testBounds() {
  return GridBounds3D{
      .origin_x = -1.0,
      .origin_y = -2.0,
      .origin_z = 0.0,
      .resolution_m = 0.1,
      .width_cells = 110,
      .height_cells = 40,
      .depth_cells = 40,
  };
}

void addPassageWalls(ObservedOccupancyGrid3D& occupancy) {
  const GridBounds3D& bounds = occupancy.bounds();
  const int lower_wall_y = 7;
  const int upper_wall_y = 32;
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int x = 0; x < bounds.width_cells; ++x) {
      static_cast<void>(
          occupancy.setState({x, lower_wall_y, z}, ObservedVoxelState::kOccupied));
      static_cast<void>(
          occupancy.setState({x, upper_wall_y, z}, ObservedVoxelState::kOccupied));
    }
  }
}

[[nodiscard]] std::vector<RouteSample3D> passageRoute() {
  return sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 2.0}, {5.0, 0.0, 2.0}, {9.0, 0.0, 2.0}}, 0.5, 5.0);
}

[[nodiscard]] TrackingErrorTubeProfile3D
profile(const ObservedOccupancyGrid3D& occupancy) {
  const std::uint64_t occupied_fingerprint =
      occupancy.occupiedSnapshot().contentFingerprint();
  return makeTrackingErrorTubeProfile3D(
      passageRoute(),
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &occupancy,
          .occupied_content_fingerprint = occupied_fingerprint,
      },
      SweptFootprintConfig{}, TrackingErrorTubeConfig3D{.response_time_s = 0.15}, 5.0);
}

TEST(TrackingErrorTube3DTest, FullSpeedErrorMatchesClosedLoopResponseHorizon) {
  EXPECT_DOUBLE_EQ(
      trackingErrorTubeRadiusM(TrackingErrorTubeConfig3D{.response_time_s = 0.15}, 5.0),
      0.75);
}

TEST(TrackingErrorTube3DTest, NarrowPhysicalPassageReducesSpeedInsteadOfPathClearance) {
  ObservedOccupancyGrid3D occupancy{testBounds()};
  addPassageWalls(occupancy);
  const std::vector<RouteSample3D> route = passageRoute();

  const SweptFootprintResult physical = validateRawSweptFootprint(
      occupancy, route.front().position, FootprintBodyAxis{}, route.back().position,
      FootprintBodyAxis{}, SweptFootprintConfig{});
  const TrackingErrorTubeProfile3D tube = profile(occupancy);

  ASSERT_TRUE(physical.accepted());
  ASSERT_TRUE(tube.valid);
  EXPECT_TRUE(tube.obstacle_evidence_available);
  EXPECT_GT(tube.constrained_segment_count, 0U);
  EXPECT_GT(tube.minimum_speed_limit_mps, 0.5);
  EXPECT_LT(tube.minimum_speed_limit_mps, 4.0);
  EXPECT_LT(tube.maximum_tracking_error_m, 0.75);
}

TEST(TrackingErrorTube3DTest, ConstraintDescriptionNamesTheConstrainedStationRanges) {
  ObservedOccupancyGrid3D occupancy{testBounds()};
  addPassageWalls(occupancy);
  const std::vector<RouteSample3D> route = passageRoute();
  const TrackingErrorTubeProfile3D tube = profile(occupancy);
  ASSERT_TRUE(tube.valid);
  ASSERT_GT(tube.constrained_segment_count, 0U);

  const std::string description =
      describeTrackingErrorTubeConstraints3D(route, tube, 8U);
  ASSERT_FALSE(description.empty());
  // The walls run along the whole route, so one range spans it end to end and
  // names the profile's lowest limit.
  char expected_limit[32];
  std::snprintf(expected_limit, sizeof(expected_limit), ":%.2f@",
                tube.minimum_speed_limit_mps);
  EXPECT_NE(description.find(expected_limit), std::string::npos) << description;
  EXPECT_EQ(description.find(';'), std::string::npos) << description;
  EXPECT_EQ(description.rfind("0.0-", 0U), 0U) << description;

  TrackingErrorTubeProfile3D unconstrained = tube;
  std::ranges::fill(unconstrained.speed_limits_mps,
                    unconstrained.unconstrained_speed_limit_mps);
  unconstrained.constrained_segment_count = 0U;
  unconstrained.minimum_speed_limit_mps = unconstrained.unconstrained_speed_limit_mps;
  unconstrained.maximum_tracking_error_m = trackingErrorTubeRadiusM(
      unconstrained.config, unconstrained.unconstrained_speed_limit_mps);
  EXPECT_TRUE(describeTrackingErrorTubeConstraints3D(route, unconstrained, 8U).empty());

  // Two separated ranges with a one-range budget name the first and count the
  // second.
  TrackingErrorTubeProfile3D split = unconstrained;
  split.speed_limits_mps[1U] = 1.0;
  split.speed_limits_mps[route.size() - 2U] = 2.0;
  split.constrained_segment_count = 2U;
  split.minimum_speed_limit_mps = 1.0;
  const std::string limited = describeTrackingErrorTubeConstraints3D(route, split, 1U);
  EXPECT_NE(limited.find(":1.00@"), std::string::npos) << limited;
  EXPECT_NE(limited.find(";+1"), std::string::npos) << limited;
}

TEST(TrackingErrorTube3DTest, FreeUnknownRelabelingLeavesSpeedProfileUnchanged) {
  ObservedOccupancyGrid3D unknown{testBounds()};
  addPassageWalls(unknown);
  ObservedOccupancyGrid3D free = unknown;
  const GridBounds3D& bounds = free.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        const GridIndex3D cell{x, y, z};
        if (!free.isOccupied(cell)) {
          static_cast<void>(free.setState(cell, ObservedVoxelState::kFree));
        }
      }
    }
  }

  const TrackingErrorTubeProfile3D unknown_profile = profile(unknown);
  const TrackingErrorTubeProfile3D free_profile = profile(free);

  ASSERT_TRUE(unknown_profile.valid);
  ASSERT_TRUE(free_profile.valid);
  EXPECT_TRUE(trackingErrorTubeProfile3DMatchesWorld(
      passageRoute(), unknown_profile,
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &free,
          .occupied_content_fingerprint = free.occupiedSnapshot().contentFingerprint(),
      }));
  EXPECT_EQ(unknown_profile.speed_limits_mps, free_profile.speed_limits_mps);
  EXPECT_DOUBLE_EQ(unknown_profile.minimum_speed_limit_mps,
                   free_profile.minimum_speed_limit_mps);
  EXPECT_EQ(unknown_profile.constrained_segment_count,
            free_profile.constrained_segment_count);
}

TEST(TrackingErrorTube3DTest, RejectsAnOccupancyOwnerFingerprintMismatch) {
  ObservedOccupancyGrid3D occupancy{testBounds()};
  addPassageWalls(occupancy);

  const TrackingErrorTubeProfile3D tube = makeTrackingErrorTubeProfile3D(
      passageRoute(),
      TrackingErrorTubeWorld3D{
          .observed_occupancy = &occupancy,
          .occupied_content_fingerprint =
              occupancy.occupiedSnapshot().contentFingerprint() + 1U,
      },
      SweptFootprintConfig{}, TrackingErrorTubeConfig3D{}, 5.0);

  EXPECT_FALSE(tube.valid);
}

TEST(TrackingErrorTube3DTest, RejectsInvalidWorldAndFootprintContracts) {
  ObservedOccupancyGrid3D occupancy{testBounds()};
  addPassageWalls(occupancy);
  const std::uint64_t occupied_fingerprint =
      occupancy.occupiedSnapshot().contentFingerprint();

  EXPECT_TRUE(makeTrackingErrorTubeProfile3D(
                  passageRoute(),
                  TrackingErrorTubeWorld3D{
                      .observed_occupancy = &occupancy,
                      .occupied_content_fingerprint = occupied_fingerprint,
                  },
                  SweptFootprintConfig{}, TrackingErrorTubeConfig3D{}, 5.0)
                  .valid);
  SweptFootprintConfig invalid_footprint;
  invalid_footprint.safe_clearance_threshold_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(makeTrackingErrorTubeProfile3D(
                   passageRoute(),
                   TrackingErrorTubeWorld3D{
                       .observed_occupancy = &occupancy,
                       .occupied_content_fingerprint = occupied_fingerprint,
                   },
                   invalid_footprint, TrackingErrorTubeConfig3D{}, 5.0)
                   .valid);
}

TEST(TrackingErrorTube3DTest, MissingDistanceEvidenceIsNeutral) {
  const TrackingErrorTubeProfile3D tube = makeTrackingErrorTubeProfile3D(
      passageRoute(), TrackingErrorTubeWorld3D{}, SweptFootprintConfig{},
      TrackingErrorTubeConfig3D{.response_time_s = 0.15}, 5.0);

  ASSERT_TRUE(tube.valid);
  EXPECT_FALSE(tube.obstacle_evidence_available);
  EXPECT_EQ(tube.constrained_segment_count, 0U);
  EXPECT_DOUBLE_EQ(tube.minimum_speed_limit_mps, 5.0);
  EXPECT_DOUBLE_EQ(tube.maximum_tracking_error_m, 0.75);
}

TEST(TrackingErrorTube3DTest, ExecutionContractEnforcesSealedSpeedAndErrorBounds) {
  const std::vector<RouteSample3D> route = passageRoute();
  const TrackingErrorTubeProfile3D tube = makeTrackingErrorTubeProfile3D(
      route, TrackingErrorTubeWorld3D{}, SweptFootprintConfig{},
      TrackingErrorTubeConfig3D{.response_time_s = 0.15}, 5.0);
  ASSERT_TRUE(trackingErrorTubeProfile3DIsValid(tube, route.size()));

  const TrackingErrorTubeExecutionAssessment3D accepted =
      assessTrackingErrorTubeExecution3D(route, tube,
                                         TrackingErrorTubeExecutionObservation3D{
                                             .station_m = 2.0,
                                             .cross_track_error_m = 0.75,
                                             .speed_mps = 5.0,
                                         });
  EXPECT_TRUE(accepted.accepted());
  EXPECT_DOUBLE_EQ(accepted.speed_limit_mps, 5.0);
  EXPECT_DOUBLE_EQ(accepted.tube_radius_m, 0.75);

  const TrackingErrorTubeExecutionAssessment3D overspeed =
      assessTrackingErrorTubeExecution3D(route, tube,
                                         TrackingErrorTubeExecutionObservation3D{
                                             .station_m = 2.0,
                                             .cross_track_error_m = 0.0,
                                             .speed_mps = 5.01,
                                         });
  EXPECT_EQ(overspeed.status, TrackingErrorTubeExecutionStatus3D::kSpeedLimitExceeded);

  const TrackingErrorTubeExecutionAssessment3D diverged =
      assessTrackingErrorTubeExecution3D(route, tube,
                                         TrackingErrorTubeExecutionObservation3D{
                                             .station_m = 2.0,
                                             .cross_track_error_m = 0.76,
                                             .speed_mps = 4.0,
                                         });
  EXPECT_EQ(diverged.status, TrackingErrorTubeExecutionStatus3D::kCrossTrackExceeded);
}

TEST(TrackingErrorTube3DTest, TamperedProfileIsNotExecutable) {
  const std::vector<RouteSample3D> route = passageRoute();
  TrackingErrorTubeProfile3D tube = makeTrackingErrorTubeProfile3D(
      route, TrackingErrorTubeWorld3D{}, SweptFootprintConfig{},
      TrackingErrorTubeConfig3D{.response_time_s = 0.15}, 5.0);
  ASSERT_TRUE(tube.valid);
  tube.speed_limits_mps.front() = 6.0;

  EXPECT_FALSE(trackingErrorTubeProfile3DIsValid(tube, route.size()));
  EXPECT_EQ(assessTrackingErrorTubeExecution3D(
                route, tube, TrackingErrorTubeExecutionObservation3D{.station_m = 1.0})
                .status,
            TrackingErrorTubeExecutionStatus3D::kInvalidProfile);
}

TEST(TrackingErrorTube3DTest,
     InitialHandoffIsBoundToTheImmutableExecutionClockAndReference) {
  const std::vector<RouteSample3D> route = passageRoute();
  const TrackingErrorTubeProfile3D tube = makeTrackingErrorTubeProfile3D(
      route, TrackingErrorTubeWorld3D{}, SweptFootprintConfig{},
      TrackingErrorTubeConfig3D{.response_time_s = 0.15}, 5.0);
  ASSERT_TRUE(tube.valid);
  const std::vector<MotionState3D> states{
      MotionState3D{.x = 0.0F, .y = 2.0F, .z = 2.0F, .vx = 1.0F},
      MotionState3D{.x = 1.0F, .y = 1.5F, .z = 2.0F, .vx = 1.0F},
      MotionState3D{.x = 2.0F, .y = 0.5F, .z = 2.0F, .vx = 1.0F},
      MotionState3D{.x = 3.0F, .y = 0.0F, .z = 2.0F, .vx = 1.0F},
  };
  constexpr std::int64_t kValidFromNs{1'000'000'000LL};
  constexpr std::int64_t kControlIntervalNs{100'000'000LL};
  constexpr std::int64_t kValidUntilNs{kValidFromNs + 3LL * kControlIntervalNs};

  const TrackingErrorTubeHandoffAssessment3D active = assessTrackingErrorTubeHandoff3D(
      route, tube, states, 0.0, kValidFromNs, kValidUntilNs, kControlIntervalNs,
      TrackingErrorTubeHandoffObservation3D{
          .stamp_ns = kValidFromNs + kControlIntervalNs / 2LL,
          .state = MotionState3D{.x = 0.5F, .y = 1.75F, .z = 2.0F, .vx = 1.0F},
      });
  EXPECT_TRUE(active.active());
  EXPECT_DOUBLE_EQ(active.reference_speed_limit_mps, 1.0);
  EXPECT_DOUBLE_EQ(active.tracking_error_radius_m, 0.15);

  TrackingErrorTubeHandoffObservation3D diverged_observation{
      .stamp_ns = kValidFromNs + kControlIntervalNs / 2LL,
      .state = MotionState3D{.x = 0.5F, .y = 2.0F, .z = 2.0F, .vx = 1.0F},
  };
  EXPECT_EQ(assessTrackingErrorTubeHandoff3D(route, tube, states, 0.0, kValidFromNs,
                                             kValidUntilNs, kControlIntervalNs,
                                             diverged_observation)
                .status,
            TrackingErrorTubeHandoffStatus3D::kTrackingErrorExceeded);

  diverged_observation.state =
      MotionState3D{.x = 0.5F, .y = 1.75F, .z = 2.0F, .vx = 1.1F};
  EXPECT_EQ(assessTrackingErrorTubeHandoff3D(route, tube, states, 0.0, kValidFromNs,
                                             kValidUntilNs, kControlIntervalNs,
                                             diverged_observation)
                .status,
            TrackingErrorTubeHandoffStatus3D::kSpeedLimitExceeded);

  EXPECT_EQ(
      assessTrackingErrorTubeHandoff3D(
          route, tube, states, 0.0, kValidFromNs, kValidUntilNs, kControlIntervalNs,
          TrackingErrorTubeHandoffObservation3D{
              .stamp_ns = kValidFromNs + 5LL * kControlIntervalNs / 2LL,
              .state = MotionState3D{.x = 2.5F, .y = 0.25F, .z = 2.0F, .vx = 1.0F},
          })
          .status,
      TrackingErrorTubeHandoffStatus3D::kReferenceAcquiredRouteTube);
}

} // namespace
} // namespace drone_city_nav
