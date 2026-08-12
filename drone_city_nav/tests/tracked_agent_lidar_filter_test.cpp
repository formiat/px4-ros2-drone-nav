#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<LidarProjectionPose> levelPoses(const std::size_t count) {
  return std::vector<LidarProjectionPose>(
      count, LidarProjectionPose{Point2{0.0, 0.0}, 10.0, 0.0, 0.0, 0.0, true, true});
}

[[nodiscard]] CooperativeFlightIntentData peerIntent(const std::string& vehicle_id,
                                                     const std::int64_t stamp_ns) {
  return CooperativeFlightIntentData{
      .vehicle_id = vehicle_id,
      .frame_id = "map",
      .stamp_ns = stamp_ns,
      .intent_generation = 1U,
      .valid_from_ns = stamp_ns,
      .valid_until_ns = stamp_ns + 2'000'000'000LL,
      .footprint_radius_m = 0.8,
      .footprint_lower_extent_m = 0.4,
      .footprint_upper_extent_m = 0.6,
      .current_position = Point3{8.0, 0.0, 10.0},
      .current_velocity = Vec3{2.0, 0.0, 0.0},
      .maneuver_state = CooperativeManeuver::kKeep,
      .conflict_generation = 0U,
      .conflicting_vehicle_ids = {},
      .channel = {},
      .trajectory =
          {
              CooperativeTrajectorySample{
                  .time_ns = stamp_ns,
                  .position = Point3{8.0, 0.0, 10.0},
                  .velocity = Vec3{2.0, 0.0, 0.0},
              },
              CooperativeTrajectorySample{
                  .time_ns = stamp_ns + 1'000'000'000LL,
                  .position = Point3{10.0, 0.0, 10.0},
                  .velocity = Vec3{2.0, 0.0, 0.0},
              },
          },
  };
}

TEST(TrackedAgentLidarFilterTest, InvalidatesHitsFromMultiplePhysicalVolumes) {
  const std::vector<float> ranges{10.0F, 10.0F, 10.0F};
  const std::vector<LidarProjectionPose> poses = levelPoses(ranges.size());
  const LidarProjectionConfig projection_config{};
  const LidarBeamProjection first_projection = projectLidarBeam(
      poses[0], projection_config, 0.1, 35.0, -0.1, 0.1, 0U, ranges[0]);
  const LidarBeamProjection second_projection = projectLidarBeam(
      poses[1], projection_config, 0.1, 35.0, -0.1, 0.1, 1U, ranges[1]);
  ASSERT_TRUE(first_projection.endpoint_xyz_valid);
  ASSERT_TRUE(second_projection.endpoint_xyz_valid);
  const std::vector<DynamicAgentLidarVolume> agents{
      DynamicAgentLidarVolume{
          .position = first_projection.endpoint_map_m,
          .radius_m = 0.25,
          .lower_extent_m = 0.5,
          .upper_extent_m = 0.5,
      },
      DynamicAgentLidarVolume{
          .position = second_projection.endpoint_map_m,
          .radius_m = 0.25,
          .lower_extent_m = 0.5,
          .upper_extent_m = 0.5,
      },
  };

  const TrackedAgentLidarFilterResult result =
      filterTrackedAgentLidarHits(ranges, TrackedAgentLidarFilterInput{
                                              .beam_projection_poses = poses,
                                              .projection_config = projection_config,
                                              .range_min_m = 0.1,
                                              .range_max_m = 35.0,
                                              .angle_min_rad = -0.1,
                                              .angle_increment_rad = 0.1,
                                              .agents = agents,
                                          });

  EXPECT_EQ(result.filtered_beams, 2U);
  EXPECT_EQ(result.matched_agents, 2U);
  EXPECT_TRUE(std::isnan(result.ranges[0]));
  EXPECT_TRUE(std::isnan(result.ranges[1]));
  EXPECT_TRUE(std::isfinite(result.ranges[2]));
}

TEST(TrackedAgentLidarFilterTest, UsesProjectedEndpointAltitude) {
  const std::vector<float> ranges{10.0F};
  std::vector<LidarProjectionPose> poses = levelPoses(ranges.size());
  poses.front().pitch_rad = -0.4;
  const std::vector<DynamicAgentLidarVolume> agents{
      DynamicAgentLidarVolume{
          .position = Point3{10.0 * std::cos(0.4), 0.0, 10.0},
          .radius_m = 1.0,
          .lower_extent_m = 0.5,
          .upper_extent_m = 0.5,
      },
  };

  const TrackedAgentLidarFilterResult result = filterTrackedAgentLidarHits(
      ranges, TrackedAgentLidarFilterInput{
                  .beam_projection_poses = poses,
                  .projection_config = LidarProjectionConfig{},
                  .range_min_m = 0.1,
                  .range_max_m = 35.0,
                  .agents = agents,
              });

  EXPECT_EQ(result.filtered_beams, 0U);
  EXPECT_EQ(result.matched_agents, 0U);
  EXPECT_FLOAT_EQ(result.ranges.front(), 10.0F);
}

TEST(DynamicAgentLidarStateTest, AlignsTrackedTargetAndPeerToScanTime) {
  constexpr std::int64_t kStampNs{9'800'000'000LL};
  constexpr std::int64_t kAcquisitionNs{9'900'000'000LL};
  constexpr std::int64_t kNowNs{10'000'000'000LL};
  DynamicAgentLidarState state{DynamicAgentLidarStateConfig{
      .cooperative_enabled = true,
      .own_vehicle_id = "civilian_0",
      .tracked_agent_radius_m = 1.0,
      .tracked_agent_vertical_tolerance_m = 1.0,
      .tracked_agent_maximum_age_s = 0.5,
      .tracked_agent_excluded_from_latest_safety = true,
      .cooperative_peer_horizontal_margin_m = 0.0,
      .cooperative_peer_vertical_margin_m = 0.0,
      .cooperative_alignment_extrapolation_s = 0.5,
      .peer_store =
          CooperativePeerStoreConfig{
              .maximum_publication_age_s = 0.5,
              .maximum_peers = 4U,
          },
  }};
  state.updateTrackedAgent(Point3{5.0, 0.0, 10.0}, Vec3{1.0, 0.0, 0.0}, true, true,
                           kStampNs);
  EXPECT_EQ(state.updateCooperativeIntent(peerIntent("civilian_0", kStampNs), kNowNs),
            CooperativePeerUpdateStatus::kIgnoredOwnship);
  EXPECT_EQ(state.updateCooperativeIntent(peerIntent("civilian_1", kStampNs), kNowNs),
            CooperativePeerUpdateStatus::kAccepted);

  const DynamicAgentLidarFilterPlan plan = state.makeFilterPlan(kNowNs, kAcquisitionNs);

  ASSERT_EQ(plan.tracked_agent_exclusions.size(), 1U);
  EXPECT_NEAR(plan.tracked_agent_exclusions.front().position.x, 5.1, 1.0e-9);
  EXPECT_TRUE(plan.tracked_agent_excluded_from_latest_safety);
  ASSERT_EQ(plan.cooperative_memory_exclusions.size(), 1U);
  EXPECT_NEAR(plan.cooperative_memory_exclusions.front().position.x, 8.2, 1.0e-9);
  EXPECT_DOUBLE_EQ(plan.cooperative_memory_exclusions.front().radius_m, 0.8);
  EXPECT_DOUBLE_EQ(plan.cooperative_memory_exclusions.front().lower_extent_m, 0.4);
  EXPECT_DOUBLE_EQ(plan.cooperative_memory_exclusions.front().upper_extent_m, 0.6);
}

TEST(DynamicAgentLidarStateTest, DropsStalePeerInsteadOfMaskingStaticMemory) {
  constexpr std::int64_t kStampNs{9'000'000'000LL};
  constexpr std::int64_t kUpdateNs{9'100'000'000LL};
  constexpr std::int64_t kNowNs{10'000'000'000LL};
  DynamicAgentLidarState state{DynamicAgentLidarStateConfig{
      .cooperative_enabled = true,
      .own_vehicle_id = "civilian_0",
      .peer_store =
          CooperativePeerStoreConfig{
              .maximum_publication_age_s = 0.5,
              .maximum_peers = 4U,
          },
  }};
  EXPECT_EQ(
      state.updateCooperativeIntent(peerIntent("civilian_1", kStampNs), kUpdateNs),
      CooperativePeerUpdateStatus::kAccepted);

  const DynamicAgentLidarFilterPlan plan = state.makeFilterPlan(kNowNs, kNowNs);

  EXPECT_TRUE(plan.cooperative_memory_exclusions.empty());
}

} // namespace
} // namespace drone_city_nav
