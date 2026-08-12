#include "drone_city_nav/cooperative_channel_execution.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecondNs{1'000'000'000LL};

[[nodiscard]] ConstrainedRouteObservation approach() {
  return ConstrainedRouteObservation{
      .phase = ConstrainedRoutePhase::kApproach,
      .route_generation = 9U,
      .span_index = 0U,
      .span_count = 1U,
      .span_available = true,
      .channel_id = "channel_t:west_east",
      .direction_sign = 1,
      .station_m = 15.0,
      .begin_station_m = 25.0,
      .end_station_m = 55.0,
      .distance_to_entry_m = 10.0,
      .distance_to_exit_m = 40.0,
      .actual_horizontal_speed_mps = 5.0,
  };
}

[[nodiscard]] CooperativeChannelLaneAssignment assignment() {
  return CooperativeChannelLaneAssignment{
      .channel_id = "channel_t:west_east",
      .route_generation = 9U,
      .span_index = 0U,
      .lane_index = 0U,
      .lane_count = 3U,
      .lateral_offset_m = -5.0,
      .status = CooperativeChannelLaneRouteStatus::kApplied,
  };
}

[[nodiscard]] CooperativeManeuverCommandData yieldCommand() {
  return CooperativeManeuverCommandData{
      .vehicle_id = "civilian_0",
      .stamp_ns = 10 * kSecondNs,
      .command_generation = 3U,
      .valid_until_ns = 11 * kSecondNs,
      .avoidance_active = false,
      .preferred_maneuver = CooperativeManeuver::kSlow,
      .preferred_acceleration_direction = {},
      .conflict_generation = 0U,
      .channel_yield_required = true,
      .channel_yield_to_vehicle_id = "civilian_1",
      .channel_id = "channel_t:west_east",
      .channel_conflict_resource_id = "channel_t",
      .channel_route_generation = 9U,
      .channel_lane_index = 0U,
      .channel_lane_count = 3U,
      .conflicting_peers = {},
  };
}

TEST(CooperativeChannelExecution, PublishesCapacityAndTimeWindow) {
  const CooperativeChannelUse channel = makeCooperativeChannelUse(
      approach(), assignment(), 10 * kSecondNs, 10.0, CooperativeChannelTimingConfig{});

  ASSERT_TRUE(channel.active());
  EXPECT_EQ(channel.conflict_resource_id, "channel_t");
  EXPECT_EQ(channel.lane_index, 0U);
  EXPECT_EQ(channel.lane_count, 3U);
  EXPECT_EQ(channel.predicted_entry_ns, 11 * kSecondNs);
  EXPECT_EQ(channel.predicted_exit_ns, 14 * kSecondNs);
}

TEST(CooperativeChannelExecution, AcceptsOnlyCurrentRouteAndLaneYield) {
  const ConstrainedRouteObservation observation = approach();
  const CooperativeChannelUse channel =
      makeCooperativeChannelUse(observation, assignment(), 10 * kSecondNs, 10.0,
                                CooperativeChannelTimingConfig{});

  const CooperativeChannelYieldDecision accepted = evaluateCooperativeChannelYield(
      yieldCommand(), channel, observation, "civilian_0", 10 * kSecondNs, 5.0,
      CooperativeChannelYieldConfig{});

  ASSERT_TRUE(accepted.active);
  EXPECT_EQ(accepted.status, CooperativeChannelYieldStatus::kAccepted);
  EXPECT_DOUBLE_EQ(accepted.hold_station_m, 23.0);
  EXPECT_GT(accepted.maximum_speed_mps, 0.0);

  CooperativeManeuverCommandData stale_route = yieldCommand();
  stale_route.channel_route_generation = 8U;
  EXPECT_EQ(evaluateCooperativeChannelYield(stale_route, channel, observation,
                                            "civilian_0", 10 * kSecondNs, 5.0,
                                            CooperativeChannelYieldConfig{})
                .status,
            CooperativeChannelYieldStatus::kRouteMismatch);

  CooperativeManeuverCommandData wrong_lane = yieldCommand();
  wrong_lane.channel_lane_index = 2U;
  EXPECT_EQ(evaluateCooperativeChannelYield(wrong_lane, channel, observation,
                                            "civilian_0", 10 * kSecondNs, 5.0,
                                            CooperativeChannelYieldConfig{})
                .status,
            CooperativeChannelYieldStatus::kLaneMismatch);
}

TEST(CooperativeChannelExecution, CommandsHoldAtConfiguredEntryBuffer) {
  ConstrainedRouteObservation observation = approach();
  observation.station_m = 23.0;
  observation.distance_to_entry_m = 2.0;
  const CooperativeChannelUse channel = makeCooperativeChannelUse(
      observation, assignment(), 10 * kSecondNs, 5.0, CooperativeChannelTimingConfig{});

  const CooperativeChannelYieldDecision decision = evaluateCooperativeChannelYield(
      yieldCommand(), channel, observation, "civilian_0", 10 * kSecondNs, 1.0,
      CooperativeChannelYieldConfig{});

  EXPECT_TRUE(decision.active);
  EXPECT_TRUE(decision.hold_at_entry);
  EXPECT_DOUBLE_EQ(decision.maximum_speed_mps, 0.0);
}

} // namespace
} // namespace drone_city_nav
