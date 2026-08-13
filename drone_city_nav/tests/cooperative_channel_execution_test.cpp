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

[[nodiscard]] CooperativeChannelAssignment assignment() {
  return CooperativeChannelAssignment{
      .channel_id = "channel_t:west_east",
      .route_generation = 9U,
      .span_index = 0U,
      .physical_width_m = 14.0,
      .minimum_lateral_offset_m = -6.0,
      .maximum_lateral_offset_m = 6.0,
      .requested_lateral_offset_m = -3.0,
      .applied_lateral_offset_m = -3.0,
      .desired_center_separation_m = 5.0,
      .status = CooperativeChannelRouteStatus::kApplied,
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
      .space_time_plan_active = false,
      .space_time_lateral_offset_m = 0.0,
      .space_time_vertical_offset_m = 0.0,
      .space_time_shift_s = 0.0,
      .space_time_predicted_minimum_separation_m = 0.0,
      .space_time_integrated_shortfall_m2_s = 0.0,
      .space_time_evaluated_candidate_count = 0U,
      .channel_yield_required = true,
      .channel_yield_to_vehicle_id = "civilian_1",
      .channel_id = "channel_t:west_east",
      .channel_conflict_resource_id = "channel_t",
      .channel_route_generation = 9U,
      .channel_lateral_offset_m = -3.0,
      .channel_minimum_lateral_offset_m = -6.0,
      .channel_maximum_lateral_offset_m = 6.0,
      .channel_entry_not_before_ns = 11 * kSecondNs,
      .conflicting_peers = {},
  };
}

TEST(CooperativeChannelExecution, PublishesCorridorOffsetAndTimeWindow) {
  const CooperativeChannelUse channel = makeCooperativeChannelUse(
      approach(), assignment(), 10 * kSecondNs, 10.0, CooperativeChannelTimingConfig{});

  ASSERT_TRUE(channel.active());
  EXPECT_EQ(channel.conflict_resource_id, "channel_t");
  EXPECT_DOUBLE_EQ(channel.lateral_offset_m, -3.0);
  EXPECT_DOUBLE_EQ(channel.minimum_lateral_offset_m, -6.0);
  EXPECT_DOUBLE_EQ(channel.maximum_lateral_offset_m, 6.0);
  EXPECT_EQ(channel.predicted_entry_ns, 11 * kSecondNs);
  EXPECT_EQ(channel.predicted_exit_ns, 14 * kSecondNs);
}

TEST(CooperativeChannelExecution, AcceptsOnlyCurrentRouteAndCorridorYield) {
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
  EXPECT_EQ(accepted.entry_not_before_ns, 11 * kSecondNs);

  CooperativeManeuverCommandData stale_route = yieldCommand();
  stale_route.channel_route_generation = 8U;
  EXPECT_EQ(evaluateCooperativeChannelYield(stale_route, channel, observation,
                                            "civilian_0", 10 * kSecondNs, 5.0,
                                            CooperativeChannelYieldConfig{})
                .status,
            CooperativeChannelYieldStatus::kRouteMismatch);

  CooperativeManeuverCommandData wrong_offset = yieldCommand();
  wrong_offset.channel_lateral_offset_m = 2.0;
  EXPECT_EQ(evaluateCooperativeChannelYield(wrong_offset, channel, observation,
                                            "civilian_0", 10 * kSecondNs, 5.0,
                                            CooperativeChannelYieldConfig{})
                .status,
            CooperativeChannelYieldStatus::kCorridorMismatch);
}

TEST(CooperativeChannelExecution, ReleasesSatisfiedEntryTime) {
  const ConstrainedRouteObservation observation = approach();
  const CooperativeChannelUse channel =
      makeCooperativeChannelUse(observation, assignment(), 10 * kSecondNs, 10.0,
                                CooperativeChannelTimingConfig{});

  const CooperativeChannelYieldDecision decision = evaluateCooperativeChannelYield(
      yieldCommand(), channel, observation, "civilian_0", 11 * kSecondNs, 5.0,
      CooperativeChannelYieldConfig{});

  EXPECT_FALSE(decision.active);
  EXPECT_EQ(decision.status, CooperativeChannelYieldStatus::kEntryTimeSatisfied);
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
