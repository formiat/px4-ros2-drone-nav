#include "drone_city_nav/cooperative_passage_execution.hpp"

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
      .passage_traversal_id = "passage_t:west_east",
      .direction_sign = 1,
      .station_m = 15.0,
      .begin_station_m = 25.0,
      .end_station_m = 55.0,
      .distance_to_entry_m = 10.0,
      .distance_to_exit_m = 40.0,
      .actual_horizontal_speed_mps = 5.0,
  };
}

[[nodiscard]] CooperativePassageAssignment assignment() {
  return CooperativePassageAssignment{
      .passage_traversal_id = "passage_t:west_east",
      .route_generation = 9U,
      .span_index = 0U,
      .physical_width_m = 14.0,
      .minimum_lateral_offset_m = -6.0,
      .maximum_lateral_offset_m = 6.0,
      .requested_lateral_offset_m = -3.0,
      .applied_lateral_offset_m = -3.0,
      .desired_center_separation_m = 5.0,
      .status = CooperativePassageRouteStatus::kApplied,
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
      .passage_yield_required = true,
      .passage_yield_to_vehicle_id = "civilian_1",
      .passage_traversal_id = "passage_t:west_east",
      .passage_conflict_resource_id = "passage_t",
      .passage_route_generation = 9U,
      .passage_lateral_offset_m = -3.0,
      .passage_minimum_lateral_offset_m = -6.0,
      .passage_maximum_lateral_offset_m = 6.0,
      .passage_entry_not_before_ns = 11 * kSecondNs,
      .conflicting_peers = {},
  };
}

TEST(CooperativePassageExecution, PublishesCorridorOffsetAndTimeWindow) {
  const CooperativePassageUse passage = makeCooperativePassageUse(
      approach(), assignment(), 10 * kSecondNs, 10.0, CooperativePassageTimingConfig{});

  ASSERT_TRUE(passage.active());
  EXPECT_EQ(passage.conflict_resource_id, "passage_t");
  EXPECT_DOUBLE_EQ(passage.lateral_offset_m, -3.0);
  EXPECT_DOUBLE_EQ(passage.minimum_lateral_offset_m, -6.0);
  EXPECT_DOUBLE_EQ(passage.maximum_lateral_offset_m, 6.0);
  EXPECT_EQ(passage.predicted_entry_ns, 11 * kSecondNs);
  EXPECT_EQ(passage.predicted_exit_ns, 14 * kSecondNs);
}

TEST(CooperativePassageExecution, GroupsPortalMovementsIntoOneConflictResource) {
  ConstrainedRouteObservation observation = approach();
  observation.passage_traversal_id = "passage_t:west_north";
  CooperativePassageAssignment movement = assignment();
  movement.passage_traversal_id = observation.passage_traversal_id;

  const CooperativePassageUse passage = makeCooperativePassageUse(
      observation, movement, 10 * kSecondNs, 10.0, CooperativePassageTimingConfig{});

  EXPECT_EQ(passage.conflict_resource_id, "passage_t");
}

TEST(CooperativePassageExecution, AcceptsOnlyCurrentRouteAndCorridorYield) {
  const ConstrainedRouteObservation observation = approach();
  const CooperativePassageUse passage =
      makeCooperativePassageUse(observation, assignment(), 10 * kSecondNs, 10.0,
                                CooperativePassageTimingConfig{});

  const CooperativePassageYieldDecision accepted = evaluateCooperativePassageYield(
      yieldCommand(), passage, observation, "civilian_0", 10 * kSecondNs, 5.0,
      CooperativePassageYieldConfig{});

  ASSERT_TRUE(accepted.active);
  EXPECT_EQ(accepted.status, CooperativePassageYieldStatus::kAccepted);
  EXPECT_DOUBLE_EQ(accepted.hold_station_m, 23.0);
  EXPECT_GT(accepted.maximum_speed_mps, 0.0);
  EXPECT_EQ(accepted.entry_not_before_ns, 11 * kSecondNs);

  CooperativeManeuverCommandData stale_route = yieldCommand();
  stale_route.passage_route_generation = 8U;
  EXPECT_EQ(evaluateCooperativePassageYield(stale_route, passage, observation,
                                            "civilian_0", 10 * kSecondNs, 5.0,
                                            CooperativePassageYieldConfig{})
                .status,
            CooperativePassageYieldStatus::kRouteMismatch);

  CooperativeManeuverCommandData wrong_offset = yieldCommand();
  wrong_offset.passage_lateral_offset_m = 2.0;
  EXPECT_EQ(evaluateCooperativePassageYield(wrong_offset, passage, observation,
                                            "civilian_0", 10 * kSecondNs, 5.0,
                                            CooperativePassageYieldConfig{})
                .status,
            CooperativePassageYieldStatus::kCorridorMismatch);
}

TEST(CooperativePassageExecution, ReleasesSatisfiedEntryTime) {
  const ConstrainedRouteObservation observation = approach();
  const CooperativePassageUse passage =
      makeCooperativePassageUse(observation, assignment(), 10 * kSecondNs, 10.0,
                                CooperativePassageTimingConfig{});

  const CooperativePassageYieldDecision decision = evaluateCooperativePassageYield(
      yieldCommand(), passage, observation, "civilian_0", 11 * kSecondNs, 5.0,
      CooperativePassageYieldConfig{});

  EXPECT_FALSE(decision.active);
  EXPECT_EQ(decision.status, CooperativePassageYieldStatus::kEntryTimeSatisfied);
}

TEST(CooperativePassageExecution, CommandsHoldAtConfiguredEntryBuffer) {
  ConstrainedRouteObservation observation = approach();
  observation.station_m = 23.0;
  observation.distance_to_entry_m = 2.0;
  const CooperativePassageUse passage = makeCooperativePassageUse(
      observation, assignment(), 10 * kSecondNs, 5.0, CooperativePassageTimingConfig{});

  const CooperativePassageYieldDecision decision = evaluateCooperativePassageYield(
      yieldCommand(), passage, observation, "civilian_0", 10 * kSecondNs, 1.0,
      CooperativePassageYieldConfig{});

  EXPECT_TRUE(decision.active);
  EXPECT_TRUE(decision.hold_at_entry);
  EXPECT_DOUBLE_EQ(decision.maximum_speed_mps, 0.0);
}

} // namespace
} // namespace drone_city_nav
