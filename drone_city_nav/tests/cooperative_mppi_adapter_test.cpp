#include "drone_city_nav/cooperative_mppi_adapter.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecondNs{1'000'000'000LL};

[[nodiscard]] CooperativeManeuverCommandData command() {
  return CooperativeManeuverCommandData{
      .vehicle_id = "civilian_0",
      .stamp_ns = 10 * kSecondNs,
      .command_generation = 4U,
      .valid_until_ns = 11 * kSecondNs,
      .avoidance_active = true,
      .preferred_maneuver = CooperativeManeuver::kClimb,
      .preferred_acceleration_direction = Vec3{0.0, 0.0, 1.0},
      .conflict_generation = 2U,
      .space_time_plan_active = true,
      .space_time_lateral_offset_m = 1.25,
      .space_time_vertical_offset_m = 0.5,
      .space_time_shift_s = 0.75,
      .space_time_predicted_minimum_separation_m = 5.5,
      .space_time_integrated_shortfall_m2_s = 0.25,
      .space_time_evaluated_candidate_count = 7U,
      .channel_yield_required = false,
      .channel_yield_to_vehicle_id = {},
      .channel_id = {},
      .channel_conflict_resource_id = {},
      .channel_route_generation = 0U,
      .channel_lateral_offset_m = 0.0,
      .channel_minimum_lateral_offset_m = 0.0,
      .channel_maximum_lateral_offset_m = 0.0,
      .channel_entry_not_before_ns = 0,
      .conflicting_peers =
          {
              CooperativePeerTrajectoryData{
                  .vehicle_id = "civilian_1",
                  .valid_from_ns = 10 * kSecondNs,
                  .valid_until_ns = 12 * kSecondNs,
                  .footprint_radius_m = 0.82,
                  .trajectory =
                      {
                          CooperativeTrajectorySample{
                              .time_ns = 10 * kSecondNs,
                              .position = Point3{0.0, 0.0, 5.0},
                              .velocity = Vec3{2.0, 0.0, 0.0},
                          },
                          CooperativeTrajectorySample{
                              .time_ns = 12 * kSecondNs,
                              .position = Point3{4.0, 0.0, 5.0},
                              .velocity = Vec3{2.0, 0.0, 0.0},
                          },
                      },
              },
          },
  };
}

TEST(CooperativeMppiAdapter, AlignsPeerTrajectoryToPlannerSteps) {
  const CooperativeMppiAdapterResult result =
      adaptCooperativeMppiCommand(command(), "civilian_0", 10 * kSecondNs, 4U, 0.5F);

  ASSERT_TRUE(result.accepted());
  ASSERT_EQ(result.conflicting_peers.size(), 1U);
  ASSERT_TRUE(result.conflicting_peers.front().samples);
  EXPECT_EQ(result.conflicting_peers.front().active_steps, 4U);
  EXPECT_FLOAT_EQ(result.conflicting_peers.front().samples->at(0U).x, 1.0F);
  EXPECT_FLOAT_EQ(result.conflicting_peers.front().samples->at(3U).x, 4.0F);
  ASSERT_TRUE(result.maneuver.has_value());
  ASSERT_TRUE(result.acquisition.has_value());
  EXPECT_TRUE(result.avoidance_active);
  EXPECT_TRUE(result.space_time_plan_active);
  EXPECT_DOUBLE_EQ(result.space_time_lateral_offset_m, 1.25);
  EXPECT_DOUBLE_EQ(result.space_time_vertical_offset_m, 0.5);
  EXPECT_DOUBLE_EQ(result.space_time_shift_s, 0.75);
  EXPECT_DOUBLE_EQ(result.space_time_predicted_minimum_separation_m, 5.5);
  EXPECT_DOUBLE_EQ(result.space_time_integrated_shortfall_m2_s, 0.25);
  EXPECT_EQ(result.space_time_evaluated_candidate_count, 7U);
  const mppi::CooperativeManeuverPreference maneuver =
      result.maneuver.value_or(mppi::CooperativeManeuverPreference{});
  EXPECT_EQ(maneuver.maneuver, mppi::CooperativeManeuver::kClimb);
  EXPECT_FLOAT_EQ(maneuver.direction_z, 1.0F);
}

TEST(CooperativeMppiAdapter, StopsAtPeerValidityBoundary) {
  CooperativeManeuverCommandData input = command();
  input.conflicting_peers.front().valid_until_ns = 11 * kSecondNs;

  const CooperativeMppiAdapterResult result =
      adaptCooperativeMppiCommand(input, "civilian_0", 10 * kSecondNs, 6U, 0.5F);

  ASSERT_TRUE(result.accepted());
  ASSERT_EQ(result.conflicting_peers.size(), 1U);
  EXPECT_EQ(result.conflicting_peers.front().active_steps, 2U);
  EXPECT_FLOAT_EQ(result.conflicting_peers.front().samples->at(2U).x,
                  result.conflicting_peers.front().samples->at(1U).x);
}

TEST(CooperativeMppiAdapter, RejectsStaleOrWrongVehicleCommand) {
  EXPECT_EQ(
      adaptCooperativeMppiCommand(command(), "civilian_1", 10 * kSecondNs, 4U, 0.5F)
          .status,
      CooperativeMppiAdapterStatus::kVehicleMismatch);
  EXPECT_EQ(
      adaptCooperativeMppiCommand(command(), "civilian_0", 12 * kSecondNs, 4U, 0.5F)
          .status,
      CooperativeMppiAdapterStatus::kStale);
}

TEST(CooperativeMppiAdapter, RejectsInvalidActiveSpaceTimeContract) {
  CooperativeManeuverCommandData input = command();
  input.space_time_evaluated_candidate_count = 0U;

  EXPECT_EQ(
      adaptCooperativeMppiCommand(input, "civilian_0", 10 * kSecondNs, 4U, 0.5F).status,
      CooperativeMppiAdapterStatus::kInvalid);
}

TEST(CooperativeMppiAdapter, InactiveCommandAddsNoPeerCostOrBias) {
  CooperativeManeuverCommandData input = command();
  input.avoidance_active = false;
  input.space_time_plan_active = false;
  input.space_time_evaluated_candidate_count = 0U;

  const CooperativeMppiAdapterResult result =
      adaptCooperativeMppiCommand(input, "civilian_0", 10 * kSecondNs, 4U, 0.5F);

  EXPECT_TRUE(result.accepted());
  EXPECT_TRUE(result.conflicting_peers.empty());
  EXPECT_FALSE(result.maneuver.has_value());
  EXPECT_FALSE(result.acquisition.has_value());
  EXPECT_FALSE(result.avoidance_active);
}

} // namespace
} // namespace drone_city_nav
