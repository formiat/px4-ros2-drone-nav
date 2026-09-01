#include <gtest/gtest.h>

#include <optional>

#include "production_mppi_execution_control.hpp"
#include "production_mppi_node_types.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] AppliedControlEvidence3D plannedControl() {
  AppliedControlEvidence3D applied;
  applied.control = MotionControl3D{.ax = 4.0F, .ay = 0.0F, .az = 0.0F};
  applied.source_stamp_ns = 1'010'000'000;
  applied.receive_stamp_ns = 1'020'000'000;
  applied.producer_instance_id = 91U;
  applied.horizon_producer_instance_id = 92U;
  applied.horizon_sequence = 17U;
  applied.execution_mode = ExecutionAuthorityMode3D::kPlanned;
  applied.control_authoritative = true;
  applied.valid = true;
  return applied;
}

[[nodiscard]] ExecutionOwnerIdentity3D plannedOwner() {
  return {
      .valid_from_ns = 1'000'000'000,
      .valid_until_ns = 2'000'000'000,
      .producer_instance_id = 92U,
      .target_offboard_instance_id = 91U,
      .sequence = 17U,
      .execution_mode = ExecutionAuthorityMode3D::kPlanned,
      .valid = true,
  };
}

TEST(ProductionMppiExecutionControlRuntimeTest,
     ExactAppliedControlBodyAxisTakesPriorityOverMeasuredAcceleration) {
  const AppliedControlEvidence3D applied = plannedControl();
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  ProductionMppiNavigation navigation;
  navigation.measured_equivalent_control =
      MotionControl3D{.ax = -4.0F, .ay = 0.0F, .az = 0.0F};
  navigation.receive_stamp_ns = 1'025'000'000;
  navigation.linear_acceleration_authoritative = true;
  navigation.valid = true;

  const std::optional<FootprintBodyAxis> selected = authoritativeBodyAxisForExecution(
      applied, owner, navigation, 1'030'000'000, 100.0, 100.0);
  ASSERT_TRUE(selected.has_value());
  const FootprintBodyAxis selected_value = selected.value_or(FootprintBodyAxis{});
  const FootprintBodyAxis expected = bodyAxisFromWorldAcceleration(Vec3{4.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(selected_value.x, expected.x);
  EXPECT_DOUBLE_EQ(selected_value.y, expected.y);
  EXPECT_DOUBLE_EQ(selected_value.z, expected.z);
}

TEST(ProductionMppiExecutionControlRuntimeTest,
     MeasuredBodyAxisIsFallbackForNonOwningAppliedControl) {
  AppliedControlEvidence3D applied = plannedControl();
  --applied.horizon_sequence;
  const ExecutionOwnerIdentity3D owner = plannedOwner();
  ProductionMppiNavigation navigation;
  navigation.measured_equivalent_control =
      MotionControl3D{.ax = -4.0F, .ay = 1.0F, .az = 0.5F};
  navigation.receive_stamp_ns = 1'025'000'000;
  navigation.linear_acceleration_authoritative = true;
  navigation.valid = true;

  const std::optional<FootprintBodyAxis> selected = authoritativeBodyAxisForExecution(
      applied, owner, navigation, 1'030'000'000, 100.0, 100.0);
  ASSERT_TRUE(selected.has_value());
  const FootprintBodyAxis selected_value = selected.value_or(FootprintBodyAxis{});
  const FootprintBodyAxis expected =
      bodyAxisFromWorldAcceleration(Vec3{-4.0, 1.0, 0.5});
  EXPECT_DOUBLE_EQ(selected_value.x, expected.x);
  EXPECT_DOUBLE_EQ(selected_value.y, expected.y);
  EXPECT_DOUBLE_EQ(selected_value.z, expected.z);
}

TEST(ProductionMppiExecutionControlRuntimeTest,
     MissingOrStaleAccelerationEvidenceDoesNotInventBodyAxis) {
  AppliedControlEvidence3D applied;
  ExecutionOwnerIdentity3D owner;
  ProductionMppiNavigation navigation;
  navigation.receive_stamp_ns = 1'000'000'000;
  navigation.valid = true;

  EXPECT_FALSE(authoritativeBodyAxisForExecution(applied, owner, navigation,
                                                 1'010'000'000, 100.0, 100.0)
                   .has_value());

  navigation.linear_acceleration_authoritative = true;
  EXPECT_FALSE(authoritativeBodyAxisForExecution(applied, owner, navigation,
                                                 1'500'000'000, 100.0, 100.0)
                   .has_value());
}

TEST(ProductionMppiExecutionControlRuntimeTest,
     AppliedControlAuthorityRequiresExactTwoSidedLiveOwnership) {
  const AppliedControlEvidence3D applied = plannedControl();
  const ExecutionOwnerIdentity3D owner = plannedOwner();

  EXPECT_TRUE(
      appliedControlAuthoritativeForExecution(applied, owner, 1'030'000'000, 100.0));

  AppliedControlEvidence3D bounded_clock_skew = applied;
  bounded_clock_skew.source_stamp_ns = 1'050'000'000;
  bounded_clock_skew.receive_stamp_ns = 1'045'000'000;
  EXPECT_TRUE(appliedControlAuthoritativeForExecution(bounded_clock_skew, owner,
                                                      1'040'000'000, 100.0));

  AppliedControlEvidence3D future_feedback = bounded_clock_skew;
  future_feedback.source_stamp_ns = 1'200'000'000;
  future_feedback.receive_stamp_ns = 1'200'000'000;
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(future_feedback, owner,
                                                       1'040'000'000, 100.0));

  AppliedControlEvidence3D wrong_offboard = applied;
  ++wrong_offboard.producer_instance_id;
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(wrong_offboard, owner,
                                                       1'030'000'000, 100.0));

  AppliedControlEvidence3D wrong_planner = applied;
  ++wrong_planner.horizon_producer_instance_id;
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(wrong_planner, owner,
                                                       1'030'000'000, 100.0));

  AppliedControlEvidence3D revoked = applied;
  revoked.control_authoritative = false;
  EXPECT_FALSE(
      appliedControlAuthoritativeForExecution(revoked, owner, 1'030'000'000, 100.0));
  AppliedControlEvidence3D delayed = applied;
  delayed.source_stamp_ns = 1'010'000'000;
  delayed.receive_stamp_ns = 1'490'000'000;
  EXPECT_FALSE(
      appliedControlAuthoritativeForExecution(delayed, owner, 1'500'000'000, 100.0));
  EXPECT_FALSE(appliedControlAuthoritativeForExecution(applied, owner,
                                                       owner.valid_until_ns, 100.0));
}

TEST(ProductionMppiExecutionControlRuntimeTest,
     VehicleStatusAuthorityRequiresFreshStableArmedObservation) {
  ProductionMppiVehicleStatus status{
      .receive_stamp_ns = 1'000'000'000,
      .source_timestamp_us = 900'000U,
      .revision = 3U,
      .armed = true,
      .valid = true,
  };

  EXPECT_TRUE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'050'000'000, 100.0));
  EXPECT_TRUE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'500'000'000, 1000.0));
  EXPECT_TRUE(
      vehicleStatusAuthoritativeForExecution(status, true, 2'000'000'000, 1000.0));
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 2'000'000'001, 1000.0));
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, false, 1'050'000'000, 100.0));
  status.armed = false;
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'050'000'000, 100.0));
  status.armed = true;
  status.valid = false;
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'050'000'000, 100.0));
  status.valid = true;
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 1'101'000'000, 100.0));
  EXPECT_FALSE(
      vehicleStatusAuthoritativeForExecution(status, true, 999'999'999, 100.0));
}

} // namespace
} // namespace drone_city_nav
