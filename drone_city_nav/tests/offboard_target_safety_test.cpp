#include "drone_city_nav/offboard_target_safety.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] TargetSegmentSafetyInput baseInput() {
  TargetSegmentSafetyInput input{};
  input.grid_available = true;
  input.start_cell_valid = true;
  input.end_cell_valid = true;
  return input;
}

} // namespace

TEST(OffboardTargetSafety, AllowsClearSegment) {
  TargetSegmentSafetyInput input = baseInput();

  const TargetSegmentSafety safety = evaluateTargetSegmentSafetyPolicy(input);

  EXPECT_TRUE(safety.allowed);
  EXPECT_FALSE(safety.escape);
  EXPECT_EQ(safety.reason, TargetSegmentSafetyReason::kAllowed);
}

TEST(OffboardTargetSafety, RejectsOccupiedSegmentEvenDuringEscape) {
  TargetSegmentSafetyInput input = baseInput();
  input.allow_escape = true;
  input.clearance_stop_requested = true;
  input.occupied_cells = 1U;
  input.blocked_cells = 1U;
  input.start_clearance_m = 0.5;
  input.end_clearance_m = 2.0;

  const TargetSegmentSafety safety = evaluateTargetSegmentSafetyPolicy(input);

  EXPECT_FALSE(safety.allowed);
  EXPECT_FALSE(safety.escape);
  EXPECT_EQ(safety.reason, TargetSegmentSafetyReason::kOccupied);
}

TEST(OffboardTargetSafety, AllowsEscapeFromInflatedStartWhenClearanceImproves) {
  TargetSegmentSafetyInput input = baseInput();
  input.allow_escape = true;
  input.start_blocked = true;
  input.blocked_cells = 2U;
  input.start_clearance_m = 0.5;
  input.end_clearance_m = 0.7;
  input.min_clearance_improvement_m = 0.1;

  const TargetSegmentSafety safety = evaluateTargetSegmentSafetyPolicy(input);

  EXPECT_TRUE(safety.allowed);
  EXPECT_TRUE(safety.escape);
  EXPECT_EQ(safety.reason, TargetSegmentSafetyReason::kEscape);
}

TEST(OffboardTargetSafety,
     AllowsEscapeWhenContinuousClearanceStopHappenedBeforeInflation) {
  TargetSegmentSafetyInput input = baseInput();
  input.allow_escape = true;
  input.clearance_stop_requested = true;
  input.blocked_cells = 1U;
  input.start_blocked = false;
  input.start_clearance_m = 0.8;
  input.end_clearance_m = 1.1;
  input.min_clearance_improvement_m = 0.1;

  const TargetSegmentSafety safety = evaluateTargetSegmentSafetyPolicy(input);

  EXPECT_TRUE(safety.allowed);
  EXPECT_TRUE(safety.escape);
  EXPECT_EQ(safety.reason, TargetSegmentSafetyReason::kEscape);
}

TEST(OffboardTargetSafety, RequestsEscapeDuringClearanceSlowdownWithPositiveSpeed) {
  ClearanceEscapeRequestInput input{};
  input.enabled = true;
  input.hold_position = false;
  input.escape_step_m = 0.5;
  input.current_position_in_inflated_safety_cell = false;
  input.speed_limit_reason = SpeedLimitReason::kClearance;
  input.clearance_limit_mps = 0.8;

  EXPECT_TRUE(clearanceEscapeRequested(input));
}

TEST(OffboardTargetSafety, DoesNotRequestEscapeDuringCruise) {
  ClearanceEscapeRequestInput input{};
  input.enabled = true;
  input.hold_position = false;
  input.escape_step_m = 0.5;
  input.current_position_in_inflated_safety_cell = false;
  input.speed_limit_reason = SpeedLimitReason::kCruise;

  EXPECT_FALSE(clearanceEscapeRequested(input));
}

TEST(OffboardTargetSafety, RejectsBlockedSegmentWithoutEscapeImprovement) {
  TargetSegmentSafetyInput input = baseInput();
  input.allow_escape = true;
  input.clearance_stop_requested = true;
  input.blocked_cells = 1U;
  input.start_clearance_m = 0.8;
  input.end_clearance_m = 0.82;
  input.min_clearance_improvement_m = 0.1;

  const TargetSegmentSafety safety = evaluateTargetSegmentSafetyPolicy(input);

  EXPECT_FALSE(safety.allowed);
  EXPECT_FALSE(safety.escape);
  EXPECT_EQ(safety.reason, TargetSegmentSafetyReason::kBlocked);
}

TEST(OffboardTargetSafety, RejectsOutsideGrid) {
  TargetSegmentSafetyInput input = baseInput();
  input.end_cell_valid = false;

  const TargetSegmentSafety safety = evaluateTargetSegmentSafetyPolicy(input);

  EXPECT_FALSE(safety.allowed);
  EXPECT_EQ(safety.reason, TargetSegmentSafetyReason::kOutsideGrid);
}

TEST(OffboardTargetSafety, AllowsClearEscapeCommandStep) {
  TargetSegmentSafety safety{};
  safety.allowed = true;
  safety.reason = TargetSegmentSafetyReason::kAllowed;
  safety.blocked_cells = 0U;

  EXPECT_TRUE(escapeCommandStepAllowed(safety, 0.1));
}

TEST(OffboardTargetSafety, RejectsEscapeCommandStepThatLosesClearance) {
  TargetSegmentSafety safety{};
  safety.allowed = false;
  safety.reason = TargetSegmentSafetyReason::kBlocked;
  safety.blocked_cells = 2U;
  safety.start_clearance_m = 0.30;
  safety.end_clearance_m = 0.25;

  EXPECT_FALSE(escapeCommandStepAllowed(safety, 0.05));
}

TEST(OffboardTargetSafety, RejectsEscapeCommandStepBelowRequiredImprovement) {
  TargetSegmentSafety safety{};
  safety.allowed = false;
  safety.reason = TargetSegmentSafetyReason::kBlocked;
  safety.blocked_cells = 2U;
  safety.start_clearance_m = 0.30;
  safety.end_clearance_m = 0.34;

  EXPECT_FALSE(escapeCommandStepAllowed(safety, 0.05));
}

TEST(OffboardTargetSafety, AllowsEscapeCommandStepWithRequiredImprovement) {
  TargetSegmentSafety safety{};
  safety.allowed = false;
  safety.reason = TargetSegmentSafetyReason::kBlocked;
  safety.blocked_cells = 2U;
  safety.start_clearance_m = 0.30;
  safety.end_clearance_m = 0.36;

  EXPECT_TRUE(escapeCommandStepAllowed(safety, 0.05));
}

} // namespace drone_city_nav
