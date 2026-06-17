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

} // namespace drone_city_nav
