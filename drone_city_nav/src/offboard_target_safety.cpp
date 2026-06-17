#include "drone_city_nav/offboard_target_safety.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kStoppedClearanceLimitMps = 1.0e-6;

} // namespace

TargetSegmentSafety
evaluateTargetSegmentSafetyPolicy(const TargetSegmentSafetyInput& input) noexcept {
  TargetSegmentSafety safety{};
  safety.grid_available = input.grid_available;
  safety.start_prohibited = input.start_prohibited;
  safety.start_occupied = input.start_occupied;
  safety.end_prohibited = input.end_prohibited;
  safety.prohibited_cells = input.prohibited_cells;
  safety.occupied_cells = input.occupied_cells;
  safety.start_clearance_m = input.start_clearance_m;
  safety.end_clearance_m = input.end_clearance_m;

  if (!input.safety_check_enabled) {
    safety.allowed = true;
    safety.reason = TargetSegmentSafetyReason::kSafetyDisabled;
    return safety;
  }
  if (!input.grid_available) {
    safety.allowed = true;
    safety.reason = TargetSegmentSafetyReason::kNoGrid;
    return safety;
  }
  if (!input.start_cell_valid || !input.end_cell_valid) {
    safety.allowed = false;
    safety.reason = TargetSegmentSafetyReason::kOutsideGrid;
    return safety;
  }
  if (input.occupied_cells > 0U) {
    safety.allowed = false;
    safety.reason = TargetSegmentSafetyReason::kProhibited;
    return safety;
  }
  if (input.prohibited_cells == 0U) {
    safety.allowed = true;
    safety.reason = TargetSegmentSafetyReason::kAllowed;
    return safety;
  }

  const double required_improvement = std::max(0.0, input.min_clearance_improvement_m);
  const bool improves_clearance =
      std::isfinite(input.start_clearance_m) && std::isfinite(input.end_clearance_m) &&
      input.end_clearance_m >= input.start_clearance_m + required_improvement;
  const bool can_start_escape =
      (input.start_prohibited || input.clearance_stop_requested) &&
      !input.start_occupied;

  safety.escape = input.allow_escape && can_start_escape && improves_clearance;
  safety.allowed = safety.escape;
  safety.reason = safety.escape ? TargetSegmentSafetyReason::kEscape
                                : TargetSegmentSafetyReason::kProhibited;
  return safety;
}

bool clearanceEscapeRequested(const ClearanceEscapeRequestInput& input) noexcept {
  if (!input.enabled || input.hold_position || !(input.escape_step_m > 0.0)) {
    return false;
  }

  if (input.current_position_in_inflated_safety_cell) {
    return true;
  }

  if (input.speed_limit_reason == SpeedLimitReason::kClearance) {
    return true;
  }

  return input.speed_limit_reason == SpeedLimitReason::kTrackingOverspeed &&
         input.clearance_limit_mps <= kStoppedClearanceLimitMps;
}

bool escapeCommandStepAllowed(const TargetSegmentSafety& safety,
                              const double min_clearance_improvement_m) noexcept {
  if (safety.reason == TargetSegmentSafetyReason::kOutsideGrid ||
      safety.occupied_cells > 0U) {
    return false;
  }
  if (safety.reason == TargetSegmentSafetyReason::kSafetyDisabled ||
      safety.reason == TargetSegmentSafetyReason::kNoGrid) {
    return safety.allowed;
  }
  if (safety.reason == TargetSegmentSafetyReason::kAllowed &&
      safety.prohibited_cells == 0U) {
    return true;
  }
  if (!std::isfinite(safety.start_clearance_m) ||
      !std::isfinite(safety.end_clearance_m)) {
    return false;
  }

  const double required_improvement = std::max(0.0, min_clearance_improvement_m);
  return safety.end_clearance_m >= safety.start_clearance_m + required_improvement;
}

bool targetCommandAllowed(const TargetSegmentSafety& safety, const bool allow_escape,
                          const double min_clearance_improvement_m) noexcept {
  if (!allow_escape) {
    return safety.allowed;
  }
  if (safety.allowed) {
    return true;
  }
  return escapeCommandStepAllowed(safety, min_clearance_improvement_m);
}

const char*
targetSegmentSafetyReasonName(const TargetSegmentSafetyReason reason) noexcept {
  switch (reason) {
    case TargetSegmentSafetyReason::kAllowed:
      return "allowed";
    case TargetSegmentSafetyReason::kSafetyDisabled:
      return "safety_disabled";
    case TargetSegmentSafetyReason::kNoGrid:
      return "no_grid";
    case TargetSegmentSafetyReason::kOutsideGrid:
      return "outside_grid";
    case TargetSegmentSafetyReason::kProhibited:
      return "prohibited";
    case TargetSegmentSafetyReason::kEscape:
      return "escape";
  }
  return "unknown";
}

} // namespace drone_city_nav
