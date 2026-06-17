#pragma once

#include "drone_city_nav/offboard_speed_controller.hpp"

#include <cstddef>
#include <limits>

namespace drone_city_nav {

enum class TargetSegmentSafetyReason {
  kAllowed,
  kSafetyDisabled,
  kNoGrid,
  kOutsideGrid,
  kProhibited,
  kEscape,
};

struct TargetSegmentSafety {
  bool grid_available{false};
  bool allowed{true};
  bool escape{false};
  bool start_prohibited{false};
  bool start_occupied{false};
  bool end_prohibited{false};
  std::size_t prohibited_cells{0U};
  std::size_t occupied_cells{0U};
  double start_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double end_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  TargetSegmentSafetyReason reason{TargetSegmentSafetyReason::kAllowed};
};

struct TargetSegmentSafetyInput {
  bool safety_check_enabled{true};
  bool grid_available{true};
  bool start_cell_valid{true};
  bool end_cell_valid{true};
  bool start_prohibited{false};
  bool start_occupied{false};
  bool end_prohibited{false};
  std::size_t prohibited_cells{0U};
  std::size_t occupied_cells{0U};
  double start_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double end_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double min_clearance_improvement_m{0.0};
  bool allow_escape{false};
  bool clearance_stop_requested{false};
};

struct ClearanceEscapeRequestInput {
  bool enabled{true};
  bool hold_position{true};
  double escape_step_m{0.0};
  bool current_position_in_inflated_safety_cell{false};
  SpeedLimitReason speed_limit_reason{SpeedLimitReason::kHold};
  double clearance_limit_mps{std::numeric_limits<double>::infinity()};
};

[[nodiscard]] TargetSegmentSafety
evaluateTargetSegmentSafetyPolicy(const TargetSegmentSafetyInput& input) noexcept;

[[nodiscard]] bool
clearanceEscapeRequested(const ClearanceEscapeRequestInput& input) noexcept;

[[nodiscard]] bool
escapeCommandStepAllowed(const TargetSegmentSafety& safety,
                         double min_clearance_improvement_m) noexcept;

[[nodiscard]] bool targetCommandAllowed(const TargetSegmentSafety& safety,
                                        bool allow_escape,
                                        double min_clearance_improvement_m) noexcept;

[[nodiscard]] const char*
targetSegmentSafetyReasonName(TargetSegmentSafetyReason reason) noexcept;

} // namespace drone_city_nav
