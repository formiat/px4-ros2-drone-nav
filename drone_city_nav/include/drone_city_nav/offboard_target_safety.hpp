#pragma once

#include <cstddef>
#include <limits>

namespace drone_city_nav {

enum class TargetSegmentSafetyReason {
  kAllowed,
  kSafetyDisabled,
  kNoGrid,
  kOutsideGrid,
  kOccupied,
  kBlocked,
  kEscape,
};

struct TargetSegmentSafety {
  bool grid_available{false};
  bool allowed{true};
  bool escape{false};
  bool start_blocked{false};
  bool start_occupied{false};
  bool end_blocked{false};
  std::size_t blocked_cells{0U};
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
  bool start_blocked{false};
  bool start_occupied{false};
  bool end_blocked{false};
  std::size_t blocked_cells{0U};
  std::size_t occupied_cells{0U};
  double start_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double end_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double min_clearance_improvement_m{0.0};
  bool allow_escape{false};
  bool clearance_stop_requested{false};
};

[[nodiscard]] TargetSegmentSafety
evaluateTargetSegmentSafetyPolicy(const TargetSegmentSafetyInput& input) noexcept;

[[nodiscard]] bool
escapeCommandStepAllowed(const TargetSegmentSafety& safety,
                         double min_clearance_improvement_m) noexcept;

[[nodiscard]] const char*
targetSegmentSafetyReasonName(TargetSegmentSafetyReason reason) noexcept;

} // namespace drone_city_nav
