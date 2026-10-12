#pragma once

#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <utility>

namespace drone_city_nav {

// The vehicle's main battery as time left in the air (roadmap item 27 block 1,
// specification K32), weighed against the way home by the rule of the
// carried light's battery (K11). The autopilot's own estimate of the time
// left is taken when it has one (a battery with its current measured and its
// capacity set); without one the time left is the charge left over the rate
// the charge fell over the last thirty seconds, none while it does not fall.
// PX4's simulated battery reports neither current nor time left (both
// invalid on every flight of 2026-10-11) and drained from full to its
// floor in its first minute, so the simulation flies it full.
class MainBatteryEstimate final {
public:
  static constexpr double kWindowS{30.0};
  static constexpr double kMinimumSpanS{10.0};

  void observe(const double stamp_s, const double remaining,
               const double autopilot_time_left_s) {
    if (std::isfinite(autopilot_time_left_s) && autopilot_time_left_s >= 0.0) {
      autopilot_time_left_s_ = autopilot_time_left_s;
    } else {
      autopilot_time_left_s_.reset();
    }
    if (!std::isfinite(stamp_s) || !std::isfinite(remaining) || remaining < 0.0) {
      return;
    }
    samples_.emplace_back(stamp_s, remaining);
    while (!samples_.empty() && samples_.front().first < stamp_s - kWindowS) {
      samples_.pop_front();
    }
  }

  // Seconds left in the air, infinite when nothing says the charge falls.
  [[nodiscard]] double timeLeftS() const noexcept {
    if (autopilot_time_left_s_.has_value()) {
      return *autopilot_time_left_s_;
    }
    if (samples_.size() < 2U) {
      return std::numeric_limits<double>::infinity();
    }
    const auto& [first_s, first_remaining] = samples_.front();
    const auto& [last_s, last_remaining] = samples_.back();
    const double span_s = last_s - first_s;
    const double fallen = first_remaining - last_remaining;
    if (span_s < kMinimumSpanS || !(fallen > 0.0)) {
      return std::numeric_limits<double>::infinity();
    }
    return last_remaining / (fallen / span_s);
  }

private:
  std::deque<std::pair<double, double>> samples_;
  std::optional<double> autopilot_time_left_s_;
};

} // namespace drone_city_nav
