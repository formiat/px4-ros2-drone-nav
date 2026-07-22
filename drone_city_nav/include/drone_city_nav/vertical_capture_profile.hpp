#pragma once

#include <limits>

namespace drone_city_nav {

struct VerticalCaptureProfileConfig {
  double max_climb_speed_mps{4.0};
  double max_descent_speed_mps{4.0};
  double max_accel_mps2{3.5};
  double max_jerk_mps3{10.0};
};

struct VerticalCaptureProfileState {
  bool initialized{false};
  double commanded_z_m{std::numeric_limits<double>::quiet_NaN()};
  double commanded_vz_mps{0.0};
  double commanded_accel_mps2{0.0};
};

struct VerticalCaptureProfileStep {
  bool valid{false};
  bool target_reached{false};
  VerticalCaptureProfileState state{};
};

[[nodiscard]] VerticalCaptureProfileStep
advanceVerticalCaptureProfile(const VerticalCaptureProfileState& previous,
                              double actual_z_m, double actual_vz_mps,
                              bool actual_vz_valid, double target_z_m, double dt_s,
                              const VerticalCaptureProfileConfig& config) noexcept;

} // namespace drone_city_nav
