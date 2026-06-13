#pragma once

#include <limits>

namespace drone_city_nav {

enum class SpeedLimitReason {
  kHold,
  kInvalidInput,
  kCruise,
  kAcceleration,
  kGoal,
  kTurn,
  kClearance,
  kHardStepCap,
};

struct SpeedControllerConfig {
  double desired_speed_mps{3.0};
  double max_accel_mps2{2.0};
  double min_command_speed_mps{0.0};
  double goal_slowdown_radius_m{10.0};
  double braking_safety_margin_m{1.0};
  double turn_slowdown_angle_rad{0.7};
  double turn_slowdown_min_speed_mps{1.5};
  double narrow_clearance_slowdown_radius_m{7.0};
  double narrow_clearance_min_speed_mps{1.0};
  double max_commanded_target_step_m{0.5};
};

struct SpeedLimitBreakdown {
  double goal_limit_mps{std::numeric_limits<double>::infinity()};
  double turn_limit_mps{std::numeric_limits<double>::infinity()};
  double clearance_limit_mps{std::numeric_limits<double>::infinity()};
  double step_cap_limit_mps{std::numeric_limits<double>::infinity()};
};

struct SpeedControllerInput {
  bool hold_position{true};
  double controller_dt_s{0.1};
  double distance_to_goal_m{std::numeric_limits<double>::infinity()};
  double turn_angle_rad{0.0};
  double local_clearance_m{std::numeric_limits<double>::infinity()};
  double actual_speed_mps{0.0};
};

struct SpeedControllerOutput {
  double requested_speed_mps{0.0};
  double allowed_speed_mps{0.0};
  double target_step_m{0.0};
  double braking_distance_m{0.0};
  SpeedLimitBreakdown limits{};
  SpeedLimitReason limit_reason{SpeedLimitReason::kHold};
};

[[nodiscard]] const char* speedLimitReasonName(SpeedLimitReason reason) noexcept;

[[nodiscard]] double brakingLimitedSpeedMps(const SpeedControllerConfig& config,
                                            double distance_to_goal_m) noexcept;

[[nodiscard]] double turnLimitedSpeedMps(const SpeedControllerConfig& config,
                                         double turn_angle_rad) noexcept;

[[nodiscard]] double clearanceLimitedSpeedMps(const SpeedControllerConfig& config,
                                              double local_clearance_m) noexcept;

[[nodiscard]] double advanceToward(double current, double target,
                                   double max_delta) noexcept;

class OffboardSpeedController {
public:
  explicit OffboardSpeedController(
      const SpeedControllerConfig& config = SpeedControllerConfig{});

  void setConfig(const SpeedControllerConfig& config);
  [[nodiscard]] const SpeedControllerConfig& config() const noexcept;

  void reset() noexcept;
  [[nodiscard]] SpeedControllerOutput update(const SpeedControllerInput& input);

private:
  [[nodiscard]] bool configIsUsable() const noexcept;

  SpeedControllerConfig config_{};
  double previous_requested_speed_mps_{0.0};
};

} // namespace drone_city_nav
