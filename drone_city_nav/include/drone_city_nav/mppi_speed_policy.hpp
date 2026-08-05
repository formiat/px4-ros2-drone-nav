#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace drone_city_nav {

enum class MppiSpeedLimiter : std::uint8_t {
  kCruise,
  kAbsolute,
  kCurvature,
  kObservation,
  kGoal,
  kRouteConstraint,
};

struct MppiSpeedPolicyConfig {
  double cruise_speed_mps{20.0};
  double absolute_speed_limit_mps{20.0};
  double maximum_lateral_acceleration_mps2{5.0};
  double maximum_braking_acceleration_mps2{8.0};
  double reaction_latency_s{0.10};
  double observation_distance_m{30.0};
  double observation_margin_m{2.0};
  double goal_margin_m{2.0};
  double curvature_preview_distance_m{100.0};
  double curvature_measurement_window_m{5.0};
  double horizon_duration_s{6.0};
  double minimum_target_lookahead_m{30.0};
  double maximum_target_lookahead_m{100.0};
};

struct MppiSpeedPolicyInput {
  mppi::State state{};
  Point3 mission_goal{};
  std::span<const Point2> guide;
  std::optional<double> route_constraint_speed_limit_mps;
  bool terminal_goal_limit_enabled{true};
};

struct MppiSpeedPolicyResult {
  bool enabled{false};
  double reference_speed_mps{0.0};
  double cruise_limit_mps{0.0};
  double absolute_limit_mps{0.0};
  double curvature_limit_mps{std::numeric_limits<double>::infinity()};
  double observation_limit_mps{std::numeric_limits<double>::infinity()};
  double goal_limit_mps{std::numeric_limits<double>::infinity()};
  double route_constraint_limit_mps{std::numeric_limits<double>::infinity()};
  double maximum_preview_curvature_1pm{0.0};
  double target_lookahead_m{0.0};
  MppiSpeedLimiter active_limiter{MppiSpeedLimiter::kGoal};
  bool terminal_goal_limit_enabled{true};
};

[[nodiscard]] double stoppingLimitedSpeed(double available_distance_m,
                                          double terminal_speed_mps,
                                          double reaction_latency_s,
                                          double braking_acceleration_mps2) noexcept;

[[nodiscard]] MppiSpeedPolicyResult
evaluateMppiSpeedPolicy(const MppiSpeedPolicyConfig& config,
                        const MppiSpeedPolicyInput& input);

[[nodiscard]] const char* mppiSpeedLimiterName(MppiSpeedLimiter limiter) noexcept;

} // namespace drone_city_nav
