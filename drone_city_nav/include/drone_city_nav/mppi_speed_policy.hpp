#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"
#include "drone_city_nav/sensor_braking_contract_3d.hpp"
#include "drone_city_nav/stopping_capability.hpp"
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
  kSensorBraking,
  kGoal,
  kRouteEndpoint,
  kRouteConstraint,
  kBlockedRoute,
  kClearance,
};

struct MppiSpeedPolicyConfig {
  double cruise_speed_mps{5.0};
  double absolute_speed_limit_mps{10.0};
  double maximum_lateral_acceleration_mps2{4.0};
  StoppingCapability stopping_capability{};
  SensorBrakingContract3D sensor_braking_contract{};
  double goal_margin_m{2.0};
  double curvature_preview_distance_m{60.0};
  double curvature_measurement_window_m{5.0};
  double horizon_duration_s{6.0};
  double minimum_target_lookahead_m{30.0};
  double maximum_target_lookahead_m{100.0};
  // Floor of the clearance limit so a tight spot stays leavable: the body
  // validation, not the speed policy, is the hard authority there.
  double clearance_minimum_progress_speed_mps{1.0};
};

struct MppiSpeedPolicyInput {
  mppi::State state{};
  Point3 mission_goal{};
  std::span<const RouteSample3D> route;
  std::optional<double> route_endpoint_remaining_m;
  std::optional<double> route_constraint_speed_limit_mps;
  // Route distance to the first sample the raw world blocks, while the route
  // is still followed and a replacement is searched: the vehicle must be able
  // to stop before it.
  std::optional<double> blocked_route_remaining_m;
  // Body clearance to known occupied evidence along the motion the vehicle
  // executes right now: the vehicle must be able to stop within it.
  std::optional<double> executed_horizon_clearance_m;
  RouteEndpointSemantics3D route_endpoint_semantics{
      RouteEndpointSemantics3D::kContinuation};
  bool terminal_goal_limit_enabled{true};
};

struct MppiSpeedPolicyResult {
  bool enabled{false};
  double reference_speed_mps{0.0};
  double cruise_limit_mps{0.0};
  double absolute_limit_mps{0.0};
  double curvature_limit_mps{std::numeric_limits<double>::infinity()};
  double sensor_braking_limit_mps{0.0};
  double goal_limit_mps{std::numeric_limits<double>::infinity()};
  double route_endpoint_limit_mps{std::numeric_limits<double>::infinity()};
  double route_constraint_limit_mps{std::numeric_limits<double>::infinity()};
  double blocked_route_limit_mps{std::numeric_limits<double>::infinity()};
  double clearance_limit_mps{std::numeric_limits<double>::infinity()};
  double maximum_preview_curvature_1pm{0.0};
  double target_lookahead_m{0.0};
  SensorBrakingAssessment3D sensor_braking_assessment{};
  MppiSpeedLimiter active_limiter{MppiSpeedLimiter::kGoal};
  RouteEndpointSemantics3D route_endpoint_semantics{
      RouteEndpointSemantics3D::kContinuation};
  bool route_endpoint_stop_required{false};
  bool terminal_goal_limit_enabled{true};
};

[[nodiscard]] double
stoppingLimitedSpeed(double available_distance_m, double terminal_speed_mps,
                     const StoppingCapability& capability) noexcept;

[[nodiscard]] MppiSpeedPolicyResult
evaluateMppiSpeedPolicy(const MppiSpeedPolicyConfig& config,
                        const MppiSpeedPolicyInput& input);

[[nodiscard]] const char* mppiSpeedLimiterName(MppiSpeedLimiter limiter) noexcept;

} // namespace drone_city_nav
