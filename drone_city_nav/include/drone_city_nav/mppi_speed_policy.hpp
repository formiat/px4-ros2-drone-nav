#pragma once

#include "drone_city_nav/executed_horizon_clearance_3d.hpp"
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
  kRouteClearance,
  kUnobservedFrontier,
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
  // Tracking-error tube law applied to live evidence: the tracking error the
  // controller can accumulate within its response time must fit inside the
  // body clearance of the motion under execution. The floor keeps a tight
  // spot leavable; the body validation, not the speed policy, is the hard
  // authority there. Both mirror the route tube configuration.
  double clearance_response_time_s{0.15};
  double clearance_minimum_progress_speed_mps{1.0};
  // The reference speed may rise no faster than this. Every limiter can still
  // cut the reference immediately — a cap is always allowed to bite at once —
  // but a limit that lifts as the horizon shifts must not snap the reference
  // back up, because the controller answers each step with a fresh burst of
  // acceleration. The vehicle's own horizontal acceleration is the honest
  // rate: the reference climbs no faster than the airframe can follow it.
  double reference_speed_rise_mps2{4.0};
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
  // Where the motion the vehicle executes right now comes close to known
  // occupied evidence: how far along it each such point lies, and the body
  // clearance there. The tube law applies at each point, and the stopping law
  // decides what the vehicle may carry on the way to it. It also says how far
  // along that motion the evidence reaches: past the first unobserved sample
  // the sensor-braking contract's guaranteed range no longer applies, and the
  // same contract is read with the observed range instead.
  std::optional<ExecutedHorizonClearance3D> executed_horizon_clearance;
  // Route length from the vehicle's projection to the first route segment
  // whose body envelope reaches unobserved space, probed as far as the latest
  // scan is checked along the route. The executed horizon ends where the
  // vehicle can rest, so its own frontier is never beyond the stopping path;
  // the route says where the evidence ends ahead of that.
  std::optional<double> route_observed_range_m;
  // Where the route ahead comes close to known occupied evidence, in the same
  // form as the executed horizon. The horizon reaches only as far as the
  // vehicle can stop, so at low speed it cannot see the tight spot the route
  // enters next; the route's own clearance does not move with the speed, so
  // the same tube and stopping laws read off it bound the reference before the
  // vehicle accelerates into the constraint.
  std::optional<ExecutedHorizonClearance3D> route_clearance;
  // Reference speed the previous cycle published, and how long ago, for the
  // rise limit. Absent on the first cycle, which then starts unconstrained.
  std::optional<double> previous_reference_speed_mps;
  double elapsed_since_previous_reference_s{0.0};
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
  double route_clearance_limit_mps{std::numeric_limits<double>::infinity()};
  double unobserved_frontier_limit_mps{std::numeric_limits<double>::infinity()};
  // The observed range the frontier limiter read the contract with.
  double unobserved_frontier_range_m{std::numeric_limits<double>::infinity()};
  // The reference before the rise limit, so diagnostics show when the limit is
  // what is holding the vehicle back.
  double unslewed_reference_speed_mps{0.0};
  bool reference_speed_rise_limited{false};
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
