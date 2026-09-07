#include "drone_city_nav/mppi/mppi_reference.hpp"

#include "drone_city_nav/derived_clearance_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/mppi/mppi_altitude_envelope.hpp"
#include "drone_city_nav/mppi/mppi_clearance_cost.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] float clampMagnitude(const float value, const float limit) noexcept {
  return std::clamp(value, -limit, limit);
}

void clampHorizontal(float& x, float& y, const float limit) noexcept {
  const float magnitude = std::hypot(x, y);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

[[nodiscard]] SweptFootprintConfig sweptConfig(const FootprintConfig& footprint,
                                               const float sweep_step_m,
                                               const float safe_clearance_m) noexcept {
  return SweptFootprintConfig{
      .radius_m = footprint.radius_m,
      .lower_extent_m = footprint.lower_extent_m,
      .upper_extent_m = footprint.upper_extent_m,
      .perimeter_samples = footprint.perimeter_samples,
      .radial_rings = footprint.radial_rings,
      .axial_samples = footprint.axial_samples,
      .sweep_step_m = sweep_step_m,
      .safe_clearance_threshold_m = safe_clearance_m,
  };
}

[[nodiscard]] float squared(const float value) noexcept {
  return value * value;
}

[[nodiscard]] float targetDistance(const State& state,
                                   const MovingTargetReference& target,
                                   const float elapsed_s) noexcept {
  const float target_x = target.state.x + target.state.vx * elapsed_s;
  const float target_y = target.state.y + target.state.vy * elapsed_s;
  const float target_z = movingTargetAltitudeAt(target, elapsed_s);
  return std::hypot(std::hypot(target_x - state.x, target_y - state.y),
                    target_z - state.z);
}

[[nodiscard]] float aircraftDistance(const State& state,
                                     const DynamicAircraftSample& aircraft) noexcept {
  return std::hypot(std::hypot(aircraft.x - state.x, aircraft.y - state.y),
                    aircraft.z - state.z);
}

[[nodiscard]] Control
preferredCooperativeAccelerationImpl(const CooperativeManeuverPreference& preference,
                                     const DynamicsConfig& dynamics,
                                     const CooperativeConfig& cooperative) noexcept {
  const float norm =
      std::hypot(std::hypot(preference.direction_x, preference.direction_y),
                 preference.direction_z);
  if (!(norm > 1.0e-5F)) {
    return {};
  }
  const float horizontal_norm =
      std::hypot(preference.direction_x, preference.direction_y);
  const float horizontal_magnitude = cooperative.candidate_acceleration_fraction *
                                     dynamics.maximum_horizontal_acceleration_mps2;
  const float vertical_magnitude = cooperative.candidate_acceleration_fraction *
                                   dynamics.maximum_vertical_acceleration_mps2;
  return Control{
      .ax = horizontal_norm > 1.0e-5F
                ? horizontal_magnitude * preference.direction_x / horizontal_norm
                : 0.0F,
      .ay = horizontal_norm > 1.0e-5F
                ? horizontal_magnitude * preference.direction_y / horizontal_norm
                : 0.0F,
      .az = vertical_magnitude * preference.direction_z / norm,
  };
}

} // namespace

Control
resolveCooperativePreferredAcceleration(const CooperativeManeuverPreference& preference,
                                        const DynamicsConfig& dynamics,
                                        const CooperativeConfig& cooperative) noexcept {
  return preferredCooperativeAccelerationImpl(preference, dynamics, cooperative);
}

bool benchmarkConfigIsValid(const BenchmarkConfig& config) noexcept {
  return !config.scenario.empty() && config.rollouts > 0U && config.steps >= 2U &&
         config.measured_ticks > 0U && std::isfinite(config.deadline_ms) &&
         config.deadline_ms > 0.0 && std::isfinite(config.dynamics.dt_s) &&
         config.dynamics.dt_s > 0.0F && std::isfinite(config.costs.temperature) &&
         config.costs.temperature > 0.0F &&
         std::isfinite(config.costs.target_effective_sample_fraction) &&
         config.costs.target_effective_sample_fraction >= 0.0F &&
         config.costs.target_effective_sample_fraction <= 1.0F &&
         std::isfinite(config.costs.maximum_temperature_growth) &&
         config.costs.maximum_temperature_growth >= 1.0F &&
         std::isfinite(config.costs.route_directed_candidate_cost_tolerance) &&
         config.costs.route_directed_candidate_cost_tolerance >= 0.0F &&
         config.costs.route_directed_candidate_switch_ticks > 0U &&
         std::isfinite(config.costs.head_progress_horizon_s) &&
         config.costs.head_progress_horizon_s > 0.0F &&
         std::isfinite(config.costs.head_progress_weight) &&
         config.costs.head_progress_weight >= 0.0F &&
         std::isfinite(config.costs.route_progress_integral_weight) &&
         config.costs.route_progress_integral_weight >= 0.0F &&
         std::isfinite(config.costs.speed_tracking_weight) &&
         config.costs.speed_tracking_weight >= 0.0F &&
         std::isfinite(config.costs.overspeed_weight) &&
         config.costs.overspeed_weight >= 0.0F &&
         std::isfinite(config.costs.planning_exposure_weight) &&
         config.costs.planning_exposure_weight >= 0.0F &&
         std::isfinite(config.costs.obstacle_approach_weight) &&
         config.costs.obstacle_approach_weight >= 0.0F &&
         std::isfinite(config.costs.peer_separation_weight) &&
         config.costs.peer_separation_weight >= 0.0F &&
         std::isfinite(config.costs.cooperative_maneuver_preference_weight) &&
         config.costs.cooperative_maneuver_preference_weight >= 0.0F &&
         std::isfinite(config.altitude_envelope.minimum_z_m) &&
         std::isfinite(config.altitude_envelope.maximum_z_m) &&
         config.altitude_envelope.maximum_z_m > config.altitude_envelope.minimum_z_m &&
         std::isfinite(
             config.altitude_envelope.guaranteed_vertical_deceleration_mps2) &&
         config.altitude_envelope.guaranteed_vertical_deceleration_mps2 > 0.0F &&
         config.altitude_envelope.guaranteed_vertical_deceleration_mps2 <=
             config.dynamics.maximum_vertical_acceleration_mps2 &&
         stoppingCapabilityIsValid(config.stopping_capability) &&
         config.stopping_capability.maximum_commanded_horizontal_deceleration_mps2 <=
             config.dynamics.maximum_horizontal_acceleration_mps2 &&
         config.stopping_capability.guaranteed_vertical_deceleration_mps2 <=
             config.dynamics.maximum_vertical_acceleration_mps2 &&
         std::isfinite(config.dynamics.maximum_translational_speed_mps) &&
         config.dynamics.maximum_translational_speed_mps > 0.0F &&
         std::isfinite(config.altitude_envelope.reaction_latency_s) &&
         config.altitude_envelope.reaction_latency_s >= 0.0F &&
         std::isfinite(config.cooperative.desired_minimum_separation_m) &&
         config.cooperative.desired_minimum_separation_m > 0.0F &&
         std::isfinite(config.cooperative.candidate_acceleration_fraction) &&
         config.cooperative.candidate_acceleration_fraction > 0.0F &&
         config.cooperative.candidate_acceleration_fraction <= 1.0F &&
         std::isfinite(config.cooperative.candidate_duration_s) &&
         config.cooperative.candidate_duration_s > 0.0F &&
         horizonSamplingConfigIsValid(config.horizon_sampling) &&
         config.footprint.radius_m >= 0.0F && config.footprint.lower_extent_m >= 0.0F &&
         config.footprint.upper_extent_m >= 0.0F &&
         (config.footprint.radius_m == 0.0F ||
          (config.footprint.perimeter_samples > 0U &&
           config.footprint.radial_rings > 0U &&
           config.footprint.axial_samples >= 2U)) &&
         config.risk.critical_distance_m > 0.0F &&
         config.risk.preferred_distance_m >= config.risk.critical_distance_m &&
         config.risk.obstacle_approach_response_time_s >= 0.0F &&
         config.risk.obstacle_approach_deceleration_mps2 > 0.0F;
}

MppiProgressDiagnostics resolveUnroutedProgressDiagnostics(
    const RolloutMetrics& metrics, const bool moving_target_enabled,
    const float fixed_target_head_progress_m,
    const float fixed_target_terminal_progress_m) noexcept {
  if (moving_target_enabled) {
    return MppiProgressDiagnostics{
        .head_progress_m = metrics.costs.head_progress,
        .terminal_progress_m = -metrics.costs.progress,
    };
  }
  return MppiProgressDiagnostics{
      .head_progress_m = fixed_target_head_progress_m,
      .terminal_progress_m = fixed_target_terminal_progress_m,
  };
}

State integrateReference(const State& state, const Control& control,
                         const DynamicsConfig& config) noexcept {
  return integrateMotionState3D(state, control, config);
}

Control equivalentControlFromMeasuredAcceleration(
    const State& state, const float measured_ax_mps2, const float measured_ay_mps2,
    const float measured_az_mps2, const DynamicsConfig& config) noexcept {
  Control control{
      .ax = measured_ax_mps2 + config.linear_drag_1ps * state.vx,
      .ay = measured_ay_mps2 + config.linear_drag_1ps * state.vy,
      .az = measured_az_mps2 + config.linear_drag_1ps * state.vz,
  };
  clampHorizontal(control.ax, control.ay, config.maximum_horizontal_acceleration_mps2);
  control.az = clampMagnitude(control.az, config.maximum_vertical_acceleration_mps2);
  return control;
}

RolloutMetrics simulateReference(
    const State& initial_state, const std::span<const Control> nominal_controls,
    const std::span<const Control> noise_controls, const DynamicsConfig& dynamics,
    const RiskConfig& risk, const CostConfig& costs, const EsdfGrid& grid,
    const std::span<const float> esdf, const float target_x_m, const float target_y_m,
    const bool early_exit_on_altitude_envelope_violation,
    const Control previous_applied_control, const float reference_speed_mps,
    const FootprintConfig& footprint,
    const std::optional<MovingTargetReference> moving_target,
    ReferenceSimulationTrace* const trace,
    const std::span<const DynamicAircraftTrajectory> dynamic_aircraft,
    const std::optional<CooperativeManeuverPreference> cooperative_maneuver,
    const CooperativeConfig& cooperative,
    const std::optional<DynamicAircraftCostPolicy> dynamic_aircraft_cost_policy,
    const AltitudeEnvelopeConfig altitude_envelope,
    const std::optional<float> fixed_target_z_m) {
  for (const DynamicAircraftTrajectory& aircraft : dynamic_aircraft) {
    if (!aircraft.samples || aircraft.samples->size() < nominal_controls.size() ||
        aircraft.active_steps == 0U ||
        aircraft.active_steps > nominal_controls.size() ||
        !(aircraft.footprint_radius_m >= 0.0F)) {
      throw std::invalid_argument{"invalid dynamic aircraft trajectory"};
    }
  }
  const DynamicAircraftCostPolicy aircraft_cost_policy =
      dynamic_aircraft_cost_policy.value_or(DynamicAircraftCostPolicy{
          .strong_separation_m = cooperative.desired_minimum_separation_m,
          .anticipation_separation_m = cooperative.desired_minimum_separation_m,
          .strong_weight = costs.peer_separation_weight,
      });
  if (!(aircraft_cost_policy.strong_separation_m > 0.0F) ||
      aircraft_cost_policy.anticipation_separation_m <
          aircraft_cost_policy.strong_separation_m ||
      !(aircraft_cost_policy.strong_weight >= 0.0F) ||
      !(aircraft_cost_policy.anticipation_weight >= 0.0F) ||
      !(aircraft_cost_policy.time_to_collision_gain_s >= 0.0F) ||
      aircraft_cost_policy.maximum_time_to_collision_multiplier < 1.0F) {
    throw std::invalid_argument{"invalid dynamic aircraft cost policy"};
  }
  RolloutMetrics metrics{};
  metrics.altitude_envelope_violation = !altitudeEnvelopeDynamicallyRecoverable(
      initial_state, previous_applied_control, dynamics, altitude_envelope);
  metrics.minimum_clearance_m = std::numeric_limits<float>::infinity();
  metrics.minimum_peer_separation_m = std::numeric_limits<float>::infinity();
  State state = initial_state;
  if (trace != nullptr) {
    trace->horizon.clear();
    trace->horizon.reserve(nominal_controls.size() + 1U);
    trace->horizon.push_back(state);
  }
  Control previous = previous_applied_control;
  const float target_z_m = fixed_target_z_m.value_or(initial_state.z);
  const auto fixedTargetDistance = [&](const State& sample) noexcept {
    return std::hypot(std::hypot(target_x_m - sample.x, target_y_m - sample.y),
                      target_z_m - sample.z);
  };
  const float initial_target_distance =
      moving_target.has_value() ? targetDistance(state, *moving_target, 0.0F)
                                : fixedTargetDistance(state);
  metrics.minimum_target_separation_m = initial_target_distance;
  const std::size_t head_steps =
      std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(
                                  costs.head_progress_horizon_s / dynamics.dt_s)),
                              1U, nominal_controls.size());
  const Control preferred_acceleration =
      cooperative_maneuver.has_value()
          ? resolveCooperativePreferredAcceleration(*cooperative_maneuver, dynamics,
                                                    cooperative)
          : Control{};
  const std::size_t cooperative_preference_steps =
      std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(
                                  cooperative.candidate_duration_s / dynamics.dt_s)),
                              1U, nominal_controls.size());
  std::size_t simulated_steps = 0U;
  for (std::size_t step = 0U; step < nominal_controls.size(); ++step) {
    Control control{
        .ax = nominal_controls[step].ax + noise_controls[step].ax,
        .ay = nominal_controls[step].ay + noise_controls[step].ay,
        .az = nominal_controls[step].az + noise_controls[step].az,
        .yaw_accel = nominal_controls[step].yaw_accel + noise_controls[step].yaw_accel,
    };
    const State previous_state = state;
    state = integrateReference(state, control, dynamics);
    if (trace != nullptr) {
      trace->horizon.push_back(state);
    }
    simulated_steps = step + 1U;
    metrics.altitude_envelope_violation =
        metrics.altitude_envelope_violation ||
        !altitudeEnvelopeDynamicallyRecoverable(state, control, dynamics,
                                                altitude_envelope);
    const float validation_step_m = std::max(0.05F, 0.5F * grid.resolution_m);
    const FootprintBodyAxis body_axis =
        bodyAxisFromWorldAcceleration(Vec3{control.ax, control.ay, control.az});
    const DerivedFootprintClearance3D footprint_clearance =
        querySweptFootprintClearance3D(
            grid, esdf, Point3{previous_state.x, previous_state.y, previous_state.z},
            body_axis, Point3{state.x, state.y, state.z}, body_axis,
            sweptConfig(footprint, validation_step_m,
                        footprint.clearance_broad_phase_enabled
                            ? risk.preferred_distance_m
                            : 0.0F));
    const bool known_clearance = footprint_clearance.evidence.known_clearance_observed;
    const float clearance =
        known_clearance
            ? static_cast<float>(footprint_clearance.evidence.minimum_known_clearance_m)
            : std::numeric_limits<float>::infinity();
    metrics.minimum_clearance_m = std::min(metrics.minimum_clearance_m, clearance);
    const float segment_speed_mps =
        std::hypot(std::hypot(state.vx, state.vy), state.vz);
    const float segment_m = dynamics.dt_s * segment_speed_mps;
    if (known_clearance && clearance < risk.critical_distance_m) {
      metrics.worst_tier = std::max(metrics.worst_tier, RiskTier::kCritical);
      metrics.critical_exposure_m += segment_m;
    } else if (known_clearance && clearance < risk.preferred_distance_m) {
      metrics.worst_tier = std::max(metrics.worst_tier, RiskTier::kPlanning);
      metrics.planning_exposure_m += segment_m;
    }
    metrics.costs.obstacle_approach_m2_s +=
        dynamics.dt_s * stoppingClearanceDeficitM2(
                            clearance, segment_speed_mps, risk.critical_distance_m,
                            risk.obstacle_approach_response_time_s,
                            risk.obstacle_approach_deceleration_mps2);

    const float target_distance =
        moving_target.has_value()
            ? targetDistance(state, *moving_target,
                             static_cast<float>(step + 1U) * dynamics.dt_s)
            : fixedTargetDistance(state);
    if (target_distance < metrics.minimum_target_separation_m) {
      metrics.minimum_target_separation_m = target_distance;
    }
    if (moving_target.has_value() && metrics.predicted_capture_time_s < 0.0F &&
        target_distance <= moving_target->capture_radius_m) {
      metrics.predicted_capture_time_s = static_cast<float>(step + 1U) * dynamics.dt_s;
    }
    for (const DynamicAircraftTrajectory& aircraft : dynamic_aircraft) {
      if (step >= aircraft.active_steps) {
        continue;
      }
      const float separation_m = aircraftDistance(state, (*aircraft.samples)[step]);
      metrics.minimum_peer_separation_m =
          std::min(metrics.minimum_peer_separation_m, separation_m);
      const float desired_separation_m =
          std::max(aircraft_cost_policy.strong_separation_m,
                   footprint.radius_m + aircraft.footprint_radius_m);
      const float shortfall_m = std::max(0.0F, desired_separation_m - separation_m);
      metrics.costs.peer_separation += squared(shortfall_m);
      DynamicAircraftCostPolicy effective_policy = aircraft_cost_policy;
      effective_policy.strong_separation_m = desired_separation_m;
      effective_policy.anticipation_separation_m =
          std::max(effective_policy.anticipation_separation_m, desired_separation_m);
      const DynamicAircraftCostContribution contribution =
          dynamicAircraftCostContribution(separation_m,
                                          static_cast<float>(step + 1U) * dynamics.dt_s,
                                          effective_policy);
      metrics.costs.dynamic_aircraft_anticipation += contribution.anticipation;
      metrics.costs.dynamic_aircraft_survival +=
          contribution.strong + contribution.anticipation;
    }
    if (cooperative_maneuver.has_value() && step < cooperative_preference_steps) {
      metrics.costs.maneuver_preference +=
          squared(control.ax - preferred_acceleration.ax) +
          squared(control.ay - preferred_acceleration.ay) +
          squared(control.az - preferred_acceleration.az);
    }
    if (step + 1U == head_steps) {
      metrics.costs.head_progress = initial_target_distance - target_distance;
    }
    metrics.costs.guide_deviation += squared(state.y - initial_state.y);
    metrics.costs.acceleration +=
        squared(control.ax) + squared(control.ay) + squared(control.az);
    metrics.costs.jerk += squared(control.ax - previous.ax) +
                          squared(control.ay - previous.ay) +
                          squared(control.az - previous.az);
    metrics.costs.yaw_change += squared(control.yaw_accel);
    metrics.costs.control_effort += squared(control.ax) + squared(control.ay) +
                                    squared(control.az) + squared(control.yaw_accel);
    if (reference_speed_mps >= 0.0F) {
      const float speed_error =
          std::hypot(std::hypot(state.vx, state.vy), state.vz) - reference_speed_mps;
      metrics.costs.speed_tracking += squared(speed_error);
      // The excess above the reference is shed like the excess above the
      // dynamics caps: the reference is the speed the policy can stop within.
      metrics.costs.overspeed += squared(std::max(0.0F, speed_error));
    }
    {
      const float horizontal_speed_mps = std::hypot(state.vx, state.vy);
      const float translational_speed_mps = std::hypot(horizontal_speed_mps, state.vz);
      const float excess_mps = std::max(
          0.0F,
          std::max(horizontal_speed_mps - dynamics.maximum_horizontal_speed_mps,
                   translational_speed_mps - dynamics.maximum_translational_speed_mps));
      metrics.costs.overspeed += squared(excess_mps);
    }
    metrics.costs.terminal = moving_target.has_value()
                                 ? std::max(0.0F, metrics.minimum_target_separation_m -
                                                      moving_target->capture_radius_m)
                                 : target_distance;
    previous = control;
    if (metrics.altitude_envelope_violation &&
        early_exit_on_altitude_envelope_violation) {
      break;
    }
  }
  if (trace != nullptr && simulated_steps < nominal_controls.size()) {
    State trace_state = state;
    for (std::size_t step = simulated_steps; step < nominal_controls.size(); ++step) {
      const Control control{
          .ax = nominal_controls[step].ax + noise_controls[step].ax,
          .ay = nominal_controls[step].ay + noise_controls[step].ay,
          .az = nominal_controls[step].az + noise_controls[step].az,
          .yaw_accel =
              nominal_controls[step].yaw_accel + noise_controls[step].yaw_accel,
      };
      trace_state = integrateReference(trace_state, control, dynamics);
      trace->horizon.push_back(trace_state);
    }
  }
  metrics.costs.progress =
      -(initial_target_distance - (moving_target.has_value()
                                       ? metrics.minimum_target_separation_m
                                       : fixedTargetDistance(state)));
  metrics.soft_cost =
      costs.head_progress_weight * -metrics.costs.head_progress +
      costs.progress_weight * metrics.costs.progress +
      costs.speed_tracking_weight * dynamics.dt_s * metrics.costs.speed_tracking +
      costs.overspeed_weight * dynamics.dt_s * metrics.costs.overspeed +
      costs.guide_deviation_weight * dynamics.dt_s * metrics.costs.guide_deviation +
      costs.acceleration_weight * dynamics.dt_s * metrics.costs.acceleration +
      costs.jerk_weight * metrics.costs.jerk +
      costs.yaw_change_weight * metrics.costs.yaw_change +
      costs.control_effort_weight * dynamics.dt_s * metrics.costs.control_effort +
      costs.planning_exposure_weight * metrics.planning_exposure_m +
      costs.obstacle_approach_weight * metrics.costs.obstacle_approach_m2_s +
      dynamics.dt_s * metrics.costs.dynamic_aircraft_survival +
      costs.cooperative_maneuver_preference_weight * dynamics.dt_s *
          metrics.costs.maneuver_preference +
      costs.terminal_weight * metrics.costs.terminal;
  metrics.terminal_state = state;
  return metrics;
}

} // namespace drone_city_nav::mppi
