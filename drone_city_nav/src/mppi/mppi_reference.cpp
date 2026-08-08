#include "drone_city_nav/mppi/mppi_reference.hpp"

#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

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

} // namespace

bool benchmarkConfigIsValid(const BenchmarkConfig& config) noexcept {
  return !config.scenario.empty() && config.rollouts > 0U && config.steps >= 2U &&
         config.measured_ticks > 0U && std::isfinite(config.deadline_ms) &&
         config.deadline_ms > 0.0 && std::isfinite(config.dynamics.dt_s) &&
         config.dynamics.dt_s > 0.0F && std::isfinite(config.costs.temperature) &&
         config.costs.temperature > 0.0F &&
         std::isfinite(config.costs.head_progress_horizon_s) &&
         config.costs.head_progress_horizon_s > 0.0F &&
         std::isfinite(config.costs.head_progress_weight) &&
         config.costs.head_progress_weight >= 0.0F &&
         std::isfinite(config.costs.speed_tracking_weight) &&
         config.costs.speed_tracking_weight >= 0.0F &&
         horizonSamplingConfigIsValid(config.horizon_sampling) &&
         config.footprint.radius_m >= 0.0F && config.footprint.lower_extent_m >= 0.0F &&
         config.footprint.upper_extent_m >= 0.0F &&
         (config.footprint.radius_m == 0.0F ||
          (config.footprint.perimeter_samples > 0U &&
           config.footprint.radial_rings > 0U &&
           config.footprint.axial_samples >= 2U)) &&
         config.risk.critical_distance_m > 0.0F &&
         config.risk.preferred_distance_m >= config.risk.critical_distance_m;
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

State integrateReference(State state, Control control,
                         const DynamicsConfig& config) noexcept {
  clampHorizontal(control.ax, control.ay, config.maximum_horizontal_acceleration_mps2);
  control.az = clampMagnitude(control.az, config.maximum_vertical_acceleration_mps2);
  control.yaw_accel =
      clampMagnitude(control.yaw_accel, config.maximum_yaw_acceleration_radps2);

  const float drag = std::max(0.0F, 1.0F - config.linear_drag_1ps * config.dt_s);
  state.vx = state.vx * drag + control.ax * config.dt_s;
  state.vy = state.vy * drag + control.ay * config.dt_s;
  state.vz = state.vz * drag + control.az * config.dt_s;
  clampHorizontal(state.vx, state.vy, config.maximum_horizontal_speed_mps);
  state.vz = clampMagnitude(state.vz, config.maximum_vertical_speed_mps);

  state.yaw_rate = clampMagnitude(state.yaw_rate + control.yaw_accel * config.dt_s,
                                  config.maximum_yaw_rate_radps);
  state.x += state.vx * config.dt_s;
  state.y += state.vy * config.dt_s;
  state.z += state.vz * config.dt_s;
  state.yaw = std::remainder(state.yaw + state.yaw_rate * config.dt_s,
                             2.0F * std::numbers::pi_v<float>);
  return state;
}

RolloutMetrics simulateReference(
    const State& initial_state, const std::span<const Control> nominal_controls,
    const std::span<const Control> noise_controls, const DynamicsConfig& dynamics,
    const RiskConfig& risk, const CostConfig& costs, const EsdfGrid& grid,
    const std::span<const float> esdf, const float target_x_m, const float target_y_m,
    const bool early_exit_on_collision, const Control previous_applied_control,
    const float reference_speed_mps, const FootprintConfig& footprint,
    const std::optional<MovingTargetReference> moving_target,
    ReferenceSimulationTrace* const trace) {
  RolloutMetrics metrics{};
  metrics.minimum_clearance_m = std::numeric_limits<float>::infinity();
  State state = initial_state;
  if (trace != nullptr) {
    trace->horizon.clear();
    trace->horizon.reserve(nominal_controls.size() + 1U);
    trace->horizon.push_back(state);
  }
  Control previous = previous_applied_control;
  const float initial_target_distance =
      moving_target.has_value()
          ? targetDistance(state, *moving_target, 0.0F)
          : std::hypot(target_x_m - state.x, target_y_m - state.y);
  metrics.minimum_target_separation_m = initial_target_distance;
  const std::size_t head_steps =
      std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(
                                  costs.head_progress_horizon_s / dynamics.dt_s)),
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
    const float validation_step_m = std::max(0.05F, 0.5F * grid.resolution_m);
    const FootprintBodyAxis body_axis =
        bodyAxisFromWorldAcceleration(Vec3{control.ax, control.ay, control.az});
    const SweptFootprintResult footprint_result = validateSweptFootprint(
        grid, esdf, Point3{previous_state.x, previous_state.y, previous_state.z},
        body_axis, Point3{state.x, state.y, state.z}, body_axis,
        sweptConfig(footprint, validation_step_m,
                    footprint.clearance_broad_phase_enabled ? risk.preferred_distance_m
                                                            : 0.0F));
    const bool raw_collision =
        footprint_result.status == SweptFootprintStatus::kRawCollision ||
        footprint_result.status == SweptFootprintStatus::kInvalidEsdf;
    const float clearance = static_cast<float>(footprint_result.minimum_clearance_m);
    metrics.minimum_clearance_m = std::min(metrics.minimum_clearance_m, clearance);
    const float segment_m = dynamics.dt_s * std::hypot(state.vx, state.vy);
    if (raw_collision) {
      metrics.collision = true;
      metrics.worst_tier = RiskTier::kCollision;
    } else if (clearance < risk.critical_distance_m) {
      metrics.worst_tier = std::max(metrics.worst_tier, RiskTier::kCritical);
      metrics.critical_exposure_m += segment_m;
    } else if (clearance < risk.preferred_distance_m) {
      metrics.worst_tier = std::max(metrics.worst_tier, RiskTier::kPlanning);
      metrics.planning_exposure_m += segment_m;
    }

    const float target_distance =
        moving_target.has_value()
            ? targetDistance(state, *moving_target,
                             static_cast<float>(step + 1U) * dynamics.dt_s)
            : std::hypot(target_x_m - state.x, target_y_m - state.y);
    if (target_distance < metrics.minimum_target_separation_m) {
      metrics.minimum_target_separation_m = target_distance;
    }
    if (moving_target.has_value() && metrics.predicted_capture_time_s < 0.0F &&
        target_distance <= moving_target->capture_radius_m) {
      metrics.predicted_capture_time_s = static_cast<float>(step + 1U) * dynamics.dt_s;
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
      metrics.costs.speed_tracking +=
          squared(std::hypot(state.vx, state.vy) - reference_speed_mps);
    }
    metrics.costs.terminal = moving_target.has_value()
                                 ? std::max(0.0F, metrics.minimum_target_separation_m -
                                                      moving_target->capture_radius_m)
                                 : target_distance;
    previous = control;
    if (metrics.collision && early_exit_on_collision) {
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
      -(initial_target_distance -
        (moving_target.has_value()
             ? metrics.minimum_target_separation_m
             : std::hypot(target_x_m - state.x, target_y_m - state.y)));
  metrics.soft_cost =
      costs.head_progress_weight * -metrics.costs.head_progress +
      costs.progress_weight * metrics.costs.progress +
      costs.speed_tracking_weight * dynamics.dt_s * metrics.costs.speed_tracking +
      costs.guide_deviation_weight * dynamics.dt_s * metrics.costs.guide_deviation +
      costs.acceleration_weight * dynamics.dt_s * metrics.costs.acceleration +
      costs.jerk_weight * metrics.costs.jerk +
      costs.yaw_change_weight * metrics.costs.yaw_change +
      costs.control_effort_weight * dynamics.dt_s * metrics.costs.control_effort +
      costs.terminal_weight * metrics.costs.terminal;
  metrics.terminal_state = state;
  return metrics;
}

} // namespace drone_city_nav::mppi
