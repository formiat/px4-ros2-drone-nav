#include "drone_city_nav/mppi/mppi_reference.hpp"

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

[[nodiscard]] float sampleEsdf(const EsdfGrid& grid, const std::span<const float> esdf,
                               const float x, const float y) noexcept {
  const float grid_x = (x - grid.origin_x_m) / grid.resolution_m - 0.5F;
  const float grid_y = (y - grid.origin_y_m) / grid.resolution_m - 0.5F;
  if (grid_x < 0.0F || grid_y < 0.0F || grid_x >= static_cast<float>(grid.width - 1) ||
      grid_y >= static_cast<float>(grid.height - 1)) {
    return 0.0F;
  }
  const int x0 = static_cast<int>(std::floor(grid_x));
  const int y0 = static_cast<int>(std::floor(grid_y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const float fx = grid_x - static_cast<float>(x0);
  const float fy = grid_y - static_cast<float>(y0);
  const auto at = [&esdf, &grid](const int cell_x, const int cell_y) {
    return esdf[static_cast<std::size_t>(cell_y) *
                    static_cast<std::size_t>(grid.width) +
                static_cast<std::size_t>(cell_x)];
  };
  const float lower = std::lerp(at(x0, y0), at(x1, y0), fx);
  const float upper = std::lerp(at(x0, y1), at(x1, y1), fx);
  return std::lerp(lower, upper, fy);
}

[[nodiscard]] float squared(const float value) noexcept {
  return value * value;
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
         config.risk.collision_radius_m >= 0.0F &&
         config.risk.critical_distance_m >= config.risk.collision_radius_m &&
         config.risk.preferred_distance_m >= config.risk.critical_distance_m;
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
    const bool early_exit_on_collision, const Control previous_applied_control) {
  RolloutMetrics metrics{};
  metrics.minimum_clearance_m = std::numeric_limits<float>::infinity();
  State state = initial_state;
  Control previous = previous_applied_control;
  const float initial_target_distance =
      std::hypot(target_x_m - state.x, target_y_m - state.y);
  const std::size_t head_steps =
      std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(
                                  costs.head_progress_horizon_s / dynamics.dt_s)),
                              1U, nominal_controls.size());
  for (std::size_t step = 0U; step < nominal_controls.size(); ++step) {
    Control control{
        .ax = nominal_controls[step].ax + noise_controls[step].ax,
        .ay = nominal_controls[step].ay + noise_controls[step].ay,
        .az = nominal_controls[step].az + noise_controls[step].az,
        .yaw_accel = nominal_controls[step].yaw_accel + noise_controls[step].yaw_accel,
    };
    state = integrateReference(state, control, dynamics);
    const float clearance = sampleEsdf(grid, esdf, state.x, state.y);
    metrics.minimum_clearance_m = std::min(metrics.minimum_clearance_m, clearance);
    const float segment_m = dynamics.dt_s * std::hypot(state.vx, state.vy);
    if (clearance <= risk.collision_radius_m) {
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
        std::hypot(target_x_m - state.x, target_y_m - state.y);
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
    metrics.costs.terminal = target_distance;
    previous = control;
    if (metrics.collision && early_exit_on_collision) {
      break;
    }
  }
  metrics.costs.progress = -(initial_target_distance -
                             std::hypot(target_x_m - state.x, target_y_m - state.y));
  metrics.soft_cost =
      costs.head_progress_weight * -metrics.costs.head_progress +
      costs.progress_weight * metrics.costs.progress +
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
