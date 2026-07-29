#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] builtin_interfaces::msg::Time
timeFromNanoseconds(const std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return time;
}

void appendStationaryHoldPoint(msg::MppiTrajectoryHorizon& horizon,
                               const PassageCoordinatorResult& passage,
                               const float time_from_start_s, const float yaw_rad) {
  msg::MppiHorizonPoint point;
  point.time_from_start_s = time_from_start_s;
  point.position.x = passage.hold_position.x;
  point.position.y = passage.hold_position.y;
  point.position.z = passage.preferred_z_m;
  point.yaw_rad = yaw_rad;
  horizon.points.push_back(point);
}

} // namespace

void ProductionMppiNode::publishExecutionHorizon(
    const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
    const ProductionMppiPreparedEsdf& esdf,
    const PassageCoordinatorResult& passage_coordinator,
    const ProductionMppiPlanningState planning_state, const std::int64_t now_ns) {
  if (!execution_horizon_pub_) {
    return;
  }

  const auto make_horizon = [&](const std::int64_t valid_until_ns) {
    msg::MppiTrajectoryHorizon horizon;
    horizon.header.stamp = now();
    horizon.header.frame_id = frame_id_;
    horizon.sequence = tick_sequence_;
    horizon.valid_from = timeFromNanoseconds(now_ns);
    horizon.valid_until = timeFromNanoseconds(valid_until_ns);
    horizon.pose_revision = input.pose_revision;
    horizon.obstacle_revision = input.obstacle_revision;
    horizon.risk_tier = static_cast<std::uint8_t>(result.selected_tier);
    horizon.passage_constrained = input.passage.has_value();
    horizon.stationary_position_hold = passage_coordinator.hold_xy;
    horizon.stationary_hold_position.x = passage_coordinator.hold_position.x;
    horizon.stationary_hold_position.y = passage_coordinator.hold_position.y;
    horizon.stationary_hold_position.z = passage_coordinator.preferred_z_m;
    return horizon;
  };

  if (passage_coordinator.hold_xy) {
    const auto hold_duration_ns = static_cast<std::int64_t>(
        std::max(0.2, 2.0 * static_cast<double>(mppi_config_.dynamics.dt_s)) * 1.0e9);
    msg::MppiTrajectoryHorizon horizon = make_horizon(now_ns + hold_duration_ns);
    horizon.points.reserve(2U);
    appendStationaryHoldPoint(horizon, passage_coordinator, 0.0F,
                              input.initial_state.yaw);
    appendStationaryHoldPoint(horizon, passage_coordinator, mppi_config_.dynamics.dt_s,
                              input.initial_state.yaw);
    execution_horizon_pub_->publish(horizon);
    return;
  }

  if (!esdf.distances_m || result.horizon.size() < 2U) {
    return;
  }
  const bool engine_collision = result.raw_collision || result.known_solid_collision;
  MppiHorizonSafetyResult safety = evaluateMppiHorizonSafety(
      input.initial_state, result.horizon, *esdf.distances_m, esdf.grid, safety_config_,
      engine_collision, known_solids_);
  const bool forced_braking_hold =
      planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold;
  const bool braking = forced_braking_hold || engine_collision ||
                       safety.decision != MppiHorizonSafetyDecision::kExecute;
  std::span<const mppi::State> states{result.horizon};
  std::span<const mppi::Control> controls{result.controls};
  if (braking && !forced_braking_hold) {
    states = safety.fallback_horizon;
    controls = safety.fallback_controls;
  }
  if (states.size() < 2U || controls.empty()) {
    return;
  }

  msg::MppiTrajectoryHorizon horizon = make_horizon(
      now_ns + static_cast<std::int64_t>(
                   static_cast<double>(controls.size()) *
                   static_cast<double>(mppi_config_.dynamics.dt_s) * 1.0e9));
  horizon.emergency_braking = braking;
  horizon.points.reserve(states.size());
  for (std::size_t index = 0U; index < states.size(); ++index) {
    const mppi::State& state = states[index];
    const mppi::Control control = controls[std::min(index, controls.size() - 1U)];
    msg::MppiHorizonPoint point;
    point.time_from_start_s = static_cast<float>(index) * mppi_config_.dynamics.dt_s;
    point.position.x = state.x;
    point.position.y = state.y;
    point.position.z = state.z;
    point.velocity.x = state.vx;
    point.velocity.y = state.vy;
    point.velocity.z = state.vz;
    point.acceleration.x = control.ax;
    point.acceleration.y = control.ay;
    point.acceleration.z = control.az;
    point.yaw_rad = state.yaw;
    point.yaw_rate_radps = state.yaw_rate;
    horizon.points.push_back(point);
  }
  execution_horizon_pub_->publish(horizon);
}

} // namespace drone_city_nav
