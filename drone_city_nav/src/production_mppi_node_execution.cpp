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

} // namespace

void ProductionMppiNode::publishExecutionHorizon(
    const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
    const ProductionMppiPreparedEsdf& esdf,
    const ProductionMppiPlanningState planning_state, const std::int64_t now_ns) {
  if (!execution_horizon_pub_ || !esdf.distances_m || result.horizon.size() < 2U) {
    return;
  }
  MppiHorizonSafetyResult safety =
      evaluateMppiHorizonSafety(input.initial_state, result.horizon, *esdf.distances_m,
                                esdf.grid, safety_config_);
  const bool engine_collision = result.raw_collision || result.known_solid_collision;
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

  msg::MppiTrajectoryHorizon horizon;
  horizon.header.stamp = now();
  horizon.header.frame_id = frame_id_;
  horizon.sequence = tick_sequence_;
  horizon.valid_from = timeFromNanoseconds(now_ns);
  horizon.valid_until = timeFromNanoseconds(
      now_ns + static_cast<std::int64_t>(
                   static_cast<double>(controls.size()) *
                   static_cast<double>(mppi_config_.dynamics.dt_s) * 1.0e9));
  horizon.pose_revision = input.pose_revision;
  horizon.obstacle_revision = input.obstacle_revision;
  horizon.risk_tier = static_cast<std::uint8_t>(result.selected_tier);
  horizon.emergency_braking = braking;
  horizon.passage_constrained = input.passage.has_value();
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
