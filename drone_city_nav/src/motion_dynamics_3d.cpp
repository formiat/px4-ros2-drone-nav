#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace drone_city_nav {
namespace {

// How far a reconstructed state may drift from the recorded one before the
// two describe different motions, and how far a control may sit outside a
// limit before it breaks it. Both absorb float round-trips only.
constexpr float kDynamicsStateToleranceMps{2.0e-3F};
constexpr float kDynamicsControlToleranceMps2{1.0e-4F};

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

void clampTranslational(float& x, float& y, float& z, const float limit) noexcept {
  const float magnitude = std::hypot(std::hypot(x, y), z);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
    z *= scale;
  }
}

} // namespace

MotionState3D integrateMotionState3D(MotionState3D state, MotionControl3D control,
                                     const MotionDynamicsConfig3D& config) noexcept {
  clampHorizontal(control.ax, control.ay, config.maximum_horizontal_acceleration_mps2);
  control.az = clampMagnitude(control.az, config.maximum_vertical_acceleration_mps2);
  control.yaw_accel =
      clampMagnitude(control.yaw_accel, config.maximum_yaw_acceleration_radps2);

  const float drag = std::max(0.0F, 1.0F - config.linear_drag_1ps * config.dt_s);
  const float inherited_horizontal_speed_mps = std::hypot(state.vx, state.vy);
  const float inherited_vertical_speed_mps = std::abs(state.vz);
  const float inherited_translational_speed_mps =
      std::hypot(inherited_horizontal_speed_mps, inherited_vertical_speed_mps);
  const float inherited_yaw_rate_radps = std::abs(state.yaw_rate);
  state.vx = state.vx * drag + control.ax * config.dt_s;
  state.vy = state.vy * drag + control.ay * config.dt_s;
  state.vz = state.vz * drag + control.az * config.dt_s;
  // A state above a cap keeps at most the speed the maximum deceleration
  // leaves it after one step: an inherited excess is shed at least as fast as
  // the vehicle can brake instead of being carried along the whole horizon.
  const float horizontal_shed_mps =
      config.maximum_horizontal_acceleration_mps2 * config.dt_s;
  const float vertical_shed_mps =
      config.maximum_vertical_acceleration_mps2 * config.dt_s;
  clampHorizontal(state.vx, state.vy,
                  std::max(config.maximum_horizontal_speed_mps,
                           inherited_horizontal_speed_mps - horizontal_shed_mps));
  state.vz = clampMagnitude(state.vz,
                            std::max(config.maximum_vertical_speed_mps,
                                     inherited_vertical_speed_mps - vertical_shed_mps));
  clampTranslational(state.vx, state.vy, state.vz,
                     std::max(config.maximum_translational_speed_mps,
                              inherited_translational_speed_mps - horizontal_shed_mps));

  state.yaw_rate =
      clampMagnitude(state.yaw_rate + control.yaw_accel * config.dt_s,
                     std::max(config.maximum_yaw_rate_radps, inherited_yaw_rate_radps));
  state.x += state.vx * config.dt_s;
  state.y += state.vy * config.dt_s;
  state.z += state.vz * config.dt_s;
  state.yaw = std::remainder(state.yaw + state.yaw_rate * config.dt_s,
                             2.0F * std::numbers::pi_v<float>);
  return state;
}

namespace {

[[nodiscard]] bool finiteControl(const MotionControl3D& control) noexcept {
  return std::isfinite(control.ax) && std::isfinite(control.ay) &&
         std::isfinite(control.az) && std::isfinite(control.yaw_accel);
}

[[nodiscard]] bool statesNearlyEqual(const MotionState3D& first,
                                     const MotionState3D& second) noexcept {
  const auto close = [](const float left, const float right) noexcept {
    return std::isfinite(left) && std::isfinite(right) &&
           std::abs(left - right) <= kDynamicsStateToleranceMps;
  };
  return close(first.x, second.x) && close(first.y, second.y) &&
         close(first.z, second.z) && close(first.vx, second.vx) &&
         close(first.vy, second.vy) && close(first.vz, second.vz) &&
         std::abs(std::remainder(first.yaw - second.yaw,
                                 2.0F * std::numbers::pi_v<float>)) <=
             kDynamicsStateToleranceMps &&
         close(first.yaw_rate, second.yaw_rate);
}

} // namespace

const char* motionDynamicsConsistency3DName(
    const MotionDynamicsConsistency3D consistency) noexcept {
  switch (consistency) {
    case MotionDynamicsConsistency3D::kConsistent:
      return "consistent";
    case MotionDynamicsConsistency3D::kNotFinite:
      return "not_finite";
    case MotionDynamicsConsistency3D::kAccelerationLimitExceeded:
      return "acceleration_limit";
    case MotionDynamicsConsistency3D::kJerkLimitExceeded:
      return "jerk_limit";
    case MotionDynamicsConsistency3D::kStateMismatch:
      return "state_mismatch";
  }
  return "unknown";
}

bool motionDynamicsConfigIsValid3D(const MotionDynamicsConfig3D& dynamics) noexcept {
  const auto positive = [](const float value) noexcept {
    return std::isfinite(value) && value > 0.0F;
  };
  return positive(dynamics.dt_s) &&
         positive(dynamics.maximum_horizontal_acceleration_mps2) &&
         positive(dynamics.maximum_vertical_acceleration_mps2) &&
         positive(dynamics.maximum_yaw_acceleration_radps2) &&
         positive(dynamics.maximum_control_jerk_mps3);
}

MotionDynamicsConsistency3D finiteMotionHorizonDynamicsConsistency3D(
    const FiniteMotionHorizon3D& horizon,
    const MotionControl3D& previous_applied_control,
    const MotionDynamicsConfig3D& dynamics) noexcept {
  if (horizon.controls.empty() ||
      horizon.states.size() != horizon.controls.size() + 1U ||
      !finiteControl(previous_applied_control) ||
      !motionDynamicsConfigIsValid3D(dynamics)) {
    return MotionDynamicsConsistency3D::kNotFinite;
  }
  MotionControl3D previous_control = previous_applied_control;
  for (std::size_t index = 0U; index < horizon.controls.size(); ++index) {
    const MotionControl3D& control = horizon.controls[index];
    const MotionDynamicsConsistency3D consistency = motionStepDynamicallyConsistent3D(
        horizon.states[index], previous_control, control, horizon.states[index + 1U],
        dynamics);
    if (consistency != MotionDynamicsConsistency3D::kConsistent) {
      return consistency;
    }
    previous_control = control;
  }
  return MotionDynamicsConsistency3D::kConsistent;
}

MotionDynamicsConsistency3D motionStepDynamicallyConsistent3D(
    const MotionState3D& from, const MotionControl3D& previous_control,
    const MotionControl3D& control, const MotionState3D& to,
    const MotionDynamicsConfig3D& dynamics) noexcept {
  if (!finiteControl(control) || !finiteControl(previous_control) ||
      !motionDynamicsConfigIsValid3D(dynamics)) {
    return MotionDynamicsConsistency3D::kNotFinite;
  }
  if (std::hypot(control.ax, control.ay) >
          dynamics.maximum_horizontal_acceleration_mps2 +
              kDynamicsControlToleranceMps2 ||
      std::abs(control.az) >
          dynamics.maximum_vertical_acceleration_mps2 + kDynamicsControlToleranceMps2 ||
      std::abs(control.yaw_accel) >
          dynamics.maximum_yaw_acceleration_radps2 + kDynamicsControlToleranceMps2) {
    return MotionDynamicsConsistency3D::kAccelerationLimitExceeded;
  }
  const float maximum_control_delta =
      dynamics.maximum_control_jerk_mps3 * dynamics.dt_s;
  if (std::abs(control.ax - previous_control.ax) >
          maximum_control_delta + kDynamicsControlToleranceMps2 ||
      std::abs(control.ay - previous_control.ay) >
          maximum_control_delta + kDynamicsControlToleranceMps2 ||
      std::abs(control.az - previous_control.az) >
          maximum_control_delta + kDynamicsControlToleranceMps2) {
    return MotionDynamicsConsistency3D::kJerkLimitExceeded;
  }
  return statesNearlyEqual(integrateMotionState3D(from, control, dynamics), to)
             ? MotionDynamicsConsistency3D::kConsistent
             : MotionDynamicsConsistency3D::kStateMismatch;
}

} // namespace drone_city_nav
