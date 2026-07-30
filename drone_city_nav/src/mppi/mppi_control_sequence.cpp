#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include <algorithm>
#include <cmath>
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

} // namespace

Control interpolateControl(const Control& first, const Control& second,
                           const float ratio) noexcept {
  const float clamped = std::clamp(ratio, 0.0F, 1.0F);
  return Control{
      .ax = std::lerp(first.ax, second.ax, clamped),
      .ay = std::lerp(first.ay, second.ay, clamped),
      .az = std::lerp(first.az, second.az, clamped),
      .yaw_accel = std::lerp(first.yaw_accel, second.yaw_accel, clamped),
  };
}

std::vector<Control> shiftControlSequence(const std::span<const Control> controls,
                                          const float dt_s, const double elapsed_s) {
  if (!(dt_s > 0.0F) || !std::isfinite(dt_s) || !std::isfinite(elapsed_s) ||
      elapsed_s < 0.0) {
    throw std::invalid_argument{"invalid MPPI control sequence timing"};
  }
  if (controls.empty()) {
    return {};
  }
  const double offset_steps = elapsed_s / static_cast<double>(dt_s);
  if (offset_steps >= static_cast<double>(controls.size())) {
    return std::vector<Control>(controls.size());
  }

  std::vector<Control> shifted;
  shifted.reserve(controls.size());
  const std::size_t last = controls.size() - 1U;
  for (std::size_t index = 0U; index < controls.size(); ++index) {
    const double source = static_cast<double>(index) + offset_steps;
    if (source >= static_cast<double>(last)) {
      shifted.push_back(controls[last]);
      continue;
    }
    const std::size_t lower = static_cast<std::size_t>(std::floor(source));
    const std::size_t upper = lower + 1U;
    shifted.push_back(
        interpolateControl(controls[lower], controls[upper],
                           static_cast<float>(source - static_cast<double>(lower))));
  }
  return shifted;
}

void limitControlSequence(const std::span<Control> controls,
                          const DynamicsConfig& dynamics,
                          const Control previous_applied_control,
                          const float first_control_interval_s) noexcept {
  Control previous = previous_applied_control;
  for (std::size_t step = 0U; step < controls.size(); ++step) {
    const float interval_s = step == 0U ? first_control_interval_s : dynamics.dt_s;
    const float maximum_delta = dynamics.maximum_control_jerk_mps3 * interval_s;
    Control& control = controls[step];
    clampHorizontal(control.ax, control.ay,
                    dynamics.maximum_horizontal_acceleration_mps2);
    control.az =
        clampMagnitude(control.az, dynamics.maximum_vertical_acceleration_mps2);
    control.yaw_accel =
        clampMagnitude(control.yaw_accel, dynamics.maximum_yaw_acceleration_radps2);
    control.ax = std::clamp(control.ax, previous.ax - maximum_delta,
                            previous.ax + maximum_delta);
    control.ay = std::clamp(control.ay, previous.ay - maximum_delta,
                            previous.ay + maximum_delta);
    control.az = std::clamp(control.az, previous.az - maximum_delta,
                            previous.az + maximum_delta);
    previous = control;
  }
}

std::vector<Control> buildGuideDirectedNominalSeed(const State& initial,
                                                   const State& target,
                                                   const DynamicsConfig& dynamics,
                                                   const std::size_t steps,
                                                   const std::uint64_t generation) {
  std::vector<Control> seed(steps);
  const float dx = target.x - initial.x;
  const float dy = target.y - initial.y;
  const float distance = std::hypot(dx, dy);
  const float direction_x = distance > 1.0e-3F ? dx / distance : 0.0F;
  const float direction_y = distance > 1.0e-3F ? dy / distance : 0.0F;
  const float lateral_x = -direction_y;
  const float lateral_y = direction_x;
  const std::uint64_t phase = generation % 3U;
  float lateral_sign = 0.0F;
  if (phase == 1U) {
    lateral_sign = 1.0F;
  } else if (phase == 2U) {
    lateral_sign = -1.0F;
  }
  const float forward_acceleration =
      std::min(0.35F * dynamics.maximum_horizontal_acceleration_mps2, 3.0F);
  const float lateral_acceleration =
      lateral_sign *
      std::min(0.20F * dynamics.maximum_horizontal_acceleration_mps2, 1.5F);
  const float vertical_acceleration = clampMagnitude(
      0.5F * (target.z - initial.z), dynamics.maximum_vertical_acceleration_mps2);

  for (std::size_t index = 0U; index < steps; ++index) {
    const float decay = 1.0F - static_cast<float>(index) / static_cast<float>(steps);
    seed[index] = Control{
        .ax = forward_acceleration * direction_x +
              lateral_acceleration * lateral_x * decay,
        .ay = forward_acceleration * direction_y +
              lateral_acceleration * lateral_y * decay,
        .az = vertical_acceleration * decay,
        .yaw_accel = 0.0F,
    };
  }
  return seed;
}

} // namespace drone_city_nav::mppi
