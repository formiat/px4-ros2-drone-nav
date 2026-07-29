#include "drone_city_nav/mppi_horizon_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace drone_city_nav {
namespace {

[[nodiscard]] float clearanceAt(const mppi::State& state,
                                const std::span<const float> esdf_m,
                                const mppi::EsdfGrid& grid) {
  const int x =
      static_cast<int>(std::floor((state.x - grid.origin_x_m) / grid.resolution_m));
  const int y =
      static_cast<int>(std::floor((state.y - grid.origin_y_m) / grid.resolution_m));
  if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) {
    return 0.0F;
  }
  return esdf_m[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
                static_cast<std::size_t>(x)];
}

[[nodiscard]] bool
intersectsKnownSolid(const mppi::State& state,
                     const std::span<const mppi::KnownSolid> known_solids) noexcept {
  return std::ranges::any_of(known_solids, [&state](const mppi::KnownSolid& solid) {
    if (state.z < solid.min_z_m || state.z > solid.max_z_m) {
      return false;
    }
    const float dx = state.x - solid.center_x_m;
    const float dy = state.y - solid.center_y_m;
    return std::abs(dx * solid.normal_x + dy * solid.normal_y) <= solid.half_depth_m &&
           std::abs(dx * solid.lateral_x + dy * solid.lateral_y) <= solid.half_width_m;
  });
}

void populateBrakingFallback(const mppi::State& initial,
                             const MppiHorizonSafetyConfig& config,
                             MppiHorizonSafetyResult& result) {
  mppi::State state = initial;
  const std::size_t steps =
      static_cast<std::size_t>(std::ceil(config.fallback_duration_s / config.dt_s));
  result.fallback_horizon.reserve(steps + 1U);
  result.fallback_controls.reserve(steps);
  result.fallback_horizon.push_back(state);
  for (std::size_t step = 0U; step < steps; ++step) {
    const double horizontal_speed = std::hypot(state.vx, state.vy);
    mppi::Control control{};
    if (horizontal_speed > 1.0e-3) {
      const double deceleration = std::min(config.maximum_braking_acceleration_mps2,
                                           horizontal_speed / config.dt_s);
      control.ax = static_cast<float>(-deceleration * state.vx / horizontal_speed);
      control.ay = static_cast<float>(-deceleration * state.vy / horizontal_speed);
    }
    if (std::abs(state.vz) > 1.0e-3) {
      control.az = static_cast<float>(
          -std::copysign(std::min(config.maximum_braking_acceleration_mps2,
                                  std::abs(state.vz) / config.dt_s),
                         state.vz));
    }
    state.vx += control.ax * static_cast<float>(config.dt_s);
    state.vy += control.ay * static_cast<float>(config.dt_s);
    state.vz += control.az * static_cast<float>(config.dt_s);
    state.x += state.vx * static_cast<float>(config.dt_s);
    state.y += state.vy * static_cast<float>(config.dt_s);
    state.z += state.vz * static_cast<float>(config.dt_s);
    result.fallback_controls.push_back(control);
    result.fallback_horizon.push_back(state);
  }
}

} // namespace

MppiHorizonSafetyResult
buildMppiBrakingFallback(const mppi::State& current_state,
                         const MppiHorizonSafetyConfig& config) {
  MppiHorizonSafetyResult result;
  const double speed =
      std::hypot(std::hypot(current_state.vx, current_state.vy), current_state.vz);
  result.decision = speed > 0.15 ? MppiHorizonSafetyDecision::kBrake
                                 : MppiHorizonSafetyDecision::kHold;
  result.stopping_time_s =
      config.reaction_latency_s +
      speed / std::max(1.0e-3, config.maximum_braking_acceleration_mps2);
  result.stopping_distance_m =
      speed * config.reaction_latency_s +
      speed * speed /
          (2.0 * std::max(1.0e-3, config.maximum_braking_acceleration_mps2));
  result.time_to_collision_s = std::numeric_limits<double>::infinity();
  populateBrakingFallback(current_state, config, result);
  return result;
}

MppiHorizonSafetyResult evaluateMppiHorizonSafety(
    const mppi::State& current_state, const std::span<const mppi::State> horizon,
    const std::span<const float> esdf_m, const mppi::EsdfGrid& grid,
    const MppiHorizonSafetyConfig& config, const bool engine_collision,
    const std::span<const mppi::KnownSolid> known_solids) {
  MppiHorizonSafetyResult result;
  const double speed =
      std::hypot(std::hypot(current_state.vx, current_state.vy), current_state.vz);
  result.stopping_time_s =
      config.reaction_latency_s +
      speed / std::max(1.0e-3, config.maximum_braking_acceleration_mps2);
  result.stopping_distance_m =
      speed * config.reaction_latency_s +
      speed * speed /
          (2.0 * std::max(1.0e-3, config.maximum_braking_acceleration_mps2));
  result.time_to_collision_s = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < horizon.size(); ++index) {
    if (clearanceAt(horizon[index], esdf_m, grid) <= config.collision_radius_m ||
        intersectsKnownSolid(horizon[index], known_solids)) {
      result.time_to_collision_s = static_cast<double>(index) * config.dt_s;
      break;
    }
  }
  if (!std::isfinite(result.time_to_collision_s) && !engine_collision) {
    result.decision = MppiHorizonSafetyDecision::kExecute;
    return result;
  }
  if (!std::isfinite(result.time_to_collision_s)) {
    return buildMppiBrakingFallback(current_state, config);
  }
  result.decision =
      result.time_to_collision_s >
              std::max(config.minimum_time_to_collision_s, result.stopping_time_s)
          ? MppiHorizonSafetyDecision::kBrake
          : MppiHorizonSafetyDecision::kHold;
  populateBrakingFallback(current_state, config, result);
  return result;
}

} // namespace drone_city_nav
