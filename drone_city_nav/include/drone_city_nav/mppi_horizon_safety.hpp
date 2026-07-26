#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <span>
#include <vector>

namespace drone_city_nav {

struct MppiHorizonSafetyConfig {
  double collision_radius_m{0.5};
  double reaction_latency_s{0.10};
  double maximum_braking_acceleration_mps2{8.0};
  double minimum_time_to_collision_s{0.50};
  double fallback_duration_s{2.0};
  double dt_s{0.05};
};

enum class MppiHorizonSafetyDecision {
  kExecute,
  kBrake,
  kHold,
};

struct MppiHorizonSafetyResult {
  MppiHorizonSafetyDecision decision{MppiHorizonSafetyDecision::kHold};
  double time_to_collision_s{0.0};
  double stopping_time_s{0.0};
  double stopping_distance_m{0.0};
  std::vector<mppi::State> fallback_horizon;
  std::vector<mppi::Control> fallback_controls;
};

[[nodiscard]] MppiHorizonSafetyResult
evaluateMppiHorizonSafety(const mppi::State& current_state,
                          std::span<const mppi::State> horizon,
                          std::span<const float> esdf_m, const mppi::EsdfGrid& grid,
                          const MppiHorizonSafetyConfig& config);

} // namespace drone_city_nav
