#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace drone_city_nav::mppi {

struct DynamicsConfig {
  float dt_s{0.05F};
  float linear_drag_1ps{0.08F};
  float maximum_horizontal_acceleration_mps2{8.0F};
  float maximum_vertical_acceleration_mps2{4.0F};
  float maximum_horizontal_speed_mps{20.0F};
  float maximum_vertical_speed_mps{5.0F};
  float maximum_yaw_acceleration_radps2{2.0F};
  float maximum_yaw_rate_radps{1.5F};
  float maximum_control_jerk_mps3{20.0F};
};

struct NoiseConfig {
  float horizontal_acceleration_sigma_mps2{3.0F};
  float vertical_acceleration_sigma_mps2{1.5F};
  float yaw_acceleration_sigma_radps2{0.6F};
};

struct RiskConfig {
  float critical_distance_m{1.0F};
  float preferred_distance_m{6.0F};
  float critical_exposure_tolerance_m{0.5F};
  float planning_exposure_tolerance_m{1.0F};
};

struct FootprintConfig {
  float radius_m{0.0F};
  float lower_extent_m{0.0F};
  float upper_extent_m{0.0F};
  std::uint32_t perimeter_samples{0U};
  std::uint32_t radial_rings{0U};
  std::uint32_t axial_samples{0U};
};

struct CostConfig {
  float guide_deviation_weight{1.0F};
  float altitude_tracking_weight{4.0F};
  float head_progress_horizon_s{0.4F};
  float head_progress_weight{8.0F};
  float progress_weight{4.0F};
  float speed_tracking_weight{1.0F};
  float acceleration_weight{0.03F};
  float jerk_weight{0.02F};
  float yaw_change_weight{0.1F};
  float control_effort_weight{0.01F};
  float terminal_weight{2.0F};
  float planning_exposure_weight{2.0F};
  float critical_exposure_weight{20.0F};
  float temperature{8.0F};
};

struct BenchmarkConfig {
  std::string scenario{"urban_blocks"};
  std::size_t rollouts{8192U};
  std::size_t steps{80U};
  std::size_t warmup_ticks{100U};
  std::size_t measured_ticks{1000U};
  double deadline_ms{50.0};
  std::uint64_t seed{42U};
  bool early_exit_on_collision{true};
  DynamicsConfig dynamics{};
  NoiseConfig noise{};
  RiskConfig risk{};
  FootprintConfig footprint{};
  CostConfig costs{};
};

[[nodiscard]] bool benchmarkConfigIsValid(const BenchmarkConfig& config) noexcept;

} // namespace drone_city_nav::mppi
