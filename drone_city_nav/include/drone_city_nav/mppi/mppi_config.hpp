#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/mppi/mppi_horizon_sampling.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/stopping_capability.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace drone_city_nav::mppi {

using DynamicsConfig = drone_city_nav::MotionDynamicsConfig3D;

struct NoiseConfig {
  float horizontal_acceleration_sigma_mps2{3.0F};
  float vertical_acceleration_sigma_mps2{1.5F};
  float yaw_acceleration_sigma_radps2{0.6F};
};

struct RiskConfig {
  float critical_distance_m{1.0F};
  float preferred_distance_m{6.0F};
  float obstacle_approach_response_time_s{0.25F};
  float obstacle_approach_deceleration_mps2{4.0F};
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
  bool clearance_broad_phase_enabled{true};
};

using AltitudeEnvelopeConfig = drone_city_nav::MotionAltitudeEnvelopeConfig3D;

struct CostConfig {
  float guide_deviation_weight{1.0F};
  float altitude_tracking_weight{4.0F};
  float head_progress_horizon_s{0.4F};
  float head_progress_weight{8.0F};
  float progress_weight{4.0F};
  float route_progress_integral_weight{2.0F};
  float speed_tracking_weight{1.0F};
  // Cost per second of squared translational speed above the dynamics speed
  // caps. The caps bound every rollout state, but a rollout that starts above
  // them inherits its speed, and without this term the progress reward keeps
  // an overspeeding vehicle at its inherited speed instead of shedding it.
  float overspeed_weight{200.0F};
  float acceleration_weight{0.03F};
  float jerk_weight{0.02F};
  float yaw_change_weight{0.1F};
  float control_effort_weight{0.01F};
  float peer_separation_weight{80.0F};
  float cooperative_maneuver_preference_weight{1.5F};
  float terminal_weight{2.0F};
  float planning_exposure_weight{2.0F};
  float critical_exposure_weight{20.0F};
  float critical_clearance_proximity_weight{400.0F};
  float obstacle_approach_weight{40.0F};
  float temperature{8.0F};
  // The softmax temperature grows with the mean feasible cost excess above the
  // best rollout so a population spread over thousands of cost units does not
  // collapse onto a single sample. Zero keeps the fixed temperature.
  float adaptive_temperature_cost_fraction{0.5F};
  // Excludes rollouts whose body enters an occupied ESDF voxel from the
  // weighted update while any other rollout stays feasible. The device ESDF is
  // coarser than the raw grid, so the gate can reject raw-valid passages and
  // leave the update hovering; it is a sampler ranking aid, off by default.
  bool body_collision_gate_enabled{false};
  // Share of that mean excess the deterministic route candidate may cost more
  // than the best feasible rollout and still override the weighted update.
  float route_directed_candidate_cost_tolerance{0.5F};
};

struct CooperativeConfig {
  float desired_minimum_separation_m{5.0F};
  float candidate_acceleration_fraction{0.75F};
  float candidate_duration_s{1.5F};
};

struct BenchmarkConfig {
  std::string scenario{"urban_blocks"};
  std::size_t rollouts{8192U};
  std::size_t steps{80U};
  std::size_t warmup_ticks{100U};
  std::size_t measured_ticks{1000U};
  double deadline_ms{50.0};
  std::uint64_t seed{42U};
  bool early_exit_on_altitude_envelope_violation{true};
  DynamicsConfig dynamics{};
  NoiseConfig noise{};
  RiskConfig risk{};
  FootprintConfig footprint{};
  AltitudeEnvelopeConfig altitude_envelope{};
  StoppingCapability stopping_capability{};
  CostConfig costs{};
  CooperativeConfig cooperative{};
  HorizonSamplingConfig horizon_sampling{};
};

[[nodiscard]] bool benchmarkConfigIsValid(const BenchmarkConfig& config) noexcept;

} // namespace drone_city_nav::mppi
