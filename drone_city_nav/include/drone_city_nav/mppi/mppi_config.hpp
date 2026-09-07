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
  // The one clearance charge a rollout carries: the squared shortfall between
  // the clearance it keeps and the clearance its own speed needs to stop
  // within. It is the same law the speed policy's clearance limiter reads in
  // the other direction, so the optimiser and the reference speed cannot
  // disagree about what "too close, too fast" means.
  //
  // Two charges used to sit here instead. A flat price per metre *travelled*
  // inside the critical band priced motion itself: in a corridor narrower than
  // the band a metre of flight cost two orders of magnitude more than the
  // progress it earned, and the weighted update converged on standing still. A
  // band-normalised squared depth priced position without regard to speed, so
  // it said nothing about running fast along a wall. Distance inside the band
  // is still measured, for the risk tier and diagnostics.
  float obstacle_approach_weight{40.0F};
  // Floor of the regulated softmax temperature.
  float temperature{8.0F};
  // Share of the feasible population the softmax should spread its weight
  // over, measured as the effective sample size (sum w)^2 / (N sum w^2). The
  // temperature is nudged toward it each tick. A fixed temperature against a
  // population spread over thousands of cost units collapses the weights onto
  // whichever sample happens to be best and the weighted update degenerates
  // into "best of N random"; scaling the temperature by a share of the mean
  // cost excess tracked the spread's scale but not how many samples actually
  // carried weight, which is the quantity that matters. Zero disables the
  // regulator and keeps the floor.
  float target_effective_sample_fraction{0.07F};
  // How far above the floor the regulator may take the temperature.
  float maximum_temperature_growth{1000.0F};
  // Excludes rollouts whose body enters an occupied ESDF voxel from the
  // weighted update while any other rollout stays feasible. The device ESDF is
  // coarser than the raw grid, so the gate can reject raw-valid passages and
  // leave the update hovering; it is a sampler ranking aid, off by default.
  bool body_collision_gate_enabled{false};
  // Share of that mean excess the deterministic route candidate may cost more
  // than the best feasible rollout and still override the weighted update.
  float route_directed_candidate_cost_tolerance{0.5F};
  // Consecutive ticks the deterministic candidate must stay preferable before
  // it takes the update over from the weighted one. The two sources produce
  // visibly different first controls, so a preference that flips tick to tick
  // is felt as a jerk; a candidate that is genuinely better stays better for
  // several ticks. Switching back the other way needs the preference to lapse,
  // which is immediate: the weighted update is the default owner.
  std::uint32_t route_directed_candidate_switch_ticks{3U};
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
