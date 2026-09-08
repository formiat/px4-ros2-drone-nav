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

// The stopping law charges every state before a rollout's first contact for
// the free path it had, and those states are known only once the contact is
// reached: each rollout keeps its path and speed history up to this many
// steps. A configuration with a longer horizon is rejected.
inline constexpr std::size_t kMaximumHorizonSteps{256U};

struct NoiseConfig {
  float horizontal_acceleration_sigma_mps2{3.0F};
  float vertical_acceleration_sigma_mps2{1.5F};
  float yaw_acceleration_sigma_radps2{0.6F};
};

struct RiskConfig {
  // Risk-tier bands, for classification and diagnostics only: distance inside
  // the critical band is the critical tier, inside the preferred band the
  // planning tier. Neither is a margin any law measures from.
  float critical_distance_m{1.0F};
  // The preferred band is also the reach of the clearance preference: depth
  // into it is priced per second, position rather than motion.
  float preferred_distance_m{6.0F};
  // Tracking-error tube law for evidence beside the motion: the error the
  // controller can accumulate within this response time must fit inside the
  // body clearance. Shared with the route certification and the speed policy.
  float tube_response_time_s{0.15F};
  // Stopping law for evidence ahead on the motion: reaction time and the
  // guaranteed deceleration the body stops with. Shared with the speed
  // policy's stopping capability.
  float stopping_response_time_s{0.10F};
  float stopping_deceleration_mps2{4.0F};
  float critical_exposure_tolerance_m{0.5F};
  float planning_exposure_tolerance_m{1.0F};
};

struct FootprintConfig {
  float radius_m{0.0F};
  float lower_extent_m{0.0F};
  float upper_extent_m{0.0F};
  // The physical body inside the envelope; see SweptFootprintConfig.
  float body_radius_m{0.55F};
  float body_lower_extent_m{0.23F};
  float body_upper_extent_m{0.35F};
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
  // Squared normalised depth into the preferred band, per second. It prices
  // position, never motion: a rollout hovering beside a wall pays as much as
  // one flying past it, so a corridor narrower than the band cannot make
  // standing still cheaper than progress. A price per metre *travelled* inside
  // the band used to sit here and did exactly that.
  float clearance_preference_weight{2.0F};
  // The obstacle approach charge, two laws under one weight. Beside the
  // motion, the squared shortfall between the clearance a rollout keeps and
  // the tracking error its speed can accumulate within the tube response
  // time — the law the route certification and the speed policy's clearance
  // limiter apply. Ahead on the motion, the squared shortfall between the free
  // path to the point where the rollout's envelope enters occupied evidence
  // and the path its speed needs to stop within. An isotropic stopping law
  // used to price both from the nearest surface: a wall beside the vehicle
  // then demanded a braking distance it never needed, and the corridor speed
  // was pinned to the floor of the speed policy.
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
