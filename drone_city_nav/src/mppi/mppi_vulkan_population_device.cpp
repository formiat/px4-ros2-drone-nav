#include "mppi_vulkan_population_device.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>

#include "mppi_engine_host_evaluation.hpp"
#include "shaders/mppi_noise.inc"
#include "shaders/mppi_rollout.inc"
#include "shaders/mppi_update.inc"
#include "vulkan_compute.hpp"

namespace drone_city_nav::mppi::detail {
namespace {

constexpr std::size_t kMaximumRoutePoints{512U};
constexpr std::size_t kMaximumDynamicAircraft{16U};
constexpr std::size_t kRouteFloats{9U};
constexpr std::size_t kTermFloats{22U};
constexpr std::size_t kDirectSequences{kMaximumRepairCandidateCount + 2U};
constexpr std::uint32_t kNoiseLocalSize{256U};
constexpr std::uint32_t kRolloutLocalSize{64U};

// The noise shader's push-constant block, as mppi_noise.comp lays it out.
struct NoisePush {
  std::uint64_t seed;
  std::uint64_t tick;
  std::uint32_t count;
  float horizontal_sigma;
  float vertical_sigma;
  float yaw_sigma;
};

static_assert(sizeof(NoisePush) == 32U);

struct UpdatePush {
  std::uint32_t rollouts;
  std::uint32_t steps;
  float denominator;
};

static_assert(sizeof(UpdatePush) == 12U);

// The tick's scalars, in the order and the 4-byte types of the shader's
// Params block (std430: scalars and float arrays packed).
struct RolloutParams {
  std::uint32_t steps;
  std::uint32_t rollouts;
  std::uint32_t direct;
  std::uint32_t write_terms;
  float initial[8];
  float target[8];
  float dt_s;
  float linear_drag_1ps;
  float maximum_horizontal_acceleration_mps2;
  float maximum_vertical_acceleration_mps2;
  float maximum_horizontal_speed_mps;
  float maximum_vertical_speed_mps;
  float maximum_translational_speed_mps;
  float translational_limit_mps[33];
  float maximum_yaw_acceleration_radps2;
  float maximum_yaw_rate_radps;
  float maximum_control_jerk_mps3;
  float critical_distance_m;
  float preferred_distance_m;
  float tube_response_time_s;
  float stopping_response_time_s;
  float stopping_deceleration_mps2;
  float footprint_radius_m;
  float footprint_lower_extent_m;
  float footprint_upper_extent_m;
  std::uint32_t perimeter_samples;
  std::uint32_t radial_rings;
  std::uint32_t axial_samples;
  std::uint32_t clearance_broad_phase_enabled;
  float minimum_z_m;
  float maximum_z_m;
  float guaranteed_vertical_deceleration_mps2;
  float reaction_latency_s;
  float guide_deviation_weight;
  float altitude_tracking_weight;
  float head_progress_horizon_s;
  float head_progress_weight;
  float progress_weight;
  float route_progress_integral_weight;
  float speed_tracking_weight;
  float overspeed_weight;
  float acceleration_weight;
  float jerk_weight;
  float yaw_change_weight;
  float cooperative_maneuver_preference_weight;
  float terminal_weight;
  float clearance_preference_weight;
  float obstacle_approach_weight;
  float full_rate_duration_s;
  std::uint32_t far_cost_stride;
  std::int32_t grid_width;
  std::int32_t grid_height;
  std::int32_t grid_depth;
  float grid_resolution_m;
  float grid_origin_x_m;
  float grid_origin_y_m;
  float grid_origin_z_m;
  std::uint32_t outside_is_unknown;
  std::uint32_t route_point_count;
  float initial_route_station_m;
  std::uint32_t dynamic_aircraft_count;
  float strong_separation_m;
  float anticipation_separation_m;
  float strong_weight;
  float anticipation_weight;
  float time_to_collision_gain_s;
  float maximum_time_to_collision_multiplier;
  float cooperative_preferred[4];
  std::uint32_t cooperative_preference_steps;
  std::uint32_t cooperative_preference_enabled;
  float previous_applied[4];
  float first_control_interval_s;
  float reference_speed_mps;
  std::uint32_t early_exit;
};

static_assert(sizeof(RolloutParams) == 126U * 4U);

void writeState(float (&out)[8], const State& state) {
  out[0] = state.x;
  out[1] = state.y;
  out[2] = state.z;
  out[3] = state.vx;
  out[4] = state.vy;
  out[5] = state.vz;
  out[6] = state.yaw;
  out[7] = state.yaw_rate;
}

void writeControl(float (&out)[4], const Control& control) {
  out[0] = control.ax;
  out[1] = control.ay;
  out[2] = control.az;
  out[3] = control.yaw_accel;
}

[[nodiscard]] RolloutParams rolloutParams(const std::size_t rollouts,
                                          const rollout::RolloutInputs& in,
                                          const bool direct, const bool write_terms) {
  RolloutParams p{};
  p.steps = static_cast<std::uint32_t>(in.steps);
  p.rollouts = static_cast<std::uint32_t>(rollouts);
  p.direct = direct ? 1U : 0U;
  p.write_terms = write_terms ? 1U : 0U;
  writeState(p.initial, in.initial);
  writeState(p.target, in.target);
  const DynamicsConfig& d = in.dynamics;
  p.dt_s = d.dt_s;
  p.linear_drag_1ps = d.linear_drag_1ps;
  p.maximum_horizontal_acceleration_mps2 = d.maximum_horizontal_acceleration_mps2;
  p.maximum_vertical_acceleration_mps2 = d.maximum_vertical_acceleration_mps2;
  p.maximum_horizontal_speed_mps = d.maximum_horizontal_speed_mps;
  p.maximum_vertical_speed_mps = d.maximum_vertical_speed_mps;
  p.maximum_translational_speed_mps = d.maximum_translational_speed_mps;
  std::copy_n(d.translational_speed_limit_by_vertical_share.limit_mps, 33U,
              p.translational_limit_mps);
  p.maximum_yaw_acceleration_radps2 = d.maximum_yaw_acceleration_radps2;
  p.maximum_yaw_rate_radps = d.maximum_yaw_rate_radps;
  p.maximum_control_jerk_mps3 = d.maximum_control_jerk_mps3;
  p.critical_distance_m = in.risk.critical_distance_m;
  p.preferred_distance_m = in.risk.preferred_distance_m;
  p.tube_response_time_s = in.risk.tube_response_time_s;
  p.stopping_response_time_s = in.risk.stopping_response_time_s;
  p.stopping_deceleration_mps2 = in.risk.stopping_deceleration_mps2;
  p.footprint_radius_m = in.footprint.radius_m;
  p.footprint_lower_extent_m = in.footprint.lower_extent_m;
  p.footprint_upper_extent_m = in.footprint.upper_extent_m;
  p.perimeter_samples = in.footprint.perimeter_samples;
  p.radial_rings = in.footprint.radial_rings;
  p.axial_samples = in.footprint.axial_samples;
  p.clearance_broad_phase_enabled =
      in.footprint.clearance_broad_phase_enabled ? 1U : 0U;
  p.minimum_z_m = in.altitude_envelope.minimum_z_m;
  p.maximum_z_m = in.altitude_envelope.maximum_z_m;
  p.guaranteed_vertical_deceleration_mps2 =
      in.altitude_envelope.guaranteed_vertical_deceleration_mps2;
  p.reaction_latency_s = in.altitude_envelope.reaction_latency_s;
  const CostConfig& c = in.costs;
  p.guide_deviation_weight = c.guide_deviation_weight;
  p.altitude_tracking_weight = c.altitude_tracking_weight;
  p.head_progress_horizon_s = c.head_progress_horizon_s;
  p.head_progress_weight = c.head_progress_weight;
  p.progress_weight = c.progress_weight;
  p.route_progress_integral_weight = c.route_progress_integral_weight;
  p.speed_tracking_weight = c.speed_tracking_weight;
  p.overspeed_weight = c.overspeed_weight;
  p.acceleration_weight = c.acceleration_weight;
  p.jerk_weight = c.jerk_weight;
  p.yaw_change_weight = c.yaw_change_weight;
  p.cooperative_maneuver_preference_weight = c.cooperative_maneuver_preference_weight;
  p.terminal_weight = c.terminal_weight;
  p.clearance_preference_weight = c.clearance_preference_weight;
  p.obstacle_approach_weight = c.obstacle_approach_weight;
  p.full_rate_duration_s = in.horizon_sampling.full_rate_duration_s;
  p.far_cost_stride = in.horizon_sampling.far_cost_stride;
  p.grid_width = in.grid.width;
  p.grid_height = in.grid.height;
  p.grid_depth = in.grid.depth;
  p.grid_resolution_m = in.grid.resolution_m;
  p.grid_origin_x_m = in.grid.origin_x_m;
  p.grid_origin_y_m = in.grid.origin_y_m;
  p.grid_origin_z_m = in.grid.origin_z_m;
  p.outside_is_unknown = in.grid.outside_is_unknown ? 1U : 0U;
  p.route_point_count = static_cast<std::uint32_t>(in.route_point_count);
  p.initial_route_station_m = in.initial_route_station_m;
  p.dynamic_aircraft_count = static_cast<std::uint32_t>(in.dynamic_aircraft_count);
  const DynamicAircraftCostPolicy& policy = in.dynamic_aircraft_cost_policy;
  p.strong_separation_m = policy.strong_separation_m;
  p.anticipation_separation_m = policy.anticipation_separation_m;
  p.strong_weight = policy.strong_weight;
  p.anticipation_weight = policy.anticipation_weight;
  p.time_to_collision_gain_s = policy.time_to_collision_gain_s;
  p.maximum_time_to_collision_multiplier = policy.maximum_time_to_collision_multiplier;
  writeControl(p.cooperative_preferred, in.cooperative_preferred_acceleration);
  p.cooperative_preference_steps =
      static_cast<std::uint32_t>(in.cooperative_preference_steps);
  p.cooperative_preference_enabled = in.cooperative_preference_enabled ? 1U : 0U;
  writeControl(p.previous_applied, in.previous_applied_control);
  p.first_control_interval_s = in.first_control_interval_s;
  p.reference_speed_mps = in.reference_speed_mps;
  p.early_exit = in.early_exit ? 1U : 0U;
  return p;
}

} // namespace

class VulkanPopulationDevice::Impl {
public:
  explicit Impl(const BenchmarkConfig& config)
      : rollouts_{config.rollouts},
        steps_{config.steps},
        noise_pipeline_{device_, kMppiNoiseSpv, 1U, sizeof(NoisePush)},
        rollout_pipeline_{device_, kMppiRolloutSpv, 10U, 0U},
        update_pipeline_{device_, kMppiUpdateSpv, 4U, sizeof(UpdatePush)},
        params_{device_, sizeof(RolloutParams), true},
        esdf_{std::make_unique<vulkan::HostBuffer>(device_, sizeof(float), true)},
        nominal_{device_, steps_ * 4U * sizeof(float), true},
        noise_{device_, 4U * rollouts_ * steps_ * sizeof(float), true},
        route_{device_, kMaximumRoutePoints * kRouteFloats * sizeof(float), true},
        aircraft_{device_,
                  (kMaximumDynamicAircraft * steps_ * 3U + 32U) * sizeof(float), true},
        direct_{device_, kDirectSequences * steps_ * 4U * sizeof(float), true},
        weights_{device_, rollouts_ * sizeof(float), true},
        out_float_{device_, 4U * rollouts_ * sizeof(float), false},
        out_uint_{device_, 3U * rollouts_ * sizeof(std::uint32_t), false},
        terms_{device_, kTermFloats * rollouts_ * sizeof(float), false},
        updated_{device_, steps_ * 4U * sizeof(float), false} {
    if (!device_.shaderInt64()) {
      throw std::runtime_error{
          "Vulkan: the device has no 64-bit integers for the noise's hash"};
    }
    if (steps_ > kMaximumHorizonSteps || steps_ == 0U || rollouts_ == 0U) {
      throw std::invalid_argument{"invalid MPPI engine configuration"};
    }
  }

  void uploadEsdf(const EsdfGrid& grid, const std::span<const float> distances) {
    const std::size_t bytes = distances.size() * sizeof(float);
    if (esdf_->bytes() < bytes) {
      esdf_.reset();
      esdf_ = std::make_unique<vulkan::HostBuffer>(device_, bytes, true);
    }
    std::memcpy(esdf_->data(), distances.data(), bytes);
    grid_ = grid;
  }

  void generateNoise(const std::size_t count, const std::size_t steps,
                     const std::uint64_t seed, const std::uint64_t tick,
                     const NoiseConfig& config,
                     const std::span<const Control> candidate_row) {
    if (count > rollouts_ * steps_ || steps != steps_) {
      throw std::invalid_argument{"Vulkan noise: count exceeds the capacity"};
    }
    noise_count_ = count;
    const NoisePush push{seed,
                         tick,
                         static_cast<std::uint32_t>(count),
                         config.horizontal_acceleration_sigma_mps2,
                         config.vertical_acceleration_sigma_mps2,
                         config.yaw_acceleration_sigma_radps2};
    const std::array<vulkan::HostBuffer*, 1U> buffers{&noise_};
    noise_pipeline_.dispatch(
        buffers, &push, sizeof(push),
        static_cast<std::uint32_t>((count + kNoiseLocalSize - 1U) / kNoiseLocalSize));
    if (!candidate_row.empty()) {
      float* const noise = static_cast<float*>(noise_.data());
      for (std::size_t step = 0U; step < steps_; ++step) {
        noise[step] = candidate_row[step].ax;
        noise[count + step] = candidate_row[step].ay;
        noise[2U * count + step] = candidate_row[step].az;
        noise[3U * count + step] = candidate_row[step].yaw_accel;
      }
    }
  }

  void simulate(const std::size_t rollouts, const rollout::RolloutInputs& in,
                const std::span<const Control> direct_controls,
                const PopulationOutputs& out) {
    const bool direct = !direct_controls.empty();
    if (rollouts > rollouts_ || (direct && rollouts > kDirectSequences) ||
        (!direct && rollouts * steps_ != noise_count_) ||
        in.route_point_count > kMaximumRoutePoints ||
        in.dynamic_aircraft_count > kMaximumDynamicAircraft) {
      throw std::invalid_argument{"Vulkan rollouts: a count exceeds the capacity"};
    }
    const RolloutParams params =
        rolloutParams(rollouts, in, direct, out.cost_terms != nullptr);
    std::memcpy(params_.data(), &params, sizeof(params));
    std::memcpy(nominal_.data(), in.nominal, steps_ * sizeof(Control));
    if (in.route_point_count > 0U) {
      float* const route = static_cast<float*>(route_.data());
      for (std::size_t index = 0U; index < in.route_point_count; ++index) {
        const RouteSample3D& point = in.route_points[index];
        float* const at = route + index * kRouteFloats;
        at[0] = point.x_m;
        at[1] = point.y_m;
        at[2] = point.z_m;
        at[3] = point.tangent_x;
        at[4] = point.tangent_y;
        at[5] = point.tangent_z;
        at[6] = point.station_m;
        at[7] = point.reference_speed_mps;
        at[8] = static_cast<float>(point.required_risk_tier);
      }
    }
    if (in.dynamic_aircraft_count > 0U) {
      float* const aircraft = static_cast<float*>(aircraft_.data());
      std::memcpy(aircraft, in.dynamic_aircraft_samples,
                  in.dynamic_aircraft_count * steps_ * sizeof(DynamicAircraftSample));
      float* const radii = aircraft + kMaximumDynamicAircraft * steps_ * 3U;
      float* const active = radii + kMaximumDynamicAircraft;
      for (std::size_t index = 0U; index < in.dynamic_aircraft_count; ++index) {
        radii[index] = in.dynamic_aircraft_radii[index];
        active[index] = static_cast<float>(in.dynamic_aircraft_active_steps[index]);
      }
    }
    if (direct) {
      std::memcpy(direct_.data(), direct_controls.data(),
                  rollouts * steps_ * sizeof(Control));
    }
    const std::array<vulkan::HostBuffer*, 10U> buffers{
        &params_,   esdf_.get(), &nominal_,   &noise_,    &route_,
        &aircraft_, &direct_,    &out_float_, &out_uint_, &terms_};
    rollout_pipeline_.dispatch(
        buffers, nullptr, 0U,
        static_cast<std::uint32_t>((rollouts + kRolloutLocalSize - 1U) /
                                   kRolloutLocalSize));
    const float* const floats = static_cast<const float*>(out_float_.data());
    std::memcpy(out.soft_cost, floats, rollouts * sizeof(float));
    std::memcpy(out.critical_exposure, floats + rollouts, rollouts * sizeof(float));
    std::memcpy(out.planning_exposure, floats + 2U * rollouts,
                rollouts * sizeof(float));
    std::memcpy(out.minimum_clearance, floats + 3U * rollouts,
                rollouts * sizeof(float));
    const std::uint32_t* const uints =
        static_cast<const std::uint32_t*>(out_uint_.data());
    for (std::size_t index = 0U; index < rollouts; ++index) {
      out.altitude_envelope_violation[index] = static_cast<std::uint8_t>(uints[index]);
      out.collision_violation[index] =
          static_cast<std::uint8_t>(uints[rollouts + index]);
      out.worst_tier[index] = static_cast<std::uint8_t>(uints[2U * rollouts + index]);
    }
    if (out.cost_terms != nullptr) {
      const float* const terms = static_cast<const float*>(terms_.data());
      for (std::size_t index = 0U; index < rollouts; ++index) {
        const float* const t = terms + index * kTermFloats;
        RolloutCostTerms& r = out.cost_terms[index];
        r.head_progress = t[0];
        r.progress = t[1];
        r.route_progress_integral = t[2];
        r.speed_tracking = t[3];
        r.overspeed = t[4];
        r.guide_deviation = t[5];
        r.altitude_tracking = t[6];
        r.acceleration = t[7];
        r.jerk = t[8];
        r.yaw_change = t[9];
        r.dynamic_aircraft = t[10];
        r.maneuver_preference = t[11];
        r.clearance_preference = t[12];
        r.obstacle_approach = t[13];
        r.stopping_deficit = t[14];
        r.terminal = t[15];
        r.soft_cost = t[16];
        r.minimum_clearance_m = t[17];
        r.contact_distance_m = t[18];
        r.head_speed_mps = t[19];
        r.collision = t[20] != 0.0F;
        r.altitude_envelope_violation = t[21] != 0.0F;
      }
    }
  }

  void weightedUpdate(const std::span<const float> weights,
                      const std::span<const Control> nominal, const float denominator,
                      const std::span<Control> updated) {
    const std::size_t rollouts = weights.size();
    if (rollouts * steps_ != noise_count_ || nominal.size() != steps_ ||
        updated.size() != steps_) {
      throw std::invalid_argument{"Vulkan update: the population's shape differs"};
    }
    std::memcpy(weights_.data(), weights.data(), rollouts * sizeof(float));
    std::memcpy(nominal_.data(), nominal.data(), steps_ * sizeof(Control));
    const UpdatePush push{static_cast<std::uint32_t>(rollouts),
                          static_cast<std::uint32_t>(steps_), denominator};
    const std::array<vulkan::HostBuffer*, 4U> buffers{&noise_, &weights_, &nominal_,
                                                      &updated_};
    update_pipeline_.dispatch(buffers, &push, sizeof(push),
                              static_cast<std::uint32_t>(steps_));
    std::memcpy(updated.data(), updated_.data(), steps_ * sizeof(Control));
  }

  void rolloutControls(const std::size_t rollout,
                       const std::span<const Control> nominal,
                       const std::span<Control> controls) {
    const float* const noise = static_cast<const float*>(noise_.data());
    for (std::size_t step = 0U; step < steps_; ++step) {
      const std::size_t at = rollout * steps_ + step;
      controls[step] = Control{nominal[step].ax + noise[at],
                               nominal[step].ay + noise[noise_count_ + at],
                               nominal[step].az + noise[2U * noise_count_ + at],
                               nominal[step].yaw_accel + noise[3U * noise_count_ + at]};
    }
  }

  [[nodiscard]] std::size_t allocatedBytes() const noexcept {
    return params_.bytes() + esdf_->bytes() + nominal_.bytes() + noise_.bytes() +
           route_.bytes() + aircraft_.bytes() + direct_.bytes() + weights_.bytes() +
           out_float_.bytes() + out_uint_.bytes() + terms_.bytes() + updated_.bytes();
  }

  [[nodiscard]] const std::string& deviceName() const noexcept {
    return device_.deviceName();
  }

private:
  std::size_t rollouts_;
  std::size_t steps_;
  std::size_t noise_count_{0U};
  EsdfGrid grid_{};
  vulkan::ComputeDevice device_;
  vulkan::ComputePipeline noise_pipeline_;
  vulkan::ComputePipeline rollout_pipeline_;
  vulkan::ComputePipeline update_pipeline_;
  vulkan::HostBuffer params_;
  std::unique_ptr<vulkan::HostBuffer> esdf_;
  vulkan::HostBuffer nominal_;
  vulkan::HostBuffer noise_;
  vulkan::HostBuffer route_;
  vulkan::HostBuffer aircraft_;
  vulkan::HostBuffer direct_;
  vulkan::HostBuffer weights_;
  vulkan::HostBuffer out_float_;
  vulkan::HostBuffer out_uint_;
  vulkan::HostBuffer terms_;
  vulkan::HostBuffer updated_;
};

VulkanPopulationDevice::VulkanPopulationDevice(const BenchmarkConfig& config)
    : impl_{std::make_unique<Impl>(config)} {
}

VulkanPopulationDevice::~VulkanPopulationDevice() = default;

void VulkanPopulationDevice::uploadEsdf(const EsdfGrid& grid,
                                        const std::span<const float> distances) {
  impl_->uploadEsdf(grid, distances);
}

void VulkanPopulationDevice::generateNoise(
    const std::size_t count, const std::size_t steps, const std::uint64_t seed,
    const std::uint64_t tick, const NoiseConfig& config,
    const std::span<const Control> candidate_row) {
  impl_->generateNoise(count, steps, seed, tick, config, candidate_row);
}

void VulkanPopulationDevice::simulate(const std::size_t rollouts,
                                      const rollout::RolloutInputs& inputs,
                                      const std::span<const Control> direct_controls,
                                      const PopulationOutputs& outputs) {
  impl_->simulate(rollouts, inputs, direct_controls, outputs);
}

void VulkanPopulationDevice::weightedUpdate(const std::span<const float> weights,
                                            const std::span<const Control> nominal,
                                            const float denominator,
                                            const std::span<Control> updated) {
  impl_->weightedUpdate(weights, nominal, denominator, updated);
}

void VulkanPopulationDevice::rolloutControls(const std::size_t rollout,
                                             const std::span<const Control> nominal,
                                             const std::span<Control> controls) {
  impl_->rolloutControls(rollout, nominal, controls);
}

std::size_t VulkanPopulationDevice::allocatedBytes() const noexcept {
  return impl_->allocatedBytes();
}

const std::string& VulkanPopulationDevice::deviceName() const noexcept {
  return impl_->deviceName();
}

} // namespace drone_city_nav::mppi::detail
