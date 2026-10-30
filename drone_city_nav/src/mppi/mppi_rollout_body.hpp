// The rollout model of the MPPI engine as one body for every backend (roadmap
// item 28, way A): the sampled noise, the dynamics, the body's clearance
// query against the ESDF and the costs of one rollout are written once here,
// host and device, and the CUDA kernels and the CPU engine are thin shells
// around them. The ESDF is read through a lookup functor (a texture on the
// device, an array on the host), the only place the backends differ. Device
// rules hold throughout: no containers, no exceptions, fixed-size arrays.
#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/esdf_grid_3d.hpp"
#include "drone_city_nav/mppi/mppi_altitude_envelope.hpp"
#include "drone_city_nav/mppi/mppi_clearance_cost.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_control_limits.hpp"
#include "drone_city_nav/mppi/mppi_horizon_sampling.hpp"
#include "drone_city_nav/mppi/mppi_route_projection.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

// The public headers above each define and undefine their own host-device
// marker; this one is the body's.
#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE
#endif

namespace drone_city_nav::mppi::rollout {

inline constexpr float kPi{3.14159265358979323846F};
inline constexpr float kInfinity{__builtin_huge_valf()};

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline bool
finiteValue(const float value) {
  return value == value && value != kInfinity && value != -kInfinity;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline int maxOf(const int a,
                                                                       const int b) {
  return a > b ? a : b;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline int minOf(const int a,
                                                                       const int b) {
  return a < b ? a : b;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline float
clampValue(const float value, const float minimum, const float maximum) {
  return fminf(maximum, fmaxf(minimum, value));
}

DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline void clampHorizontal(float& x, float& y,
                                                                    const float limit) {
  const float magnitude = hypotf(x, y);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline void
clampTranslational(float& x, float& y, float& z, const float limit) {
  const float magnitude = hypotf(hypotf(x, y), z);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
    z *= scale;
  }
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline std::uint64_t
mixBits(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline float
uniform01(const std::uint64_t value) {
  return static_cast<float>((mixBits(value) >> 40U) + 1U) * (1.0F / 16777217.0F);
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline float
gaussian(const std::uint64_t first, const std::uint64_t second) {
  const float u1 = fmaxf(uniform01(first), 1.0e-7F);
  return sqrtf(-2.0F * logf(u1)) * cosf(2.0F * kPi * uniform01(second));
}

struct NoiseSample {
  float ax;
  float ay;
  float az;
  float yaw;
};

// The noise of one control sample: a hash of the seed, the tick and the
// index, the same on every backend (no generator state).
[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline NoiseSample
noiseSample(const std::size_t index, const std::uint64_t seed, const std::uint64_t tick,
            const NoiseConfig config) {
  const std::uint64_t base = seed ^ (tick * 0xd1342543de82ef95ULL) ^ index;
  return NoiseSample{
      config.horizontal_acceleration_sigma_mps2 *
          gaussian(base, base ^ 0xa0761d6478bd642fULL),
      config.horizontal_acceleration_sigma_mps2 *
          gaussian(base ^ 0xe7037ed1a0b428dbULL, base ^ 0x8ebc6af09c88c6e3ULL),
      config.vertical_acceleration_sigma_mps2 *
          gaussian(base ^ 0x589965cc75374cc3ULL, base ^ 0x1d8e4e27c47d124fULL),
      config.yaw_acceleration_sigma_radps2 *
          gaussian(base ^ 0xeb44accab455d165ULL, base ^ 0x9e3779b97f4a7c15ULL)};
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline State
integrate(State state, Control control, const DynamicsConfig config) {
  clampHorizontal(control.ax, control.ay, config.maximum_horizontal_acceleration_mps2);
  control.az = clampValue(control.az, -config.maximum_vertical_acceleration_mps2,
                          config.maximum_vertical_acceleration_mps2);
  control.yaw_accel =
      clampValue(control.yaw_accel, -config.maximum_yaw_acceleration_radps2,
                 config.maximum_yaw_acceleration_radps2);
  const float drag = fmaxf(0.0F, 1.0F - config.linear_drag_1ps * config.dt_s);
  const float inherited_horizontal_speed_mps = hypotf(state.vx, state.vy);
  const float inherited_vertical_speed_mps = fabsf(state.vz);
  const float inherited_translational_speed_mps =
      hypotf(inherited_horizontal_speed_mps, inherited_vertical_speed_mps);
  const float inherited_yaw_rate_radps = fabsf(state.yaw_rate);
  state.vx = state.vx * drag + control.ax * config.dt_s;
  state.vy = state.vy * drag + control.ay * config.dt_s;
  state.vz = state.vz * drag + control.az * config.dt_s;
  // Mirrors integrateMotionState3D: an inherited excess above a cap is shed
  // at least as fast as the maximum deceleration allows.
  const float horizontal_shed_mps =
      config.maximum_horizontal_acceleration_mps2 * config.dt_s;
  const float vertical_shed_mps =
      config.maximum_vertical_acceleration_mps2 * config.dt_s;
  clampHorizontal(state.vx, state.vy,
                  fmaxf(config.maximum_horizontal_speed_mps,
                        inherited_horizontal_speed_mps - horizontal_shed_mps));
  const float vertical_speed_limit_mps =
      fmaxf(config.maximum_vertical_speed_mps,
            inherited_vertical_speed_mps - vertical_shed_mps);
  state.vz = clampValue(state.vz, -vertical_speed_limit_mps, vertical_speed_limit_mps);
  clampTranslational(
      state.vx, state.vy, state.vz,
      fmaxf(translationalSpeedLimitAlong3D(config, state.vx, state.vy, state.vz),
            inherited_translational_speed_mps - horizontal_shed_mps));
  const float yaw_rate_limit_radps =
      fmaxf(config.maximum_yaw_rate_radps, inherited_yaw_rate_radps);
  state.yaw_rate = clampValue(state.yaw_rate + control.yaw_accel * config.dt_s,
                              -yaw_rate_limit_radps, yaw_rate_limit_radps);
  state.x += state.vx * config.dt_s;
  state.y += state.vy * config.dt_s;
  state.z += state.vz * config.dt_s;
  state.yaw = remainderf(state.yaw + state.yaw_rate * config.dt_s, 2.0F * kPi);
  return state;
}

struct BodyAxis {
  float x{0.0F};
  float y{0.0F};
  float z{1.0F};
};

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline BodyAxis
normalizeAxis(const BodyAxis axis) {
  const float length = sqrtf(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  return length > 1.0e-6F && finiteValue(length)
             ? BodyAxis{axis.x / length, axis.y / length, axis.z / length}
             : BodyAxis{};
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline BodyAxis
bodyAxisFromControl(const Control control) {
  constexpr float kGravityMps2{9.80665F};
  return normalizeAxis(BodyAxis{control.ax, control.ay, control.az + kGravityMps2});
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline BodyAxis
crossAxis(const BodyAxis first, const BodyAxis second) {
  return BodyAxis{first.y * second.z - first.z * second.y,
                  first.z * second.x - first.x * second.z,
                  first.x * second.y - first.y * second.x};
}

struct EsdfQuery {
  float clearance_m;
  bool unknown_space;
  // The sampled point lies inside a raw occupied voxel: its own distance is
  // exactly zero. Unlike the conservative clearance this is an exact fact.
  bool inside_occupied;
};

// The ESDF cell's stored distance, through the backend's lookup: a texture
// read on the device, an array read on the host, both of cell (x, y, z).
template<typename Lookup>
[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline EsdfQuery
queryEsdfPoint(const float x, const float y, const float z, const EsdfGrid grid,
               const Lookup& lookup) {
  const float cell_x_float = (x - grid.origin_x_m) / grid.resolution_m;
  const float cell_y_float = (y - grid.origin_y_m) / grid.resolution_m;
  const int cell_x = static_cast<int>(floorf(cell_x_float));
  const int cell_y = static_cast<int>(floorf(cell_y_float));
  const int depth = maxOf(1, grid.depth);
  const int cell_z =
      depth > 1 ? static_cast<int>(floorf((z - grid.origin_z_m) / grid.resolution_m))
                : 0;
  if (cell_x < 0 || cell_y < 0 || cell_z < 0 || cell_x >= grid.width ||
      cell_y >= grid.height || cell_z >= depth) {
    return {grid.outside_is_unknown ? 0.0F : kInfinity, grid.outside_is_unknown, false};
  }
  const float center_distance_m = lookup(cell_x, cell_y, cell_z);
  if (center_distance_m == kUnknownEsdfDistanceM) {
    return {0.0F, true, false};
  }
  if (center_distance_m == kInfinity) {
    return {center_distance_m, false, false};
  }
  if (!finiteValue(center_distance_m) || center_distance_m < 0.0F) {
    return {0.0F, true, false};
  }
  const float center_x_m =
      grid.origin_x_m + (static_cast<float>(cell_x) + 0.5F) * grid.resolution_m;
  const float center_y_m =
      grid.origin_y_m + (static_cast<float>(cell_y) + 0.5F) * grid.resolution_m;
  const float center_z_m =
      grid.origin_z_m + (static_cast<float>(cell_z) + 0.5F) * grid.resolution_m;
  const float dx = x - center_x_m;
  const float dy = y - center_y_m;
  const float dz = depth > 1 ? z - center_z_m : 0.0F;
  const float half_diagonal_scale =
      depth > 1 ? 0.86602540378443864676F : 0.70710678118654752440F;
  const float correction_m =
      sqrtf(dx * dx + dy * dy + dz * dz) + half_diagonal_scale * grid.resolution_m;
  return {fmaxf(0.0F, center_distance_m - correction_m), false,
          center_distance_m == 0.0F};
}

template<typename Lookup>
[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline EsdfQuery
queryFootprint(const State& state, const BodyAxis body_axis,
               const FootprintConfig footprint, const float safe_clearance_m,
               const EsdfGrid grid, const Lookup& lookup) {
  EsdfQuery result = queryEsdfPoint(state.x, state.y, state.z, grid, lookup);
  if (result.unknown_space || !(footprint.radius_m > 0.0F) ||
      footprint.perimeter_samples == 0U) {
    return result;
  }
  if (grid.depth <= 1) {
    const float minimum_x = state.x - footprint.radius_m;
    const float maximum_x = state.x + footprint.radius_m;
    const float minimum_y = state.y - footprint.radius_m;
    const float maximum_y = state.y + footprint.radius_m;
    const int minimum_cell_x = maxOf(
        0, static_cast<int>(floorf((minimum_x - grid.origin_x_m) / grid.resolution_m)));
    const int maximum_cell_x = minOf(
        grid.width - 1,
        static_cast<int>(ceilf((maximum_x - grid.origin_x_m) / grid.resolution_m)) - 1);
    const int minimum_cell_y = maxOf(
        0, static_cast<int>(floorf((minimum_y - grid.origin_y_m) / grid.resolution_m)));
    const int maximum_cell_y = minOf(
        grid.height - 1,
        static_cast<int>(ceilf((maximum_y - grid.origin_y_m) / grid.resolution_m)) - 1);
    const float radius_squared = footprint.radius_m * footprint.radius_m;
    for (int cell_y = minimum_cell_y; cell_y <= maximum_cell_y; ++cell_y) {
      for (int cell_x = minimum_cell_x; cell_x <= maximum_cell_x; ++cell_x) {
        const float center_distance_m = lookup(cell_x, cell_y, 0);
        if (center_distance_m == kUnknownEsdfDistanceM) {
          result.unknown_space = true;
          return result;
        }
        if ((!finiteValue(center_distance_m) && center_distance_m != kInfinity) ||
            center_distance_m < 0.0F) {
          result.unknown_space = true;
          return result;
        }
        if (center_distance_m != 0.0F) {
          continue;
        }
        const float cell_minimum_x =
            grid.origin_x_m + static_cast<float>(cell_x) * grid.resolution_m;
        const float cell_minimum_y =
            grid.origin_y_m + static_cast<float>(cell_y) * grid.resolution_m;
        const float nearest_x =
            clampValue(state.x, cell_minimum_x, cell_minimum_x + grid.resolution_m);
        const float nearest_y =
            clampValue(state.y, cell_minimum_y, cell_minimum_y + grid.resolution_m);
        const float dx = state.x - nearest_x;
        const float dy = state.y - nearest_y;
        if (dx * dx + dy * dy <= radius_squared) {
          result.clearance_m = 0.0F;
          result.inside_occupied = true;
        }
      }
    }
    result.clearance_m = fmaxf(0.0F, result.clearance_m - footprint.radius_m);
    return result;
  }
  const BodyAxis axis = normalizeAxis(body_axis);
  const BodyAxis reference =
      fabsf(axis.z) < 0.9F ? BodyAxis{0.0F, 0.0F, 1.0F} : BodyAxis{1.0F, 0.0F, 0.0F};
  const BodyAxis radial_x = normalizeAxis(crossAxis(axis, reference));
  const BodyAxis radial_y = normalizeAxis(crossAxis(axis, radial_x));
  const float bounding_radius_m = hypotf(
      footprint.radius_m, fmaxf(footprint.lower_extent_m, footprint.upper_extent_m));
  if (safe_clearance_m > 0.0F &&
      result.clearance_m - bounding_radius_m >= safe_clearance_m) {
    result.clearance_m -= bounding_radius_m;
    return result;
  }
  const std::uint32_t axial_samples =
      footprint.axial_samples > 2U ? footprint.axial_samples : 2U;
  const std::uint32_t radial_rings =
      footprint.radial_rings > 1U ? footprint.radial_rings : 1U;
  for (std::uint32_t ring = 0U; ring <= radial_rings; ++ring) {
    const std::uint32_t angular_samples = ring == 0U ? 1U : footprint.perimeter_samples;
    const float radial_offset = footprint.radius_m * static_cast<float>(ring) /
                                static_cast<float>(radial_rings);
    for (std::uint32_t angular = 0U; angular < angular_samples; ++angular) {
      const float angle = 2.0F * kPi * static_cast<float>(angular) /
                          static_cast<float>(angular_samples);
      const float sin_angle = sinf(angle);
      const float cos_angle = cosf(angle);
      const float radial_x_offset =
          radial_offset * (cos_angle * radial_x.x + sin_angle * radial_y.x);
      const float radial_y_offset =
          radial_offset * (cos_angle * radial_x.y + sin_angle * radial_y.y);
      const float radial_z_offset =
          radial_offset * (cos_angle * radial_x.z + sin_angle * radial_y.z);
      for (std::uint32_t axial_sample = 0U; axial_sample < axial_samples;
           ++axial_sample) {
        const float axial_ratio =
            static_cast<float>(axial_sample) / static_cast<float>(axial_samples - 1U);
        const float axial_offset =
            -footprint.lower_extent_m +
            axial_ratio * (footprint.lower_extent_m + footprint.upper_extent_m);
        const EsdfQuery query = queryEsdfPoint(
            state.x + axial_offset * axis.x + radial_x_offset,
            state.y + axial_offset * axis.y + radial_y_offset,
            state.z + axial_offset * axis.z + radial_z_offset, grid, lookup);
        result.clearance_m = fminf(result.clearance_m, query.clearance_m);
        result.unknown_space = result.unknown_space || query.unknown_space;
        result.inside_occupied = result.inside_occupied || query.inside_occupied;
        if (result.unknown_space) {
          return result;
        }
      }
    }
  }
  return result;
}

// Everything a rollout reads, by value: the population's buffers (device or
// host memory), the configuration and the tick's scalars.
struct RolloutInputs {
  const float* noise_ax{nullptr};
  const float* noise_ay{nullptr};
  const float* noise_az{nullptr};
  const float* noise_yaw{nullptr};
  const Control* nominal{nullptr};
  std::size_t steps{0U};
  State initial{};
  State target{};
  DynamicsConfig dynamics{};
  RiskConfig risk{};
  FootprintConfig footprint{};
  AltitudeEnvelopeConfig altitude_envelope{};
  CostConfig costs{};
  HorizonSamplingConfig horizon_sampling{};
  EsdfGrid grid{};
  const RouteSample3D* route_points{nullptr};
  std::size_t route_point_count{0U};
  float initial_route_station_m{0.0F};
  const DynamicAircraftSample* dynamic_aircraft_samples{nullptr};
  const float* dynamic_aircraft_radii{nullptr};
  const std::uint32_t* dynamic_aircraft_active_steps{nullptr};
  std::size_t dynamic_aircraft_count{0U};
  DynamicAircraftCostPolicy dynamic_aircraft_cost_policy{};
  CooperativeConfig cooperative{};
  Control cooperative_preferred_acceleration{};
  std::size_t cooperative_preference_steps{0U};
  bool cooperative_preference_enabled{false};
  Control previous_applied_control{};
  float first_control_interval_s{0.0F};
  float reference_speed_mps{-1.0F};
  bool early_exit{false};
  // The sequences simulated in place of nominal plus noise (the repair
  // candidates, the reported sequences); nullptr for the population.
  const Control* direct_controls{nullptr};
};

// What one rollout leaves: the population's arrays at its index.
struct RolloutOutputs {
  float* soft_cost{nullptr};
  float* critical_exposure{nullptr};
  float* planning_exposure{nullptr};
  float* minimum_clearance{nullptr};
  std::uint8_t* altitude_envelope_violation{nullptr};
  std::uint8_t* collision_violation{nullptr};
  std::uint8_t* worst_tier{nullptr};
  RolloutCostTerms* cost_terms{nullptr}; // nullptr: not written
};

// One rollout of the population: the controls (nominal plus the sampled
// noise, or a direct sequence) limited and integrated step by step, the body
// swept against the ESDF, the costs summed, the outputs written at `rollout`.
template<typename Lookup>
DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline void
simulateRollout(const std::size_t rollout, const RolloutInputs& in,
                const RolloutOutputs& out, const Lookup& lookup) {
  const std::size_t steps = in.steps;
  State state = in.initial;
  Control previous = in.previous_applied_control;
  float guide_cost = 0.0F;
  float acceleration_cost = 0.0F;
  float jerk_cost = 0.0F;
  float yaw_cost = 0.0F;
  float altitude_cost = 0.0F;
  float speed_tracking_cost = 0.0F;
  float overspeed_cost = 0.0F;
  float dynamic_aircraft_survival_cost = 0.0F;
  float maneuver_preference_cost = 0.0F;
  float critical_m = 0.0F;
  float planning_m = 0.0F;
  float obstacle_approach_m2_s = 0.0F;
  float stopping_deficit_m2_s = 0.0F;
  float clearance_preference_s = 0.0F;
  float minimum_clearance_m = kInfinity;
  float contact_distance_m = kInfinity;
  float head_speed_mps = 0.0F;
  bool collision_hit = false;
  // Path length and speed of each state, for the stopping law once the first
  // contact is reached. Index 0 is the initial state.
  float trace_station_m[kMaximumHorizonSteps + 1U];
  float trace_speed_mps[kMaximumHorizonSteps + 1U];
  trace_station_m[0] = 0.0F;
  trace_speed_mps[0] = hypotf(hypotf(in.initial.vx, in.initial.vy), in.initial.vz);
  bool altitude_envelope_hit = !altitudeEnvelopeDynamicallyRecoverable(
      in.initial, in.previous_applied_control, in.dynamics, in.altitude_envelope);
  std::uint8_t tier = static_cast<std::uint8_t>(RiskTier::kPreferred);
  const float initial_distance =
      hypotf(hypotf(in.target.x - in.initial.x, in.target.y - in.initial.y),
             in.target.z - in.initial.z);
  float head_progress = 0.0F;
  float terminal_route_progress = 0.0F;
  float route_progress_integral_m_s = 0.0F;
  float traveled_distance_m = 0.0F;
  float rollout_route_station_m = in.initial_route_station_m;
  const std::size_t requested_head_steps = static_cast<std::size_t>(
      ceilf(in.costs.head_progress_horizon_s / in.dynamics.dt_s));
  const std::size_t head_steps =
      requested_head_steps == 0U
          ? 1U
          : (requested_head_steps > steps ? steps : requested_head_steps);
  for (std::size_t step = 0U; step < steps; ++step) {
    const std::size_t index = rollout * steps + step;
    Control control = in.direct_controls != nullptr
                          ? in.direct_controls[index]
                          : Control{in.nominal[step].ax + in.noise_ax[index],
                                    in.nominal[step].ay + in.noise_ay[index],
                                    in.nominal[step].az + in.noise_az[index],
                                    in.nominal[step].yaw_accel + in.noise_yaw[index]};
    control = limitMotionControlStep3D(control, previous, in.dynamics,
                                       step == 0U ? in.first_control_interval_s
                                                  : in.dynamics.dt_s);
    const State previous_state = state;
    state = integrate(state, control, in.dynamics);
    altitude_envelope_hit =
        altitude_envelope_hit || !altitudeEnvelopeDynamicallyRecoverable(
                                     state, control, in.dynamics, in.altitude_envelope);
    const float segment_length_m =
        hypotf(hypotf(state.x - previous_state.x, state.y - previous_state.y),
               state.z - previous_state.z);
    traveled_distance_m += segment_length_m;
    const float validation_step_m = fmaxf(0.05F, 0.5F * in.grid.resolution_m);
    const int validation_samples =
        maxOf(1, static_cast<int>(ceilf(segment_length_m / validation_step_m)));
    float clearance = kInfinity;
    bool step_contact = false;
    const BodyAxis body_axis = bodyAxisFromControl(control);
    for (int sample = 1; sample <= validation_samples; ++sample) {
      const float ratio =
          static_cast<float>(sample) / static_cast<float>(validation_samples);
      State swept_state = state;
      swept_state.x = previous_state.x + ratio * (state.x - previous_state.x);
      swept_state.y = previous_state.y + ratio * (state.y - previous_state.y);
      swept_state.z = previous_state.z + ratio * (state.z - previous_state.z);
      const EsdfQuery esdf_query = queryFootprint(
          swept_state, body_axis, in.footprint,
          in.footprint.clearance_broad_phase_enabled ? in.risk.preferred_distance_m
                                                     : 0.0F,
          in.grid, lookup);
      // The envelope reaching occupied evidence is a physical intersection,
      // not a clearance preference: the raw validators reject such a horizon,
      // so the rollout cannot be executed. It is the same event under the
      // same conservative query, not a sample inside a voxel: a body sample
      // stays outside every voxel across the whole band the validators
      // reject, and a rollout priced by that test dived through a mapped
      // surface at four metres a second paying only the tube law.
      if (!esdf_query.unknown_space) {
        clearance = fminf(clearance, esdf_query.clearance_m);
        step_contact = step_contact || esdf_query.inside_occupied ||
                       !(esdf_query.clearance_m > 0.0F);
      }
    }
    minimum_clearance_m = fminf(minimum_clearance_m, clearance);
    const float segment_speed_mps = hypotf(hypotf(state.vx, state.vy), state.vz);
    const float segment_m = in.dynamics.dt_s * segment_speed_mps;
    if (clearance < in.risk.critical_distance_m) {
      tier = tier > static_cast<std::uint8_t>(RiskTier::kCritical)
                 ? tier
                 : static_cast<std::uint8_t>(RiskTier::kCritical);
      critical_m += segment_m;
    } else if (clearance < in.risk.preferred_distance_m) {
      tier = tier > static_cast<std::uint8_t>(RiskTier::kPlanning)
                 ? tier
                 : static_cast<std::uint8_t>(RiskTier::kPlanning);
      planning_m += segment_m;
    }
    {
      const float depth =
          clearancePreferenceDepth(clearance, in.risk.preferred_distance_m);
      clearance_preference_s += in.dynamics.dt_s * depth * depth;
    }
    // Evidence beside the motion: the tube law at this state's clearance.
    obstacle_approach_m2_s +=
        in.dynamics.dt_s * tubeClearanceDeficitM2(clearance, segment_speed_mps,
                                                  in.risk.tube_response_time_s);
    // Evidence ahead on the motion: once the envelope enters occupied
    // evidence, every state before it owed the stopping law the free path it
    // had to that point.
    if (!collision_hit) {
      trace_station_m[step + 1U] = traveled_distance_m;
      trace_speed_mps[step + 1U] = segment_speed_mps;
      if (step_contact) {
        collision_hit = true;
        contact_distance_m = traveled_distance_m;
        for (std::size_t prior = 0U; prior <= step; ++prior) {
          stopping_deficit_m2_s +=
              in.dynamics.dt_s *
              stoppingDistanceDeficitM2(
                  traveled_distance_m - trace_station_m[prior], trace_speed_mps[prior],
                  in.risk.stopping_response_time_s, in.risk.stopping_deceleration_mps2);
        }
      }
    }
    // A state in contact has no free path at all: it owes the stopping law
    // the whole path its speed needs. Without this a rollout already
    // intersecting paid only the tube law, and with the body collision gate
    // off nothing else priced carrying speed through evidence, so the update
    // kept the speed and the raw validators rejected every horizon it
    // produced until the vehicle braked blind into the surface.
    if (step_contact) {
      stopping_deficit_m2_s +=
          in.dynamics.dt_s *
          stoppingDistanceDeficitM2(0.0F, segment_speed_mps,
                                    in.risk.stopping_response_time_s,
                                    in.risk.stopping_deceleration_mps2);
    }
    for (std::size_t aircraft_index = 0U; aircraft_index < in.dynamic_aircraft_count;
         ++aircraft_index) {
      if (step >= in.dynamic_aircraft_active_steps[aircraft_index]) {
        continue;
      }
      const DynamicAircraftSample aircraft =
          in.dynamic_aircraft_samples[aircraft_index * steps + step];
      const float peer_separation_m = hypotf(
          hypotf(aircraft.x - state.x, aircraft.y - state.y), aircraft.z - state.z);
      DynamicAircraftCostPolicy effective_policy = in.dynamic_aircraft_cost_policy;
      effective_policy.strong_separation_m =
          fmaxf(effective_policy.strong_separation_m,
                in.footprint.radius_m + in.dynamic_aircraft_radii[aircraft_index]);
      effective_policy.anticipation_separation_m =
          fmaxf(effective_policy.anticipation_separation_m,
                effective_policy.strong_separation_m);
      const DynamicAircraftCostContribution contribution =
          dynamicAircraftCostContribution(
              peer_separation_m, static_cast<float>(step + 1U) * in.dynamics.dt_s,
              effective_policy);
      dynamic_aircraft_survival_cost += contribution.strong + contribution.anticipation;
    }
    if (in.cooperative_preference_enabled && step < in.cooperative_preference_steps) {
      const float preference_ax = control.ax - in.cooperative_preferred_acceleration.ax;
      const float preference_ay = control.ay - in.cooperative_preferred_acceleration.ay;
      const float preference_az = control.az - in.cooperative_preferred_acceleration.az;
      maneuver_preference_cost += preference_ax * preference_ax +
                                  preference_ay * preference_ay +
                                  preference_az * preference_az;
    }
    const HorizonCostSample path_cost_sample =
        in.route_point_count >= 2U
            ? horizonCostSample(step, steps, in.dynamics.dt_s,
                                in.costs.head_progress_horizon_s, in.horizon_sampling)
            : HorizonCostSample{1U, true};
    MppiRouteProjection3D route_projection;
    if (path_cost_sample.evaluate && in.route_point_count >= 2U) {
      route_projection = projectOntoMppiRoute3D(
          state, in.route_points, in.route_point_count, rollout_route_station_m);
    }
    if (path_cost_sample.evaluate) {
      const float sample_weight =
          static_cast<float>(path_cost_sample.represented_steps);
      if (route_projection.valid) {
        rollout_route_station_m = route_projection.station_m;
        guide_cost +=
            sample_weight * route_projection.distance_m * route_projection.distance_m;
        terminal_route_progress =
            creditedRouteProgressM(route_projection.station_m,
                                   in.initial_route_station_m, traveled_distance_m);
        route_progress_integral_m_s +=
            sample_weight * in.dynamics.dt_s * terminal_route_progress;
      } else {
        const float guide_x = in.target.x - in.initial.x;
        const float guide_y = in.target.y - in.initial.y;
        const float guide_z = in.target.z - in.initial.z;
        const float offset_x = state.x - in.initial.x;
        const float offset_y = state.y - in.initial.y;
        const float offset_z = state.z - in.initial.z;
        const float cross_x = offset_y * guide_z - offset_z * guide_y;
        const float cross_y = offset_z * guide_x - offset_x * guide_z;
        const float cross_z = offset_x * guide_y - offset_y * guide_x;
        const float guide_length =
            fmaxf(1.0F, hypotf(hypotf(guide_x, guide_y), guide_z));
        const float cross_track =
            hypotf(hypotf(cross_x, cross_y), cross_z) / guide_length;
        guide_cost += sample_weight * cross_track * cross_track;
      }
      const float z_reference =
          route_projection.valid ? route_projection.reference_z_m : in.target.z;
      const float altitude_error = state.z - z_reference;
      altitude_cost += sample_weight * altitude_error * altitude_error;
      const float active_reference_speed_mps =
          route_projection.valid && route_projection.reference_speed_mps > 0.0F
              ? fminf(in.reference_speed_mps, route_projection.reference_speed_mps)
              : in.reference_speed_mps;
      if (active_reference_speed_mps >= 0.0F) {
        const float speed_mps = route_projection.valid
                                    ? routeTrackingSpeedMps(state, route_projection)
                                    : hypotf(hypotf(state.vx, state.vy), state.vz);
        const float speed_error = speed_mps - active_reference_speed_mps;
        speed_tracking_cost += sample_weight * speed_error * speed_error;
        // The reference is the speed the policy can still stop within
        // (a blocked route, a route endpoint, the goal, a turn, the sensor
        // range): the excess above it is shed like the excess above the
        // dynamics caps, so the stopping law ahead outweighs the progress a
        // faster rollout would buy.
        const float reference_excess_mps = fmaxf(0.0F, speed_error);
        overspeed_cost += sample_weight * reference_excess_mps * reference_excess_mps;
      }
      {
        const float horizontal_speed_mps = hypotf(state.vx, state.vy);
        const float translational_speed_mps = hypotf(horizontal_speed_mps, state.vz);
        const float excess_mps = fmaxf(
            0.0F, fmaxf(horizontal_speed_mps - in.dynamics.maximum_horizontal_speed_mps,
                        translational_speed_mps -
                            translationalSpeedLimitAlong3D(in.dynamics, state.vx,
                                                           state.vy, state.vz)));
        overspeed_cost += sample_weight * excess_mps * excess_mps;
      }
    }
    if (step + 1U == head_steps) {
      head_speed_mps = segment_speed_mps;
      head_progress = route_projection.valid
                          ? terminal_route_progress
                          : initial_distance - hypotf(hypotf(in.target.x - state.x,
                                                             in.target.y - state.y),
                                                      in.target.z - state.z);
    }
    acceleration_cost +=
        control.ax * control.ax + control.ay * control.ay + control.az * control.az;
    jerk_cost += (control.ax - previous.ax) * (control.ax - previous.ax) +
                 (control.ay - previous.ay) * (control.ay - previous.ay) +
                 (control.az - previous.az) * (control.az - previous.az);
    yaw_cost += control.yaw_accel * control.yaw_accel;
    previous = control;
    if (altitude_envelope_hit && in.early_exit) {
      break;
    }
  }
  const float terminal_distance = hypotf(
      hypotf(in.target.x - state.x, in.target.y - state.y), in.target.z - state.z);
  const float progress = in.route_point_count >= 2U
                             ? terminal_route_progress
                             : initial_distance - terminal_distance;
  RolloutCostTerms terms;
  terms.head_progress = in.costs.head_progress_weight * -head_progress;
  terms.progress = in.costs.progress_weight * -progress;
  terms.route_progress_integral =
      in.costs.route_progress_integral_weight * -route_progress_integral_m_s;
  terms.speed_tracking =
      in.costs.speed_tracking_weight * in.dynamics.dt_s * speed_tracking_cost;
  terms.overspeed = in.costs.overspeed_weight * in.dynamics.dt_s * overspeed_cost;
  terms.guide_deviation =
      in.costs.guide_deviation_weight * in.dynamics.dt_s * guide_cost;
  terms.altitude_tracking =
      in.costs.altitude_tracking_weight * in.dynamics.dt_s * altitude_cost;
  terms.acceleration =
      in.costs.acceleration_weight * in.dynamics.dt_s * acceleration_cost;
  terms.jerk = in.costs.jerk_weight * jerk_cost;
  terms.yaw_change = in.costs.yaw_change_weight * yaw_cost;
  terms.dynamic_aircraft = in.dynamics.dt_s * dynamic_aircraft_survival_cost;
  terms.maneuver_preference = in.costs.cooperative_maneuver_preference_weight *
                              in.dynamics.dt_s * maneuver_preference_cost;
  terms.clearance_preference =
      in.costs.clearance_preference_weight * clearance_preference_s;
  terms.obstacle_approach = in.costs.obstacle_approach_weight * obstacle_approach_m2_s;
  terms.stopping_deficit = in.costs.obstacle_approach_weight * stopping_deficit_m2_s;
  terms.terminal = in.costs.terminal_weight * terminal_distance;
  terms.soft_cost =
      terms.head_progress + terms.progress + terms.route_progress_integral +
      terms.speed_tracking + terms.overspeed + terms.guide_deviation +
      terms.altitude_tracking + terms.acceleration + terms.jerk + terms.yaw_change +
      terms.dynamic_aircraft + terms.maneuver_preference + terms.clearance_preference +
      terms.obstacle_approach + terms.stopping_deficit + terms.terminal;
  terms.minimum_clearance_m = minimum_clearance_m;
  terms.contact_distance_m = contact_distance_m;
  terms.head_speed_mps = head_speed_mps;
  terms.collision = collision_hit;
  terms.altitude_envelope_violation = altitude_envelope_hit;
  out.soft_cost[rollout] = terms.soft_cost;
  if (out.cost_terms != nullptr) {
    out.cost_terms[rollout] = terms;
  }
  out.critical_exposure[rollout] = critical_m;
  out.planning_exposure[rollout] = planning_m;
  out.minimum_clearance[rollout] = minimum_clearance_m;
  out.altitude_envelope_violation[rollout] = altitude_envelope_hit ? 1U : 0U;
  out.collision_violation[rollout] = collision_hit ? 1U : 0U;
  out.worst_tier[rollout] = tier;
}

// A rollout takes part in the weighted update only when it is executable:
// inside the altitude envelope and, unless the caller lifted that gate because
// every rollout intersected raw occupancy, free of body-sample intersections.
[[nodiscard]] DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline bool
rolloutFeasible(const std::uint8_t altitude_envelope_violation,
                const std::uint8_t collision_violation, const bool ignore_collision) {
  return altitude_envelope_violation == 0U &&
         (ignore_collision || collision_violation == 0U);
}

// The sequence limited in place to the dynamics' caps and the jerk between
// consecutive controls, from the previously applied control.
DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE inline void limitControlSequence(
    Control* controls, const std::size_t steps, const DynamicsConfig dynamics,
    const Control previous_applied_control, const float first_control_interval_s) {
  Control previous = previous_applied_control;
  for (std::size_t step = 0U; step < steps; ++step) {
    const float interval_s = step == 0U ? first_control_interval_s : dynamics.dt_s;
    const float maximum_delta = dynamics.maximum_control_jerk_mps3 * interval_s;
    Control control = controls[step];
    clampHorizontal(control.ax, control.ay,
                    dynamics.maximum_horizontal_acceleration_mps2);
    control.az = clampValue(control.az, -dynamics.maximum_vertical_acceleration_mps2,
                            dynamics.maximum_vertical_acceleration_mps2);
    control.yaw_accel =
        clampValue(control.yaw_accel, -dynamics.maximum_yaw_acceleration_radps2,
                   dynamics.maximum_yaw_acceleration_radps2);
    control.ax = clampValue(control.ax, previous.ax - maximum_delta,
                            previous.ax + maximum_delta);
    control.ay = clampValue(control.ay, previous.ay - maximum_delta,
                            previous.ay + maximum_delta);
    control.az = clampValue(control.az, previous.az - maximum_delta,
                            previous.az + maximum_delta);
    controls[step] = control;
    previous = control;
  }
}

} // namespace drone_city_nav::mppi::rollout

#undef DRONE_CITY_NAV_MPPI_ROLLOUT_HOST_DEVICE
