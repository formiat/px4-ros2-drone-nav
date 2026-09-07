#pragma once

__device__ float clampValue(const float value, const float minimum,
                            const float maximum) {
  return fminf(maximum, fmaxf(minimum, value));
}

__device__ void clampHorizontal(float& x, float& y, const float limit) {
  const float magnitude = hypotf(x, y);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

__device__ void clampTranslational(float& x, float& y, float& z, const float limit) {
  const float magnitude = hypotf(hypotf(x, y), z);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
    z *= scale;
  }
}

__device__ std::uint64_t mixBits(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

__device__ float uniform01(const std::uint64_t value) {
  return static_cast<float>((mixBits(value) >> 40U) + 1U) * (1.0F / 16777217.0F);
}

__device__ float gaussian(const std::uint64_t first, const std::uint64_t second) {
  const float u1 = fmaxf(uniform01(first), 1.0e-7F);
  return sqrtf(-2.0F * logf(u1)) * cosf(2.0F * kPi * uniform01(second));
}

__global__ void generateNoise(float* noise_ax, float* noise_ay, float* noise_az,
                              float* noise_yaw, std::size_t count, std::uint64_t seed,
                              std::uint64_t tick, NoiseConfig config) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const std::uint64_t base = seed ^ (tick * 0xd1342543de82ef95ULL) ^ index;
  noise_ax[index] = config.horizontal_acceleration_sigma_mps2 *
                    gaussian(base, base ^ 0xa0761d6478bd642fULL);
  noise_ay[index] =
      config.horizontal_acceleration_sigma_mps2 *
      gaussian(base ^ 0xe7037ed1a0b428dbULL, base ^ 0x8ebc6af09c88c6e3ULL);
  noise_az[index] =
      config.vertical_acceleration_sigma_mps2 *
      gaussian(base ^ 0x589965cc75374cc3ULL, base ^ 0x1d8e4e27c47d124fULL);
  noise_yaw[index] =
      config.yaw_acceleration_sigma_radps2 *
      gaussian(base ^ 0xeb44accab455d165ULL, base ^ 0x9e3779b97f4a7c15ULL);
}

__device__ State integrate(State state, Control control, DynamicsConfig config) {
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
  clampTranslational(state.vx, state.vy, state.vz,
                     fmaxf(config.maximum_translational_speed_mps,
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

// The device rollouts and the host sampler share one admissible-control law:
// see limitMotionControlStep3D.
__device__ Control limitControlStep(Control control, const Control previous,
                                    const DynamicsConfig config,
                                    const float interval_s) {
  return limitMotionControlStep3D(control, previous, config, interval_s);
}

struct DeviceBodyAxis {
  float x{0.0F};
  float y{0.0F};
  float z{1.0F};
};

__device__ DeviceBodyAxis normalizeAxis(const DeviceBodyAxis axis) {
  const float length = sqrtf(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  return length > 1.0e-6F && isfinite(length)
             ? DeviceBodyAxis{axis.x / length, axis.y / length, axis.z / length}
             : DeviceBodyAxis{};
}

__device__ DeviceBodyAxis bodyAxisFromControl(const Control control) {
  constexpr float kGravityMps2{9.80665F};
  return normalizeAxis(
      DeviceBodyAxis{control.ax, control.ay, control.az + kGravityMps2});
}

struct DeviceEsdfQuery {
  float clearance_m;
  bool unknown_space;
  // The sampled point lies inside a raw occupied voxel: its own distance is
  // exactly zero. Unlike the conservative clearance this is an exact fact.
  bool inside_occupied;
};

__device__ DeviceEsdfQuery queryEsdfPoint(const float x, const float y, const float z,
                                          const EsdfGrid grid,
                                          const cudaTextureObject_t esdf_texture) {
  const float cell_x_float = (x - grid.origin_x_m) / grid.resolution_m;
  const float cell_y_float = (y - grid.origin_y_m) / grid.resolution_m;
  const int cell_x = static_cast<int>(floorf(cell_x_float));
  const int cell_y = static_cast<int>(floorf(cell_y_float));
  const int depth = max(1, grid.depth);
  const int cell_z =
      depth > 1 ? static_cast<int>(floorf((z - grid.origin_z_m) / grid.resolution_m))
                : 0;
  if (cell_x < 0 || cell_y < 0 || cell_z < 0 || cell_x >= grid.width ||
      cell_y >= grid.height || cell_z >= depth) {
    return {grid.outside_is_unknown ? 0.0F : kInfinity, grid.outside_is_unknown, false};
  }
  const float center_distance_m = tex3D<float>(
      esdf_texture, static_cast<float>(cell_x) + 0.5F,
      static_cast<float>(cell_y) + 0.5F, static_cast<float>(cell_z) + 0.5F);
  if (center_distance_m == kUnknownEsdfDistanceM) {
    return {0.0F, true, false};
  }
  if (isinf(center_distance_m) && center_distance_m > 0.0F) {
    return {center_distance_m, false, false};
  }
  if (!isfinite(center_distance_m) || center_distance_m < 0.0F) {
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

__device__ DeviceBodyAxis crossAxis(const DeviceBodyAxis first,
                                    const DeviceBodyAxis second) {
  return DeviceBodyAxis{first.y * second.z - first.z * second.y,
                        first.z * second.x - first.x * second.z,
                        first.x * second.y - first.y * second.x};
}

__device__ DeviceEsdfQuery queryFootprint(const State& state,
                                          const DeviceBodyAxis body_axis,
                                          const FootprintConfig footprint,
                                          const float safe_clearance_m,
                                          const EsdfGrid grid,
                                          const cudaTextureObject_t esdf_texture) {
  DeviceEsdfQuery result =
      queryEsdfPoint(state.x, state.y, state.z, grid, esdf_texture);
  if (result.unknown_space || !(footprint.radius_m > 0.0F) ||
      footprint.perimeter_samples == 0U) {
    return result;
  }
  if (grid.depth <= 1) {
    const float minimum_x = state.x - footprint.radius_m;
    const float maximum_x = state.x + footprint.radius_m;
    const float minimum_y = state.y - footprint.radius_m;
    const float maximum_y = state.y + footprint.radius_m;
    const int minimum_cell_x = max(
        0, static_cast<int>(floorf((minimum_x - grid.origin_x_m) / grid.resolution_m)));
    const int maximum_cell_x = min(
        grid.width - 1,
        static_cast<int>(ceilf((maximum_x - grid.origin_x_m) / grid.resolution_m)) - 1);
    const int minimum_cell_y = max(
        0, static_cast<int>(floorf((minimum_y - grid.origin_y_m) / grid.resolution_m)));
    const int maximum_cell_y = min(
        grid.height - 1,
        static_cast<int>(ceilf((maximum_y - grid.origin_y_m) / grid.resolution_m)) - 1);
    const float radius_squared = footprint.radius_m * footprint.radius_m;
    for (int cell_y = minimum_cell_y; cell_y <= maximum_cell_y; ++cell_y) {
      for (int cell_x = minimum_cell_x; cell_x <= maximum_cell_x; ++cell_x) {
        const float center_distance_m =
            tex3D<float>(esdf_texture, static_cast<float>(cell_x) + 0.5F,
                         static_cast<float>(cell_y) + 0.5F, 0.5F);
        if (center_distance_m == kUnknownEsdfDistanceM) {
          result.unknown_space = true;
          return result;
        }
        if ((!isfinite(center_distance_m) &&
             !(isinf(center_distance_m) && center_distance_m > 0.0F)) ||
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
  const DeviceBodyAxis axis = normalizeAxis(body_axis);
  const DeviceBodyAxis reference = fabsf(axis.z) < 0.9F
                                       ? DeviceBodyAxis{0.0F, 0.0F, 1.0F}
                                       : DeviceBodyAxis{1.0F, 0.0F, 0.0F};
  const DeviceBodyAxis radial_x = normalizeAxis(crossAxis(axis, reference));
  const DeviceBodyAxis radial_y = normalizeAxis(crossAxis(axis, radial_x));
  const float bounding_radius_m = hypotf(
      footprint.radius_m, fmaxf(footprint.lower_extent_m, footprint.upper_extent_m));
  if (safe_clearance_m > 0.0F &&
      result.clearance_m - bounding_radius_m >= safe_clearance_m) {
    result.clearance_m -= bounding_radius_m;
    return result;
  }
  const std::uint32_t axial_samples = max(2U, footprint.axial_samples);
  const std::uint32_t radial_rings = max(1U, footprint.radial_rings);
  for (std::uint32_t ring = 0U; ring <= radial_rings; ++ring) {
    const std::uint32_t angular_samples = ring == 0U ? 1U : footprint.perimeter_samples;
    const float radial_offset = footprint.radius_m * static_cast<float>(ring) /
                                static_cast<float>(radial_rings);
    for (std::uint32_t angular = 0U; angular < angular_samples; ++angular) {
      const float angle = 2.0F * kPi * static_cast<float>(angular) /
                          static_cast<float>(angular_samples);
      float sin_angle = 0.0F;
      float cos_angle = 0.0F;
      sincosf(angle, &sin_angle, &cos_angle);
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
        const DeviceEsdfQuery query = queryEsdfPoint(
            state.x + axial_offset * axis.x + radial_x_offset,
            state.y + axial_offset * axis.y + radial_y_offset,
            state.z + axial_offset * axis.z + radial_z_offset, grid, esdf_texture);
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

__global__ void
simulate(const float* noise_ax, const float* noise_ay, const float* noise_az,
         const float* noise_yaw, const Control* nominal, float* soft_cost,
         float* critical_exposure, float* planning_exposure, float* minimum_clearance,
         std::uint8_t* altitude_envelope_violation, std::uint8_t* collision_violation,
         std::uint8_t* worst_tier, std::size_t rollouts, std::size_t steps,
         State initial, State target, MovingTargetReference moving_target,
         bool moving_target_enabled, DynamicsConfig dynamics, RiskConfig risk,
         FootprintConfig footprint, AltitudeEnvelopeConfig altitude_envelope,
         CostConfig costs, HorizonSamplingConfig horizon_sampling, EsdfGrid grid,
         cudaTextureObject_t esdf_texture, const RouteSample3D* route_points,
         std::size_t route_point_count, float initial_route_station_m,
         const DynamicAircraftSample* dynamic_aircraft_samples,
         const float* dynamic_aircraft_radii,
         const std::uint32_t* dynamic_aircraft_active_steps,
         std::size_t dynamic_aircraft_count,
         DynamicAircraftCostPolicy dynamic_aircraft_cost_policy,
         CooperativeConfig cooperative, Control cooperative_preferred_acceleration,
         std::size_t cooperative_preference_steps, bool cooperative_preference_enabled,
         Control previous_applied_control, float first_control_interval_s,
         float reference_speed_mps, bool early_exit, const Control* direct_controls,
         RolloutCostTerms* cost_terms) {
  const std::size_t rollout =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (rollout >= rollouts) {
    return;
  }
  State state = initial;
  Control previous = previous_applied_control;
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
  trace_speed_mps[0] = hypotf(hypotf(initial.vx, initial.vy), initial.vz);
  bool altitude_envelope_hit = !altitudeEnvelopeDynamicallyRecoverable(
      initial, previous_applied_control, dynamics, altitude_envelope);
  std::uint8_t tier = static_cast<std::uint8_t>(RiskTier::kPreferred);
  const float initial_distance =
      moving_target_enabled ? hypotf(hypotf(moving_target.state.x - initial.x,
                                            moving_target.state.y - initial.y),
                                     moving_target.state.z - initial.z)
                            : hypotf(hypotf(target.x - initial.x, target.y - initial.y),
                                     target.z - initial.z);
  float minimum_target_separation_m = initial_distance;
  float head_progress = 0.0F;
  float terminal_route_progress = 0.0F;
  float route_progress_integral_m_s = 0.0F;
  float traveled_distance_m = 0.0F;
  float rollout_route_station_m = initial_route_station_m;
  const std::size_t requested_head_steps =
      static_cast<std::size_t>(ceilf(costs.head_progress_horizon_s / dynamics.dt_s));
  const std::size_t head_steps =
      requested_head_steps == 0U
          ? 1U
          : (requested_head_steps > steps ? steps : requested_head_steps);
  for (std::size_t step = 0U; step < steps; ++step) {
    const std::size_t index = rollout * steps + step;
    Control control = direct_controls != nullptr
                          ? direct_controls[index]
                          : Control{nominal[step].ax + noise_ax[index],
                                    nominal[step].ay + noise_ay[index],
                                    nominal[step].az + noise_az[index],
                                    nominal[step].yaw_accel + noise_yaw[index]};
    control = limitControlStep(control, previous, dynamics,
                               step == 0U ? first_control_interval_s : dynamics.dt_s);
    const State previous_state = state;
    state = integrate(state, control, dynamics);
    altitude_envelope_hit =
        altitude_envelope_hit || !altitudeEnvelopeDynamicallyRecoverable(
                                     state, control, dynamics, altitude_envelope);
    const float segment_length_m =
        hypotf(hypotf(state.x - previous_state.x, state.y - previous_state.y),
               state.z - previous_state.z);
    traveled_distance_m += segment_length_m;
    const float validation_step_m = fmaxf(0.05F, 0.5F * grid.resolution_m);
    const int validation_samples =
        max(1, static_cast<int>(ceilf(segment_length_m / validation_step_m)));
    float clearance = kInfinity;
    bool step_contact = false;
    const DeviceBodyAxis body_axis = bodyAxisFromControl(control);
    for (int sample = 1; sample <= validation_samples; ++sample) {
      const float ratio =
          static_cast<float>(sample) / static_cast<float>(validation_samples);
      State swept_state = state;
      swept_state.x = previous_state.x + ratio * (state.x - previous_state.x);
      swept_state.y = previous_state.y + ratio * (state.y - previous_state.y);
      swept_state.z = previous_state.z + ratio * (state.z - previous_state.z);
      const DeviceEsdfQuery esdf_query = queryFootprint(
          swept_state, body_axis, footprint,
          footprint.clearance_broad_phase_enabled ? risk.preferred_distance_m : 0.0F,
          grid, esdf_texture);
      // A body sample inside a raw occupied voxel is a physical intersection,
      // not a clearance preference: such a rollout cannot be executed.
      step_contact = step_contact || esdf_query.inside_occupied;
      if (!esdf_query.unknown_space) {
        clearance = fminf(clearance, esdf_query.clearance_m);
      }
    }
    minimum_clearance_m = fminf(minimum_clearance_m, clearance);
    const float segment_speed_mps = hypotf(hypotf(state.vx, state.vy), state.vz);
    const float segment_m = dynamics.dt_s * segment_speed_mps;
    if (clearance < risk.critical_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kCritical));
      critical_m += segment_m;
    } else if (clearance < risk.preferred_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kPlanning));
      planning_m += segment_m;
    }
    {
      const float depth =
          clearancePreferenceDepth(clearance, risk.preferred_distance_m);
      clearance_preference_s += dynamics.dt_s * depth * depth;
    }
    // Evidence beside the motion: the tube law at this state's clearance.
    obstacle_approach_m2_s +=
        dynamics.dt_s *
        tubeClearanceDeficitM2(clearance, segment_speed_mps, risk.tube_response_time_s);
    // Evidence ahead on the motion: once the envelope enters occupied
    // evidence, every state before it owed the stopping law the free path it
    // had to that point. States after the first contact are not charged
    // again; the rollout is already intersecting.
    if (!collision_hit) {
      trace_station_m[step + 1U] = traveled_distance_m;
      trace_speed_mps[step + 1U] = segment_speed_mps;
      if (step_contact) {
        collision_hit = true;
        contact_distance_m = traveled_distance_m;
        for (std::size_t prior = 0U; prior <= step; ++prior) {
          stopping_deficit_m2_s +=
              dynamics.dt_s * stoppingDistanceDeficitM2(
                                  traveled_distance_m - trace_station_m[prior],
                                  trace_speed_mps[prior], risk.stopping_response_time_s,
                                  risk.stopping_deceleration_mps2);
        }
      }
    }
    const float target_elapsed_s = static_cast<float>(step + 1U) * dynamics.dt_s;
    const float target_distance =
        moving_target_enabled
            ? hypotf(hypotf(moving_target.state.x +
                                moving_target.state.vx * target_elapsed_s - state.x,
                            moving_target.state.y +
                                moving_target.state.vy * target_elapsed_s - state.y),
                     movingTargetAltitudeAt(moving_target, target_elapsed_s) - state.z)
            : hypotf(hypotf(target.x - state.x, target.y - state.y),
                     target.z - state.z);
    minimum_target_separation_m = fminf(minimum_target_separation_m, target_distance);
    for (std::size_t aircraft_index = 0U; aircraft_index < dynamic_aircraft_count;
         ++aircraft_index) {
      if (step >= dynamic_aircraft_active_steps[aircraft_index]) {
        continue;
      }
      const DynamicAircraftSample aircraft =
          dynamic_aircraft_samples[aircraft_index * steps + step];
      const float peer_separation_m = hypotf(
          hypotf(aircraft.x - state.x, aircraft.y - state.y), aircraft.z - state.z);
      DynamicAircraftCostPolicy effective_policy = dynamic_aircraft_cost_policy;
      effective_policy.strong_separation_m =
          fmaxf(effective_policy.strong_separation_m,
                footprint.radius_m + dynamic_aircraft_radii[aircraft_index]);
      effective_policy.anticipation_separation_m =
          fmaxf(effective_policy.anticipation_separation_m,
                effective_policy.strong_separation_m);
      const DynamicAircraftCostContribution contribution =
          dynamicAircraftCostContribution(peer_separation_m,
                                          static_cast<float>(step + 1U) * dynamics.dt_s,
                                          effective_policy);
      dynamic_aircraft_survival_cost += contribution.strong + contribution.anticipation;
    }
    if (cooperative_preference_enabled && step < cooperative_preference_steps) {
      const float preference_ax = control.ax - cooperative_preferred_acceleration.ax;
      const float preference_ay = control.ay - cooperative_preferred_acceleration.ay;
      const float preference_az = control.az - cooperative_preferred_acceleration.az;
      maneuver_preference_cost += preference_ax * preference_ax +
                                  preference_ay * preference_ay +
                                  preference_az * preference_az;
    }
    const HorizonCostSample path_cost_sample =
        route_point_count >= 2U
            ? horizonCostSample(step, steps, dynamics.dt_s,
                                costs.head_progress_horizon_s, horizon_sampling)
            : HorizonCostSample{1U, true};
    MppiRouteProjection3D route_projection;
    if (path_cost_sample.evaluate && route_point_count >= 2U) {
      route_projection = projectOntoMppiRoute3D(state, route_points, route_point_count,
                                                rollout_route_station_m);
    }
    if (path_cost_sample.evaluate) {
      const float sample_weight =
          static_cast<float>(path_cost_sample.represented_steps);
      if (route_projection.valid) {
        rollout_route_station_m = route_projection.station_m;
        guide_cost +=
            sample_weight * route_projection.distance_m * route_projection.distance_m;
        terminal_route_progress = creditedRouteProgressM(
            route_projection.station_m, initial_route_station_m, traveled_distance_m);
        if (!moving_target_enabled) {
          route_progress_integral_m_s +=
              sample_weight * dynamics.dt_s * terminal_route_progress;
        }
      } else {
        const float guide_x = target.x - initial.x;
        const float guide_y = target.y - initial.y;
        const float guide_z = target.z - initial.z;
        const float offset_x = state.x - initial.x;
        const float offset_y = state.y - initial.y;
        const float offset_z = state.z - initial.z;
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
          route_projection.valid ? route_projection.reference_z_m : target.z;
      const float altitude_error = state.z - z_reference;
      altitude_cost += sample_weight * altitude_error * altitude_error;
      const float active_reference_speed_mps =
          route_projection.valid && route_projection.reference_speed_mps > 0.0F
              ? fminf(reference_speed_mps, route_projection.reference_speed_mps)
              : reference_speed_mps;
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
            0.0F,
            fmaxf(horizontal_speed_mps - dynamics.maximum_horizontal_speed_mps,
                  translational_speed_mps - dynamics.maximum_translational_speed_mps));
        overspeed_cost += sample_weight * excess_mps * excess_mps;
      }
    }
    if (step + 1U == head_steps) {
      head_speed_mps = segment_speed_mps;
      head_progress = moving_target_enabled ? initial_distance - target_distance
                      : route_projection.valid
                          ? terminal_route_progress
                          : initial_distance -
                                hypotf(hypotf(target.x - state.x, target.y - state.y),
                                       target.z - state.z);
    }
    acceleration_cost +=
        control.ax * control.ax + control.ay * control.ay + control.az * control.az;
    jerk_cost += (control.ax - previous.ax) * (control.ax - previous.ax) +
                 (control.ay - previous.ay) * (control.ay - previous.ay) +
                 (control.az - previous.az) * (control.az - previous.az);
    yaw_cost += control.yaw_accel * control.yaw_accel;
    previous = control;
    if (altitude_envelope_hit && early_exit) {
      break;
    }
  }
  const float terminal_distance =
      moving_target_enabled
          ? fmaxf(0.0F, minimum_target_separation_m - moving_target.capture_radius_m)
          : hypotf(hypotf(target.x - state.x, target.y - state.y), target.z - state.z);
  const float progress =
      moving_target_enabled     ? initial_distance - minimum_target_separation_m
      : route_point_count >= 2U ? terminal_route_progress
                                : initial_distance - terminal_distance;
  RolloutCostTerms terms;
  terms.head_progress = costs.head_progress_weight * -head_progress;
  terms.progress = costs.progress_weight * -progress;
  terms.route_progress_integral =
      costs.route_progress_integral_weight * -route_progress_integral_m_s;
  terms.speed_tracking =
      costs.speed_tracking_weight * dynamics.dt_s * speed_tracking_cost;
  terms.overspeed = costs.overspeed_weight * dynamics.dt_s * overspeed_cost;
  terms.guide_deviation = costs.guide_deviation_weight * dynamics.dt_s * guide_cost;
  terms.altitude_tracking =
      costs.altitude_tracking_weight * dynamics.dt_s * altitude_cost;
  terms.acceleration = costs.acceleration_weight * dynamics.dt_s * acceleration_cost;
  terms.jerk = costs.jerk_weight * jerk_cost;
  terms.yaw_change = costs.yaw_change_weight * yaw_cost;
  terms.dynamic_aircraft = dynamics.dt_s * dynamic_aircraft_survival_cost;
  terms.maneuver_preference = costs.cooperative_maneuver_preference_weight *
                              dynamics.dt_s * maneuver_preference_cost;
  terms.clearance_preference =
      costs.clearance_preference_weight * clearance_preference_s;
  terms.obstacle_approach = costs.obstacle_approach_weight * obstacle_approach_m2_s;
  terms.stopping_deficit = costs.obstacle_approach_weight * stopping_deficit_m2_s;
  terms.terminal = costs.terminal_weight * terminal_distance;
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
  soft_cost[rollout] = terms.soft_cost;
  if (cost_terms != nullptr) {
    cost_terms[rollout] = terms;
  }
  critical_exposure[rollout] = critical_m;
  planning_exposure[rollout] = planning_m;
  minimum_clearance[rollout] = minimum_clearance_m;
  altitude_envelope_violation[rollout] = altitude_envelope_hit ? 1U : 0U;
  collision_violation[rollout] = collision_hit ? 1U : 0U;
  worst_tier[rollout] = tier;
}

__device__ float atomicMinFloat(float* address, float value) {
  int* const address_as_int = reinterpret_cast<int*>(address);
  int old = *address_as_int;
  int assumed = 0;
  while (value < __int_as_float(old)) {
    assumed = old;
    old = atomicCAS(address_as_int, assumed, __float_as_int(value));
    if (old == assumed) {
      break;
    }
  }
  return __int_as_float(old);
}

__global__ void initializeReduction(float* minimum_soft, float* weight_sum,
                                    float* weight_square_sum, int* best_rollout,
                                    float* feasible_cost_sum,
                                    unsigned int* feasible_count) {
  if (blockIdx.x == 0 && threadIdx.x == 0) {
    *minimum_soft = kInfinity;
    *weight_sum = 0.0F;
    *weight_square_sum = 0.0F;
    *best_rollout = INT_MAX;
    *feasible_cost_sum = 0.0F;
    *feasible_count = 0U;
  }
}

// A rollout takes part in the weighted update only when it is executable:
// inside the altitude envelope and, unless the caller lifted that gate because
// every rollout intersected raw occupancy, free of body-sample intersections.
__device__ bool rolloutFeasible(const std::uint8_t altitude_envelope_violation,
                                const std::uint8_t collision_violation,
                                const bool ignore_collision) {
  return altitude_envelope_violation == 0U &&
         (ignore_collision || collision_violation == 0U);
}

template<typename T> __device__ T blockMinimum(T value, const T identity) {
  constexpr unsigned int kFullWarpMask{0xffffffffU};
  constexpr int kWarpSize{32};
  __shared__ T warp_minima[kThreadsPerBlock / kWarpSize];
  const int lane = static_cast<int>(threadIdx.x) % kWarpSize;
  const int warp = static_cast<int>(threadIdx.x) / kWarpSize;
  for (int offset = kWarpSize / 2; offset > 0; offset /= 2) {
    const T other = __shfl_down_sync(kFullWarpMask, value, offset);
    value = value < other ? value : other;
  }
  if (lane == 0) {
    warp_minima[warp] = value;
  }
  __syncthreads();
  if (warp != 0) {
    return identity;
  }
  value = lane < kThreadsPerBlock / kWarpSize ? warp_minima[lane] : identity;
  for (int offset = kWarpSize / 2; offset > 0; offset /= 2) {
    const T other = __shfl_down_sync(kFullWarpMask, value, offset);
    value = value < other ? value : other;
  }
  return value;
}

__device__ float blockSum(float value) {
  constexpr unsigned int kFullWarpMask{0xffffffffU};
  constexpr int kWarpSize{32};
  __shared__ float warp_sums[kThreadsPerBlock / kWarpSize];
  const int lane = static_cast<int>(threadIdx.x) % kWarpSize;
  const int warp = static_cast<int>(threadIdx.x) / kWarpSize;
  for (int offset = kWarpSize / 2; offset > 0; offset /= 2) {
    value += __shfl_down_sync(kFullWarpMask, value, offset);
  }
  if (lane == 0) {
    warp_sums[warp] = value;
  }
  __syncthreads();
  if (warp != 0) {
    return 0.0F;
  }
  value = lane < kThreadsPerBlock / kWarpSize ? warp_sums[lane] : 0.0F;
  for (int offset = kWarpSize / 2; offset > 0; offset /= 2) {
    value += __shfl_down_sync(kFullWarpMask, value, offset);
  }
  return value;
}

__global__ void selectBestFeasibleRollout(const float* weights, const float* soft_cost,
                                          const float* minimum_soft,
                                          const std::size_t count, int* best_rollout) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  int candidate = INT_MAX;
  if (index < count && weights[index] > 0.0F &&
      soft_cost[index] <= *minimum_soft + 1.0e-5F) {
    candidate = static_cast<int>(index);
  }
  const int block_candidate = blockMinimum(candidate, INT_MAX);
  if (threadIdx.x == 0 && block_candidate != INT_MAX) {
    atomicMin(best_rollout, block_candidate);
  }
}

__global__ void buildBestFeasibleControls(
    const Control* nominal, Control* best_feasible, const float* noise_ax,
    const float* noise_ay, const float* noise_az, const float* noise_yaw,
    const int* best_rollout, const std::size_t rollouts, const std::size_t steps) {
  const std::size_t step =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (step >= steps) {
    return;
  }
  const int rollout = *best_rollout;
  if (rollout < 0 || static_cast<std::size_t>(rollout) >= rollouts) {
    best_feasible[step] = nominal[step];
    return;
  }
  const std::size_t index = static_cast<std::size_t>(rollout) * steps + step;
  best_feasible[step] = Control{
      nominal[step].ax + noise_ax[index], nominal[step].ay + noise_ay[index],
      nominal[step].az + noise_az[index], nominal[step].yaw_accel + noise_yaw[index]};
}

__global__ void reduceSoft(const float* soft,
                           const std::uint8_t* altitude_envelope_violation,
                           const std::uint8_t* collision_violation,
                           bool ignore_collision, std::size_t count,
                           float* minimum_soft) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const bool valid = index < count;
  float candidate = kInfinity;
  if (valid && rolloutFeasible(altitude_envelope_violation[index],
                               collision_violation[index], ignore_collision)) {
    candidate = soft[index];
  }
  const float block_candidate = blockMinimum(candidate, kInfinity);
  if (threadIdx.x == 0) {
    atomicMinFloat(minimum_soft, block_candidate);
  }
}

// Sums the feasible costs above the minimum so the temperature can follow the
// spread of the population instead of a fixed constant.
__global__ void accumulateFeasibleCost(
    const float* soft, const std::uint8_t* altitude_envelope_violation,
    const std::uint8_t* collision_violation, bool ignore_collision, std::size_t count,
    const float* minimum_soft, float* feasible_cost_sum, unsigned int* feasible_count) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const bool valid = index < count;
  float excess = 0.0F;
  float feasible = 0.0F;
  if (valid && rolloutFeasible(altitude_envelope_violation[index],
                               collision_violation[index], ignore_collision)) {
    excess = fmaxf(0.0F, soft[index] - *minimum_soft);
    feasible = 1.0F;
  }
  const float block_excess = blockSum(excess);
  const float block_feasible = blockSum(feasible);
  if (threadIdx.x == 0) {
    atomicAdd(feasible_cost_sum, block_excess);
    atomicAdd(feasible_count, static_cast<unsigned int>(block_feasible + 0.5F));
  }
}

// The temperature is chosen by the host from the effective sample size the
// previous tick achieved, so the softmax is a distribution over a target share
// of the population instead of collapsing onto whichever sample happens to be
// best. The squared weight sum is what makes that measurable: with it,
// ESS = (sum w)^2 / sum w^2.
__global__ void
calculateWeights(const float* soft, const std::uint8_t* altitude_envelope_violation,
                 const std::uint8_t* collision_violation, bool ignore_collision,
                 float* weights, std::size_t count, const float* minimum_soft,
                 float temperature, float* weight_sum, float* weight_square_sum,
                 float* effective_temperature) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const bool valid = index < count;
  float weight = 0.0F;
  if (valid && rolloutFeasible(altitude_envelope_violation[index],
                               collision_violation[index], ignore_collision)) {
    weight = expf(-(soft[index] - *minimum_soft) / temperature);
  }
  if (valid) {
    weights[index] = weight;
  }
  const float block_weight_sum = blockSum(weight);
  const float block_weight_square_sum = blockSum(weight * weight);
  if (threadIdx.x == 0) {
    atomicAdd(weight_sum, block_weight_sum);
    atomicAdd(weight_square_sum, block_weight_square_sum);
    if (blockIdx.x == 0) {
      *effective_temperature = temperature;
    }
  }
}

__global__ void accumulateControlUpdatePartials(
    const float* noise_ax, const float* noise_ay, const float* noise_az,
    const float* noise_yaw, const float* weights, Control* partials,
    std::size_t rollouts, std::size_t steps, std::size_t partitions) {
  // Each warp keeps adjacent timesteps coalesced while Y blocks partition rollouts.
  __shared__ Control tile[kThreadsPerBlock];
  const std::size_t step_lane = threadIdx.x % kControlUpdateStepTile;
  const std::size_t rollout_lane = threadIdx.x / kControlUpdateStepTile;
  const std::size_t step =
      static_cast<std::size_t>(blockIdx.x) * kControlUpdateStepTile + step_lane;
  const std::size_t partition = blockIdx.y;
  float ax = 0.0F;
  float ay = 0.0F;
  float az = 0.0F;
  float yaw = 0.0F;
  if (step < steps) {
    const std::size_t rollout_stride = partitions * kControlUpdateRolloutLanes;
    for (std::size_t rollout = partition * kControlUpdateRolloutLanes + rollout_lane;
         rollout < rollouts; rollout += rollout_stride) {
      const std::size_t index = rollout * steps + step;
      const float weight = weights[rollout];
      ax += weight * noise_ax[index];
      ay += weight * noise_ay[index];
      az += weight * noise_az[index];
      yaw += weight * noise_yaw[index];
    }
  }
  tile[threadIdx.x] = Control{ax, ay, az, yaw};
  __syncthreads();
  if (rollout_lane != 0U || step >= steps) {
    return;
  }
  Control partial{};
  for (std::size_t lane = 0U; lane < kControlUpdateRolloutLanes; ++lane) {
    const Control value = tile[lane * kControlUpdateStepTile + step_lane];
    partial.ax += value.ax;
    partial.ay += value.ay;
    partial.az += value.az;
    partial.yaw_accel += value.yaw_accel;
  }
  partials[partition * steps + step] = partial;
}

__global__ void finalizeControlUpdate(const Control* nominal, Control* updated,
                                      const Control* partials, const float* weight_sum,
                                      std::size_t steps, std::size_t partitions) {
  const std::size_t step =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (step >= steps) {
    return;
  }
  Control sum{};
  for (std::size_t partition = 0U; partition < partitions; ++partition) {
    const Control partial = partials[partition * steps + step];
    sum.ax += partial.ax;
    sum.ay += partial.ay;
    sum.az += partial.az;
    sum.yaw_accel += partial.yaw_accel;
  }
  const float denominator = fmaxf(*weight_sum, 1.0e-12F);
  updated[step] = Control{nominal[step].ax + sum.ax / denominator,
                          nominal[step].ay + sum.ay / denominator,
                          nominal[step].az + sum.az / denominator,
                          nominal[step].yaw_accel + sum.yaw_accel / denominator};
}

__global__ void limitControls(Control* controls, std::size_t steps,
                              DynamicsConfig dynamics, Control previous_applied_control,
                              float first_control_interval_s) {
  if (blockIdx.x != 0 || threadIdx.x != 0) {
    return;
  }
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
