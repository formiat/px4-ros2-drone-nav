#pragma once

__device__ float clampValue(const float value, const float minimum,
                            const float maximum) {
  return fminf(maximum, fmaxf(minimum, value));
}

__device__ void clampHorizontalDevice(float& x, float& y, const float limit) {
  const float magnitude = hypotf(x, y);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

__device__ std::uint64_t mixBits(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

__device__ float uniform01(const std::uint64_t value) {
  return (static_cast<float>((mixBits(value) >> 40U) + 1U)) * (1.0F / 16777217.0F);
}

__device__ float gaussian(const std::uint64_t first, const std::uint64_t second) {
  const float u1 = fmaxf(uniform01(first), 1.0e-7F);
  const float u2 = uniform01(second);
  return sqrtf(-2.0F * logf(u1)) * cosf(2.0F * kPi * u2);
}

__global__ void generateNoiseKernel(float* const noise_ax, float* const noise_ay,
                                    float* const noise_az, float* const noise_yaw,
                                    const std::size_t count, const std::uint64_t seed,
                                    const std::uint64_t tick,
                                    const NoiseConfig config) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const std::uint64_t base =
      seed ^ (tick * 0xd1342543de82ef95ULL) ^ static_cast<std::uint64_t>(index);
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

__device__ State integrateDevice(State state, Control control,
                                 const DynamicsConfig config) {
  clampHorizontalDevice(control.ax, control.ay,
                        config.maximum_horizontal_acceleration_mps2);
  control.az = clampValue(control.az, -config.maximum_vertical_acceleration_mps2,
                          config.maximum_vertical_acceleration_mps2);
  control.yaw_accel =
      clampValue(control.yaw_accel, -config.maximum_yaw_acceleration_radps2,
                 config.maximum_yaw_acceleration_radps2);
  const float drag = fmaxf(0.0F, 1.0F - config.linear_drag_1ps * config.dt_s);
  const float inherited_horizontal_speed_mps = hypotf(state.vx, state.vy);
  const float inherited_vertical_speed_mps = fabsf(state.vz);
  const float inherited_yaw_rate_radps = fabsf(state.yaw_rate);
  state.vx = state.vx * drag + control.ax * config.dt_s;
  state.vy = state.vy * drag + control.ay * config.dt_s;
  state.vz = state.vz * drag + control.az * config.dt_s;
  clampHorizontalDevice(
      state.vx, state.vy,
      fmaxf(config.maximum_horizontal_speed_mps, inherited_horizontal_speed_mps));
  const float vertical_speed_limit_mps =
      fmaxf(config.maximum_vertical_speed_mps, inherited_vertical_speed_mps);
  state.vz = clampValue(state.vz, -vertical_speed_limit_mps, vertical_speed_limit_mps);
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

struct DeviceEsdfQuery {
  float clearance_m;
  bool raw_collision;
};

__device__ DeviceEsdfQuery queryEsdf(const State& state, const EsdfGrid grid,
                                     const cudaTextureObject_t esdf_texture) {
  const float cell_x_float = (state.x - grid.origin_x_m) / grid.resolution_m;
  const float cell_y_float = (state.y - grid.origin_y_m) / grid.resolution_m;
  const int cell_x = static_cast<int>(floorf(cell_x_float));
  const int cell_y = static_cast<int>(floorf(cell_y_float));
  if (cell_x < 0 || cell_y < 0 || cell_x >= grid.width || cell_y >= grid.height) {
    return {0.0F, true};
  }
  const float center_distance_m =
      tex2D<float>(esdf_texture, static_cast<float>(cell_x) + 0.5F,
                   static_cast<float>(cell_y) + 0.5F);
  if (isinf(center_distance_m) && center_distance_m > 0.0F) {
    return {center_distance_m, false};
  }
  if (!isfinite(center_distance_m) || center_distance_m < 0.0F) {
    return {0.0F, true};
  }
  const float center_x_m =
      grid.origin_x_m + (static_cast<float>(cell_x) + 0.5F) * grid.resolution_m;
  const float center_y_m =
      grid.origin_y_m + (static_cast<float>(cell_y) + 0.5F) * grid.resolution_m;
  constexpr float kHalfDiagonalScale{0.70710678118654752440F};
  const float correction_m = hypotf(state.x - center_x_m, state.y - center_y_m) +
                             kHalfDiagonalScale * grid.resolution_m;
  return {fmaxf(0.0F, center_distance_m - correction_m), center_distance_m == 0.0F};
}

__global__ void
simulateKernel(const float* const noise_ax, const float* const noise_ay,
               const float* const noise_az, const float* const noise_yaw,
               const Control* const nominal, float* const soft_cost,
               float* const critical_exposure, float* const planning_exposure,
               float* const minimum_clearance, std::uint8_t* const worst_tier,
               std::uint8_t* const collision, const std::size_t rollouts,
               const std::size_t steps, const State initial, const float target_x,
               const float target_y, const DynamicsConfig dynamics,
               const RiskConfig risk, const CostConfig costs, const EsdfGrid grid,
               const cudaTextureObject_t esdf_texture, const bool early_exit) {
  const std::size_t rollout =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (rollout >= rollouts) {
    return;
  }
  State state = initial;
  Control previous{};
  float guide_cost = 0.0F;
  float acceleration_cost = 0.0F;
  float jerk_cost = 0.0F;
  float yaw_cost = 0.0F;
  float effort_cost = 0.0F;
  float critical_m = 0.0F;
  float planning_m = 0.0F;
  float critical_clearance_proximity_s = 0.0F;
  float obstacle_approach_m2_s = 0.0F;
  float minimum_clearance_m = kInfinity;
  float previous_clearance_m = kInfinity;
  std::uint8_t tier = static_cast<std::uint8_t>(RiskTier::kPreferred);
  bool collided = false;
  const float initial_distance = hypotf(target_x - state.x, target_y - state.y);
  float head_progress = 0.0F;
  const std::size_t requested_head_steps =
      static_cast<std::size_t>(ceilf(costs.head_progress_horizon_s / dynamics.dt_s));
  const std::size_t head_steps =
      requested_head_steps == 0U
          ? 1U
          : (requested_head_steps > steps ? steps : requested_head_steps);
  for (std::size_t step = 0U; step < steps; ++step) {
    const std::size_t index = rollout * steps + step;
    Control control{
        nominal[step].ax + noise_ax[index],
        nominal[step].ay + noise_ay[index],
        nominal[step].az + noise_az[index],
        nominal[step].yaw_accel + noise_yaw[index],
    };
    state = integrateDevice(state, control, dynamics);
    const DeviceEsdfQuery esdf_query = queryEsdf(state, grid, esdf_texture);
    const float clearance = esdf_query.clearance_m;
    minimum_clearance_m = fminf(minimum_clearance_m, clearance);
    const float segment_speed_mps = hypotf(state.vx, state.vy);
    const float segment_m = dynamics.dt_s * segment_speed_mps;
    if (esdf_query.raw_collision) {
      collided = true;
      tier = static_cast<std::uint8_t>(RiskTier::kCollision);
    } else if (clearance < risk.critical_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kCritical));
      critical_m += segment_m;
      critical_clearance_proximity_s +=
          dynamics.dt_s *
          criticalClearanceProximitySeverity(clearance, risk.critical_distance_m);
    } else if (clearance < risk.preferred_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kPlanning));
      planning_m += segment_m;
    }
    obstacle_approach_m2_s +=
        dynamics.dt_s *
        obstacleApproachSeverityM2(previous_clearance_m, clearance, segment_speed_mps,
                                   dynamics.dt_s, risk.critical_distance_m,
                                   risk.obstacle_approach_response_time_s,
                                   risk.obstacle_approach_deceleration_mps2);
    previous_clearance_m = clearance;
    guide_cost += (state.y - initial.y) * (state.y - initial.y);
    acceleration_cost +=
        control.ax * control.ax + control.ay * control.ay + control.az * control.az;
    jerk_cost += (control.ax - previous.ax) * (control.ax - previous.ax) +
                 (control.ay - previous.ay) * (control.ay - previous.ay) +
                 (control.az - previous.az) * (control.az - previous.az);
    yaw_cost += control.yaw_accel * control.yaw_accel;
    effort_cost += control.ax * control.ax + control.ay * control.ay +
                   control.az * control.az + control.yaw_accel * control.yaw_accel;
    if (step + 1U == head_steps) {
      head_progress = initial_distance - hypotf(target_x - state.x, target_y - state.y);
    }
    previous = control;
    if (collided && early_exit) {
      break;
    }
  }
  const float terminal_distance = hypotf(target_x - state.x, target_y - state.y);
  const float progress_cost = -(initial_distance - terminal_distance);
  soft_cost[rollout] =
      costs.head_progress_weight * -head_progress +
      costs.progress_weight * progress_cost +
      costs.guide_deviation_weight * dynamics.dt_s * guide_cost +
      costs.acceleration_weight * dynamics.dt_s * acceleration_cost +
      costs.jerk_weight * jerk_cost + costs.yaw_change_weight * yaw_cost +
      costs.control_effort_weight * dynamics.dt_s * effort_cost +
      costs.planning_exposure_weight * planning_m +
      costs.critical_exposure_weight * critical_m +
      costs.critical_clearance_proximity_weight * critical_clearance_proximity_s +
      costs.obstacle_approach_weight * obstacle_approach_m2_s +
      costs.terminal_weight * terminal_distance;
  critical_exposure[rollout] = critical_m;
  planning_exposure[rollout] = planning_m;
  minimum_clearance[rollout] = minimum_clearance_m;
  worst_tier[rollout] = tier;
  collision[rollout] = collided ? 1U : 0U;
}

__device__ float atomicMinFloat(float* const address, const float value) {
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

__global__ void initializeReductionKernel(int* const best_tier,
                                          float* const best_critical,
                                          float* const best_planning,
                                          float* const minimum_soft,
                                          float* const weight_sum) {
  if (blockIdx.x == 0 && threadIdx.x == 0) {
    *best_tier = static_cast<int>(RiskTier::kCollision);
    *best_critical = kInfinity;
    *best_planning = kInfinity;
    *minimum_soft = kInfinity;
    *weight_sum = 0.0F;
  }
}

__global__ void reduceTierKernel(const std::uint8_t* const tier,
                                 const std::uint8_t* const collision,
                                 const std::size_t count, int* const best_tier) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && collision[index] == 0U) {
    atomicMin(best_tier, static_cast<int>(tier[index]));
  }
}

__global__ void
reduceCriticalKernel(const std::uint8_t* const tier, const float* const critical,
                     const std::uint8_t* const collision, const std::size_t count,
                     const int* const best_tier, float* const best_critical) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && collision[index] == 0U &&
      static_cast<int>(tier[index]) == *best_tier) {
    atomicMinFloat(best_critical, critical[index]);
  }
}

__global__ void
reducePlanningKernel(const std::uint8_t* const tier, const float* const critical,
                     const float* const planning, const std::uint8_t* const collision,
                     const std::size_t count, const int* const best_tier,
                     const float* const best_critical, const float critical_tolerance,
                     float* const best_planning) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && collision[index] == 0U &&
      static_cast<int>(tier[index]) == *best_tier &&
      critical[index] <= *best_critical + critical_tolerance) {
    atomicMinFloat(best_planning, planning[index]);
  }
}

__global__ void reduceSoftKernel(const std::uint8_t* const tier,
                                 const float* const critical,
                                 const float* const planning, const float* const soft,
                                 const std::uint8_t* const collision,
                                 const std::size_t count, const int* const best_tier,
                                 const float* const best_critical,
                                 const float* const best_planning,
                                 const RiskConfig risk, float* const minimum_soft) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && collision[index] == 0U &&
      static_cast<int>(tier[index]) == *best_tier &&
      critical[index] <= *best_critical + risk.critical_exposure_tolerance_m &&
      planning[index] <= *best_planning + risk.planning_exposure_tolerance_m) {
    atomicMinFloat(minimum_soft, soft[index]);
  }
}

__global__ void calculateWeightsKernel(
    const std::uint8_t* const tier, const float* const critical,
    const float* const planning, const float* const soft,
    const std::uint8_t* const collision, float* const weights, const std::size_t count,
    const int* const best_tier, const float* const best_critical,
    const float* const best_planning, const float* const minimum_soft,
    const RiskConfig risk, const float temperature, float* const weight_sum) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  float weight = 0.0F;
  if (collision[index] == 0U && static_cast<int>(tier[index]) == *best_tier &&
      critical[index] <= *best_critical + risk.critical_exposure_tolerance_m &&
      planning[index] <= *best_planning + risk.planning_exposure_tolerance_m) {
    weight = expf(-(soft[index] - *minimum_soft) / temperature);
  }
  weights[index] = weight;
  atomicAdd(weight_sum, weight);
}

__global__ void
updateControlsKernel(const Control* const nominal, Control* const updated,
                     const float* const noise_ax, const float* const noise_ay,
                     const float* const noise_az, const float* const noise_yaw,
                     const float* const weights, const float* const weight_sum,
                     const std::size_t rollouts, const std::size_t steps) {
  const std::size_t step =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (step >= steps) {
    return;
  }
  float ax = 0.0F;
  float ay = 0.0F;
  float az = 0.0F;
  float yaw = 0.0F;
  for (std::size_t rollout = 0U; rollout < rollouts; ++rollout) {
    const float weight = weights[rollout];
    const std::size_t index = rollout * steps + step;
    ax += weight * noise_ax[index];
    ay += weight * noise_ay[index];
    az += weight * noise_az[index];
    yaw += weight * noise_yaw[index];
  }
  const float denominator = fmaxf(*weight_sum, 1.0e-12F);
  updated[step] = Control{
      nominal[step].ax + ax / denominator, nominal[step].ay + ay / denominator,
      nominal[step].az + az / denominator, nominal[step].yaw_accel + yaw / denominator};
}

__global__ void warmStartKernel(Control* const nominal, const Control* const updated,
                                const std::size_t steps) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= steps) {
    return;
  }
  nominal[index] = updated[index + (index + 1U < steps ? 1U : 0U)];
}
