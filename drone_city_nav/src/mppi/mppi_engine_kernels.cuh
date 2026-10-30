#pragma once

// The CUDA shells of the rollout model: the bodies live in
// mppi_rollout_body.hpp, shared with the CPU engine (roadmap item 28, way A);
// here only what is the device's own, the thread indexing, the texture read
// and the block reductions.

// The ESDF as the device reads it: a point-filtered 3D texture of the grid.
struct TextureLookup {
  cudaTextureObject_t texture;

  __device__ float operator()(const int x, const int y, const int z) const {
    return tex3D<float>(texture, static_cast<float>(x) + 0.5F,
                        static_cast<float>(y) + 0.5F, static_cast<float>(z) + 0.5F);
  }
};

__global__ void generateNoise(float* noise_ax, float* noise_ay, float* noise_az,
                              float* noise_yaw, std::size_t count, std::uint64_t seed,
                              std::uint64_t tick, NoiseConfig config) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const rollout::NoiseSample sample = rollout::noiseSample(index, seed, tick, config);
  noise_ax[index] = sample.ax;
  noise_ay[index] = sample.ay;
  noise_az[index] = sample.az;
  noise_yaw[index] = sample.yaw;
}

__global__ void simulate(const rollout::RolloutInputs inputs,
                         const rollout::RolloutOutputs outputs,
                         const cudaTextureObject_t esdf_texture, std::size_t rollouts) {
  const std::size_t rollout =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (rollout >= rollouts) {
    return;
  }
  rollout::simulateRollout(rollout, inputs, outputs, TextureLookup{esdf_texture});
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
  if (valid && rollout::rolloutFeasible(altitude_envelope_violation[index],
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
  if (valid && rollout::rolloutFeasible(altitude_envelope_violation[index],
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
  if (valid && rollout::rolloutFeasible(altitude_envelope_violation[index],
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
  rollout::limitControlSequence(controls, steps, dynamics, previous_applied_control,
                                first_control_interval_s);
}
