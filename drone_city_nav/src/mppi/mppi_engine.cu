#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cuda_runtime.h>
#include <limits>
#include <mutex>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_city_nav::mppi {
namespace {

constexpr int kThreadsPerBlock{256};
constexpr std::size_t kMaximumKnownSolids{2048U};
constexpr float kPi{3.14159265358979323846F};
constexpr float kInfinity{std::numeric_limits<float>::infinity()};

void checkCuda(const cudaError_t error, const char* const operation) {
  if (error != cudaSuccess) {
    throw std::runtime_error{std::string{operation} + ": " + cudaGetErrorString(error)};
  }
}

template<typename T> class DeviceBuffer {
public:
  DeviceBuffer() = default;

  explicit DeviceBuffer(const std::size_t count) {
    allocate(count);
  }

  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  ~DeviceBuffer() {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
    }
  }

  void allocate(const std::size_t count) {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
    }
    count_ = count;
    checkCuda(cudaMalloc(reinterpret_cast<void**>(&data_), count * sizeof(T)),
              "cudaMalloc");
  }

  [[nodiscard]] T* get() noexcept {
    return data_;
  }

  [[nodiscard]] const T* get() const noexcept {
    return data_;
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return count_ * sizeof(T);
  }

private:
  T* data_{nullptr};
  std::size_t count_{0U};
};

class Event {
public:
  Event() {
    checkCuda(cudaEventCreate(&event_), "cudaEventCreate");
  }

  Event(const Event&) = delete;
  Event& operator=(const Event&) = delete;

  ~Event() {
    if (event_ != nullptr) {
      (void)cudaEventDestroy(event_);
    }
  }

  void record(cudaStream_t stream) {
    checkCuda(cudaEventRecord(event_, stream), "cudaEventRecord");
  }

  void synchronize() {
    checkCuda(cudaEventSynchronize(event_), "cudaEventSynchronize");
  }

  [[nodiscard]] cudaEvent_t get() const noexcept {
    return event_;
  }

private:
  cudaEvent_t event_{nullptr};
};

[[nodiscard]] double elapsedMs(const Event& start, const Event& end) {
  float elapsed_ms = 0.0F;
  checkCuda(cudaEventElapsedTime(&elapsed_ms, start.get(), end.get()),
            "cudaEventElapsedTime");
  return elapsed_ms;
}

class EsdfTexture {
public:
  EsdfTexture() = default;
  EsdfTexture(const EsdfTexture&) = delete;
  EsdfTexture& operator=(const EsdfTexture&) = delete;

  ~EsdfTexture() {
    reset();
  }

  double upload(const EsdfSnapshot& snapshot, cudaStream_t stream) {
    const auto started = std::chrono::steady_clock::now();
    if (array_ == nullptr || grid_.width != snapshot.grid.width ||
        grid_.height != snapshot.grid.height) {
      reset();
      const cudaChannelFormatDesc channel = cudaCreateChannelDesc<float>();
      checkCuda(cudaMallocArray(&array_, &channel,
                                static_cast<std::size_t>(snapshot.grid.width),
                                static_cast<std::size_t>(snapshot.grid.height)),
                "cudaMallocArray");
      cudaResourceDesc resource{};
      resource.resType = cudaResourceTypeArray;
      resource.res.array.array = array_;
      cudaTextureDesc texture_description{};
      texture_description.addressMode[0] = cudaAddressModeBorder;
      texture_description.addressMode[1] = cudaAddressModeBorder;
      texture_description.filterMode = cudaFilterModeLinear;
      texture_description.readMode = cudaReadModeElementType;
      texture_description.normalizedCoords = 0;
      checkCuda(
          cudaCreateTextureObject(&texture_, &resource, &texture_description, nullptr),
          "cudaCreateTextureObject");
    }
    checkCuda(cudaMemcpy2DToArrayAsync(
                  array_, 0U, 0U, snapshot.distances_m.data(),
                  static_cast<std::size_t>(snapshot.grid.width) * sizeof(float),
                  static_cast<std::size_t>(snapshot.grid.width) * sizeof(float),
                  static_cast<std::size_t>(snapshot.grid.height),
                  cudaMemcpyHostToDevice, stream),
              "cudaMemcpy2DToArrayAsync");
    checkCuda(cudaStreamSynchronize(stream), "synchronize ESDF upload");
    grid_ = snapshot.grid;
    revision_ = snapshot.revision;
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                     started)
        .count();
  }

  [[nodiscard]] cudaTextureObject_t texture() const noexcept {
    return texture_;
  }

  [[nodiscard]] const EsdfGrid& grid() const noexcept {
    return grid_;
  }

  [[nodiscard]] std::uint64_t revision() const noexcept {
    return revision_;
  }

  [[nodiscard]] bool ready() const noexcept {
    return texture_ != 0U;
  }

private:
  void reset() noexcept {
    if (texture_ != 0U) {
      (void)cudaDestroyTextureObject(texture_);
    }
    if (array_ != nullptr) {
      (void)cudaFreeArray(array_);
    }
    texture_ = 0U;
    array_ = nullptr;
  }

  cudaArray_t array_{nullptr};
  cudaTextureObject_t texture_{0U};
  EsdfGrid grid_{};
  std::uint64_t revision_{0U};
};

struct DeviceBuffers {
  DeviceBuffer<float> noise_ax;
  DeviceBuffer<float> noise_ay;
  DeviceBuffer<float> noise_az;
  DeviceBuffer<float> noise_yaw;
  DeviceBuffer<float> soft_cost;
  DeviceBuffer<float> critical_exposure;
  DeviceBuffer<float> planning_exposure;
  DeviceBuffer<float> minimum_clearance;
  DeviceBuffer<std::uint8_t> worst_tier;
  DeviceBuffer<std::uint8_t> raw_collision;
  DeviceBuffer<std::uint8_t> solid_collision;
  DeviceBuffer<float> weights;
  DeviceBuffer<Control> nominal;
  DeviceBuffer<Control> updated;
  DeviceBuffer<int> best_tier;
  DeviceBuffer<float> best_critical;
  DeviceBuffer<float> best_planning;
  DeviceBuffer<float> minimum_soft;
  DeviceBuffer<float> weight_sum;
  DeviceBuffer<KnownSolid> solids{kMaximumKnownSolids};

  DeviceBuffers(const std::size_t rollouts, const std::size_t steps)
      : noise_ax{rollouts * steps},
        noise_ay{rollouts * steps},
        noise_az{rollouts * steps},
        noise_yaw{rollouts * steps},
        soft_cost{rollouts},
        critical_exposure{rollouts},
        planning_exposure{rollouts},
        minimum_clearance{rollouts},
        worst_tier{rollouts},
        raw_collision{rollouts},
        solid_collision{rollouts},
        weights{rollouts},
        nominal{steps},
        updated{steps},
        best_tier{1U},
        best_critical{1U},
        best_planning{1U},
        minimum_soft{1U},
        weight_sum{1U} {
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return noise_ax.bytes() + noise_ay.bytes() + noise_az.bytes() + noise_yaw.bytes() +
           soft_cost.bytes() + critical_exposure.bytes() + planning_exposure.bytes() +
           minimum_clearance.bytes() + worst_tier.bytes() + raw_collision.bytes() +
           solid_collision.bytes() + weights.bytes() + nominal.bytes() +
           updated.bytes() + best_tier.bytes() + best_critical.bytes() +
           best_planning.bytes() + minimum_soft.bytes() + weight_sum.bytes() +
           solids.bytes();
  }
};

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
  state.vx = state.vx * drag + control.ax * config.dt_s;
  state.vy = state.vy * drag + control.ay * config.dt_s;
  state.vz = state.vz * drag + control.az * config.dt_s;
  clampHorizontal(state.vx, state.vy, config.maximum_horizontal_speed_mps);
  state.vz = clampValue(state.vz, -config.maximum_vertical_speed_mps,
                        config.maximum_vertical_speed_mps);
  state.yaw_rate =
      clampValue(state.yaw_rate + control.yaw_accel * config.dt_s,
                 -config.maximum_yaw_rate_radps, config.maximum_yaw_rate_radps);
  state.x += state.vx * config.dt_s;
  state.y += state.vy * config.dt_s;
  state.z += state.vz * config.dt_s;
  state.yaw = remainderf(state.yaw + state.yaw_rate * config.dt_s, 2.0F * kPi);
  return state;
}

__device__ bool intersectsSolid(const State& state, const KnownSolid& solid) {
  if (state.z < solid.min_z_m || state.z > solid.max_z_m) {
    return false;
  }
  const float dx = state.x - solid.center_x_m;
  const float dy = state.y - solid.center_y_m;
  const float depth = dx * solid.normal_x + dy * solid.normal_y;
  const float lateral = dx * solid.lateral_x + dy * solid.lateral_y;
  return fabsf(depth) <= solid.half_depth_m && fabsf(lateral) <= solid.half_width_m;
}

__global__ void
simulate(const float* noise_ax, const float* noise_ay, const float* noise_az,
         const float* noise_yaw, const Control* nominal, float* soft_cost,
         float* critical_exposure, float* planning_exposure, float* minimum_clearance,
         std::uint8_t* worst_tier, std::uint8_t* raw_collision,
         std::uint8_t* solid_collision, std::size_t rollouts, std::size_t steps,
         State initial, State target, DynamicsConfig dynamics, RiskConfig risk,
         CostConfig costs, EsdfGrid grid, cudaTextureObject_t esdf_texture,
         const KnownSolid* solids, std::size_t solid_count, PassageConstraint passage,
         bool passage_active, Control previous_applied_control,
         float reference_speed_mps, bool early_exit) {
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
  float critical_m = 0.0F;
  float planning_m = 0.0F;
  float minimum_clearance_m = kInfinity;
  bool raw_hit = false;
  bool solid_hit = false;
  std::uint8_t tier = static_cast<std::uint8_t>(RiskTier::kPreferred);
  const float initial_distance = hypotf(target.x - initial.x, target.y - initial.y);
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
        nominal[step].ax + noise_ax[index], nominal[step].ay + noise_ay[index],
        nominal[step].az + noise_az[index], nominal[step].yaw_accel + noise_yaw[index]};
    state = integrate(state, control, dynamics);
    const float texture_x = (state.x - grid.origin_x_m) / grid.resolution_m;
    const float texture_y = (state.y - grid.origin_y_m) / grid.resolution_m;
    const float clearance = tex2D<float>(esdf_texture, texture_x, texture_y);
    minimum_clearance_m = fminf(minimum_clearance_m, clearance);
    raw_hit = raw_hit || clearance <= risk.collision_radius_m;
    for (std::size_t solid_index = 0U; solid_index < solid_count && !solid_hit;
         ++solid_index) {
      solid_hit = intersectsSolid(state, solids[solid_index]);
    }
    if (passage_active) {
      const float passage_dx = state.x - passage.center_x_m;
      const float passage_dy = state.y - passage.center_y_m;
      const float longitudinal =
          passage_dx * passage.normal_x + passage_dy * passage.normal_y;
      const bool inside_passage_window = longitudinal >= -passage.approach_distance_m &&
                                         longitudinal <= passage.exit_distance_m;
      const bool inside_opening = fabsf(longitudinal) <= passage.half_depth_m;
      if (inside_opening && (state.z < passage.min_z_m || state.z > passage.max_z_m)) {
        solid_hit = true;
      }
      if (inside_passage_window) {
        const float altitude_error = state.z - passage.preferred_z_m;
        guide_cost += altitude_error * altitude_error;
        const float speed = hypotf(state.vx, state.vy);
        if (passage.speed_limit_mps > 0.0F && speed > passage.speed_limit_mps) {
          const float speed_error = speed - passage.speed_limit_mps;
          acceleration_cost += speed_error * speed_error;
        }
      }
    }
    const float segment_m = dynamics.dt_s * hypotf(state.vx, state.vy);
    if (raw_hit || solid_hit) {
      tier = static_cast<std::uint8_t>(RiskTier::kCollision);
    } else if (clearance < risk.critical_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kCritical));
      critical_m += segment_m;
    } else if (clearance < risk.preferred_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kPlanning));
      planning_m += segment_m;
    }
    const float guide_cross = (state.y - initial.y) * (target.x - initial.x) -
                              (state.x - initial.x) * (target.y - initial.y);
    const float guide_length =
        fmaxf(1.0F, hypotf(target.x - initial.x, target.y - initial.y));
    guide_cost += (guide_cross / guide_length) * (guide_cross / guide_length);
    const float altitude_error = state.z - target.z;
    altitude_cost += altitude_error * altitude_error;
    if (step + 1U == head_steps) {
      head_progress = initial_distance - hypotf(target.x - state.x, target.y - state.y);
    }
    acceleration_cost +=
        control.ax * control.ax + control.ay * control.ay + control.az * control.az;
    if (reference_speed_mps >= 0.0F) {
      const float speed_error = hypotf(state.vx, state.vy) - reference_speed_mps;
      speed_tracking_cost += speed_error * speed_error;
    }
    jerk_cost += (control.ax - previous.ax) * (control.ax - previous.ax) +
                 (control.ay - previous.ay) * (control.ay - previous.ay) +
                 (control.az - previous.az) * (control.az - previous.az);
    yaw_cost += control.yaw_accel * control.yaw_accel;
    previous = control;
    if ((raw_hit || solid_hit) && early_exit) {
      break;
    }
  }
  const float terminal_distance = hypotf(target.x - state.x, target.y - state.y);
  soft_cost[rollout] = costs.head_progress_weight * -head_progress +
                       costs.progress_weight * -(initial_distance - terminal_distance) +
                       costs.speed_tracking_weight * dynamics.dt_s *
                           speed_tracking_cost +
                       costs.guide_deviation_weight * dynamics.dt_s * guide_cost +
                       costs.altitude_tracking_weight * dynamics.dt_s * altitude_cost +
                       costs.acceleration_weight * dynamics.dt_s * acceleration_cost +
                       costs.jerk_weight * jerk_cost +
                       costs.yaw_change_weight * yaw_cost +
                       costs.terminal_weight * terminal_distance;
  critical_exposure[rollout] = critical_m;
  planning_exposure[rollout] = planning_m;
  minimum_clearance[rollout] = minimum_clearance_m;
  worst_tier[rollout] = tier;
  raw_collision[rollout] = raw_hit ? 1U : 0U;
  solid_collision[rollout] = solid_hit ? 1U : 0U;
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

__global__ void initializeReduction(int* best_tier, float* best_critical,
                                    float* best_planning, float* minimum_soft,
                                    float* weight_sum) {
  if (blockIdx.x == 0 && threadIdx.x == 0) {
    *best_tier = static_cast<int>(RiskTier::kCollision);
    *best_critical = kInfinity;
    *best_planning = kInfinity;
    *minimum_soft = kInfinity;
    *weight_sum = 0.0F;
  }
}

__global__ void reduceTier(const std::uint8_t* tier, const std::uint8_t* raw_collision,
                           const std::uint8_t* solid_collision, std::size_t count,
                           int* best_tier) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && raw_collision[index] == 0U && solid_collision[index] == 0U) {
    atomicMin(best_tier, static_cast<int>(tier[index]));
  }
}

__global__ void reduceCritical(const std::uint8_t* tier, const float* critical,
                               const std::uint8_t* raw_collision,
                               const std::uint8_t* solid_collision, std::size_t count,
                               const int* best_tier, float* best_critical) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && raw_collision[index] == 0U && solid_collision[index] == 0U &&
      static_cast<int>(tier[index]) == *best_tier) {
    atomicMinFloat(best_critical, critical[index]);
  }
}

__global__ void reducePlanning(const std::uint8_t* tier, const float* critical,
                               const float* planning, const std::uint8_t* raw_collision,
                               const std::uint8_t* solid_collision, std::size_t count,
                               const int* best_tier, const float* best_critical,
                               float critical_tolerance, float* best_planning) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && raw_collision[index] == 0U && solid_collision[index] == 0U &&
      static_cast<int>(tier[index]) == *best_tier &&
      critical[index] <= *best_critical + critical_tolerance) {
    atomicMinFloat(best_planning, planning[index]);
  }
}

__global__ void reduceSoft(const std::uint8_t* tier, const float* critical,
                           const float* planning, const float* soft,
                           const std::uint8_t* raw_collision,
                           const std::uint8_t* solid_collision, std::size_t count,
                           const int* best_tier, const float* best_critical,
                           const float* best_planning, float* minimum_soft,
                           RiskConfig risk) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && raw_collision[index] == 0U && solid_collision[index] == 0U &&
      static_cast<int>(tier[index]) == *best_tier &&
      critical[index] <= *best_critical + risk.critical_exposure_tolerance_m &&
      planning[index] <= *best_planning + risk.planning_exposure_tolerance_m) {
    atomicMinFloat(minimum_soft, soft[index]);
  }
}

__global__ void calculateWeights(const std::uint8_t* tier, const float* critical,
                                 const float* planning, const float* soft,
                                 const std::uint8_t* raw_collision,
                                 const std::uint8_t* solid_collision, float* weights,
                                 std::size_t count, const int* best_tier,
                                 const float* best_critical, const float* best_planning,
                                 const float* minimum_soft, RiskConfig risk,
                                 float temperature, float* weight_sum) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  float weight = 0.0F;
  if (raw_collision[index] == 0U && solid_collision[index] == 0U &&
      static_cast<int>(tier[index]) == *best_tier &&
      critical[index] <= *best_critical + risk.critical_exposure_tolerance_m &&
      planning[index] <= *best_planning + risk.planning_exposure_tolerance_m) {
    weight = expf(-(soft[index] - *minimum_soft) / temperature);
  }
  weights[index] = weight;
  atomicAdd(weight_sum, weight);
}

__global__ void updateControls(const Control* nominal, Control* updated,
                               const float* noise_ax, const float* noise_ay,
                               const float* noise_az, const float* noise_yaw,
                               const float* weights, const float* weight_sum,
                               std::size_t rollouts, std::size_t steps) {
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
    const std::size_t index = rollout * steps + step;
    const float weight = weights[rollout];
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

[[nodiscard]] bool hostSolidCollision(const State& state,
                                      std::span<const KnownSolid> solids) {
  return std::ranges::any_of(solids, [&state](const KnownSolid& solid) {
    if (state.z < solid.min_z_m || state.z > solid.max_z_m) {
      return false;
    }
    const float dx = state.x - solid.center_x_m;
    const float dy = state.y - solid.center_y_m;
    return std::abs(dx * solid.normal_x + dy * solid.normal_y) <= solid.half_depth_m &&
           std::abs(dx * solid.lateral_x + dy * solid.lateral_y) <= solid.half_width_m;
  });
}

} // namespace

class MppiCudaEngine::Impl {
public:
  explicit Impl(BenchmarkConfig config)
      : config_{std::move(config)},
        buffers_{config_.rollouts, config_.steps},
        nominal_(config_.steps),
        updated_(config_.steps),
        zero_noise_(config_.steps) {
    if (!benchmarkConfigIsValid(config_)) {
      throw std::invalid_argument{"invalid MPPI engine configuration"};
    }
    checkCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
              "cudaStreamCreateWithFlags");
    checkCuda(cudaMemcpyAsync(buffers_.nominal.get(), nominal_.data(),
                              nominal_.size() * sizeof(Control), cudaMemcpyHostToDevice,
                              stream_),
              "copy initial nominal controls");
    checkCuda(cudaStreamSynchronize(stream_), "initialize MPPI engine");
  }

  ~Impl() {
    if (stream_ != nullptr) {
      (void)cudaStreamDestroy(stream_);
    }
  }

  EsdfUploadResult updateEsdf(const EsdfSnapshot& snapshot) {
    std::scoped_lock lock{mutex_};
    const std::size_t expected = static_cast<std::size_t>(snapshot.grid.width) *
                                 static_cast<std::size_t>(snapshot.grid.height);
    if (snapshot.grid.width <= 1 || snapshot.grid.height <= 1 ||
        !(snapshot.grid.resolution_m > 0.0F) ||
        snapshot.distances_m.size() != expected) {
      return {};
    }
    const std::size_t inactive = 1U - active_texture_;
    const double upload_ms = textures_[inactive].upload(snapshot, stream_);
    esdf_host_[inactive].assign(snapshot.distances_m.begin(),
                                snapshot.distances_m.end());
    active_texture_ = inactive;
    return EsdfUploadResult{true, upload_ms, snapshot.revision};
  }

  void updateKnownSolids(std::span<const KnownSolid> solids) {
    std::scoped_lock lock{mutex_};
    if (solids.size() > kMaximumKnownSolids) {
      throw std::invalid_argument{"too many known solids for MPPI engine"};
    }
    known_solids_.assign(solids.begin(), solids.end());
    solid_count_ = solids.size();
    if (!solids.empty()) {
      checkCuda(cudaMemcpyAsync(buffers_.solids.get(), solids.data(),
                                solids.size_bytes(), cudaMemcpyHostToDevice, stream_),
                "upload known solids");
      checkCuda(cudaStreamSynchronize(stream_), "synchronize known solids upload");
    }
  }

  MppiTickResult plan(const MppiTickInput& input) {
    std::scoped_lock lock{mutex_};
    if (!textures_[active_texture_].ready()) {
      throw std::runtime_error{"MPPI engine has no ESDF"};
    }
    const auto host_started = std::chrono::steady_clock::now();
    const std::size_t noise_count = config_.rollouts * config_.steps;
    const int noise_blocks =
        static_cast<int>((noise_count + kThreadsPerBlock - 1U) / kThreadsPerBlock);
    const int rollout_blocks =
        static_cast<int>((config_.rollouts + kThreadsPerBlock - 1U) / kThreadsPerBlock);
    const int control_blocks =
        static_cast<int>((config_.steps + kThreadsPerBlock - 1U) / kThreadsPerBlock);
    double elapsed_s = 0.0;
    if (has_updated_) {
      if (last_planning_stamp_ns_ > 0 &&
          input.planning_stamp_ns > last_planning_stamp_ns_) {
        elapsed_s =
            static_cast<double>(input.planning_stamp_ns - last_planning_stamp_ns_) /
            1.0e9;
      } else {
        elapsed_s = config_.dynamics.dt_s;
      }
    }
    const bool nominal_reseeded =
        input.nominal_reseed_generation > nominal_reseed_generation_;
    if (nominal_reseeded) {
      nominal_ = buildGuideDirectedNominalSeed(input.initial_state, input.target,
                                               config_.dynamics, config_.steps,
                                               input.nominal_reseed_generation);
      nominal_reseed_generation_ = input.nominal_reseed_generation;
    } else if (has_updated_) {
      nominal_ = shiftControlSequence(updated_, config_.dynamics.dt_s, elapsed_s);
    }
    const Control previous_applied_control = input.previous_applied_control.value_or(
        last_output_control_.value_or(Control{}));
    const float first_control_interval_s =
        has_updated_
            ? std::clamp(static_cast<float>(elapsed_s), 1.0e-3F, config_.dynamics.dt_s)
            : config_.dynamics.dt_s;

    started_.record(stream_);
    checkCuda(cudaMemcpyAsync(buffers_.nominal.get(), nominal_.data(),
                              nominal_.size() * sizeof(Control), cudaMemcpyHostToDevice,
                              stream_),
              "copy time-shifted nominal controls");
    warm_done_.record(stream_);
    generateNoise<<<noise_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
        buffers_.noise_yaw.get(), noise_count, config_.seed, tick_sequence_++,
        config_.noise);
    noise_done_.record(stream_);
    simulate<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.noise_ax.get(), buffers_.noise_ay.get(), buffers_.noise_az.get(),
        buffers_.noise_yaw.get(), buffers_.nominal.get(), buffers_.soft_cost.get(),
        buffers_.critical_exposure.get(), buffers_.planning_exposure.get(),
        buffers_.minimum_clearance.get(), buffers_.worst_tier.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(), config_.rollouts,
        config_.steps, input.initial_state, input.target, config_.dynamics,
        config_.risk, config_.costs, textures_[active_texture_].grid(),
        textures_[active_texture_].texture(), buffers_.solids.get(), solid_count_,
        input.passage.value_or(PassageConstraint{}), input.passage.has_value(),
        previous_applied_control, input.reference_speed_mps,
        config_.early_exit_on_collision);
    simulation_done_.record(stream_);
    initializeReduction<<<1, 1, 0U, stream_>>>(
        buffers_.best_tier.get(), buffers_.best_critical.get(),
        buffers_.best_planning.get(), buffers_.minimum_soft.get(),
        buffers_.weight_sum.get());
    reduceTier<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.raw_collision.get(),
        buffers_.solid_collision.get(), config_.rollouts, buffers_.best_tier.get());
    reduceCritical<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(), config_.rollouts,
        buffers_.best_tier.get(), buffers_.best_critical.get());
    reducePlanning<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.planning_exposure.get(), buffers_.raw_collision.get(),
        buffers_.solid_collision.get(), config_.rollouts, buffers_.best_tier.get(),
        buffers_.best_critical.get(), config_.risk.critical_exposure_tolerance_m,
        buffers_.best_planning.get());
    reduceSoft<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.planning_exposure.get(), buffers_.soft_cost.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(), config_.rollouts,
        buffers_.best_tier.get(), buffers_.best_critical.get(),
        buffers_.best_planning.get(), buffers_.minimum_soft.get(), config_.risk);
    reduction_done_.record(stream_);
    calculateWeights<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.planning_exposure.get(), buffers_.soft_cost.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(),
        buffers_.weights.get(), config_.rollouts, buffers_.best_tier.get(),
        buffers_.best_critical.get(), buffers_.best_planning.get(),
        buffers_.minimum_soft.get(), config_.risk, config_.costs.temperature,
        buffers_.weight_sum.get());
    weights_done_.record(stream_);
    updateControls<<<control_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.nominal.get(), buffers_.updated.get(), buffers_.noise_ax.get(),
        buffers_.noise_ay.get(), buffers_.noise_az.get(), buffers_.noise_yaw.get(),
        buffers_.weights.get(), buffers_.weight_sum.get(), config_.rollouts,
        config_.steps);
    limitControls<<<1, 1, 0U, stream_>>>(buffers_.updated.get(), config_.steps,
                                         config_.dynamics, previous_applied_control,
                                         first_control_interval_s);
    update_done_.record(stream_);
    checkCuda(cudaMemcpyAsync(updated_.data(), buffers_.updated.get(),
                              updated_.size() * sizeof(Control), cudaMemcpyDeviceToHost,
                              stream_),
              "copy selected controls");
    completed_.record(stream_);
    completed_.synchronize();
    checkCuda(cudaGetLastError(), "MPPI engine kernels");

    MppiTickResult result;
    result.controls = updated_;
    result.warm_start_shift_s = elapsed_s;
    result.nominal_reseeded = nominal_reseeded;
    result.esdf_revision = textures_[active_texture_].revision();
    result.timings.warm_start_ms = elapsedMs(started_, warm_done_);
    result.timings.noise_generation_ms = elapsedMs(warm_done_, noise_done_);
    result.timings.rollout_simulation_ms = elapsedMs(noise_done_, simulation_done_);
    result.timings.risk_reduction_ms = elapsedMs(simulation_done_, reduction_done_);
    result.timings.weight_calculation_ms = elapsedMs(reduction_done_, weights_done_);
    result.timings.control_update_ms = elapsedMs(weights_done_, update_done_);
    result.timings.gpu_total_ms = elapsedMs(started_, completed_);
    const auto reconstruction_started = std::chrono::steady_clock::now();
    State state = input.initial_state;
    result.horizon.reserve(updated_.size() + 1U);
    result.horizon.push_back(state);
    const float initial_distance =
        std::hypot(input.target.x - state.x, input.target.y - state.y);
    result.minimum_esdf_distance_m = kInfinity;
    result.selected_tier = RiskTier::kPreferred;
    Control previous_control = previous_applied_control;
    if (!updated_.empty()) {
      result.first_control_delta =
          std::hypot(std::hypot(updated_.front().ax - previous_applied_control.ax,
                                updated_.front().ay - previous_applied_control.ay),
                     updated_.front().az - previous_applied_control.az);
    }
    const std::size_t head_steps = std::clamp<std::size_t>(
        static_cast<std::size_t>(
            std::ceil(config_.costs.head_progress_horizon_s / config_.dynamics.dt_s)),
        1U, updated_.size());
    for (std::size_t index = 0U; index < updated_.size(); ++index) {
      const Control& control = updated_[index];
      result.maximum_acceleration_mps2 =
          std::max(result.maximum_acceleration_mps2,
                   std::hypot(std::hypot(control.ax, control.ay), control.az));
      result.maximum_jerk_mps3 = std::max(
          result.maximum_jerk_mps3,
          std::hypot(std::hypot(control.ax - previous_control.ax,
                                control.ay - previous_control.ay),
                     control.az - previous_control.az) /
              (index == 0U ? first_control_interval_s : config_.dynamics.dt_s));
      previous_control = control;
      state = integrateReference(state, control, config_.dynamics);
      result.horizon.push_back(state);
      if (index + 1U == head_steps) {
        result.head_progress_m =
            initial_distance -
            std::hypot(input.target.x - state.x, input.target.y - state.y);
      }
      if (hostSolidCollision(state, known_solids_)) {
        result.known_solid_collision = true;
      }
    }
    const RolloutMetrics metrics = simulateReference(
        input.initial_state, updated_, zero_noise_, config_.dynamics, config_.risk,
        config_.costs, textures_[active_texture_].grid(), activeEsdfHost(),
        input.target.x, input.target.y, config_.early_exit_on_collision,
        previous_applied_control, input.reference_speed_mps);
    result.raw_collision = metrics.collision;
    result.critical_exposure_m = metrics.critical_exposure_m;
    result.planning_exposure_m = metrics.planning_exposure_m;
    result.minimum_esdf_distance_m = metrics.minimum_clearance_m;
    result.selected_tier =
        result.known_solid_collision ? RiskTier::kCollision : metrics.worst_tier;
    result.terminal_progress_m =
        initial_distance -
        std::hypot(input.target.x - state.x, input.target.y - state.y);
    result.timings.horizon_reconstruction_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  reconstruction_started)
            .count();
    result.timings.host_total_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - host_started)
                                       .count();
    has_updated_ = true;
    last_planning_stamp_ns_ = input.planning_stamp_ns;
    if (!updated_.empty()) {
      last_output_control_ = updated_.front();
    }
    return result;
  }

  [[nodiscard]] std::size_t allocatedBytes() const noexcept {
    return buffers_.bytes();
  }

  [[nodiscard]] bool ready() const noexcept {
    return textures_[active_texture_].ready();
  }

private:
  [[nodiscard]] std::span<const float> activeEsdfHost() const {
    return active_texture_ == 0U ? esdf_host_[0] : esdf_host_[1];
  }

  BenchmarkConfig config_;
  DeviceBuffers buffers_;
  EsdfTexture textures_[2];
  std::vector<float> esdf_host_[2];
  std::size_t active_texture_{0U};
  std::vector<KnownSolid> known_solids_;
  std::size_t solid_count_{0U};
  std::vector<Control> nominal_;
  std::vector<Control> updated_;
  std::vector<Control> zero_noise_;
  std::optional<Control> last_output_control_;
  std::int64_t last_planning_stamp_ns_{0};
  std::uint64_t nominal_reseed_generation_{0U};
  bool has_updated_{false};
  std::uint64_t tick_sequence_{0U};
  cudaStream_t stream_{nullptr};
  Event started_;
  Event noise_done_;
  Event simulation_done_;
  Event reduction_done_;
  Event weights_done_;
  Event update_done_;
  Event warm_done_;
  Event completed_;
  mutable std::mutex mutex_;
};

MppiCudaEngine::MppiCudaEngine(BenchmarkConfig config)
    : impl_{std::make_unique<Impl>(std::move(config))} {
}

MppiCudaEngine::~MppiCudaEngine() = default;
MppiCudaEngine::MppiCudaEngine(MppiCudaEngine&&) noexcept = default;
MppiCudaEngine& MppiCudaEngine::operator=(MppiCudaEngine&&) noexcept = default;

EsdfUploadResult MppiCudaEngine::updateEsdf(const EsdfSnapshot& snapshot) {
  return impl_->updateEsdf(snapshot);
}

void MppiCudaEngine::updateKnownSolids(std::span<const KnownSolid> solids) {
  impl_->updateKnownSolids(solids);
}

MppiTickResult MppiCudaEngine::plan(const MppiTickInput& input) {
  return impl_->plan(input);
}

std::size_t MppiCudaEngine::allocatedBytes() const noexcept {
  return impl_->allocatedBytes();
}

bool MppiCudaEngine::ready() const noexcept {
  return impl_->ready();
}

} // namespace drone_city_nav::mppi
