#include "drone_city_nav/mppi/mppi_cuda.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cuda_runtime.h>
#include <iomanip>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

constexpr float kPi = 3.14159265358979323846F;

constexpr int kThreadsPerBlock{256};
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

  DeviceBuffer(DeviceBuffer&& other) noexcept
      : data_{std::exchange(other.data_, nullptr)},
        count_{std::exchange(other.count_, 0U)} {
  }

  DeviceBuffer& operator=(DeviceBuffer&& other) noexcept {
    if (this != &other) {
      release();
      data_ = std::exchange(other.data_, nullptr);
      count_ = std::exchange(other.count_, 0U);
    }
    return *this;
  }

  ~DeviceBuffer() {
    release();
  }

  void allocate(const std::size_t count) {
    release();
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
  void release() noexcept {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
    }
    data_ = nullptr;
    count_ = 0U;
  }

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

  void record() {
    checkCuda(cudaEventRecord(event_), "cudaEventRecord");
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

[[nodiscard]] float elapsedMs(const Event& start, const Event& end) {
  float elapsed = 0.0F;
  checkCuda(cudaEventElapsedTime(&elapsed, start.get(), end.get()),
            "cudaEventElapsedTime");
  return elapsed;
}

struct Scenario {
  EsdfGrid grid;
  std::vector<float> esdf;
  State initial;
  float target_x_m{0.0F};
  float target_y_m{0.0F};
};

struct QueueCell {
  float distance_cells{0.0F};
  int index{0};

  bool operator>(const QueueCell& other) const noexcept {
    return distance_cells > other.distance_cells;
  }
};

void fillRectangle(std::vector<std::uint8_t>& occupancy, const EsdfGrid& grid,
                   const float min_x, const float min_y, const float max_x,
                   const float max_y) {
  for (int y = 0; y < grid.height; ++y) {
    const float world_y =
        grid.origin_y_m + (static_cast<float>(y) + 0.5F) * grid.resolution_m;
    for (int x = 0; x < grid.width; ++x) {
      const float world_x =
          grid.origin_x_m + (static_cast<float>(x) + 0.5F) * grid.resolution_m;
      if (world_x >= min_x && world_x <= max_x && world_y >= min_y &&
          world_y <= max_y) {
        occupancy[static_cast<std::size_t>(y * grid.width + x)] = 1U;
      }
    }
  }
}

[[nodiscard]] std::vector<float> buildEsdf(const std::vector<std::uint8_t>& occupancy,
                                           const EsdfGrid& grid) {
  std::vector<float> distances(occupancy.size(), kInfinity);
  std::priority_queue<QueueCell, std::vector<QueueCell>, std::greater<>> queue;
  for (std::size_t index = 0U; index < occupancy.size(); ++index) {
    if (occupancy[index] != 0U) {
      distances[index] = 0.0F;
      queue.push(QueueCell{.distance_cells = 0.0F, .index = static_cast<int>(index)});
    }
  }
  constexpr std::array<std::pair<int, int>, 8U> kNeighbors{{
      {-1, -1},
      {0, -1},
      {1, -1},
      {-1, 0},
      {1, 0},
      {-1, 1},
      {0, 1},
      {1, 1},
  }};
  while (!queue.empty()) {
    const QueueCell current = queue.top();
    queue.pop();
    if (current.distance_cells >
        distances[static_cast<std::size_t>(current.index)] + 1.0e-6F) {
      continue;
    }
    const int x = current.index % grid.width;
    const int y = current.index / grid.width;
    for (const auto [dx, dy] : kNeighbors) {
      const int next_x = x + dx;
      const int next_y = y + dy;
      if (next_x < 0 || next_x >= grid.width || next_y < 0 || next_y >= grid.height) {
        continue;
      }
      const int next_index = next_y * grid.width + next_x;
      const float step = dx != 0 && dy != 0 ? std::sqrt(2.0F) : 1.0F;
      const float candidate = current.distance_cells + step;
      if (candidate + 1.0e-6F < distances[static_cast<std::size_t>(next_index)]) {
        distances[static_cast<std::size_t>(next_index)] = candidate;
        queue.push(QueueCell{.distance_cells = candidate, .index = next_index});
      }
    }
  }
  for (float& distance : distances) {
    distance *= grid.resolution_m;
  }
  return distances;
}

[[nodiscard]] Scenario makeScenario(const std::string& name,
                                    double& build_duration_ms) {
  Scenario scenario{
      .grid = EsdfGrid{.width = 512,
                       .height = 512,
                       .resolution_m = 0.5F,
                       .origin_x_m = 0.0F,
                       .origin_y_m = 0.0F},
      .initial = State{.x = 24.0F, .y = 128.0F, .z = 18.0F, .vx = 8.0F},
      .target_x_m = 230.0F,
      .target_y_m = 128.0F,
  };
  std::vector<std::uint8_t> occupancy(
      static_cast<std::size_t>(scenario.grid.width * scenario.grid.height), 0U);
  fillRectangle(occupancy, scenario.grid, 0.0F, 0.0F, 255.5F, 0.5F);
  fillRectangle(occupancy, scenario.grid, 0.0F, 255.0F, 255.5F, 255.5F);
  fillRectangle(occupancy, scenario.grid, 0.0F, 0.0F, 0.5F, 255.5F);
  fillRectangle(occupancy, scenario.grid, 255.0F, 0.0F, 255.5F, 255.5F);

  if (name == "single_wall") {
    fillRectangle(occupancy, scenario.grid, 100.0F, 80.0F, 102.0F, 176.0F);
  } else if (name == "parallel_walls") {
    fillRectangle(occupancy, scenario.grid, 40.0F, 105.0F, 230.0F, 110.0F);
    fillRectangle(occupancy, scenario.grid, 40.0F, 146.0F, 230.0F, 151.0F);
  } else if (name == "narrow_corridor") {
    fillRectangle(occupancy, scenario.grid, 40.0F, 118.0F, 230.0F, 123.0F);
    fillRectangle(occupancy, scenario.grid, 40.0F, 133.0F, 230.0F, 138.0F);
  } else if (name == "building_block") {
    fillRectangle(occupancy, scenario.grid, 105.0F, 98.0F, 145.0F, 158.0F);
  } else if (name == "u_shaped_obstacle") {
    fillRectangle(occupancy, scenario.grid, 100.0F, 80.0F, 105.0F, 175.0F);
    fillRectangle(occupancy, scenario.grid, 100.0F, 80.0F, 165.0F, 85.0F);
    fillRectangle(occupancy, scenario.grid, 100.0F, 170.0F, 165.0F, 175.0F);
  } else if (name == "urban_blocks") {
    for (int column = 0; column < 4; ++column) {
      for (int row = 0; row < 3; ++row) {
        const float x = 62.0F + static_cast<float>(column) * 45.0F;
        const float y = 55.0F + static_cast<float>(row) * 65.0F;
        fillRectangle(occupancy, scenario.grid, x, y, x + 25.0F, y + 40.0F);
      }
    }
  } else if (name == "passage_lower_upper") {
    fillRectangle(occupancy, scenario.grid, 105.0F, 75.0F, 115.0F, 120.0F);
    fillRectangle(occupancy, scenario.grid, 105.0F, 136.0F, 115.0F, 181.0F);
  } else if (name == "random_occupancy") {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;
    for (int index = 0; index < 30; ++index) {
      state ^= state >> 12U;
      state ^= state << 25U;
      state ^= state >> 27U;
      const float x = 45.0F + static_cast<float>(state % 350U) * 0.45F;
      state *= 2685821657736338717ULL;
      const float y = 30.0F + static_cast<float>(state % 380U) * 0.5F;
      fillRectangle(occupancy, scenario.grid, x, y, x + 5.0F, y + 8.0F);
    }
  } else if (name != "open_space") {
    throw std::invalid_argument{"unknown MPPI benchmark scenario: " + name};
  }
  const auto started_at = std::chrono::steady_clock::now();
  scenario.esdf = buildEsdf(occupancy, scenario.grid);
  build_duration_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - started_at)
                          .count();
  return scenario;
}

class EsdfTexture {
public:
  EsdfTexture(const Scenario& scenario, double& upload_ms) {
    const auto started_at = std::chrono::steady_clock::now();
    const cudaChannelFormatDesc channel = cudaCreateChannelDesc<float>();
    checkCuda(cudaMallocArray(&array_, &channel,
                              static_cast<std::size_t>(scenario.grid.width),
                              static_cast<std::size_t>(scenario.grid.height)),
              "cudaMallocArray");
    checkCuda(cudaMemcpy2DToArray(
                  array_, 0U, 0U, scenario.esdf.data(),
                  static_cast<std::size_t>(scenario.grid.width) * sizeof(float),
                  static_cast<std::size_t>(scenario.grid.width) * sizeof(float),
                  static_cast<std::size_t>(scenario.grid.height),
                  cudaMemcpyHostToDevice),
              "cudaMemcpy2DToArray");
    cudaResourceDesc resource{};
    resource.resType = cudaResourceTypeArray;
    resource.res.array.array = array_;
    cudaTextureDesc texture{};
    texture.addressMode[0] = cudaAddressModeBorder;
    texture.addressMode[1] = cudaAddressModeBorder;
    texture.filterMode = cudaFilterModePoint;
    texture.readMode = cudaReadModeElementType;
    texture.normalizedCoords = 0;
    checkCuda(cudaCreateTextureObject(&texture_, &resource, &texture, nullptr),
              "cudaCreateTextureObject");
    upload_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started_at)
                    .count();
  }

  EsdfTexture(const EsdfTexture&) = delete;
  EsdfTexture& operator=(const EsdfTexture&) = delete;

  ~EsdfTexture() {
    if (texture_ != 0U) {
      (void)cudaDestroyTextureObject(texture_);
    }
    if (array_ != nullptr) {
      (void)cudaFreeArray(array_);
    }
  }

  [[nodiscard]] cudaTextureObject_t get() const noexcept {
    return texture_;
  }

private:
  cudaArray_t array_{nullptr};
  cudaTextureObject_t texture_{0U};
};

struct DeviceRolloutBuffers {
  DeviceBuffer<float> noise_ax;
  DeviceBuffer<float> noise_ay;
  DeviceBuffer<float> noise_az;
  DeviceBuffer<float> noise_yaw;
  DeviceBuffer<float> soft_cost;
  DeviceBuffer<float> critical_exposure;
  DeviceBuffer<float> planning_exposure;
  DeviceBuffer<float> minimum_clearance;
  DeviceBuffer<std::uint8_t> worst_tier;
  DeviceBuffer<std::uint8_t> collision;
  DeviceBuffer<float> weights;
  DeviceBuffer<Control> nominal;
  DeviceBuffer<Control> updated;
  DeviceBuffer<int> best_tier;
  DeviceBuffer<float> best_critical;
  DeviceBuffer<float> best_planning;
  DeviceBuffer<float> minimum_soft;
  DeviceBuffer<float> weight_sum;

  DeviceRolloutBuffers(const std::size_t rollouts, const std::size_t steps)
      : noise_ax{rollouts * steps},
        noise_ay{rollouts * steps},
        noise_az{rollouts * steps},
        noise_yaw{rollouts * steps},
        soft_cost{rollouts},
        critical_exposure{rollouts},
        planning_exposure{rollouts},
        minimum_clearance{rollouts},
        worst_tier{rollouts},
        collision{rollouts},
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
           minimum_clearance.bytes() + worst_tier.bytes() + collision.bytes() +
           weights.bytes() + nominal.bytes() + updated.bytes() + best_tier.bytes() +
           best_critical.bytes() + best_planning.bytes() + minimum_soft.bytes() +
           weight_sum.bytes();
  }
};

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
  state.vx = state.vx * drag + control.ax * config.dt_s;
  state.vy = state.vy * drag + control.ay * config.dt_s;
  state.vz = state.vz * drag + control.az * config.dt_s;
  clampHorizontalDevice(state.vx, state.vy, config.maximum_horizontal_speed_mps);
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

struct DeviceEsdfQuery {
  float clearance_m;
  bool raw_collision;
};

__device__ DeviceEsdfQuery
queryEsdf(const State& state, const EsdfGrid grid,
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
  const float correction_m =
      hypotf(state.x - center_x_m, state.y - center_y_m) +
      kHalfDiagonalScale * grid.resolution_m;
  return {fmaxf(0.0F, center_distance_m - correction_m),
          center_distance_m == 0.0F};
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
  float minimum_clearance_m = kInfinity;
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
    const float segment_m = dynamics.dt_s * hypotf(state.vx, state.vy);
    if (esdf_query.raw_collision) {
      collided = true;
      tier = static_cast<std::uint8_t>(RiskTier::kCollision);
    } else if (clearance < risk.critical_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kCritical));
      critical_m += segment_m;
    } else if (clearance < risk.preferred_distance_m) {
      tier = max(tier, static_cast<std::uint8_t>(RiskTier::kPlanning));
      planning_m += segment_m;
    }
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
  soft_cost[rollout] = costs.head_progress_weight * -head_progress +
                       costs.progress_weight * progress_cost +
                       costs.guide_deviation_weight * dynamics.dt_s * guide_cost +
                       costs.acceleration_weight * dynamics.dt_s * acceleration_cost +
                       costs.jerk_weight * jerk_cost +
                       costs.yaw_change_weight * yaw_cost +
                       costs.control_effort_weight * dynamics.dt_s * effort_cost +
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

[[nodiscard]] TimingStatistics statistics(std::vector<double> samples) {
  TimingStatistics result{};
  if (samples.empty()) {
    return result;
  }
  result.mean_ms = std::accumulate(samples.begin(), samples.end(), 0.0) /
                   static_cast<double>(samples.size());
  std::sort(samples.begin(), samples.end());
  const auto percentile = [&samples](const double ratio) {
    const std::size_t index =
        std::min(samples.size() - 1U,
                 static_cast<std::size_t>(
                     std::ceil(ratio * static_cast<double>(samples.size()))) -
                     1U);
    return samples[index];
  };
  result.p50_ms = percentile(0.50);
  result.p90_ms = percentile(0.90);
  result.p95_ms = percentile(0.95);
  result.p99_ms = percentile(0.99);
  result.maximum_ms = samples.back();
  return result;
}

struct TimingSamples {
  std::vector<double> noise;
  std::vector<double> simulation;
  std::vector<double> risk;
  std::vector<double> weights;
  std::vector<double> update;
  std::vector<double> warm_start;
  std::vector<double> gpu_total;
  std::vector<double> host_total;
};

} // namespace

BenchmarkResult runCudaBenchmark(const BenchmarkConfig& config) {
  if (!benchmarkConfigIsValid(config)) {
    throw std::invalid_argument{"invalid MPPI benchmark configuration"};
  }
  BenchmarkResult result{};
  int device = 0;
  checkCuda(cudaGetDevice(&device), "cudaGetDevice");
  cudaDeviceProp properties{};
  checkCuda(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
  result.gpu_name = properties.name;
  result.compute_major = properties.major;
  result.compute_minor = properties.minor;

  Scenario scenario = makeScenario(config.scenario, result.esdf_build_ms);
  EsdfTexture texture{scenario, result.esdf_upload_ms};
  const auto allocation_started = std::chrono::steady_clock::now();
  DeviceRolloutBuffers buffers{config.rollouts, config.steps};
  std::vector<Control> nominal(config.steps);
  checkCuda(cudaMemcpy(buffers.nominal.get(), nominal.data(),
                       nominal.size() * sizeof(Control), cudaMemcpyHostToDevice),
            "copy nominal controls");
  result.allocated_bytes = buffers.bytes() + scenario.esdf.size() * sizeof(float);
  result.allocation_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - allocation_started)
                             .count();

  Event tick_start;
  Event after_noise;
  Event after_simulation;
  Event after_risk;
  Event after_weights;
  Event after_update;
  Event after_warm_start;
  const std::size_t noise_count = config.rollouts * config.steps;
  const int noise_blocks =
      static_cast<int>((noise_count + kThreadsPerBlock - 1U) / kThreadsPerBlock);
  const int rollout_blocks =
      static_cast<int>((config.rollouts + kThreadsPerBlock - 1U) / kThreadsPerBlock);
  const int control_blocks =
      static_cast<int>((config.steps + kThreadsPerBlock - 1U) / kThreadsPerBlock);
  TimingSamples measured;
  const std::size_t total_ticks = config.warmup_ticks + config.measured_ticks;

  for (std::size_t tick = 0U; tick < total_ticks; ++tick) {
    const auto host_started = std::chrono::steady_clock::now();
    tick_start.record();
    generateNoiseKernel<<<noise_blocks, kThreadsPerBlock>>>(
        buffers.noise_ax.get(), buffers.noise_ay.get(), buffers.noise_az.get(),
        buffers.noise_yaw.get(), noise_count, config.seed,
        static_cast<std::uint64_t>(tick), config.noise);
    after_noise.record();
    simulateKernel<<<rollout_blocks, kThreadsPerBlock>>>(
        buffers.noise_ax.get(), buffers.noise_ay.get(), buffers.noise_az.get(),
        buffers.noise_yaw.get(), buffers.nominal.get(), buffers.soft_cost.get(),
        buffers.critical_exposure.get(), buffers.planning_exposure.get(),
        buffers.minimum_clearance.get(), buffers.worst_tier.get(),
        buffers.collision.get(), config.rollouts, config.steps, scenario.initial,
        scenario.target_x_m, scenario.target_y_m, config.dynamics, config.risk,
        config.costs, scenario.grid, texture.get(), config.early_exit_on_collision);
    after_simulation.record();
    initializeReductionKernel<<<1, 1>>>(
        buffers.best_tier.get(), buffers.best_critical.get(),
        buffers.best_planning.get(), buffers.minimum_soft.get(),
        buffers.weight_sum.get());
    reduceTierKernel<<<rollout_blocks, kThreadsPerBlock>>>(
        buffers.worst_tier.get(), buffers.collision.get(), config.rollouts,
        buffers.best_tier.get());
    reduceCriticalKernel<<<rollout_blocks, kThreadsPerBlock>>>(
        buffers.worst_tier.get(), buffers.critical_exposure.get(),
        buffers.collision.get(), config.rollouts, buffers.best_tier.get(),
        buffers.best_critical.get());
    reducePlanningKernel<<<rollout_blocks, kThreadsPerBlock>>>(
        buffers.worst_tier.get(), buffers.critical_exposure.get(),
        buffers.planning_exposure.get(), buffers.collision.get(), config.rollouts,
        buffers.best_tier.get(), buffers.best_critical.get(),
        config.risk.critical_exposure_tolerance_m, buffers.best_planning.get());
    reduceSoftKernel<<<rollout_blocks, kThreadsPerBlock>>>(
        buffers.worst_tier.get(), buffers.critical_exposure.get(),
        buffers.planning_exposure.get(), buffers.soft_cost.get(),
        buffers.collision.get(), config.rollouts, buffers.best_tier.get(),
        buffers.best_critical.get(), buffers.best_planning.get(), config.risk,
        buffers.minimum_soft.get());
    after_risk.record();
    calculateWeightsKernel<<<rollout_blocks, kThreadsPerBlock>>>(
        buffers.worst_tier.get(), buffers.critical_exposure.get(),
        buffers.planning_exposure.get(), buffers.soft_cost.get(),
        buffers.collision.get(), buffers.weights.get(), config.rollouts,
        buffers.best_tier.get(), buffers.best_critical.get(),
        buffers.best_planning.get(), buffers.minimum_soft.get(), config.risk,
        config.costs.temperature, buffers.weight_sum.get());
    after_weights.record();
    updateControlsKernel<<<control_blocks, kThreadsPerBlock>>>(
        buffers.nominal.get(), buffers.updated.get(), buffers.noise_ax.get(),
        buffers.noise_ay.get(), buffers.noise_az.get(), buffers.noise_yaw.get(),
        buffers.weights.get(), buffers.weight_sum.get(), config.rollouts, config.steps);
    after_update.record();
    warmStartKernel<<<control_blocks, kThreadsPerBlock>>>(
        buffers.nominal.get(), buffers.updated.get(), config.steps);
    after_warm_start.record();
    after_warm_start.synchronize();
    checkCuda(cudaGetLastError(), "MPPI benchmark kernels");
    const double host_total_ms = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - host_started)
                                     .count();
    if (tick < config.warmup_ticks) {
      continue;
    }
    measured.noise.push_back(elapsedMs(tick_start, after_noise));
    measured.simulation.push_back(elapsedMs(after_noise, after_simulation));
    measured.risk.push_back(elapsedMs(after_simulation, after_risk));
    measured.weights.push_back(elapsedMs(after_risk, after_weights));
    measured.update.push_back(elapsedMs(after_weights, after_update));
    measured.warm_start.push_back(elapsedMs(after_update, after_warm_start));
    measured.gpu_total.push_back(elapsedMs(tick_start, after_warm_start));
    measured.host_total.push_back(host_total_ms);
    if (host_total_ms > config.deadline_ms) {
      ++result.deadline_misses;
    }
  }
  result.deadline_miss_ratio = static_cast<double>(result.deadline_misses) /
                               static_cast<double>(config.measured_ticks);
  result.timings.noise_generation = statistics(std::move(measured.noise));
  result.timings.rollout_simulation = statistics(std::move(measured.simulation));
  result.timings.risk_reduction = statistics(std::move(measured.risk));
  result.timings.weight_calculation = statistics(std::move(measured.weights));
  result.timings.control_update = statistics(std::move(measured.update));
  result.timings.warm_start = statistics(std::move(measured.warm_start));
  result.timings.gpu_total = statistics(std::move(measured.gpu_total));
  result.timings.host_total = statistics(std::move(measured.host_total));

  checkCuda(cudaMemcpy(nominal.data(), buffers.nominal.get(),
                       nominal.size() * sizeof(Control), cudaMemcpyDeviceToHost),
            "copy selected controls");
  const std::vector<Control> zero_noise(config.steps);
  result.selected = simulateReference(
      scenario.initial, nominal, zero_noise, config.dynamics, config.risk, config.costs,
      scenario.grid, scenario.esdf, scenario.target_x_m, scenario.target_y_m,
      config.early_exit_on_collision);

  const std::size_t check_steps = std::min<std::size_t>(config.steps, 10U);
  std::vector<Control> check_noise(check_steps);
  std::vector<Control> check_nominal(
      nominal.begin(), nominal.begin() + static_cast<std::ptrdiff_t>(check_steps));
  std::vector<float> copied_ax(check_steps);
  std::vector<float> copied_ay(check_steps);
  std::vector<float> copied_az(check_steps);
  std::vector<float> copied_yaw(check_steps);
  checkCuda(cudaMemcpy(copied_ax.data(), buffers.noise_ax.get(),
                       check_steps * sizeof(float), cudaMemcpyDeviceToHost),
            "copy reference ax noise");
  checkCuda(cudaMemcpy(copied_ay.data(), buffers.noise_ay.get(),
                       check_steps * sizeof(float), cudaMemcpyDeviceToHost),
            "copy reference ay noise");
  checkCuda(cudaMemcpy(copied_az.data(), buffers.noise_az.get(),
                       check_steps * sizeof(float), cudaMemcpyDeviceToHost),
            "copy reference az noise");
  checkCuda(cudaMemcpy(copied_yaw.data(), buffers.noise_yaw.get(),
                       check_steps * sizeof(float), cudaMemcpyDeviceToHost),
            "copy reference yaw noise");
  for (std::size_t step = 0U; step < check_steps; ++step) {
    check_noise[step] =
        Control{copied_ax[step], copied_ay[step], copied_az[step], copied_yaw[step]};
  }
  checkCuda(cudaMemcpy(buffers.nominal.get(), check_nominal.data(),
                       check_nominal.size() * sizeof(Control), cudaMemcpyHostToDevice),
            "copy reference nominal controls");
  simulateKernel<<<1, 1>>>(
      buffers.noise_ax.get(), buffers.noise_ay.get(), buffers.noise_az.get(),
      buffers.noise_yaw.get(), buffers.nominal.get(), buffers.soft_cost.get(),
      buffers.critical_exposure.get(), buffers.planning_exposure.get(),
      buffers.minimum_clearance.get(), buffers.worst_tier.get(),
      buffers.collision.get(), 1U, check_steps, scenario.initial, scenario.target_x_m,
      scenario.target_y_m, config.dynamics, config.risk, config.costs, scenario.grid,
      texture.get(), config.early_exit_on_collision);
  checkCuda(cudaDeviceSynchronize(), "synchronize reference rollout");

  float gpu_soft_cost = 0.0F;
  float gpu_critical_exposure = 0.0F;
  float gpu_planning_exposure = 0.0F;
  float gpu_minimum_clearance = 0.0F;
  std::uint8_t gpu_tier = 0U;
  std::uint8_t gpu_collision = 0U;
  checkCuda(cudaMemcpy(&gpu_soft_cost, buffers.soft_cost.get(), sizeof(float),
                       cudaMemcpyDeviceToHost),
            "copy reference soft cost");
  checkCuda(cudaMemcpy(&gpu_critical_exposure, buffers.critical_exposure.get(),
                       sizeof(float), cudaMemcpyDeviceToHost),
            "copy reference critical exposure");
  checkCuda(cudaMemcpy(&gpu_planning_exposure, buffers.planning_exposure.get(),
                       sizeof(float), cudaMemcpyDeviceToHost),
            "copy reference planning exposure");
  checkCuda(cudaMemcpy(&gpu_minimum_clearance, buffers.minimum_clearance.get(),
                       sizeof(float), cudaMemcpyDeviceToHost),
            "copy reference minimum clearance");
  checkCuda(cudaMemcpy(&gpu_tier, buffers.worst_tier.get(), sizeof(std::uint8_t),
                       cudaMemcpyDeviceToHost),
            "copy reference risk tier");
  checkCuda(cudaMemcpy(&gpu_collision, buffers.collision.get(), sizeof(std::uint8_t),
                       cudaMemcpyDeviceToHost),
            "copy reference collision");
  const RolloutMetrics reference = simulateReference(
      scenario.initial, check_nominal, check_noise, config.dynamics, config.risk,
      config.costs, scenario.grid, scenario.esdf, scenario.target_x_m,
      scenario.target_y_m, config.early_exit_on_collision);
  const auto approximately_equal = [](const float lhs, const float rhs) {
    const float scale = std::max({1.0F, std::abs(lhs), std::abs(rhs)});
    return std::abs(lhs - rhs) <= 2.0e-3F * scale;
  };
  result.reference_check_passed =
      approximately_equal(gpu_soft_cost, reference.soft_cost) &&
      approximately_equal(gpu_critical_exposure, reference.critical_exposure_m) &&
      approximately_equal(gpu_planning_exposure, reference.planning_exposure_m) &&
      approximately_equal(gpu_minimum_clearance, reference.minimum_clearance_m) &&
      gpu_tier == static_cast<std::uint8_t>(reference.worst_tier) &&
      (gpu_collision != 0U) == reference.collision;

  constexpr std::size_t kReplayValues{256U};
  const std::size_t replay_values = std::min(kReplayValues, noise_count);
  std::vector<float> first_noise(replay_values);
  std::vector<float> replayed_noise(replay_values);
  const std::uint64_t replay_tick = static_cast<std::uint64_t>(total_ticks);
  generateNoiseKernel<<<noise_blocks, kThreadsPerBlock>>>(
      buffers.noise_ax.get(), buffers.noise_ay.get(), buffers.noise_az.get(),
      buffers.noise_yaw.get(), noise_count, config.seed, replay_tick, config.noise);
  checkCuda(cudaMemcpy(first_noise.data(), buffers.noise_ax.get(),
                       replay_values * sizeof(float), cudaMemcpyDeviceToHost),
            "copy first deterministic noise");
  generateNoiseKernel<<<noise_blocks, kThreadsPerBlock>>>(
      buffers.noise_ax.get(), buffers.noise_ay.get(), buffers.noise_az.get(),
      buffers.noise_yaw.get(), noise_count, config.seed, replay_tick, config.noise);
  checkCuda(cudaMemcpy(replayed_noise.data(), buffers.noise_ax.get(),
                       replay_values * sizeof(float), cudaMemcpyDeviceToHost),
            "copy replayed deterministic noise");
  result.deterministic_replay_passed = first_noise == replayed_noise;
  return result;
}

BenchmarkResult runPersistentCudaBenchmark(const BenchmarkConfig& config) {
  if (!benchmarkConfigIsValid(config)) {
    throw std::invalid_argument{"invalid MPPI benchmark configuration"};
  }
  BenchmarkResult result{};
  int device = 0;
  checkCuda(cudaGetDevice(&device), "cudaGetDevice");
  cudaDeviceProp properties{};
  checkCuda(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
  result.gpu_name = properties.name;
  result.compute_major = properties.major;
  result.compute_minor = properties.minor;

  Scenario scenario = makeScenario(config.scenario, result.esdf_build_ms);
  const auto allocation_started = std::chrono::steady_clock::now();
  MppiCudaEngine engine{config};
  result.allocation_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - allocation_started)
                             .count();
  const EsdfUploadResult upload =
      engine.updateEsdf(EsdfSnapshot{scenario.grid, scenario.esdf, 1U});
  if (!upload.accepted) {
    throw std::runtime_error{"persistent MPPI engine rejected synthetic ESDF"};
  }
  result.esdf_upload_ms = upload.upload_ms;
  result.allocated_bytes =
      engine.allocatedBytes() + scenario.esdf.size() * sizeof(float) * 2U;

  TimingSamples measured;
  MppiTickResult selected;
  const State target{scenario.target_x_m, scenario.target_y_m, scenario.initial.z};
  const std::size_t total_ticks = config.warmup_ticks + config.measured_ticks;
  for (std::size_t tick = 0U; tick < total_ticks; ++tick) {
    selected = engine.plan(MppiTickInput{.initial_state = scenario.initial,
                                         .target = target,
                                         .pose_revision = static_cast<std::uint64_t>(tick),
                                         .obstacle_revision = 1U});
    if (tick < config.warmup_ticks) {
      continue;
    }
    measured.noise.push_back(selected.timings.noise_generation_ms);
    measured.simulation.push_back(selected.timings.rollout_simulation_ms);
    measured.risk.push_back(selected.timings.risk_reduction_ms);
    measured.weights.push_back(selected.timings.weight_calculation_ms);
    measured.update.push_back(selected.timings.control_update_ms);
    measured.warm_start.push_back(selected.timings.warm_start_ms);
    measured.gpu_total.push_back(selected.timings.gpu_total_ms);
    measured.host_total.push_back(selected.timings.host_total_ms);
    if (selected.timings.host_total_ms > config.deadline_ms) {
      ++result.deadline_misses;
    }
  }
  result.deadline_miss_ratio = static_cast<double>(result.deadline_misses) /
                               static_cast<double>(config.measured_ticks);
  result.timings.noise_generation = statistics(std::move(measured.noise));
  result.timings.rollout_simulation = statistics(std::move(measured.simulation));
  result.timings.risk_reduction = statistics(std::move(measured.risk));
  result.timings.weight_calculation = statistics(std::move(measured.weights));
  result.timings.control_update = statistics(std::move(measured.update));
  result.timings.warm_start = statistics(std::move(measured.warm_start));
  result.timings.gpu_total = statistics(std::move(measured.gpu_total));
  result.timings.host_total = statistics(std::move(measured.host_total));

  const std::vector<Control> zero_noise(config.steps);
  result.selected = simulateReference(
      scenario.initial, selected.controls, zero_noise, config.dynamics, config.risk,
      config.costs, scenario.grid, scenario.esdf, scenario.target_x_m,
      scenario.target_y_m, config.early_exit_on_collision);
  result.reference_check_passed = result.selected.collision == selected.raw_collision &&
                                  result.selected.worst_tier == selected.selected_tier;

  MppiCudaEngine replay_engine{config};
  const EsdfUploadResult replay_upload =
      replay_engine.updateEsdf(EsdfSnapshot{scenario.grid, scenario.esdf, 1U});
  if (!replay_upload.accepted) {
    throw std::runtime_error{"persistent replay engine rejected synthetic ESDF"};
  }
  const MppiTickInput replay_input{.initial_state = scenario.initial,
                                   .target = target,
                                   .obstacle_revision = 1U};
  MppiCudaEngine first_engine{config};
  const EsdfUploadResult first_upload =
      first_engine.updateEsdf(EsdfSnapshot{scenario.grid, scenario.esdf, 1U});
  if (!first_upload.accepted) {
    throw std::runtime_error{"persistent reference engine rejected synthetic ESDF"};
  }
  const std::vector<Control> first_controls = first_engine.plan(replay_input).controls;
  const std::vector<Control> replay_controls =
      replay_engine.plan(replay_input).controls;
  result.deterministic_replay_passed =
      first_controls.size() == replay_controls.size() &&
      std::equal(first_controls.begin(), first_controls.end(), replay_controls.begin(),
                 [](const Control& first, const Control& replay) {
                   constexpr float kReplayTolerance{1.0e-4F};
                   return std::abs(first.ax - replay.ax) <= kReplayTolerance &&
                          std::abs(first.ay - replay.ay) <= kReplayTolerance &&
                          std::abs(first.az - replay.az) <= kReplayTolerance &&
                          std::abs(first.yaw_accel - replay.yaw_accel) <=
                              kReplayTolerance;
                 });
  return result;
}

} // namespace drone_city_nav::mppi
