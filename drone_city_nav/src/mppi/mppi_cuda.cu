#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/mppi/mppi_cuda.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cuda_runtime.h>
#include <iomanip>
#include <iterator>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
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
  OccupancyGrid2D source{GridBounds{grid.origin_x_m, grid.origin_y_m, grid.resolution_m,
                                    grid.width, grid.height}};
  source.reset(CellState::kFree);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y * grid.width + x);
      if (occupancy[index] != 0U) {
        source.setOccupied(GridIndex{x, y});
      }
    }
  }
  const DistanceField2D field =
      DistanceField2D::build(source, 1000.0, DistanceFieldSource::kOccupied);
  std::vector<float> distances;
  distances.reserve(field.distancesM().size());
  std::ranges::transform(field.distancesM(), std::back_inserter(distances),
                         [](const double value) { return static_cast<float>(value); });
  return distances;
}

[[nodiscard]] Scenario makeScenario(const std::string& name,
                                    double& build_duration_ms) {
  constexpr std::string_view kThreeDimensionalSuffix{"_3d"};
  const bool extrude_to_3d = name.ends_with(kThreeDimensionalSuffix);
  const std::string_view base_name =
      extrude_to_3d ? std::string_view{name}.substr(
                          0U, name.size() - kThreeDimensionalSuffix.size())
                    : std::string_view{name};
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

  if (base_name == "single_wall") {
    fillRectangle(occupancy, scenario.grid, 100.0F, 80.0F, 102.0F, 176.0F);
  } else if (base_name == "parallel_walls") {
    fillRectangle(occupancy, scenario.grid, 40.0F, 105.0F, 230.0F, 110.0F);
    fillRectangle(occupancy, scenario.grid, 40.0F, 146.0F, 230.0F, 151.0F);
  } else if (base_name == "narrow_corridor") {
    fillRectangle(occupancy, scenario.grid, 40.0F, 118.0F, 230.0F, 123.0F);
    fillRectangle(occupancy, scenario.grid, 40.0F, 133.0F, 230.0F, 138.0F);
  } else if (base_name == "building_block") {
    fillRectangle(occupancy, scenario.grid, 105.0F, 98.0F, 145.0F, 158.0F);
  } else if (base_name == "u_shaped_obstacle") {
    fillRectangle(occupancy, scenario.grid, 100.0F, 80.0F, 105.0F, 175.0F);
    fillRectangle(occupancy, scenario.grid, 100.0F, 80.0F, 165.0F, 85.0F);
    fillRectangle(occupancy, scenario.grid, 100.0F, 170.0F, 165.0F, 175.0F);
  } else if (base_name == "urban_blocks") {
    for (int column = 0; column < 4; ++column) {
      for (int row = 0; row < 3; ++row) {
        const float x = 62.0F + static_cast<float>(column) * 45.0F;
        const float y = 55.0F + static_cast<float>(row) * 65.0F;
        fillRectangle(occupancy, scenario.grid, x, y, x + 25.0F, y + 40.0F);
      }
    }
  } else if (base_name == "passage_lower_upper") {
    fillRectangle(occupancy, scenario.grid, 105.0F, 75.0F, 115.0F, 120.0F);
    fillRectangle(occupancy, scenario.grid, 105.0F, 136.0F, 115.0F, 181.0F);
  } else if (base_name == "random_occupancy") {
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
  } else if (base_name != "open_space") {
    throw std::invalid_argument{"unknown MPPI benchmark scenario: " + name};
  }
  const auto started_at = std::chrono::steady_clock::now();
  scenario.esdf = buildEsdf(occupancy, scenario.grid);
  if (extrude_to_3d) {
    constexpr int kDepthCells{80};
    const std::vector<float> planar_esdf = std::move(scenario.esdf);
    scenario.grid.depth = kDepthCells;
    scenario.grid.origin_z_m = 0.0F;
    scenario.esdf.resize(planar_esdf.size() * static_cast<std::size_t>(kDepthCells));
    for (int z = 0; z < kDepthCells; ++z) {
      std::ranges::copy(planar_esdf,
                        scenario.esdf.begin() +
                            static_cast<std::ptrdiff_t>(z) * planar_esdf.size());
    }
  }
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

#include "mppi_benchmark_kernels.cuh"

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
  std::vector<double> post_update_evaluation;
  std::vector<double> horizon_reconstruction;
  std::vector<double> host_total;
};

} // namespace

BenchmarkResult runCudaBenchmark(const BenchmarkConfig& config) {
  if (config.scenario.ends_with("_3d")) {
    return runPersistentCudaBenchmark(config);
  }
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
    selected =
        engine.plan(MppiTickInput{.initial_state = scenario.initial,
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
    measured.post_update_evaluation.push_back(
        selected.timings.post_update_evaluation_ms);
    measured.horizon_reconstruction.push_back(
        selected.timings.horizon_reconstruction_ms);
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
  result.timings.post_update_evaluation =
      statistics(std::move(measured.post_update_evaluation));
  result.timings.horizon_reconstruction =
      statistics(std::move(measured.horizon_reconstruction));
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
  const MppiTickInput replay_input{
      .initial_state = scenario.initial, .target = target, .obstacle_revision = 1U};
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
