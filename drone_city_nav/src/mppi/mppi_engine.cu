#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
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
constexpr std::size_t kMaximumRoutePoints{512U};
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
      texture_description.filterMode = cudaFilterModePoint;
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
  DeviceBuffer<Control> best_eligible;
  DeviceBuffer<int> best_tier;
  DeviceBuffer<int> best_rollout;
  DeviceBuffer<float> best_critical;
  DeviceBuffer<float> best_planning;
  DeviceBuffer<float> minimum_soft;
  DeviceBuffer<float> weight_sum;
  DeviceBuffer<KnownSolid> solids{kMaximumKnownSolids};
  DeviceBuffer<RoutePoint> route_points{kMaximumRoutePoints};

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
        best_eligible{steps},
        best_tier{1U},
        best_rollout{1U},
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
           updated.bytes() + best_eligible.bytes() + best_tier.bytes() +
           best_rollout.bytes() + best_critical.bytes() +
           best_planning.bytes() + minimum_soft.bytes() + weight_sum.bytes() +
           solids.bytes() + route_points.bytes();
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

__device__ Control limitControlStep(Control control, const Control previous,
                                    const DynamicsConfig config,
                                    const float interval_s) {
  clampHorizontal(control.ax, control.ay, config.maximum_horizontal_acceleration_mps2);
  control.az = clampValue(control.az, -config.maximum_vertical_acceleration_mps2,
                          config.maximum_vertical_acceleration_mps2);
  control.yaw_accel =
      clampValue(control.yaw_accel, -config.maximum_yaw_acceleration_radps2,
                 config.maximum_yaw_acceleration_radps2);
  const float maximum_delta = config.maximum_control_jerk_mps3 * interval_s;
  control.ax =
      clampValue(control.ax, previous.ax - maximum_delta, previous.ax + maximum_delta);
  control.ay =
      clampValue(control.ay, previous.ay - maximum_delta, previous.ay + maximum_delta);
  control.az =
      clampValue(control.az, previous.az - maximum_delta, previous.az + maximum_delta);
  return control;
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

struct RouteProjection {
  float station_m{0.0F};
  float cross_track_m{0.0F};
  bool valid{false};
};

__device__ RouteProjection projectOntoRoute(const State& state,
                                            const RoutePoint* route_points,
                                            const std::size_t route_point_count,
                                            const float minimum_station_m) {
  RouteProjection result;
  float best_squared_distance = kInfinity;
  for (std::size_t index = 0U; index + 1U < route_point_count; ++index) {
    const RoutePoint first = route_points[index];
    const RoutePoint second = route_points[index + 1U];
    if (second.station_m + 2.0F < minimum_station_m) {
      continue;
    }
    const float dx = second.x_m - first.x_m;
    const float dy = second.y_m - first.y_m;
    const float squared_length = dx * dx + dy * dy;
    if (!(squared_length > 1.0e-8F)) {
      continue;
    }
    const float ratio = clampValue(
        ((state.x - first.x_m) * dx + (state.y - first.y_m) * dy) / squared_length,
        0.0F, 1.0F);
    const float projected_x = first.x_m + ratio * dx;
    const float projected_y = first.y_m + ratio * dy;
    const float offset_x = state.x - projected_x;
    const float offset_y = state.y - projected_y;
    const float squared_distance = offset_x * offset_x + offset_y * offset_y;
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      result.station_m = first.station_m + ratio * (second.station_m - first.station_m);
      result.cross_track_m = sqrtf(squared_distance);
      result.valid = true;
    }
  }
  return result;
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

__device__ float passageZReference(const PassageConstraint& passage,
                                   const float station_m) {
  if (station_m < passage.approach_station_m ||
      station_m > passage.departure_station_m) {
    return passage.normal_flight_z_m;
  }
  if (passage.phase == PassagePhase::kVerticalAlignment &&
      station_m <= passage.exit_station_m) {
    return passage.preferred_z_m;
  }
  const auto smooth_step = [](const float ratio) {
    const float value = fminf(1.0F, fmaxf(0.0F, ratio));
    return value * value * value * (value * (value * 6.0F - 15.0F) + 10.0F);
  };
  if (station_m < passage.alignment_station_m) {
    const float length =
        fmaxf(1.0e-3F, passage.alignment_station_m - passage.approach_station_m);
    const float ratio = smooth_step((station_m - passage.approach_station_m) / length);
    return passage.normal_flight_z_m +
           ratio * (passage.preferred_z_m - passage.normal_flight_z_m);
  }
  if (station_m <= passage.exit_station_m) {
    return passage.preferred_z_m;
  }
  const float length =
      fmaxf(1.0e-3F, passage.departure_station_m - passage.exit_station_m);
  const float ratio = smooth_step((station_m - passage.exit_station_m) / length);
  return passage.preferred_z_m +
         ratio * (passage.normal_flight_z_m - passage.preferred_z_m);
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
         bool passage_active, const RoutePoint* route_points,
         std::size_t route_point_count, float initial_route_station_m,
         Control previous_applied_control, float first_control_interval_s,
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
  float terminal_route_progress = 0.0F;
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
    control = limitControlStep(
        control, previous, dynamics, step == 0U ? first_control_interval_s : dynamics.dt_s);
    const State previous_state = state;
    state = integrate(state, control, dynamics);
    const float segment_length_m =
        hypotf(state.x - previous_state.x, state.y - previous_state.y);
    const float validation_step_m = fmaxf(0.05F, 0.5F * grid.resolution_m);
    const int validation_samples =
        max(1, static_cast<int>(ceilf(segment_length_m / validation_step_m)));
    float clearance = kInfinity;
    bool segment_raw_hit = false;
    for (int sample = 1; sample <= validation_samples; ++sample) {
      const float ratio =
          static_cast<float>(sample) / static_cast<float>(validation_samples);
      State swept_state = state;
      swept_state.x = previous_state.x + ratio * (state.x - previous_state.x);
      swept_state.y = previous_state.y + ratio * (state.y - previous_state.y);
      swept_state.z = previous_state.z + ratio * (state.z - previous_state.z);
      const DeviceEsdfQuery esdf_query =
          queryEsdf(swept_state, grid, esdf_texture);
      clearance = fminf(clearance, esdf_query.clearance_m);
      segment_raw_hit = segment_raw_hit || esdf_query.raw_collision;
      for (std::size_t solid_index = 0U; solid_index < solid_count && !solid_hit;
           ++solid_index) {
        solid_hit = intersectsSolid(swept_state, solids[solid_index]);
      }
      if (passage_active) {
        const float passage_dx = swept_state.x - passage.center_x_m;
        const float passage_dy = swept_state.y - passage.center_y_m;
        const float longitudinal =
            passage_dx * passage.normal_x + passage_dy * passage.normal_y;
        const bool inside_opening = fabsf(longitudinal) <= passage.half_depth_m;
        if (inside_opening &&
            (swept_state.z < passage.min_z_m || swept_state.z > passage.max_z_m)) {
          solid_hit = true;
        }
      }
    }
    minimum_clearance_m = fminf(minimum_clearance_m, clearance);
    raw_hit = raw_hit || segment_raw_hit;
    const RouteProjection route_projection =
        route_point_count >= 2U
            ? projectOntoRoute(state, route_points, route_point_count,
                               initial_route_station_m)
            : RouteProjection{};
    if (passage_active) {
      if (route_projection.valid &&
          route_projection.station_m >= passage.approach_station_m &&
          route_projection.station_m <= passage.departure_station_m) {
        const float altitude_error =
            state.z - passageZReference(passage, route_projection.station_m);
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
    if (route_projection.valid) {
      guide_cost += route_projection.cross_track_m * route_projection.cross_track_m;
      terminal_route_progress = route_projection.station_m - initial_route_station_m;
    } else {
      const float guide_cross = (state.y - initial.y) * (target.x - initial.x) -
                                (state.x - initial.x) * (target.y - initial.y);
      const float guide_length =
          fmaxf(1.0F, hypotf(target.x - initial.x, target.y - initial.y));
      guide_cost += (guide_cross / guide_length) * (guide_cross / guide_length);
    }
    const float z_reference =
        passage_active && route_projection.valid
            ? passageZReference(passage, route_projection.station_m)
            : target.z;
    const float altitude_error = state.z - z_reference;
    altitude_cost += altitude_error * altitude_error;
    if (step + 1U == head_steps) {
      head_progress =
          route_projection.valid
              ? route_projection.station_m - initial_route_station_m
              : initial_distance - hypotf(target.x - state.x, target.y - state.y);
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
  const float progress = route_point_count >= 2U ? terminal_route_progress
                                                 : initial_distance - terminal_distance;
  soft_cost[rollout] =
      costs.head_progress_weight * -head_progress + costs.progress_weight * -progress +
      costs.speed_tracking_weight * dynamics.dt_s * speed_tracking_cost +
      costs.guide_deviation_weight * dynamics.dt_s * guide_cost +
      costs.altitude_tracking_weight * dynamics.dt_s * altitude_cost +
      costs.acceleration_weight * dynamics.dt_s * acceleration_cost +
      costs.jerk_weight * jerk_cost + costs.yaw_change_weight * yaw_cost +
      costs.planning_exposure_weight * planning_m +
      costs.critical_exposure_weight * critical_m +
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
                                    float* weight_sum, int* best_rollout) {
  if (blockIdx.x == 0 && threadIdx.x == 0) {
    *best_tier = static_cast<int>(RiskTier::kCollision);
    *best_critical = kInfinity;
    *best_planning = kInfinity;
    *minimum_soft = kInfinity;
    *weight_sum = 0.0F;
    *best_rollout = INT_MAX;
  }
}

__global__ void selectBestEligibleRollout(const float* weights, const float* soft_cost,
                                          const float* minimum_soft,
                                          const std::size_t count,
                                          int* best_rollout) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && weights[index] > 0.0F &&
      soft_cost[index] <= *minimum_soft + 1.0e-5F) {
    atomicMin(best_rollout, static_cast<int>(index));
  }
}

__global__ void buildBestEligibleControls(
    const Control* nominal, Control* best_eligible, const float* noise_ax,
    const float* noise_ay, const float* noise_az, const float* noise_yaw,
    const int* best_rollout, const std::size_t rollouts, const std::size_t steps) {
  const std::size_t step =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (step >= steps) {
    return;
  }
  const int rollout = *best_rollout;
  if (rollout < 0 || static_cast<std::size_t>(rollout) >= rollouts) {
    best_eligible[step] = nominal[step];
    return;
  }
  const std::size_t index = static_cast<std::size_t>(rollout) * steps + step;
  best_eligible[step] =
      Control{nominal[step].ax + noise_ax[index], nominal[step].ay + noise_ay[index],
              nominal[step].az + noise_az[index],
              nominal[step].yaw_accel + noise_yaw[index]};
}

__global__ void reduceTier(const std::uint8_t* tier, const std::uint8_t* raw_collision,
                           const std::uint8_t* solid_collision, std::size_t count,
                           int maximum_tier, int* best_tier) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count && raw_collision[index] == 0U && solid_collision[index] == 0U &&
      static_cast<int>(tier[index]) <= maximum_tier) {
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
                           RiskConfig risk, int maximum_tier) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const bool escalated = maximum_tier > static_cast<int>(RiskTier::kPreferred);
  const bool eligible =
      escalated
          ? static_cast<int>(tier[index]) <= maximum_tier
          : static_cast<int>(tier[index]) == *best_tier &&
                critical[index] <=
                    *best_critical + risk.critical_exposure_tolerance_m &&
                planning[index] <= *best_planning + risk.planning_exposure_tolerance_m;
  if (raw_collision[index] == 0U && solid_collision[index] == 0U && eligible) {
    atomicMinFloat(minimum_soft, soft[index]);
  }
}

__global__ void
calculateWeights(const std::uint8_t* tier, const float* critical, const float* planning,
                 const float* soft, const std::uint8_t* raw_collision,
                 const std::uint8_t* solid_collision, float* weights, std::size_t count,
                 const int* best_tier, const float* best_critical,
                 const float* best_planning, const float* minimum_soft, RiskConfig risk,
                 float temperature, int maximum_tier, float* weight_sum) {
  const std::size_t index =
      static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  float weight = 0.0F;
  const bool escalated = maximum_tier > static_cast<int>(RiskTier::kPreferred);
  const bool eligible =
      escalated
          ? static_cast<int>(tier[index]) <= maximum_tier
          : static_cast<int>(tier[index]) == *best_tier &&
                critical[index] <=
                    *best_critical + risk.critical_exposure_tolerance_m &&
                planning[index] <= *best_planning + risk.planning_exposure_tolerance_m;
  if (raw_collision[index] == 0U && solid_collision[index] == 0U && eligible) {
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

[[nodiscard]] bool
hostSweptSolidCollision(const State& initial, const std::span<const Control> controls,
                        const DynamicsConfig& dynamics,
                        const std::span<const KnownSolid> solids) {
  State previous = initial;
  for (const Control& control : controls) {
    const State next = integrateReference(previous, control, dynamics);
    const float segment_length_m =
        std::hypot(next.x - previous.x, next.y - previous.y);
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(segment_length_m / 0.25F)));
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const float ratio = static_cast<float>(sample) / static_cast<float>(samples);
      State state = next;
      state.x = std::lerp(previous.x, next.x, ratio);
      state.y = std::lerp(previous.y, next.y, ratio);
      state.z = std::lerp(previous.z, next.z, ratio);
      if (hostSolidCollision(state, solids)) {
        return true;
      }
    }
    previous = next;
  }
  return false;
}

} // namespace

class MppiCudaEngine::Impl {
public:
  explicit Impl(BenchmarkConfig config)
      : config_{std::move(config)},
        buffers_{config_.rollouts, config_.steps},
        nominal_(config_.steps),
        updated_(config_.steps),
        best_eligible_(config_.steps),
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
    const Control previous_applied_control = input.previous_applied_control.value_or(
        last_output_control_.value_or(Control{}));
    const float first_control_interval_s =
        has_updated_
            ? std::clamp(static_cast<float>(elapsed_s), 1.0e-3F, config_.dynamics.dt_s)
            : config_.dynamics.dt_s;
    const bool route_active = input.route.has_value() && input.route->points &&
                              input.route->points->size() >= 2U;
    const bool nominal_reseeded =
        input.nominal_reseed_generation > nominal_reseed_generation_;
    if (nominal_reseeded) {
      const std::span<const RoutePoint> route =
          route_active ? std::span<const RoutePoint>{*input.route->points}
                       : std::span<const RoutePoint>{};
      nominal_ = buildGuideDirectedNominalSeed(
          input.initial_state, input.target, route,
          route_active ? input.route->initial_station_m : 0.0F, input.passage,
          input.reference_speed_mps, config_.dynamics, config_.steps,
          previous_applied_control);
      nominal_reseed_generation_ = input.nominal_reseed_generation;
    } else if (has_updated_) {
      nominal_ = shiftControlSequence(updated_, config_.dynamics.dt_s, elapsed_s);
    }
    if (route_active) {
      if (input.route->points->size() > kMaximumRoutePoints) {
        throw std::invalid_argument{"MPPI route exceeds device route capacity"};
      }
      if (!route_uploaded_ || route_generation_ != input.route->generation ||
          route_points_host_.get() != input.route->points.get()) {
        checkCuda(cudaMemcpyAsync(buffers_.route_points.get(),
                                  input.route->points->data(),
                                  input.route->points->size() * sizeof(RoutePoint),
                                  cudaMemcpyHostToDevice, stream_),
                  "upload semantic route");
        route_points_host_ = input.route->points;
        route_generation_ = input.route->generation;
        route_point_count_ = input.route->points->size();
        route_uploaded_ = true;
      }
    }

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
        buffers_.route_points.get(), route_active ? route_point_count_ : 0U,
        route_active ? input.route->initial_station_m : 0.0F, previous_applied_control,
        first_control_interval_s, input.reference_speed_mps,
        config_.early_exit_on_collision);
    simulation_done_.record(stream_);
    initializeReduction<<<1, 1, 0U, stream_>>>(
        buffers_.best_tier.get(), buffers_.best_critical.get(),
        buffers_.best_planning.get(), buffers_.minimum_soft.get(),
        buffers_.weight_sum.get(), buffers_.best_rollout.get());
    reduceTier<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.raw_collision.get(),
        buffers_.solid_collision.get(), config_.rollouts,
        static_cast<int>(input.maximum_eligible_risk_tier), buffers_.best_tier.get());
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
        buffers_.best_planning.get(), buffers_.minimum_soft.get(), config_.risk,
        static_cast<int>(input.maximum_eligible_risk_tier));
    reduction_done_.record(stream_);
    calculateWeights<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.worst_tier.get(), buffers_.critical_exposure.get(),
        buffers_.planning_exposure.get(), buffers_.soft_cost.get(),
        buffers_.raw_collision.get(), buffers_.solid_collision.get(),
        buffers_.weights.get(), config_.rollouts, buffers_.best_tier.get(),
        buffers_.best_critical.get(), buffers_.best_planning.get(),
        buffers_.minimum_soft.get(), config_.risk, config_.costs.temperature,
        static_cast<int>(input.maximum_eligible_risk_tier), buffers_.weight_sum.get());
    selectBestEligibleRollout<<<rollout_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.weights.get(), buffers_.soft_cost.get(), buffers_.minimum_soft.get(),
        config_.rollouts, buffers_.best_rollout.get());
    weights_done_.record(stream_);
    updateControls<<<control_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.nominal.get(), buffers_.updated.get(), buffers_.noise_ax.get(),
        buffers_.noise_ay.get(), buffers_.noise_az.get(), buffers_.noise_yaw.get(),
        buffers_.weights.get(), buffers_.weight_sum.get(), config_.rollouts,
        config_.steps);
    limitControls<<<1, 1, 0U, stream_>>>(buffers_.updated.get(), config_.steps,
                                         config_.dynamics, previous_applied_control,
                                         first_control_interval_s);
    buildBestEligibleControls<<<control_blocks, kThreadsPerBlock, 0U, stream_>>>(
        buffers_.nominal.get(), buffers_.best_eligible.get(), buffers_.noise_ax.get(),
        buffers_.noise_ay.get(), buffers_.noise_az.get(), buffers_.noise_yaw.get(),
        buffers_.best_rollout.get(), config_.rollouts, config_.steps);
    limitControls<<<1, 1, 0U, stream_>>>(
        buffers_.best_eligible.get(), config_.steps, config_.dynamics,
        previous_applied_control, first_control_interval_s);
    update_done_.record(stream_);
    int eligible_tier_value = static_cast<int>(RiskTier::kCollision);
    float best_critical_exposure_m = kInfinity;
    float best_planning_exposure_m = kInfinity;
    float eligible_weight_sum = 0.0F;
    checkCuda(cudaMemcpyAsync(updated_.data(), buffers_.updated.get(),
                              updated_.size() * sizeof(Control), cudaMemcpyDeviceToHost,
                              stream_),
              "copy selected controls");
    checkCuda(cudaMemcpyAsync(best_eligible_.data(), buffers_.best_eligible.get(),
                              best_eligible_.size() * sizeof(Control),
                              cudaMemcpyDeviceToHost, stream_),
              "copy best eligible controls");
    checkCuda(cudaMemcpyAsync(&eligible_tier_value, buffers_.best_tier.get(),
                              sizeof(eligible_tier_value), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible risk tier");
    checkCuda(cudaMemcpyAsync(&best_critical_exposure_m, buffers_.best_critical.get(),
                              sizeof(best_critical_exposure_m), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible critical exposure");
    checkCuda(cudaMemcpyAsync(&best_planning_exposure_m, buffers_.best_planning.get(),
                              sizeof(best_planning_exposure_m), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible planning exposure");
    checkCuda(cudaMemcpyAsync(&eligible_weight_sum, buffers_.weight_sum.get(),
                              sizeof(eligible_weight_sum), cudaMemcpyDeviceToHost,
                              stream_),
              "copy eligible weight sum");
    completed_.record(stream_);
    completed_.synchronize();
    checkCuda(cudaGetLastError(), "MPPI engine kernels");

    MppiTickResult result;
    const bool eligible_tier_available =
        eligible_tier_value >= static_cast<int>(RiskTier::kPreferred) &&
        eligible_tier_value < static_cast<int>(RiskTier::kCollision);
    if (input.maximum_eligible_risk_tier != RiskTier::kPreferred &&
        eligible_tier_available) {
      eligible_tier_value = static_cast<int>(input.maximum_eligible_risk_tier);
      best_critical_exposure_m = std::numeric_limits<float>::max() * 0.25F;
      best_planning_exposure_m = std::numeric_limits<float>::max() * 0.25F;
    }
    result.eligible_risk_contract = MppiEligibleRiskContract{
        .available = eligible_tier_available && eligible_weight_sum > 0.0F,
        .tier = eligible_tier_available ? static_cast<RiskTier>(eligible_tier_value)
                                        : RiskTier::kCollision,
        .best_critical_exposure_m = best_critical_exposure_m,
        .best_planning_exposure_m = best_planning_exposure_m,
        .critical_exposure_tolerance_m = config_.risk.critical_exposure_tolerance_m,
        .planning_exposure_tolerance_m = config_.risk.planning_exposure_tolerance_m,
        .weight_sum = eligible_weight_sum,
    };
    const auto classify_controls =
        [&](const std::span<const Control> controls) {
          const RolloutMetrics metrics = simulateReference(
              input.initial_state, controls, zero_noise_, config_.dynamics, config_.risk,
              config_.costs, textures_[active_texture_].grid(), activeEsdfHost(),
              input.target.x, input.target.y, config_.early_exit_on_collision,
              previous_applied_control, input.reference_speed_mps);
          const bool known_solid_collision = hostSweptSolidCollision(
              input.initial_state, controls, config_.dynamics, known_solids_);
          return classifyMppiPostUpdate(
              result.eligible_risk_contract,
              MppiPostUpdateObservation{
                  .tier = known_solid_collision ? RiskTier::kCollision
                                                : metrics.worst_tier,
                  .raw_collision = metrics.collision,
                  .known_solid_collision = known_solid_collision,
                  .critical_exposure_m = metrics.critical_exposure_m,
                  .planning_exposure_m = metrics.planning_exposure_m,
              });
        };
    MppiPostUpdateClassificationResult weighted_classification =
        classify_controls(updated_);
    if (!weighted_classification.contract_preserved &&
        result.eligible_risk_contract.available) {
      std::vector<Control> limited_nominal = nominal_;
      limitControlSequence(limited_nominal, config_.dynamics, previous_applied_control,
                           first_control_interval_s);
      constexpr std::array backtrack_ratios{0.5F, 0.25F, 0.125F, 0.0625F, 0.0F};
      bool repaired = false;
      std::vector<Control> candidate(updated_.size());
      for (const float ratio : backtrack_ratios) {
        for (std::size_t index = 0U; index < candidate.size(); ++index) {
          candidate[index] =
              interpolateControl(limited_nominal[index], updated_[index], ratio);
        }
        limitControlSequence(candidate, config_.dynamics, previous_applied_control,
                             first_control_interval_s);
        if (classify_controls(candidate).contract_preserved) {
          updated_ = candidate;
          result.post_update_repair = MppiPostUpdateRepair::kBacktracked;
          result.post_update_backtrack_ratio = ratio;
          repaired = true;
          break;
        }
      }
      if (!repaired &&
          classify_controls(best_eligible_).contract_preserved) {
        updated_ = best_eligible_;
        result.post_update_repair = MppiPostUpdateRepair::kBestEligibleRollout;
        result.post_update_backtrack_ratio = 0.0F;
        repaired = true;
      }
      if (!repaired) {
        result.post_update_repair = MppiPostUpdateRepair::kFailed;
      }
    }
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
    const auto hostRouteProjection =
        [&input](const State& candidate) -> std::optional<float> {
      if (!input.route.has_value() || !input.route->points ||
          input.route->points->size() < 2U) {
        return std::nullopt;
      }
      float best_squared_distance = kInfinity;
      float best_station = input.route->initial_station_m;
      const auto& points = *input.route->points;
      for (std::size_t route_index = 0U; route_index + 1U < points.size();
           ++route_index) {
        const RoutePoint& first = points[route_index];
        const RoutePoint& second = points[route_index + 1U];
        if (second.station_m + 2.0F < input.route->initial_station_m) {
          continue;
        }
        const float dx = second.x_m - first.x_m;
        const float dy = second.y_m - first.y_m;
        const float squared_length = dx * dx + dy * dy;
        if (!(squared_length > 1.0e-8F)) {
          continue;
        }
        const float ratio = std::clamp(
            ((candidate.x - first.x_m) * dx + (candidate.y - first.y_m) * dy) /
                squared_length,
            0.0F, 1.0F);
        const float offset_x = candidate.x - (first.x_m + ratio * dx);
        const float offset_y = candidate.y - (first.y_m + ratio * dy);
        const float squared_distance = offset_x * offset_x + offset_y * offset_y;
        if (squared_distance < best_squared_distance) {
          best_squared_distance = squared_distance;
          best_station = first.station_m + ratio * (second.station_m - first.station_m);
        }
      }
      return best_squared_distance < kInfinity ? std::optional<float>{best_station}
                                               : std::nullopt;
    };
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
        const std::optional<float> route_station = hostRouteProjection(state);
        result.head_progress_m =
            route_station.has_value()
                ? *route_station - input.route->initial_station_m
                : initial_distance -
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
    result.post_update_classification = classifyMppiPostUpdate(
        result.eligible_risk_contract,
        MppiPostUpdateObservation{
            .tier = result.selected_tier,
            .raw_collision = result.raw_collision,
            .known_solid_collision = result.known_solid_collision,
            .critical_exposure_m = result.critical_exposure_m,
            .planning_exposure_m = result.planning_exposure_m,
        });
    const std::optional<float> terminal_route_station = hostRouteProjection(state);
    result.terminal_progress_m =
        terminal_route_station.has_value()
            ? *terminal_route_station - input.route->initial_station_m
            : initial_distance -
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
  std::shared_ptr<const std::vector<RoutePoint>> route_points_host_;
  std::size_t route_point_count_{0U};
  std::uint64_t route_generation_{0U};
  bool route_uploaded_{false};
  std::vector<Control> nominal_;
  std::vector<Control> updated_;
  std::vector<Control> best_eligible_;
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
