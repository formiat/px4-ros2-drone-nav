#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace drone_city_nav::mppi {

struct HorizonSamplingConfig {
  float full_rate_duration_s{2.0F};
  std::uint32_t far_cost_stride{2U};
};

struct HorizonCostSample {
  std::size_t represented_steps{0U};
  bool evaluate{false};
};

[[nodiscard]] inline bool
horizonSamplingConfigIsValid(const HorizonSamplingConfig& config) noexcept {
  return std::isfinite(config.full_rate_duration_s) &&
         config.full_rate_duration_s >= 0.0F && config.far_cost_stride > 0U &&
         config.far_cost_stride <= 16U;
}

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_SAMPLING_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_SAMPLING_HOST_DEVICE
#endif

[[nodiscard]] DRONE_CITY_NAV_MPPI_SAMPLING_HOST_DEVICE inline std::size_t
durationSteps(const float duration_s, const float dt_s) noexcept {
  if (!(duration_s > 0.0F) || !(dt_s > 0.0F)) {
    return 0U;
  }
  std::size_t steps = static_cast<std::size_t>(duration_s / dt_s);
  if (static_cast<float>(steps) * dt_s + 1.0e-6F < duration_s) {
    ++steps;
  }
  return steps;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_SAMPLING_HOST_DEVICE inline HorizonCostSample
horizonCostSample(const std::size_t step, const std::size_t total_steps,
                  const float dt_s, const float head_progress_horizon_s,
                  const HorizonSamplingConfig config) noexcept {
  if (step >= total_steps || config.far_cost_stride == 0U) {
    return {};
  }
  std::size_t full_rate_steps = durationSteps(config.full_rate_duration_s, dt_s);
  const std::size_t head_steps = durationSteps(head_progress_horizon_s, dt_s);
  if (full_rate_steps < head_steps) {
    full_rate_steps = head_steps;
  }
  if (full_rate_steps > total_steps) {
    full_rate_steps = total_steps;
  }
  if (step < full_rate_steps || config.far_cost_stride == 1U) {
    return HorizonCostSample{1U, true};
  }
  const std::size_t far_step_count = step - full_rate_steps + 1U;
  const std::size_t remainder =
      far_step_count % static_cast<std::size_t>(config.far_cost_stride);
  if (remainder == 0U) {
    return HorizonCostSample{config.far_cost_stride, true};
  }
  if (step + 1U == total_steps) {
    return HorizonCostSample{remainder, true};
  }
  return {};
}

#undef DRONE_CITY_NAV_MPPI_SAMPLING_HOST_DEVICE

} // namespace drone_city_nav::mppi
