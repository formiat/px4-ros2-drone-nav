// The population's work on Vulkan compute (roadmap item 28, way B): the noise
// (mppi_noise.comp), the rollouts through the body written in GLSL
// (mppi_rollout.comp) and the weighted sum of the noise (mppi_update.comp),
// behind the CPU engine's population device, so the tick's orchestration, the
// reductions, the repair and the evaluation stay the host's and one code.
#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#include "mppi_population_device.hpp"

namespace drone_city_nav::mppi::detail {

class VulkanPopulationDevice final : public MppiPopulationDevice {
public:
  // Sized by the configuration's rollouts and steps. Throws
  // std::runtime_error without a Vulkan device with a compute queue and
  // 64-bit integers.
  explicit VulkanPopulationDevice(const BenchmarkConfig& config);
  ~VulkanPopulationDevice() override;
  VulkanPopulationDevice(const VulkanPopulationDevice&) = delete;
  VulkanPopulationDevice& operator=(const VulkanPopulationDevice&) = delete;
  VulkanPopulationDevice(VulkanPopulationDevice&&) = delete;
  VulkanPopulationDevice& operator=(VulkanPopulationDevice&&) = delete;

  void uploadEsdf(const EsdfGrid& grid, std::span<const float> distances) override;
  void generateNoise(std::size_t count, std::size_t steps, std::uint64_t seed,
                     std::uint64_t tick, const NoiseConfig& config,
                     std::span<const Control> candidate_row) override;
  void simulate(std::size_t rollouts, const rollout::RolloutInputs& inputs,
                std::span<const Control> direct_controls,
                const PopulationOutputs& outputs) override;
  void weightedUpdate(std::span<const float> weights, std::span<const Control> nominal,
                      float denominator, std::span<Control> updated) override;
  void rolloutControls(std::size_t rollout, std::span<const Control> nominal,
                       std::span<Control> controls) override;
  [[nodiscard]] std::size_t allocatedBytes() const noexcept override;
  [[nodiscard]] const std::string& deviceName() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav::mppi::detail
