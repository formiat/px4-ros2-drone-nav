// The first kernel of the MPPI engine on Vulkan compute (roadmap item 28, way
// B): the population's noise, generated on the device by mppi_noise.comp and
// read back, for the parity against the shared body's rollout::noiseSample.
#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace drone_city_nav::mppi {

struct VulkanNoise {
  std::vector<float> ax;
  std::vector<float> ay;
  std::vector<float> az;
  std::vector<float> yaw;
};

class MppiVulkanNoise {
public:
  // shader_path: the compiled mppi_noise.spv. Throws without a Vulkan device
  // with a compute queue and 64-bit integers.
  explicit MppiVulkanNoise(const std::string& shader_path, std::size_t capacity);
  ~MppiVulkanNoise();
  MppiVulkanNoise(const MppiVulkanNoise&) = delete;
  MppiVulkanNoise& operator=(const MppiVulkanNoise&) = delete;
  MppiVulkanNoise(MppiVulkanNoise&&) = delete;
  MppiVulkanNoise& operator=(MppiVulkanNoise&&) = delete;

  [[nodiscard]] VulkanNoise generate(std::size_t count, std::uint64_t seed,
                                     std::uint64_t tick, const NoiseConfig& config);
  [[nodiscard]] const std::string& deviceName() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav::mppi
