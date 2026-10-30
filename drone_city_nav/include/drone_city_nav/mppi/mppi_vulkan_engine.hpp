#pragma once

#include "drone_city_nav/mppi/mppi_cpu_engine.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <string>

namespace drone_city_nav::mppi {

// The MPPI engine on Vulkan compute (roadmap item 28, way B): the CPU
// engine's tick with the population's noise, rollouts and weighted sum on the
// device (`src/mppi/shaders/*.comp`, the rollout body once more in GLSL), the
// reductions, the repair and the evaluation on the host. Built where the
// Vulkan loader and glslc are (`DRONE_CITY_NAV_ENABLE_VULKAN_MPPI`); the
// parity test holds it to the CPU and CUDA engines on the same inputs. The
// CPU engine's limits hold: no cooperative candidates, no separation
// acquisition.
class MppiVulkanEngine final : public MppiEngine {
public:
  // Throws std::runtime_error without a Vulkan device with a compute queue.
  explicit MppiVulkanEngine(const BenchmarkConfig& config);
  ~MppiVulkanEngine() override;

  MppiVulkanEngine(const MppiVulkanEngine&) = delete;
  MppiVulkanEngine& operator=(const MppiVulkanEngine&) = delete;
  MppiVulkanEngine(MppiVulkanEngine&&) noexcept;
  MppiVulkanEngine& operator=(MppiVulkanEngine&&) noexcept;

  [[nodiscard]] EsdfUploadResult updateEsdf(const EsdfSnapshot& snapshot) override;
  [[nodiscard]] MppiTickResult plan(const MppiTickInput& input) override;
  [[nodiscard]] std::size_t allocatedBytes() const noexcept override;
  [[nodiscard]] bool ready() const noexcept override;
  [[nodiscard]] const std::string& deviceName() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav::mppi
