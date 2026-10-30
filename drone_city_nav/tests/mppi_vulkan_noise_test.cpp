// The Vulkan noise kernel against the shared body (roadmap item 28, way B):
// the same seed, tick and index give the same hash bit for bit and a draw
// within the GPU's transcendental functions' spread of the host's. Skipped
// where no Vulkan device with a compute queue is found.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "mppi_rollout_body.hpp"
#include "mppi_vulkan_noise.hpp"

#ifndef DRONE_CITY_NAV_MPPI_SHADER_DIR
#error "DRONE_CITY_NAV_MPPI_SHADER_DIR names the compiled shaders' directory"
#endif

namespace {

using namespace drone_city_nav::mppi;

TEST(MppiVulkanNoise, SamplesThePopulationAsTheSharedBodyDoes) {
  std::unique_ptr<MppiVulkanNoise> vulkan;
  try {
    vulkan = std::make_unique<MppiVulkanNoise>(
        std::string{DRONE_CITY_NAV_MPPI_SHADER_DIR} + "/mppi_noise.spv", 65536U);
  } catch (const std::runtime_error& error) {
    GTEST_SKIP() << error.what();
  }
  NoiseConfig config;
  config.horizontal_acceleration_sigma_mps2 = 1.5F;
  config.vertical_acceleration_sigma_mps2 = 0.8F;
  config.yaw_acceleration_sigma_radps2 = 0.4F;
  const std::size_t count = 40000U;
  const VulkanNoise noise = vulkan->generate(count, 42U, 7U, config);
  float worst = 0.0F;
  double sum = 0.0;
  for (std::size_t index = 0U; index < count; ++index) {
    const rollout::NoiseSample expected = rollout::noiseSample(index, 42U, 7U, config);
    worst = std::max({worst, std::abs(noise.ax[index] - expected.ax),
                      std::abs(noise.ay[index] - expected.ay),
                      std::abs(noise.az[index] - expected.az),
                      std::abs(noise.yaw[index] - expected.yaw)});
    sum += static_cast<double>(noise.ax[index]);
  }
  std::printf("MPPI_VULKAN_NOISE device='%s' worst_gap=%.3e mean_ax=%.4f\n",
              vulkan->deviceName().c_str(), static_cast<double>(worst),
              sum / static_cast<double>(count));
  // The hash is exact; the draw's log and cos differ by ulps: the bound is
  // ten times the spread seen on the host of record.
  EXPECT_LT(worst, 1.0e-4F);
  EXPECT_LT(std::abs(sum / static_cast<double>(count)), 0.05);
}

} // namespace
