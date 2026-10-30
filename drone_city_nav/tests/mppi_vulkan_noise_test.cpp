// The Vulkan noise kernel against the shared body (roadmap item 28, way B):
// the same seed, tick and index give the same hash bit for bit and a draw
// within the GPU's transcendental functions' spread of the host's. Skipped
// where no Vulkan device with a compute queue is found.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

#include "mppi_rollout_body.hpp"
#include "mppi_vulkan_population_device.hpp"

namespace {

using namespace drone_city_nav::mppi;

TEST(MppiVulkanNoise, SamplesThePopulationAsTheSharedBodyDoes) {
  BenchmarkConfig config;
  config.rollouts = 1024U;
  config.steps = 40U;
  config.noise.horizontal_acceleration_sigma_mps2 = 1.5F;
  config.noise.vertical_acceleration_sigma_mps2 = 0.8F;
  config.noise.yaw_acceleration_sigma_radps2 = 0.4F;
  std::unique_ptr<detail::VulkanPopulationDevice> device;
  try {
    device = std::make_unique<detail::VulkanPopulationDevice>(config);
  } catch (const std::runtime_error& error) {
    GTEST_SKIP() << error.what();
  }
  const std::size_t count = config.rollouts * config.steps;
  device->generateNoise(count, config.steps, 42U, 7U, config.noise, {});
  const std::vector<Control> zero_nominal(config.steps);
  std::vector<Control> row(config.steps);
  float worst = 0.0F;
  double sum = 0.0;
  for (std::size_t rollout = 0U; rollout < config.rollouts; ++rollout) {
    device->rolloutControls(rollout, zero_nominal, row);
    for (std::size_t step = 0U; step < config.steps; ++step) {
      const rollout::NoiseSample expected =
          rollout::noiseSample(rollout * config.steps + step, 42U, 7U, config.noise);
      worst = std::max({worst, std::abs(row[step].ax - expected.ax),
                        std::abs(row[step].ay - expected.ay),
                        std::abs(row[step].az - expected.az),
                        std::abs(row[step].yaw_accel - expected.yaw)});
      sum += static_cast<double>(row[step].ax);
    }
  }
  std::printf("MPPI_VULKAN_NOISE device='%s' worst_gap=%.3e mean_ax=%.4f\n",
              device->deviceName().c_str(), static_cast<double>(worst),
              sum / static_cast<double>(count));
  // The hash is exact; the draw's log and cos differ by ulps: the bound is
  // ten times the spread seen on the host of record (8.7e-6).
  EXPECT_LT(worst, 1.0e-4F);
  EXPECT_LT(std::abs(sum / static_cast<double>(count)), 0.05);
}

} // namespace
