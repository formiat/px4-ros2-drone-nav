#include "mppi_vulkan_noise.hpp"

#include <array>
#include <cstring>
#include <stdexcept>

#include "vulkan_compute.hpp"

namespace drone_city_nav::mppi {

namespace {

// The shader's push-constant block, as mppi_noise.comp lays it out.
struct NoisePush {
  std::uint64_t seed;
  std::uint64_t tick;
  std::uint32_t count;
  float horizontal_sigma;
  float vertical_sigma;
  float yaw_sigma;
};

static_assert(sizeof(NoisePush) == 32U);
constexpr std::uint32_t kLocalSize{256U};

} // namespace

class MppiVulkanNoise::Impl {
public:
  Impl(const std::string& shader_path, const std::size_t capacity)
      : capacity_{capacity},
        spirv_{vulkan::readSpirv(shader_path)},
        pipeline_{device_, spirv_, 4U, sizeof(NoisePush)},
        ax_{device_, capacity * sizeof(float)},
        ay_{device_, capacity * sizeof(float)},
        az_{device_, capacity * sizeof(float)},
        yaw_{device_, capacity * sizeof(float)} {
    if (!device_.shaderInt64()) {
      throw std::runtime_error{
          "Vulkan: the device has no 64-bit integers for the noise's hash"};
    }
  }

  VulkanNoise generate(const std::size_t count, const std::uint64_t seed,
                       const std::uint64_t tick, const NoiseConfig& config) {
    if (count > capacity_) {
      throw std::invalid_argument{"Vulkan noise: count exceeds the capacity"};
    }
    const NoisePush push{seed,
                         tick,
                         static_cast<std::uint32_t>(count),
                         config.horizontal_acceleration_sigma_mps2,
                         config.vertical_acceleration_sigma_mps2,
                         config.yaw_acceleration_sigma_radps2};
    const std::array<vulkan::HostBuffer*, 4U> buffers{&ax_, &ay_, &az_, &yaw_};
    pipeline_.dispatch(
        buffers, &push, sizeof(push),
        static_cast<std::uint32_t>((count + kLocalSize - 1U) / kLocalSize));
    VulkanNoise noise;
    noise.ax.resize(count);
    noise.ay.resize(count);
    noise.az.resize(count);
    noise.yaw.resize(count);
    std::memcpy(noise.ax.data(), ax_.data(), count * sizeof(float));
    std::memcpy(noise.ay.data(), ay_.data(), count * sizeof(float));
    std::memcpy(noise.az.data(), az_.data(), count * sizeof(float));
    std::memcpy(noise.yaw.data(), yaw_.data(), count * sizeof(float));
    return noise;
  }

  [[nodiscard]] const std::string& deviceName() const noexcept {
    return device_.deviceName();
  }

private:
  std::size_t capacity_;
  vulkan::ComputeDevice device_;
  std::vector<std::uint32_t> spirv_;
  vulkan::ComputePipeline pipeline_;
  vulkan::HostBuffer ax_;
  vulkan::HostBuffer ay_;
  vulkan::HostBuffer az_;
  vulkan::HostBuffer yaw_;
};

MppiVulkanNoise::MppiVulkanNoise(const std::string& shader_path,
                                 const std::size_t capacity)
    : impl_{std::make_unique<Impl>(shader_path, capacity)} {
}

MppiVulkanNoise::~MppiVulkanNoise() = default;

VulkanNoise MppiVulkanNoise::generate(const std::size_t count, const std::uint64_t seed,
                                      const std::uint64_t tick,
                                      const NoiseConfig& config) {
  return impl_->generate(count, seed, tick, config);
}

const std::string& MppiVulkanNoise::deviceName() const noexcept {
  return impl_->deviceName();
}

} // namespace drone_city_nav::mppi
