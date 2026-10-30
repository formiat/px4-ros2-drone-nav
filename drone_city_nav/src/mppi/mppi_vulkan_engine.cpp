#include "drone_city_nav/mppi/mppi_vulkan_engine.hpp"

#include <memory>
#include <utility>

#include "mppi_vulkan_population_device.hpp"

namespace drone_city_nav::mppi {

class MppiVulkanEngine::Impl {
public:
  explicit Impl(const BenchmarkConfig& config)
      : device_{std::make_shared<detail::VulkanPopulationDevice>(config)},
        engine_{config, device_} {
  }

  detail::VulkanPopulationDevice& device() noexcept {
    return *device_;
  }

  MppiCpuEngine& engine() noexcept {
    return engine_;
  }

private:
  std::shared_ptr<detail::VulkanPopulationDevice> device_;
  MppiCpuEngine engine_;
};

MppiVulkanEngine::MppiVulkanEngine(const BenchmarkConfig& config)
    : impl_{std::make_unique<Impl>(config)} {
}

MppiVulkanEngine::~MppiVulkanEngine() = default;
MppiVulkanEngine::MppiVulkanEngine(MppiVulkanEngine&&) noexcept = default;
MppiVulkanEngine& MppiVulkanEngine::operator=(MppiVulkanEngine&&) noexcept = default;

EsdfUploadResult MppiVulkanEngine::updateEsdf(const EsdfSnapshot& snapshot) {
  return impl_->engine().updateEsdf(snapshot);
}

MppiTickResult MppiVulkanEngine::plan(const MppiTickInput& input) {
  return impl_->engine().plan(input);
}

std::size_t MppiVulkanEngine::allocatedBytes() const noexcept {
  return impl_->engine().allocatedBytes();
}

bool MppiVulkanEngine::ready() const noexcept {
  return impl_->engine().ready();
}

const std::string& MppiVulkanEngine::deviceName() const noexcept {
  return impl_->device().deviceName();
}

} // namespace drone_city_nav::mppi
