#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <memory>

namespace drone_city_nav::mppi {

namespace detail {
class MppiPopulationDevice;
}

// The MPPI engine on the host alone (roadmap item 28, way A): the same rollout
// body as the CUDA kernels (`src/mppi/mppi_rollout_body.hpp`), the same
// reductions, the same weighted update, repair and evaluation, in plain loops
// over the population. It builds and runs without CUDA, which is what the
// continuous integration and the boards without NVIDIA need; it is not the
// flight's engine (73 rollouts in the tick's budget on this host, item 28), and
// the parity test holds it to the CUDA engine on the same inputs. Not every
// branch of the CUDA engine is here: the cooperative candidates and the
// separation acquisition of item 15's missions are the CUDA engine's alone.
// The population's work (the noise, the rollouts, the weighted sum) goes
// through a population device (`src/mppi/mppi_population_device.hpp`): the
// host loops by default, the Vulkan device for the Vulkan engine (way B).
class MppiCpuEngine final : public MppiEngine {
public:
  explicit MppiCpuEngine(BenchmarkConfig config);
  MppiCpuEngine(BenchmarkConfig config,
                std::shared_ptr<detail::MppiPopulationDevice> device);
  ~MppiCpuEngine() override;

  MppiCpuEngine(const MppiCpuEngine&) = delete;
  MppiCpuEngine& operator=(const MppiCpuEngine&) = delete;
  MppiCpuEngine(MppiCpuEngine&&) noexcept;
  MppiCpuEngine& operator=(MppiCpuEngine&&) noexcept;

  [[nodiscard]] EsdfUploadResult updateEsdf(const EsdfSnapshot& snapshot) override;
  [[nodiscard]] MppiTickResult plan(const MppiTickInput& input) override;
  [[nodiscard]] std::size_t allocatedBytes() const noexcept override;
  [[nodiscard]] bool ready() const noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav::mppi
