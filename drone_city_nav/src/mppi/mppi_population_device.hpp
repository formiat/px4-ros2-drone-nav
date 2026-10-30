// The population's work of an MPPI tick, which a device may take from the
// host loops (roadmap item 28): the noise, the rollouts through the shared
// body, the weighted sum of the noise, one rollout's controls. The CPU engine
// orchestrates the tick and calls one of these: its own host device (the
// loops in mppi_cpu_engine.cpp) or the Vulkan device (way B). The reductions
// over the population stay on the host in both.
#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

#include "mppi_rollout_body.hpp"

namespace drone_city_nav::mppi::detail {

// Where one simulation leaves the population's results: host arrays of at
// least the simulated count; cost_terms may be null.
struct PopulationOutputs {
  float* soft_cost{nullptr};
  float* critical_exposure{nullptr};
  float* planning_exposure{nullptr};
  float* minimum_clearance{nullptr};
  std::uint8_t* altitude_envelope_violation{nullptr};
  std::uint8_t* collision_violation{nullptr};
  std::uint8_t* worst_tier{nullptr};
  RolloutCostTerms* cost_terms{nullptr};
};

class MppiPopulationDevice {
public:
  virtual ~MppiPopulationDevice() = default;
  virtual void uploadEsdf(const EsdfGrid& grid, std::span<const float> distances) = 0;
  // The tick's noise: `count` samples of the hash (rollout-major, `steps` a
  // rollout); the candidate row, when given, written over row 0.
  virtual void generateNoise(std::size_t count, std::size_t steps, std::uint64_t seed,
                             std::uint64_t tick, const NoiseConfig& config,
                             std::span<const Control> candidate_row) = 0;
  // `rollouts` rollouts through the body: the population when
  // direct_controls is empty, the direct sequences otherwise. The inputs'
  // noise pointers are the device's own and are ignored.
  virtual void simulate(std::size_t rollouts, const rollout::RolloutInputs& inputs,
                        std::span<const Control> direct_controls,
                        const PopulationOutputs& outputs) = 0;
  // updated[step] = nominal[step] + sum over the rollouts of weight * noise,
  // over the denominator.
  virtual void weightedUpdate(std::span<const float> weights,
                              std::span<const Control> nominal, float denominator,
                              std::span<Control> updated) = 0;
  // One rollout's controls: the nominal plus its noise.
  virtual void rolloutControls(std::size_t rollout, std::span<const Control> nominal,
                               std::span<Control> controls) = 0;
  [[nodiscard]] virtual std::size_t allocatedBytes() const noexcept = 0;

protected:
  MppiPopulationDevice() = default;
  MppiPopulationDevice(const MppiPopulationDevice&) = default;
  MppiPopulationDevice& operator=(const MppiPopulationDevice&) = default;
  MppiPopulationDevice(MppiPopulationDevice&&) = default;
  MppiPopulationDevice& operator=(MppiPopulationDevice&&) = default;
};

} // namespace drone_city_nav::mppi::detail
