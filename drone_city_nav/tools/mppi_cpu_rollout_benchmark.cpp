// What one MPPI rollout costs on a CPU core (roadmap item 28): the CPU reference
// simulation the engine re-simulates its selection with (mppi_reference.cpp),
// 120 steps of 0.05 s (the 6 s static horizon), the urban body (the 0.82 m
// envelope, 12 x 2 x 3 samples, the broad phase on), against a 0.25 m ESDF of
// 40 x 40 x 16 m: open (8 m everywhere) and a corridor (walls 1.5 m either
// side of the path, the floor 1.5 m below). Prints the time of one rollout on
// one core and how many rollouts eight threads finish in 8 ms.
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <random>
#include <span>
#include <thread>
#include <vector>

namespace {

namespace mppi = drone_city_nav::mppi;

constexpr int kWidthCells{160};
constexpr int kHeightCells{160};
constexpr int kDepthCells{64};
constexpr float kResolutionM{0.25F};
constexpr float kOpenClearanceM{8.0F};
constexpr float kCorridorLowerWallM{18.5F};
constexpr float kCorridorUpperWallM{21.5F};
constexpr float kFloorM{2.5F};
constexpr std::size_t kSteps{120U};
constexpr int kWarmUpRollouts{50};
constexpr int kTimedRollouts{1000};
constexpr unsigned kThreads{8U};
constexpr double kTickBudgetUs{8000.0};

[[nodiscard]] std::size_t cellIndex(const int x, const int y, const int z) {
  return (static_cast<std::size_t>(z) * kHeightCells + static_cast<std::size_t>(y)) *
             kWidthCells +
         static_cast<std::size_t>(x);
}

[[nodiscard]] std::vector<float> corridorField() {
  std::vector<float> field(cellIndex(0, 0, kDepthCells));
  for (int z = 0; z < kDepthCells; ++z) {
    for (int y = 0; y < kHeightCells; ++y) {
      for (int x = 0; x < kWidthCells; ++x) {
        const float wall_y = (static_cast<float>(y) + 0.5F) * kResolutionM;
        const float wall_z = (static_cast<float>(z) + 0.5F) * kResolutionM;
        field[cellIndex(x, y, z)] =
            std::fmin(std::fmin(std::fabs(wall_y - kCorridorLowerWallM),
                                std::fabs(wall_y - kCorridorUpperWallM)),
                      std::fabs(wall_z - kFloorM));
      }
    }
  }
  return field;
}

struct Scenario {
  mppi::EsdfGrid grid{};
  mppi::DynamicsConfig dynamics{};
  mppi::FootprintConfig footprint{};
  std::vector<mppi::Control> nominal;
  mppi::State start{};
};

[[nodiscard]] Scenario urbanScenario() {
  Scenario scenario;
  scenario.grid.width = kWidthCells;
  scenario.grid.height = kHeightCells;
  scenario.grid.depth = kDepthCells;
  scenario.grid.resolution_m = kResolutionM;
  scenario.dynamics.dt_s = 0.05F;
  scenario.footprint.radius_m = 0.82F;
  scenario.footprint.lower_extent_m = 0.23F;
  scenario.footprint.upper_extent_m = 0.35F;
  scenario.footprint.perimeter_samples = 12U;
  scenario.footprint.radial_rings = 2U;
  scenario.footprint.axial_samples = 3U;
  scenario.footprint.clearance_broad_phase_enabled = true;
  scenario.nominal.assign(kSteps, mppi::Control{1.0F, 0.0F, 0.0F, 0.0F});
  scenario.start = mppi::State{2.0F, 20.0F, 4.0F};
  return scenario;
}

// Rollouts of random noise around the nominal sequence; the sum keeps the
// optimizer from dropping them.
float runRollouts(const Scenario& scenario, const std::span<const float> field,
                  const int rollouts, const unsigned seed) {
  std::mt19937 generator(seed);
  std::normal_distribution<float> noise_mps2(0.0F, 2.0F);
  std::vector<mppi::Control> noise(kSteps);
  float sink = 0.0F;
  for (int rollout = 0; rollout < rollouts; ++rollout) {
    for (mppi::Control& control : noise) {
      control =
          mppi::Control{noise_mps2(generator), noise_mps2(generator),
                        0.3F * noise_mps2(generator), 0.2F * noise_mps2(generator)};
    }
    const mppi::RolloutMetrics metrics = mppi::simulateReference(
        scenario.start, scenario.nominal, noise, scenario.dynamics, mppi::RiskConfig{},
        mppi::CostConfig{}, scenario.grid, field, 38.0F, 20.0F, false, mppi::Control{},
        3.0F, scenario.footprint);
    sink += metrics.minimum_clearance_m;
  }
  return sink;
}

void measure(const char* name, const Scenario& scenario,
             const std::span<const float> field) {
  static_cast<void>(runRollouts(scenario, field, kWarmUpRollouts, 1U));
  const auto single_start = std::chrono::steady_clock::now();
  const float single_sink = runRollouts(scenario, field, kTimedRollouts, 2U);
  const auto single_end = std::chrono::steady_clock::now();
  const double single_us =
      std::chrono::duration<double, std::micro>(single_end - single_start).count() /
      kTimedRollouts;
  std::vector<float> sinks(kThreads, 0.0F);
  const auto parallel_start = std::chrono::steady_clock::now();
  {
    std::vector<std::jthread> pool;
    pool.reserve(kThreads);
    for (unsigned thread = 0U; thread < kThreads; ++thread) {
      pool.emplace_back([&scenario, field, &sinks, thread] {
        sinks[thread] = runRollouts(scenario, field, kTimedRollouts, 10U + thread);
      });
    }
  }
  const auto parallel_end = std::chrono::steady_clock::now();
  const double parallel_us =
      std::chrono::duration<double, std::micro>(parallel_end - parallel_start).count() /
      (kThreads * kTimedRollouts);
  std::printf("%s: %.1f us a rollout on one core; %u threads %.1f us a rollout "
              "(%.0f rollouts in 8 ms) [%g]\n",
              name, single_us, kThreads, parallel_us, kTickBudgetUs / parallel_us,
              static_cast<double>(single_sink + sinks.front()));
}

} // namespace

int main() {
  const Scenario scenario = urbanScenario();
  const std::vector<float> open(cellIndex(0, 0, kDepthCells), kOpenClearanceM);
  const std::vector<float> corridor = corridorField();
  measure("open", scenario, open);
  measure("corridor", scenario, corridor);
  return 0;
}
