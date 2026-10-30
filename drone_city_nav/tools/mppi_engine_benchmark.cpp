// mppi_engine_benchmark: the tick's cost on each MPPI engine (roadmap item 28,
// specification K54): cuda, cpu or vulkan, at a rollout count, on one scene
// (a 40 x 40 x 16 m corridor at 0.25 m cells, walls 1.5 m either side of the
// path and a floor, the urban body of 0.82 m with 12 x 2 x 3 samples and the
// broad phase, 120 steps of 0.05 s), the population's milliseconds and the
// tick's as p50, p95 and maximum over the measured ticks.
//   mppi_engine_benchmark ENGINE ROLLOUTS [TICKS]
#include "drone_city_nav/mppi/mppi_cpu_engine.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#if DRONE_CITY_NAV_HAS_VULKAN_MPPI
#include "drone_city_nav/mppi/mppi_vulkan_engine.hpp"
#endif

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace mppi = drone_city_nav::mppi;

constexpr int kWidthCells{160};
constexpr int kHeightCells{160};
constexpr int kDepthCells{64};
constexpr float kResolutionM{0.25F};
constexpr std::size_t kWarmupTicks{20U};

[[nodiscard]] std::vector<float> corridorField() {
  std::vector<float> field(static_cast<std::size_t>(kWidthCells) * kHeightCells *
                           kDepthCells);
  for (int z = 0; z < kDepthCells; ++z) {
    for (int y = 0; y < kHeightCells; ++y) {
      for (int x = 0; x < kWidthCells; ++x) {
        const float wall_y = (static_cast<float>(y) + 0.5F) * kResolutionM;
        const float wall_z = (static_cast<float>(z) + 0.5F) * kResolutionM;
        field[(static_cast<std::size_t>(z) * kHeightCells +
               static_cast<std::size_t>(y)) *
                  kWidthCells +
              static_cast<std::size_t>(x)] =
            std::fmin(std::fmin(std::fabs(wall_y - 18.5F), std::fabs(wall_y - 21.5F)),
                      std::fabs(wall_z - 2.5F));
      }
    }
  }
  return field;
}

[[nodiscard]] mppi::BenchmarkConfig benchmarkConfig(const std::size_t rollouts) {
  mppi::BenchmarkConfig config;
  config.rollouts = rollouts;
  config.steps = 120U;
  config.dynamics.dt_s = 0.05F;
  config.footprint.radius_m = 0.82F;
  config.footprint.lower_extent_m = 0.23F;
  config.footprint.upper_extent_m = 0.35F;
  config.footprint.perimeter_samples = 12U;
  config.footprint.radial_rings = 2U;
  config.footprint.axial_samples = 3U;
  config.footprint.clearance_broad_phase_enabled = true;
  return config;
}

[[nodiscard]] std::unique_ptr<mppi::MppiEngine>
makeEngine(const mppi::MppiEngineKind kind, const mppi::BenchmarkConfig& config) {
  switch (kind) {
    case mppi::MppiEngineKind::kCpu:
      return std::make_unique<mppi::MppiCpuEngine>(config);
    case mppi::MppiEngineKind::kVulkan:
#if DRONE_CITY_NAV_HAS_VULKAN_MPPI
      return std::make_unique<mppi::MppiVulkanEngine>(config);
#else
      return nullptr;
#endif
    case mppi::MppiEngineKind::kCuda:
#if DRONE_CITY_NAV_HAS_CUDA_MPPI
      return std::make_unique<mppi::MppiCudaEngine>(config);
#else
      return nullptr;
#endif
  }
  return nullptr;
}

[[nodiscard]] double percentile(std::vector<double> values, const double share) {
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
      std::min(static_cast<double>(values.size() - 1U),
               std::floor(share * static_cast<double>(values.size()))));
  return values[index];
}

[[nodiscard]] std::optional<std::size_t> parseCount(const std::string_view text) {
  std::size_t value = 0U;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
      value == 0U) {
    return std::nullopt;
  }
  return value;
}

} // namespace

int main(const int argc, char** argv) {
  if (argc < 3) {
    std::fputs("usage: mppi_engine_benchmark cuda|cpu|vulkan ROLLOUTS [TICKS]\n",
               stderr);
    return 2;
  }
  const std::optional<mppi::MppiEngineKind> kind = mppi::parseMppiEngineKind(argv[1]);
  const std::optional<std::size_t> rollouts = parseCount(argv[2]);
  const std::optional<std::size_t> ticks =
      argc > 3 ? parseCount(argv[3]) : std::optional<std::size_t>{200U};
  if (!kind || !rollouts || !ticks) {
    std::fputs("mppi_engine_benchmark: bad engine, rollouts or ticks\n", stderr);
    return 2;
  }
  const mppi::BenchmarkConfig config = benchmarkConfig(*rollouts);
  std::unique_ptr<mppi::MppiEngine> engine;
  try {
    engine = makeEngine(*kind, config);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "mppi_engine_benchmark: %s\n", error.what());
    return 1;
  }
  if (!engine) {
    std::fprintf(stderr, "mppi_engine_benchmark: the %s engine is not in this build\n",
                 mppi::mppiEngineKindName(*kind));
    return 1;
  }
  const std::vector<float> field = corridorField();
  mppi::EsdfGrid grid;
  grid.width = kWidthCells;
  grid.height = kHeightCells;
  grid.depth = kDepthCells;
  grid.resolution_m = kResolutionM;
  const mppi::EsdfSnapshot snapshot{grid, field, 1U, {}};
  if (!engine->updateEsdf(snapshot).accepted) {
    std::fputs("mppi_engine_benchmark: the ESDF was not accepted\n", stderr);
    return 1;
  }
  std::vector<double> population_ms;
  std::vector<double> simulation_ms;
  std::vector<double> tick_ms;
  for (std::size_t tick = 0U; tick < kWarmupTicks + *ticks; ++tick) {
    mppi::MppiTickInput input;
    input.initial_state = mppi::State{.x = 2.0F, .y = 20.0F, .z = 4.0F};
    input.target = mppi::State{.x = 36.0F, .y = 20.0F, .z = 4.0F};
    input.planning_stamp_ns = static_cast<std::int64_t>(tick + 1U) * 50'000'000;
    input.reference_speed_mps = 3.0F;
    const mppi::MppiTickResult result = engine->plan(input);
    if (tick < kWarmupTicks) {
      continue;
    }
    population_ms.push_back(result.timings.gpu_total_ms);
    simulation_ms.push_back(result.timings.rollout_simulation_ms);
    tick_ms.push_back(result.timings.host_total_ms);
  }
  std::printf("MPPI_ENGINE_BENCHMARK engine=%s rollouts=%zu steps=%zu ticks=%zu "
              "simulation_ms p50=%.2f p95=%.2f max=%.2f | population_ms p50=%.2f "
              "p95=%.2f max=%.2f | tick_ms p50=%.2f p95=%.2f max=%.2f\n",
              mppi::mppiEngineKindName(*kind), *rollouts, config.steps, *ticks,
              percentile(simulation_ms, 0.5), percentile(simulation_ms, 0.95),
              percentile(simulation_ms, 1.0), percentile(population_ms, 0.5),
              percentile(population_ms, 0.95), percentile(population_ms, 1.0),
              percentile(tick_ms, 0.5), percentile(tick_ms, 0.95),
              percentile(tick_ms, 1.0));
  return 0;
}
