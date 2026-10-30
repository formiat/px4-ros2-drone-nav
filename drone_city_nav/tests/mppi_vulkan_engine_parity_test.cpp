// The Vulkan engine against the CPU engine (and the CUDA engine where the
// build carries it) on the same inputs (roadmap item 28, way B): the same
// seed, tick, ESDF, state and route give the same population, so the weighted
// update, the feasible weight sum and the selected sequence's cost must agree
// to the float arithmetic's spread (the shader's sin, cos, log and fused
// multiply-adds differ in their last bits from the host's). The tolerance is
// what the engines measure, written in the specification (K54), never
// loosened to pass. Skipped where no Vulkan device is found.
#include "drone_city_nav/mppi/mppi_cpu_engine.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_vulkan_engine.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace drone_city_nav::mppi;

struct Scene {
  EsdfGrid grid{};
  std::vector<float> distances;
};

// A 20 m by 20 m by 5 m box at 0.5 m cells with one wall plane at x = 15 m
// and a pillar of occupied cells beside the straight line to the target, so
// that the body's clearance query, the tube and stopping laws and the
// occupied-cell branch all run.
Scene wallScene() {
  Scene scene;
  scene.grid.width = 40;
  scene.grid.height = 40;
  scene.grid.depth = 10;
  scene.grid.resolution_m = 0.5F;
  scene.grid.outside_is_unknown = false;
  scene.distances.resize(static_cast<std::size_t>(40 * 40 * 10));
  for (int z = 0; z < 10; ++z) {
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 40; ++x) {
        const float centre_x = (static_cast<float>(x) + 0.5F) * 0.5F;
        const float centre_y = (static_cast<float>(y) + 0.5F) * 0.5F;
        const float wall = std::max(0.0F, 15.0F - centre_x);
        const float pillar =
            std::max(0.0F, std::hypot(centre_x - 8.0F, centre_y - 11.5F) - 0.5F);
        scene.distances[(static_cast<std::size_t>(z) * 40U +
                         static_cast<std::size_t>(y)) *
                            40U +
                        static_cast<std::size_t>(x)] = std::min(wall, pillar);
      }
    }
  }
  return scene;
}

BenchmarkConfig smallConfig() {
  BenchmarkConfig config;
  config.rollouts = 512U;
  config.steps = 40U;
  config.seed = 7U;
  config.footprint.radius_m = 0.6F;
  config.footprint.lower_extent_m = 0.2F;
  config.footprint.upper_extent_m = 0.3F;
  config.footprint.perimeter_samples = 8U;
  config.footprint.radial_rings = 1U;
  config.footprint.axial_samples = 2U;
  return config;
}

MppiTickInput tickInput(const std::int64_t stamp_ns, const bool with_route) {
  MppiTickInput input;
  input.initial_state = State{.x = 2.0F, .y = 10.0F, .z = 2.5F};
  input.target = State{.x = 12.0F, .y = 10.0F, .z = 2.5F};
  input.planning_stamp_ns = stamp_ns;
  input.reference_speed_mps = 2.0F;
  if (with_route) {
    auto points = std::make_shared<std::vector<RouteSample3D>>();
    for (int index = 0; index <= 14; ++index) {
      RouteSample3D point;
      point.x_m = static_cast<float>(index);
      point.y_m = 10.0F + 0.5F * std::sin(0.5F * static_cast<float>(index));
      point.z_m = 2.5F;
      point.tangent_x = 1.0F;
      point.station_m = static_cast<float>(index);
      point.reference_speed_mps = 2.0F;
      points->push_back(point);
    }
    input.route = RouteReference{.points = points,
                                 .generation = 1U,
                                 .initial_station_m = 2.0F,
                                 .terminal_cross_track_tolerance_m = 1.0F};
    input.deterministic_candidate = DeterministicCandidateKind::kRouteDirectedCruise;
  }
  return input;
}

float relativeGap(const float a, const float b) {
  return std::abs(a - b) / std::max({std::abs(a), std::abs(b), 1.0e-3F});
}

struct Gaps {
  float control{0.0F};
  float weight{0.0F};
  float cost{0.0F};
};

void compare(MppiEngine& reference, MppiEngine& candidate, const bool with_route,
             Gaps& gaps) {
  for (std::int64_t tick = 1; tick <= 3; ++tick) {
    const MppiTickInput input = tickInput(tick * 50'000'000, with_route);
    const MppiTickResult from_reference = reference.plan(input);
    const MppiTickResult from_candidate = candidate.plan(input);
    ASSERT_EQ(from_reference.controls.size(), from_candidate.controls.size());
    EXPECT_EQ(from_reference.collision_gate_lifted,
              from_candidate.collision_gate_lifted);
    EXPECT_EQ(from_reference.post_update_repair, from_candidate.post_update_repair);
    EXPECT_EQ(from_reference.selected_tier, from_candidate.selected_tier);
    EXPECT_EQ(from_reference.control_selection, from_candidate.control_selection);
    gaps.weight = std::max(gaps.weight,
                           relativeGap(from_reference.feasibility_contract.weight_sum,
                                       from_candidate.feasibility_contract.weight_sum));
    gaps.weight =
        std::max(gaps.weight, relativeGap(from_reference.effective_sample_fraction,
                                          from_candidate.effective_sample_fraction));
    gaps.cost =
        std::max(gaps.cost, relativeGap(from_reference.selected_cost_terms.soft_cost,
                                        from_candidate.selected_cost_terms.soft_cost));
    gaps.cost =
        std::max(gaps.cost, relativeGap(from_reference.minimum_esdf_distance_m,
                                        from_candidate.minimum_esdf_distance_m));
    for (std::size_t step = 0U; step < from_reference.controls.size(); ++step) {
      const Control& a = from_reference.controls[step];
      const Control& b = from_candidate.controls[step];
      gaps.control =
          std::max({gaps.control, std::abs(a.ax - b.ax), std::abs(a.ay - b.ay),
                    std::abs(a.az - b.az), std::abs(a.yaw_accel - b.yaw_accel)});
    }
  }
}

void report(const char* const pair, const Gaps& gaps) {
  std::printf("MPPI_VULKAN_PARITY %s control_gap_mps2=%.3e weight_gap=%.3e "
              "cost_gap=%.3e\n",
              pair, static_cast<double>(gaps.control), static_cast<double>(gaps.weight),
              static_cast<double>(gaps.cost));
}

TEST(MppiVulkanEngineParity, TheVulkanEngineRanksAndUpdatesAsTheOtherEnginesDo) {
  const Scene scene = wallScene();
  const EsdfSnapshot snapshot{scene.grid, scene.distances, 1U, {}};
  std::unique_ptr<MppiVulkanEngine> vulkan_plain;
  try {
    vulkan_plain = std::make_unique<MppiVulkanEngine>(smallConfig());
  } catch (const std::runtime_error& error) {
    GTEST_SKIP() << error.what();
  }
  std::printf("MPPI_VULKAN_PARITY device='%s'\n", vulkan_plain->deviceName().c_str());
  Gaps cpu_gaps;
  {
    MppiCpuEngine cpu{smallConfig()};
    ASSERT_TRUE(cpu.updateEsdf(snapshot).accepted);
    ASSERT_TRUE(vulkan_plain->updateEsdf(snapshot).accepted);
    compare(cpu, *vulkan_plain, false, cpu_gaps);
  }
  {
    MppiCpuEngine cpu{smallConfig()};
    MppiVulkanEngine vulkan{smallConfig()};
    ASSERT_TRUE(cpu.updateEsdf(snapshot).accepted);
    ASSERT_TRUE(vulkan.updateEsdf(snapshot).accepted);
    compare(cpu, vulkan, true, cpu_gaps);
  }
  report("cpu-vulkan", cpu_gaps);
  RecordProperty("cpu_vulkan_control_gap_mps2", std::to_string(cpu_gaps.control));
  RecordProperty("cpu_vulkan_weight_gap_relative", std::to_string(cpu_gaps.weight));
  RecordProperty("cpu_vulkan_cost_gap_relative", std::to_string(cpu_gaps.cost));
  EXPECT_LT(cpu_gaps.control, 1.0e-5F);
  EXPECT_LT(cpu_gaps.weight, 1.0e-5F);
  EXPECT_LT(cpu_gaps.cost, 1.0e-5F);
#if DRONE_CITY_NAV_HAS_CUDA_MPPI
  Gaps cuda_gaps;
  {
    MppiCudaEngine cuda{smallConfig()};
    MppiVulkanEngine vulkan{smallConfig()};
    ASSERT_TRUE(cuda.updateEsdf(snapshot).accepted);
    ASSERT_TRUE(vulkan.updateEsdf(snapshot).accepted);
    compare(cuda, vulkan, false, cuda_gaps);
  }
  {
    MppiCudaEngine cuda{smallConfig()};
    MppiVulkanEngine vulkan{smallConfig()};
    ASSERT_TRUE(cuda.updateEsdf(snapshot).accepted);
    ASSERT_TRUE(vulkan.updateEsdf(snapshot).accepted);
    compare(cuda, vulkan, true, cuda_gaps);
  }
  report("cuda-vulkan", cuda_gaps);
  RecordProperty("cuda_vulkan_control_gap_mps2", std::to_string(cuda_gaps.control));
  RecordProperty("cuda_vulkan_weight_gap_relative", std::to_string(cuda_gaps.weight));
  RecordProperty("cuda_vulkan_cost_gap_relative", std::to_string(cuda_gaps.cost));
  EXPECT_LT(cuda_gaps.control, 1.0e-5F);
  EXPECT_LT(cuda_gaps.weight, 1.0e-5F);
  EXPECT_LT(cuda_gaps.cost, 1.0e-5F);
#endif
}

} // namespace
