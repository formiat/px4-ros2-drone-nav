#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "mppi_controller_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] mppi::BenchmarkConfig controllerConfig() {
  mppi::BenchmarkConfig config;
  config.rollouts = 64U;
  config.steps = 8U;
  return config;
}

[[nodiscard]] std::shared_ptr<const WorldSnapshot3D>
controllerWorld(const std::uint64_t generation = 3U) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 4, 4, 4};
  auto occupancy = std::make_shared<OccupancyGrid3D>(bounds, 71U + generation);
  const std::uint64_t revision = occupancy->fingerprint();
  auto world = std::make_shared<WorldSnapshot3D>();
  world->revision = revision;
  world->source_occupied_fingerprint = revision;
  world->raw_occupied_fingerprint = occupancy->contentFingerprint();
  world->grid = EsdfGrid3D{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .depth = bounds.depth_cells,
  };
  world->distances_m = std::make_shared<const std::vector<float>>(64U, 20.0F);
  world->static_occupancy = std::move(occupancy);
  world->local_world_generation = LocalWorldGeneration{
      .generation = generation,
      .raw_map =
          RawMapVersion{
              .producer_instance_id = 0U,
              .base_snapshot_revision = revision,
              .revision = revision,
          },
      .pose_revision = 5U,
      .esdf_revision = revision,
      .gpu_esdf_revision = revision,
  };
  return world;
}

[[nodiscard]] MppiControllerRequest3D
holdRequest(const MppiNominalReseedObservation reseed = MppiNominalReseedObservation{
                .route_generation = 7U}) {
  mppi::MppiTickInput input;
  input.target = mppi::State{.x = 1.0F, .y = 2.0F, .z = 3.0F};
  return MppiControllerRequest3D{
      .input = std::move(input),
      .nominal_reseed = reseed,
      .tick_started = std::chrono::steady_clock::now(),
      .world_revision = 41U,
      .mode = MppiControllerMode3D::kStationaryHold,
  };
}

TEST(MppiController3DTest, OwnsStationaryHoldAndExactNominalReseedInput) {
  MppiController3D controller{controllerConfig()};

  const MppiControllerResult3D result = controller.run(holdRequest());

  EXPECT_TRUE(result.executable());
  EXPECT_EQ(result.status, MppiControllerStatus3D::kStationaryHold);
  EXPECT_EQ(result.input.nominal_reseed_generation, 1U);
  ASSERT_EQ(result.result.horizon.size(), 2U);
  EXPECT_FLOAT_EQ(result.result.horizon.front().x, result.input.target.x);
  EXPECT_FLOAT_EQ(result.result.horizon.front().y, result.input.target.y);
  EXPECT_FLOAT_EQ(result.result.horizon.front().z, result.input.target.z);
  EXPECT_FLOAT_EQ(result.result.horizon.back().x, result.input.target.x);
  EXPECT_FLOAT_EQ(result.result.horizon.back().y, result.input.target.y);
  EXPECT_FLOAT_EQ(result.result.horizon.back().z, result.input.target.z);
  ASSERT_EQ(result.result.controls.size(), 1U);
  EXPECT_FLOAT_EQ(result.result.controls.front().ax, 0.0F);
  EXPECT_FLOAT_EQ(result.result.controls.front().ay, 0.0F);
  EXPECT_FLOAT_EQ(result.result.controls.front().az, 0.0F);
  EXPECT_FLOAT_EQ(result.result.controls.front().yaw_accel, 0.0F);
  EXPECT_EQ(result.result.selected_tier, mppi::RiskTier::kPreferred);
  EXPECT_EQ(result.result.esdf_revision, 41U);
  EXPECT_GE(result.result.timings.host_total_ms, 0.0);
  EXPECT_EQ(result.no_eligible_recovery.phase, MppiNoEligiblePhase::kHealthy);
}

TEST(MppiController3DTest, RejectsInvalidOrSupersededControllerWorlds) {
  const std::shared_ptr<const WorldSnapshot3D> captured = controllerWorld();
  ASSERT_NE(captured, nullptr);

  EXPECT_TRUE(assessMppiControllerWorldCurrentness3D(*captured, captured).current());
  EXPECT_EQ(assessMppiControllerWorldCurrentness3D(*captured, nullptr).status,
            MppiControllerWorldStatus3D::kMissingResident);
  EXPECT_EQ(
      assessMppiControllerWorldCurrentness3D(*captured, controllerWorld(4U)).status,
      MppiControllerWorldStatus3D::kSuperseded);

  WorldSnapshot3D invalid_captured = *captured;
  ++invalid_captured.local_world_generation.gpu_esdf_revision;
  EXPECT_EQ(assessMppiControllerWorldCurrentness3D(invalid_captured, captured).status,
            MppiControllerWorldStatus3D::kInvalidCaptured);
  auto invalid_resident = std::make_shared<WorldSnapshot3D>(*captured);
  ++invalid_resident->local_world_generation.gpu_esdf_revision;
  EXPECT_EQ(assessMppiControllerWorldCurrentness3D(*captured, invalid_resident).status,
            MppiControllerWorldStatus3D::kInvalidResident);
}

TEST(MppiController3DTest, RetainsTheSoleNominalReseedLifecycleAcrossTicks) {
  MppiController3D controller{controllerConfig()};

  const MppiControllerResult3D first = controller.run(holdRequest());
  const MppiControllerResult3D unchanged = controller.run(holdRequest());
  const MppiControllerResult3D liveness_changed =
      controller.run(holdRequest(MppiNominalReseedObservation{
          .route_generation = 7U,
          .local_liveness_generation = 2U,
      }));

  EXPECT_EQ(first.input.nominal_reseed_generation, 1U);
  EXPECT_EQ(unchanged.input.nominal_reseed_generation, 1U);
  EXPECT_EQ(liveness_changed.input.nominal_reseed_generation, 2U);
}

TEST(MppiController3DTest, SerializesConcurrentControllerStateTransitions) {
  MppiController3D controller{controllerConfig()};
  constexpr std::size_t kThreadCount{8U};
  std::array<std::uint64_t, kThreadCount> generations{};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t index = 0U; index < kThreadCount; ++index) {
    threads.emplace_back([&controller, &generations, index] {
      const MppiControllerResult3D result = controller.run(
          holdRequest(MppiNominalReseedObservation{.route_generation = index + 1U}));
      generations.at(index) = result.input.nominal_reseed_generation;
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  std::ranges::sort(generations);

  for (std::size_t index = 0U; index < kThreadCount; ++index) {
    EXPECT_EQ(generations.at(index), index + 1U);
  }
}

TEST(MppiController3DTest, ReportsUnavailableBackendWithoutAnExecutableResult) {
  MppiController3D controller{controllerConfig()};
  ASSERT_FALSE(controller.ready());

  MppiControllerRequest3D request;
  request.nominal_reseed.route_generation = 3U;
  const MppiControllerResult3D result = controller.run(std::move(request));

  EXPECT_FALSE(result.executable());
  EXPECT_EQ(result.status, MppiControllerStatus3D::kBackendUnavailable);
  EXPECT_EQ(result.input.nominal_reseed_generation, 1U);
  EXPECT_TRUE(result.result.horizon.empty());
  EXPECT_TRUE(result.result.controls.empty());
  EXPECT_TRUE(result.failure_message.empty());
}

TEST(MppiController3DTest, StatusNamesAreStable) {
  EXPECT_STREQ(mppiControllerStatus3DName(MppiControllerStatus3D::kPlanned), "planned");
  EXPECT_STREQ(mppiControllerStatus3DName(MppiControllerStatus3D::kStationaryHold),
               "stationary_hold");
  EXPECT_STREQ(mppiControllerStatus3DName(MppiControllerStatus3D::kBackendUnavailable),
               "backend_unavailable");
  EXPECT_STREQ(mppiControllerStatus3DName(MppiControllerStatus3D::kBackendFailure),
               "backend_failure");
}

} // namespace
} // namespace drone_city_nav
