#include "drone_city_nav/mppi/static_route_handoff.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] BenchmarkConfig config() {
  BenchmarkConfig value;
  value.steps = 40U;
  value.dynamics.dt_s = 0.05F;
  value.dynamics.linear_drag_1ps = 0.0F;
  value.altitude_envelope =
      AltitudeEnvelopeConfig{.minimum_z_m = 1.0F, .maximum_z_m = 32.0F};
  return value;
}

[[nodiscard]] std::vector<RouteSample3D> straightRoute() {
  return {
      RouteSample3D{.x_m = 2.0F, .y_m = 2.0F, .z_m = 10.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 18.0F, .y_m = 2.0F, .z_m = 10.0F, .station_m = 16.0F},
  };
}

TEST(StaticRouteHandoff, AcceptsRawSafeRouteFromCurrentMotion) {
  const EsdfGrid grid{24, 8, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth), 20.0F);
  const std::vector<RouteSample3D> route = straightRoute();

  const StaticRouteHandoffResult result =
      validateStaticRouteHandoff(State{.x = 3.0F, .y = 2.0F, .z = 10.0F, .vx = 6.0F},
                                 Control{}, route, 20.0F, 15.0F, config(), grid, esdf);

  EXPECT_EQ(result.status, StaticRouteHandoffStatus::kAccepted);
  EXPECT_TRUE(result.accepted);
}

TEST(StaticRouteHandoff, RejectsRawCollisionWithoutClearanceGate) {
  const EsdfGrid grid{24, 8, 1.0F, 0.0F, 0.0F};
  std::vector<float> esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth), 20.0F);
  const std::size_t width = static_cast<std::size_t>(grid.width);
  for (int y = 0; y < grid.height; ++y) {
    esdf[static_cast<std::size_t>(y) * width + 10U] = 0.0F;
  }

  const std::vector<RouteSample3D> route = straightRoute();
  const StaticRouteHandoffResult result =
      validateStaticRouteHandoff(State{.x = 3.0F, .y = 2.0F, .z = 10.0F, .vx = 6.0F},
                                 Control{}, route, 20.0F, 15.0F, config(), grid, esdf);

  EXPECT_EQ(result.status, StaticRouteHandoffStatus::kRawCollision);
  EXPECT_FALSE(result.accepted);
}

TEST(StaticRouteHandoff, CriticalClearanceRemainsExecutable) {
  const EsdfGrid grid{24, 8, 1.0F, 0.0F, 0.0F};
  const std::vector<float> esdf(
      static_cast<std::size_t>(grid.width * grid.height * grid.depth), 0.5F);
  const std::vector<RouteSample3D> route = straightRoute();

  const StaticRouteHandoffResult result =
      validateStaticRouteHandoff(State{.x = 3.0F, .y = 2.0F, .z = 10.0F, .vx = 6.0F},
                                 Control{}, route, 20.0F, 15.0F, config(), grid, esdf);

  EXPECT_EQ(result.status, StaticRouteHandoffStatus::kAccepted);
  EXPECT_TRUE(result.accepted);
  EXPECT_GT(result.critical_exposure_m, 0.0F);
}

} // namespace
} // namespace drone_city_nav::mppi
