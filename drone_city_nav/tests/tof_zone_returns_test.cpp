#include "drone_city_nav/tof_zone_returns.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

TEST(TofZoneReturns, AZoneIsAConeNotARay) {
  // One zone saw a surface 2 m away, the others nothing: the surface fills
  // that zone's cross-section at 2 m, and every other zone is free across its
  // own as far as the rated range.
  TofZoneReturnsConfig config;
  config.zones_per_side = 2U;
  config.sub_rays = 3U;
  config.field_of_view_rad = 0.8;
  config.maximum_range_m = 2.8;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Point3> zones{Point3{2.0, 0.0, 0.0}, Point3{nan, nan, nan},
                                  Point3{nan, nan, nan}, Point3{nan, nan, nan}};

  const std::vector<StereoDepthReturn> returns = tofZoneReturns(zones, config);

  ASSERT_EQ(returns.size(), 4U * 9U);
  std::size_t hits{0U};
  for (const StereoDepthReturn& ray : returns) {
    const double range_m =
        std::hypot(std::hypot(ray.point.x, ray.point.y), ray.point.z);
    EXPECT_NEAR(range_m, ray.hit ? 2.0 : 2.8, 1.0e-9);
    // Looking up: every ray climbs.
    EXPECT_GT(ray.point.z, 0.0);
    hits += ray.hit ? 1U : 0U;
  }
  EXPECT_EQ(hits, 9U);
}

TEST(TofZoneReturns, TheRaysLeaveTheSensorAndADownwardSensorLooksDown) {
  TofZoneReturnsConfig config;
  config.zones_per_side = 1U;
  config.sub_rays = 1U;
  config.looks_up = false;
  config.position_m = Point3{-0.32, -0.10, -0.12};
  const std::vector<Point3> zones{Point3{1.5, 0.0, 0.0}};

  const std::vector<StereoDepthReturn> returns = tofZoneReturns(zones, config);

  ASSERT_EQ(returns.size(), 1U);
  EXPECT_TRUE(returns[0].hit);
  EXPECT_NEAR(returns[0].point.x, -0.32, 1.0e-9);
  EXPECT_NEAR(returns[0].point.y, -0.10, 1.0e-9);
  EXPECT_NEAR(returns[0].point.z, -0.12 - 1.5, 1.0e-9);
  EXPECT_TRUE(tofZoneReturns(std::vector<Point3>(3U), config).empty());
}

} // namespace
} // namespace drone_city_nav
