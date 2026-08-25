#include <gtest/gtest.h>

#include <array>
#include <span>
#include <vector>

#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<RouteSample3D> straightRoute() {
  const std::array<Point3, 3> points{
      Point3{0.0, 0.0, 5.0},
      Point3{5.0, 0.0, 5.0},
      Point3{10.0, 0.0, 5.0},
  };
  return sampleRoute3D(points, 5.0, 8.0);
}

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
profile(const RouteEndpointSemantics3D semantics) {
  const std::vector<RouteSample3D> route = straightRoute();
  return makeMppiRoute3D(route, std::span<const ConstrainedRouteSpan>{}, 8.0, 4.0,
                         semantics, MppiSpeedPolicyConfig{});
}

TEST(ProductionMppiRouteHelpersTest,
     ContinuationUsesTheCanonicalPolicyLimitedSpeedAtTheLocalBoundary) {
  const auto route = profile(RouteEndpointSemantics3D::kContinuation);

  ASSERT_NE(route, nullptr);
  ASSERT_EQ(route->size(), 3U);
  EXPECT_FLOAT_EQ(route->front().reference_speed_mps, 5.0F);
  EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 5.0F);
}

TEST(ProductionMppiRouteHelpersTest, RealStopsTaperTheNominalProfileToRest) {
  for (const RouteEndpointSemantics3D semantics :
       {RouteEndpointSemantics3D::kObservationStop,
        RouteEndpointSemantics3D::kMissionStop,
        RouteEndpointSemantics3D::kEmergencyBrakeTail}) {
    const auto route = profile(semantics);

    ASSERT_NE(route, nullptr);
    ASSERT_EQ(route->size(), 3U);
    EXPECT_GT(route->front().reference_speed_mps, 0.0F);
    EXPECT_GT((*route)[1].reference_speed_mps, 0.0F);
    EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 0.0F);
  }
}

TEST(ProductionMppiRouteHelpersTest, UnknownEndpointSemanticsFailsClosedToRest) {
  const auto route = profile(static_cast<RouteEndpointSemantics3D>(255U));

  ASSERT_NE(route, nullptr);
  ASSERT_FALSE(route->empty());
  EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 0.0F);
}

TEST(ProductionMppiRouteHelpersTest,
     TwoDimensionalContinuationAlsoUsesTheCanonicalPolicyLimitedSpeed) {
  const std::array<Point2, 3> points{
      Point2{0.0, 0.0},
      Point2{5.0, 0.0},
      Point2{10.0, 0.0},
  };

  const auto route =
      makeMppiRoute2D(points, 5.0, 6.0, RouteEndpointSemantics3D::kContinuation);

  ASSERT_NE(route, nullptr);
  ASSERT_FALSE(route->empty());
  EXPECT_FLOAT_EQ(route->back().reference_speed_mps, 5.0F);
}

} // namespace
} // namespace drone_city_nav
