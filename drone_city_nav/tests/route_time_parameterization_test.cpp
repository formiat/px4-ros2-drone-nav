#include "drone_city_nav/route_time_parameterization.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<RouteSample3D> straightRoute(const double length_m) {
  return {
      {.position = {0.0, 0.0, 0.0}, .tangent = {1.0, 0.0, 0.0}, .station_m = 0.0},
      {.position = {length_m * 0.5, 0.0, 0.0},
       .tangent = {1.0, 0.0, 0.0},
       .station_m = length_m * 0.5},
      {.position = {length_m, 0.0, 0.0},
       .tangent = {1.0, 0.0, 0.0},
       .station_m = length_m},
  };
}

TEST(RouteTimeParameterizationTest,
     TerminalStopProfilesBrakingIntoCanonicalTravelTime) {
  const std::vector<RouteSample3D> route = straightRoute(20.0);
  const RouteTimeParameterization3D profile = parameterizeRouteTime3D(
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kMissionStop,
      MppiSpeedPolicyConfig{}, mppi::DynamicsConfig{}, Vec3{});

  ASSERT_TRUE(profile.valid);
  ASSERT_EQ(profile.reference_speeds_mps.size(), route.size());
  EXPECT_DOUBLE_EQ(profile.reference_speeds_mps.back(), 0.0);
  EXPECT_DOUBLE_EQ(profile.reference_speeds_mps.front(), 0.0);
  EXPECT_GT(profile.travel_time_s, 20.0 / 5.0);
  EXPECT_TRUE(std::isfinite(profile.travel_time_s));
}

TEST(RouteTimeParameterizationTest, CurvatureAndVerticalMotionCapTheSameProfile) {
  const std::vector<RouteSample3D> turn_route{
      {.position = {0.0, 0.0, 0.0}, .tangent = {1.0, 0.0, 0.0}, .station_m = 0.0},
      {.position = {2.0, 0.0, 0.0}, .tangent = {0.0, 1.0, 0.0}, .station_m = 2.0},
      {.position = {2.0, 2.0, 0.0}, .tangent = {0.0, 1.0, 0.0}, .station_m = 4.0},
      {.position = {2.0, 4.0, 0.0}, .tangent = {0.0, 1.0, 0.0}, .station_m = 6.0},
  };
  MppiSpeedPolicyConfig speed_policy;
  speed_policy.cruise_speed_mps = 10.0;
  speed_policy.absolute_speed_limit_mps = 10.0;
  speed_policy.maximum_lateral_acceleration_mps2 = 1.0;
  const RouteTimeParameterization3D turn_profile = parameterizeRouteTime3D(
      turn_route, {}, 10.0, 3.0, RouteEndpointSemantics3D::kContinuation, speed_policy,
      mppi::DynamicsConfig{});

  ASSERT_TRUE(turn_profile.valid);
  EXPECT_LT(turn_profile.reference_speeds_mps[1U], 3.0);

  const std::vector<RouteSample3D> vertical_route{
      {.position = {0.0, 0.0, 0.0}, .tangent = {0.0, 0.0, 1.0}, .station_m = 0.0},
      {.position = {0.0, 0.0, 5.0}, .tangent = {0.0, 0.0, 1.0}, .station_m = 5.0},
  };
  mppi::DynamicsConfig dynamics;
  dynamics.maximum_vertical_speed_mps = 2.0F;
  const RouteTimeParameterization3D vertical_profile = parameterizeRouteTime3D(
      vertical_route, {}, 10.0, 3.0, RouteEndpointSemantics3D::kContinuation,
      speed_policy, dynamics);
  ASSERT_TRUE(vertical_profile.valid);
  EXPECT_LE(vertical_profile.reference_speeds_mps.front(), 2.0);
  EXPECT_LE(vertical_profile.reference_speeds_mps.back(), 2.0);
}

TEST(RouteTimeParameterizationTest, StopAndTurnStartsAnIndependentJerkLimitedLeg) {
  std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {0.0, 0.0, 5.0}, {0.1, 0.0, 5.0}, {0.1, 0.1, 5.0}, {0.1, 0.2, 5.0}},
      0.1, 5.0);
  ASSERT_EQ(route.size(), 4U);
  ASSERT_EQ(route[1U].transition, RouteKinematicTransition3D::kStopAndTurn);

  const RouteTimeParameterization3D profile = parameterizeRouteTime3D(
      route, {}, 5.0, 3.0, RouteEndpointSemantics3D::kContinuation,
      MppiSpeedPolicyConfig{}, mppi::DynamicsConfig{});

  ASSERT_TRUE(profile.valid);
  ASSERT_EQ(profile.reference_speeds_mps.size(), route.size());
  EXPECT_DOUBLE_EQ(profile.reference_speeds_mps[1U], 0.0);
  EXPECT_GT(profile.reference_speeds_mps[2U], 0.0);
  EXPECT_GT(profile.stationary_turn_time_s, 0.0);
  EXPECT_GT(profile.travel_time_s, profile.translation_time_s);
  EXPECT_TRUE(std::isfinite(profile.travel_time_s));
}

TEST(RouteTimeParameterizationTest,
     InitialThreeDimensionalVelocityChangesTheSharedEtaProfile) {
  const std::vector<RouteSample3D> route = straightRoute(40.0);
  const RouteTimeParameterization3D from_rest = parameterizeRouteTime3D(
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kContinuation,
      MppiSpeedPolicyConfig{}, mppi::DynamicsConfig{}, Vec3{});
  const RouteTimeParameterization3D already_moving = parameterizeRouteTime3D(
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kContinuation,
      MppiSpeedPolicyConfig{}, mppi::DynamicsConfig{}, Vec3{5.0, 0.0, 0.0});

  ASSERT_TRUE(from_rest.valid);
  ASSERT_TRUE(already_moving.valid);
  EXPECT_DOUBLE_EQ(from_rest.reference_speeds_mps.front(), 0.0);
  EXPECT_DOUBLE_EQ(already_moving.reference_speeds_mps.front(), 5.0);
  EXPECT_LT(already_moving.travel_time_s, from_rest.travel_time_s);
}

TEST(RouteTimeParameterizationTest,
     TrackingTubeCeilingConstrainsTheCompleteStopToStopLeg) {
  const std::vector<RouteSample3D> route = straightRoute(20.0);
  const std::vector<double> tracking_speed_limits_mps(route.size(), 1.0);

  const RouteTimeParameterization3D profile = parameterizeRouteTime3D(
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kMissionStop,
      MppiSpeedPolicyConfig{}, mppi::DynamicsConfig{}, Vec3{},
      tracking_speed_limits_mps);

  ASSERT_TRUE(profile.valid);
  ASSERT_EQ(profile.reference_speeds_mps.size(), route.size());
  EXPECT_TRUE(
      std::ranges::all_of(profile.reference_speeds_mps,
                          [](const double speed_mps) { return speed_mps <= 1.0; }));
  EXPECT_GE(profile.translation_time_s, 20.0);
}

} // namespace
} // namespace drone_city_nav
