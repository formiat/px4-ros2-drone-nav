#include "drone_city_nav/route_time_parameterization.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] FlightTimeModel3D testTimeModel() {
  FlightTimeModel3D model;
  model.maximum_horizontal_speed_mps = 10.0;
  model.maximum_vertical_speed_mps = 10.0;
  model.maximum_translational_speed_mps = 10.0;
  return model;
}

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
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kMissionStop, 4.0, testTimeModel(),
      Vec3{});

  ASSERT_TRUE(profile.valid);
  ASSERT_EQ(profile.reference_speeds_mps.size(), route.size());
  ASSERT_EQ(profile.arrival_times_s.size(), route.size());
  ASSERT_EQ(profile.departure_times_s.size(), route.size());
  EXPECT_DOUBLE_EQ(profile.reference_speeds_mps.back(), 0.0);
  EXPECT_DOUBLE_EQ(profile.reference_speeds_mps.front(), 0.0);
  EXPECT_DOUBLE_EQ(profile.arrival_times_s.front(), 0.0);
  EXPECT_DOUBLE_EQ(profile.departure_times_s.front(), 0.0);
  EXPECT_DOUBLE_EQ(profile.arrival_times_s.back(), profile.travel_time_s);
  EXPECT_DOUBLE_EQ(profile.departure_times_s.back(), profile.travel_time_s);
  EXPECT_GT(profile.travel_time_s, 20.0 / 5.0);
  EXPECT_TRUE(std::isfinite(profile.travel_time_s));
}

TEST(RouteTimeParameterizationTest, TheTimeProfileDoesNotDependOnTheSampleSpacing) {
  // The same 30 m straight, entered at 1 m/s and limited to 5 m/s, sampled
  // every half metre and sampled twice. The profile carries its acceleration
  // across samples and charges the jerk ramps once per acceleration phase, so
  // both readings are the one physical answer: 3 m and 1 s to reach 5 m/s, 27 m
  // of cruise, and one jerk allowance of a third of a second.
  std::vector<RouteSample3D> dense;
  for (int index = 0; index <= 60; ++index) {
    const double station_m = 0.5 * static_cast<double>(index);
    dense.push_back({.position = {station_m, 0.0, 0.0},
                     .tangent = {1.0, 0.0, 0.0},
                     .station_m = station_m});
  }
  const std::vector<RouteSample3D> coarse = straightRoute(30.0);
  const Vec3 entry_velocity{1.0, 0.0, 0.0};
  FlightTimeModel3D model = testTimeModel();
  model.maximum_horizontal_speed_mps = 5.0;
  const RouteTimeParameterization3D dense_profile = parameterizeRouteTime3D(
      dense, {}, 5.0, 3.0, RouteEndpointSemantics3D::kContinuation, 4.0, model,
      entry_velocity);
  const RouteTimeParameterization3D coarse_profile = parameterizeRouteTime3D(
      coarse, {}, 5.0, 3.0, RouteEndpointSemantics3D::kContinuation, 4.0, model,
      entry_velocity);

  ASSERT_TRUE(dense_profile.valid);
  ASSERT_TRUE(coarse_profile.valid);
  const double expected_s = 1.0 + 27.0 / 5.0 + (4.0 / 4.0 + 4.0 / 12.0 - 1.0);
  EXPECT_NEAR(dense_profile.travel_time_s, expected_s, 1.0e-3);
  EXPECT_NEAR(coarse_profile.travel_time_s, expected_s, 1.0e-3);
  // Interior samples carry the acceleration: at 1.5 m the profile has reached
  // sqrt(1 + 2 * 4 * 1.5) m/s, not a from-rest transition's fraction of it.
  EXPECT_NEAR(dense_profile.reference_speeds_mps[3U], std::sqrt(13.0), 1.0e-9);
  EXPECT_DOUBLE_EQ(dense_profile.reference_speeds_mps[30U], 5.0);
}

TEST(RouteTimeParameterizationTest, CurvatureAndVerticalMotionCapTheSameProfile) {
  const std::vector<RouteSample3D> turn_route{
      {.position = {0.0, 0.0, 0.0}, .tangent = {1.0, 0.0, 0.0}, .station_m = 0.0},
      {.position = {2.0, 0.0, 0.0}, .tangent = {0.0, 1.0, 0.0}, .station_m = 2.0},
      {.position = {2.0, 2.0, 0.0}, .tangent = {0.0, 1.0, 0.0}, .station_m = 4.0},
      {.position = {2.0, 4.0, 0.0}, .tangent = {0.0, 1.0, 0.0}, .station_m = 6.0},
  };
  const RouteTimeParameterization3D turn_profile = parameterizeRouteTime3D(
      turn_route, {}, 10.0, 3.0, RouteEndpointSemantics3D::kContinuation, 1.0,
      testTimeModel(), Vec3{});

  ASSERT_TRUE(turn_profile.valid);
  EXPECT_LT(turn_profile.reference_speeds_mps[1U], 3.0);

  const std::vector<RouteSample3D> vertical_route{
      {.position = {0.0, 0.0, 0.0}, .tangent = {0.0, 0.0, 1.0}, .station_m = 0.0},
      {.position = {0.0, 0.0, 5.0}, .tangent = {0.0, 0.0, 1.0}, .station_m = 5.0},
  };
  FlightTimeModel3D vertical_model = testTimeModel();
  vertical_model.maximum_vertical_speed_mps = 2.0;
  const RouteTimeParameterization3D vertical_profile = parameterizeRouteTime3D(
      vertical_route, {}, 10.0, 3.0, RouteEndpointSemantics3D::kContinuation, 4.0,
      vertical_model, Vec3{});
  ASSERT_TRUE(vertical_profile.valid);
  EXPECT_LE(vertical_profile.reference_speeds_mps.front(), 2.0);
  EXPECT_LE(vertical_profile.reference_speeds_mps.back(), 2.0);
}

TEST(RouteTimeParameterizationTest,
     DiagonalMotionHonorsTheCompleteTranslationalSpeedLimit) {
  constexpr double kLengthM{20.0};
  const double station_m = std::numbers::sqrt2 * kLengthM;
  const std::vector<RouteSample3D> route{
      {.position = {0.0, 0.0, 0.0},
       .tangent = {std::sqrt(0.5), 0.0, std::sqrt(0.5)},
       .station_m = 0.0},
      {.position = {kLengthM, 0.0, kLengthM},
       .tangent = {std::sqrt(0.5), 0.0, std::sqrt(0.5)},
       .station_m = station_m},
  };
  FlightTimeModel3D model = testTimeModel();
  model.maximum_translational_speed_mps = 2.0;

  const RouteTimeParameterization3D profile = parameterizeRouteTime3D(
      route, {}, 10.0, 3.0, RouteEndpointSemantics3D::kContinuation, 4.0, model,
      Vec3{});

  ASSERT_TRUE(profile.valid);
  EXPECT_TRUE(
      std::ranges::all_of(profile.reference_speeds_mps,
                          [](const double speed_mps) { return speed_mps <= 2.0; }));
  EXPECT_GE(profile.translation_time_s, station_m / 2.0);
}

TEST(RouteTimeParameterizationTest, StopAndTurnStartsAnIndependentJerkLimitedLeg) {
  std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{
          {0.0, 0.0, 5.0}, {0.1, 0.0, 5.0}, {0.1, 0.1, 5.0}, {0.1, 0.2, 5.0}},
      0.1, 5.0);
  ASSERT_EQ(route.size(), 4U);
  ASSERT_EQ(route[1U].transition, RouteKinematicTransition3D::kStopAndTurn);

  const RouteTimeParameterization3D profile = parameterizeRouteTime3D(
      route, {}, 5.0, 3.0, RouteEndpointSemantics3D::kContinuation, 4.0,
      testTimeModel(), Vec3{});

  ASSERT_TRUE(profile.valid);
  ASSERT_EQ(profile.reference_speeds_mps.size(), route.size());
  EXPECT_DOUBLE_EQ(profile.reference_speeds_mps[1U], 0.0);
  EXPECT_GT(profile.reference_speeds_mps[2U], 0.0);
  EXPECT_GT(profile.departure_times_s[1U], profile.arrival_times_s[1U]);
  EXPECT_GT(profile.arrival_times_s[2U], profile.departure_times_s[1U]);
  EXPECT_GT(profile.stationary_turn_time_s, 0.0);
  EXPECT_GT(profile.travel_time_s, profile.translation_time_s);
  EXPECT_TRUE(std::isfinite(profile.travel_time_s));
}

TEST(RouteTimeParameterizationTest,
     InitialThreeDimensionalVelocityChangesTheSharedEtaProfile) {
  const std::vector<RouteSample3D> route = straightRoute(40.0);
  const RouteTimeParameterization3D from_rest = parameterizeRouteTime3D(
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kContinuation, 4.0,
      testTimeModel(), Vec3{});
  const RouteTimeParameterization3D already_moving = parameterizeRouteTime3D(
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kContinuation, 4.0,
      testTimeModel(), Vec3{5.0, 0.0, 0.0});

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
      route, {}, 8.0, 3.0, RouteEndpointSemantics3D::kMissionStop, 4.0, testTimeModel(),
      Vec3{}, tracking_speed_limits_mps);

  ASSERT_TRUE(profile.valid);
  ASSERT_EQ(profile.reference_speeds_mps.size(), route.size());
  EXPECT_TRUE(
      std::ranges::all_of(profile.reference_speeds_mps,
                          [](const double speed_mps) { return speed_mps <= 1.0; }));
  EXPECT_GE(profile.translation_time_s, 20.0);
}

} // namespace
} // namespace drone_city_nav
