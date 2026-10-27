#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

// The gaze of a vehicle whose sensor looks forward (mppi_control_sequence.hpp):
// the motion's heading, the rest heading and the survey of a shaft.
namespace drone_city_nav::mppi {
namespace {

TEST(MppiGazeTest, TheGazeTurnsTheHeadingToTheMotionWithoutOvershoot) {
  // The vehicle flies north at 3 m/s while it looks east: the heading turns a
  // quarter turn to the motion inside the yaw limits, arrives without
  // overshoot and stays, and the translation is not touched.
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.maximum_yaw_rate_radps = 1.5F;
  dynamics.maximum_yaw_acceleration_radps2 = 2.0F;
  std::vector<Control> controls(80U);
  std::vector<State> horizon{State{.vy = 3.0F}};
  for (const Control& control : controls) {
    horizon.push_back(integrateReference(horizon.back(), control, dynamics));
  }
  const std::vector<State> translation = horizon;

  applyGazeYawControls(controls, horizon, dynamics, 1.5F, 0.55F, std::nullopt, 0.0F);

  const float north = 0.5F * std::numbers::pi_v<float>;
  float peak_yaw{0.0F};
  for (std::size_t index = 0U; index < controls.size(); ++index) {
    EXPECT_LE(std::abs(controls[index].yaw_accel), 2.0F + 1.0e-5F);
    EXPECT_LE(std::abs(horizon[index + 1U].yaw_rate), 1.5F + 1.0e-5F);
    EXPECT_FLOAT_EQ(horizon[index + 1U].y, translation[index + 1U].y);
    EXPECT_FLOAT_EQ(controls[index].ay, 0.0F);
    peak_yaw = std::max(peak_yaw, horizon[index + 1U].yaw);
  }
  EXPECT_NEAR(horizon.back().yaw, north, 0.03F);
  EXPECT_LT(peak_yaw, north + 0.05F);
  EXPECT_NEAR(horizon.back().yaw_rate, 0.0F, 0.1F);
  // Two seconds in, the quarter turn is within ten degrees of done.
  EXPECT_NEAR(horizon[40U].yaw, north, 0.18F);
}

TEST(MppiGazeTest, TheGazeHoldsTheHeadingOfAVehicleThatHardlyMoves) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.05F;
  std::vector<Control> controls(10U);
  std::vector<State> horizon{State{.vx = -0.2F, .yaw = 1.0F}};
  for (const Control& control : controls) {
    horizon.push_back(integrateReference(horizon.back(), control, dynamics));
  }

  applyGazeYawControls(controls, horizon, dynamics, 1.5F, 0.55F, std::nullopt, 0.0F);

  EXPECT_NEAR(horizon.back().yaw, 1.0F, 1.0e-5F);
}

TEST(MppiGazeTest, TheGazeOfAVehicleAtRestTurnsToWhereItsRouteLeaves) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.05F;
  std::vector<Control> controls(80U);
  std::vector<State> horizon{State{.yaw = 1.0F}};
  for (const Control& control : controls) {
    horizon.push_back(integrateReference(horizon.back(), control, dynamics));
  }
  const auto route = std::make_shared<std::vector<RouteSample3D>>(std::vector{
      RouteSample3D{.z_m = 5.0F, .tangent_y = -1.0F},
      RouteSample3D{.y_m = -4.0F, .z_m = 5.0F, .tangent_y = -1.0F, .station_m = 4.0F},
  });
  const std::optional<float> south =
      gazeRestHeading(RouteReference{.points = route}, 0.38F);
  ASSERT_TRUE(south.has_value());
  EXPECT_NEAR(*south, -1.5707964F, 1.0e-5F);

  applyGazeYawControls(controls, horizon, dynamics, 1.5F, 0.55F, south, 0.0F);

  EXPECT_NEAR(horizon.back().yaw, -1.5707964F, 0.05F);
  EXPECT_NEAR(horizon.back().x, 0.0F, 1.0e-6F);

  // A climb inside the cone of the sensors that look up has no heading.
  const auto shaft = std::make_shared<std::vector<RouteSample3D>>(std::vector{
      RouteSample3D{.z_m = 5.0F, .tangent_x = 0.2F, .tangent_z = 0.98F},
      RouteSample3D{.x_m = 0.8F,
                    .z_m = 9.0F,
                    .tangent_x = 0.2F,
                    .tangent_z = 0.98F,
                    .station_m = 4.08F},
  });
  EXPECT_FALSE(gazeRestHeading(RouteReference{.points = shaft}, 0.38F).has_value());
  EXPECT_FALSE(gazeRestHeading(std::nullopt, 0.38F).has_value());
}

TEST(MppiGazeTest, TheGazeSurveysTheWallsOfAShaftTheVehicleClimbs) {
  // A climb at 1 m/s with no heading to face: the heading turns at the survey
  // rate, inside the yaw limits, and keeps turning the way it already turns.
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.05F;
  dynamics.maximum_yaw_rate_radps = 2.0F;
  dynamics.maximum_yaw_acceleration_radps2 = 3.0F;
  std::vector<Control> controls(80U);
  std::vector<State> horizon{State{.vz = 1.0F, .yaw = 1.0F, .yaw_rate = -0.2F}};
  for (const Control& control : controls) {
    horizon.push_back(integrateReference(horizon.back(), control, dynamics));
  }

  const GazeDecision surveying = applyGazeYawControls(controls, horizon, dynamics, 1.5F,
                                                      0.55F, std::nullopt, 1.5F);

  EXPECT_EQ(surveying.rule, GazeRule::kSurvey);
  for (std::size_t index = 0U; index < controls.size(); ++index) {
    EXPECT_LE(std::abs(controls[index].yaw_accel), 3.0F + 1.0e-5F);
    EXPECT_LE(std::abs(horizon[index + 1U].yaw_rate), 2.0F + 1.0e-5F);
    EXPECT_FLOAT_EQ(controls[index].az, 0.0F);
  }
  // Two seconds in the sweep runs at the survey rate (it ends with the
  // horizon, whose last lookahead climbs less than the rule reads).
  EXPECT_NEAR(horizon[40U].yaw_rate, -1.5F, 1.0e-3F);
  float swept_rad{0.0F};
  for (std::size_t index = 0U; index < 40U; ++index) {
    swept_rad += std::abs(horizon[index + 1U].yaw_rate) * dynamics.dt_s;
  }
  EXPECT_GT(swept_rad, 2.5F);
}

} // namespace
} // namespace drone_city_nav::mppi
