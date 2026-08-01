#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace drone_city_nav::mppi {
namespace {

TEST(MppiControlSequenceTest, FractionalShiftInterpolatesWithoutDroppingWholeTick) {
  const std::array<Control, 3> controls{
      Control{.ax = 0.0F},
      Control{.ax = 10.0F},
      Control{.ax = 20.0F},
  };

  const std::vector<Control> shifted = shiftControlSequence(controls, 1.0F, 0.5);

  ASSERT_EQ(shifted.size(), controls.size());
  EXPECT_FLOAT_EQ(shifted[0].ax, 5.0F);
  EXPECT_FLOAT_EQ(shifted[1].ax, 15.0F);
  EXPECT_FLOAT_EQ(shifted[2].ax, 20.0F);
}

TEST(MppiControlSequenceTest, ShiftBeyondHorizonDropsStaleNominal) {
  const std::array<Control, 2> controls{
      Control{.ax = 1.0F},
      Control{.ax = 2.0F},
  };

  const std::vector<Control> shifted = shiftControlSequence(controls, 0.05F, 0.2);

  ASSERT_EQ(shifted.size(), controls.size());
  EXPECT_FLOAT_EQ(shifted[0].ax, 0.0F);
  EXPECT_FLOAT_EQ(shifted[1].ax, 0.0F);
}

TEST(MppiControlSequenceTest, ReseedFollowsRouteWithoutAlternatingLateralBias) {
  DynamicsConfig dynamics;
  const State initial{};
  const State target{.x = 10.0F};
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .y_m = 0.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 10.0F, .y_m = 0.0F, .station_m = 10.0F},
  };

  const std::vector<Control> seed = buildGuideDirectedNominalSeed(
      initial, target, route, 0.0F, 5.0F, dynamics, 8U, Control{});

  ASSERT_FALSE(seed.empty());
  EXPECT_GT(seed.front().ax, 0.0F);
  EXPECT_NEAR(seed.front().ay, 0.0F, 1.0e-6F);
}

TEST(MppiControlSequenceTest, ReseedUsesCurrentRouteAltitudeProfile) {
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  const State initial{.x = 5.0F, .z = 5.0F};
  const State distant_target{.x = 30.0F, .z = 18.0F};
  const std::array route{
      RouteSample3D{.x_m = 0.0F, .y_m = 0.0F, .z_m = 5.0F, .station_m = 0.0F},
      RouteSample3D{.x_m = 30.0F, .y_m = 0.0F, .z_m = 5.0F, .station_m = 30.0F},
  };

  const std::vector<Control> seed = buildGuideDirectedNominalSeed(
      initial, distant_target, route, 5.0F, 5.0F, dynamics, 8U, Control{});

  ASSERT_FALSE(seed.empty());
  EXPECT_NEAR(seed.front().az, 0.0F, 1.0e-6F);
}

TEST(MppiControlSequenceTest, HostLimiterMatchesAccelerationAndJerkContract) {
  std::array<Control, 2> controls{
      Control{.ax = 20.0F, .ay = 20.0F, .az = 20.0F},
      Control{.ax = -20.0F, .ay = -20.0F, .az = -20.0F},
  };
  DynamicsConfig dynamics;
  dynamics.dt_s = 0.1F;
  dynamics.maximum_horizontal_acceleration_mps2 = 4.0F;
  dynamics.maximum_vertical_acceleration_mps2 = 3.0F;
  dynamics.maximum_control_jerk_mps3 = 10.0F;

  limitControlSequence(controls, dynamics, Control{}, 0.05F);

  EXPECT_NEAR(controls[0].ax, 0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[0].ay, 0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[0].az, 0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[1].ax, -0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[1].ay, -0.5F, 1.0e-6F);
  EXPECT_NEAR(controls[1].az, -0.5F, 1.0e-6F);
}

} // namespace
} // namespace drone_city_nav::mppi
