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

TEST(MppiControlSequenceTest, ReseedAlternatesLateralBias) {
  DynamicsConfig dynamics;
  const State initial{};
  const State target{.x = 10.0F};

  const std::vector<Control> left =
      buildGuideDirectedNominalSeed(initial, target, dynamics, 8U, 1U);
  const std::vector<Control> right =
      buildGuideDirectedNominalSeed(initial, target, dynamics, 8U, 2U);

  ASSERT_FALSE(left.empty());
  ASSERT_EQ(left.size(), right.size());
  EXPECT_GT(left.front().ay, 0.0F);
  EXPECT_LT(right.front().ay, 0.0F);
  EXPECT_GT(left.front().ax, 0.0F);
  EXPECT_GT(right.front().ax, 0.0F);
}

} // namespace
} // namespace drone_city_nav::mppi
