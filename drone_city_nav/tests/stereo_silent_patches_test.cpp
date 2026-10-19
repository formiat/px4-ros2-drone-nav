#include "drone_city_nav/stereo_silent_patches.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

// A 1280 x 960 pair at the stride of 4: 320 x 240 cells.
constexpr std::size_t kWidth{1280U};
constexpr std::size_t kHeight{960U};
const StereoPairGeometry kGeometry{.focal_px = 369.5,
                                   .principal_x_px = 640.0,
                                   .principal_y_px = 480.0,
                                   .baseline_m = 0.2};
const StereoDepthReturnsConfig kReturns{};

StereoDepthReturn returnAt(const std::size_t column_cell, const std::size_t row_cell,
                           const double depth_m, const bool hit) {
  const double column_px = static_cast<double>(column_cell * 4U);
  const double row_px = static_cast<double>(row_cell * 4U);
  return StereoDepthReturn{
      .point =
          Point3{depth_m,
                 -(column_px - kGeometry.principal_x_px) / kGeometry.focal_px * depth_m,
                 -(row_px - kGeometry.principal_y_px) / kGeometry.focal_px * depth_m},
      .hit = hit};
}

// Every cell a hit at `background_m`, except a silent square of `side` cells
// at the centre whose rim sits at `rim_m` (one cell wide), the rest of the
// frame a free ray where `free_frame` says so.
std::vector<StereoDepthReturn> frameWithSquare(const std::size_t side,
                                               const double rim_m,
                                               const double background_m,
                                               const bool recede = false) {
  std::vector<StereoDepthReturn> returns;
  const std::size_t columns = kWidth / 4U;
  const std::size_t rows = kHeight / 4U;
  const std::size_t c0 = columns / 2U - side / 2U;
  const std::size_t r0 = rows / 2U - side / 2U;
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t column = 0U; column < columns; ++column) {
      const bool inside =
          column >= c0 && column < c0 + side && row >= r0 && row < r0 + side;
      if (inside) {
        continue;
      }
      const bool rim = column + 1U >= c0 && column <= c0 + side && row + 1U >= r0 &&
                       row <= r0 + side;
      double depth_m = rim ? rim_m : background_m;
      if (rim && recede) {
        // A corridor's walls: the rim's depth runs from near to far round it.
        depth_m = rim_m + 2.0 * static_cast<double>(column - c0 + 1U) /
                              static_cast<double>(side + 2U);
      }
      returns.push_back(returnAt(column, row, depth_m, true));
    }
  }
  return returns;
}

TEST(SilentPatches, AFlatRimWithinReachClosesThePatchAndBoundsTheMotionIntoIt) {
  const std::vector<StereoDepthReturn> frame = frameWithSquare(40U, 3.0, 6.0);
  const std::vector<SilentPatch> patches = silentPatches(
      frame, kGeometry, kReturns, kWidth, kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(patches.size(), 1U);
  EXPECT_TRUE(patches[0].closed);
  EXPECT_EQ(patches[0].cells, 1600U);
  EXPECT_NEAR(patches[0].rim_depth_m, 3.0, 1.0e-6);
  // The patch holds the frame's centre: a motion along the axis is bounded
  // by the rim's depth; a cone that misses the patch is not.
  EXPECT_NEAR(
      silentRangeAheadM(patches, kGeometry, 4U, kWidth / 4U, kHeight / 4U, 0.26), 3.0,
      1.0e-6);
  std::vector<StereoDepthReturn> aside = frameWithSquare(40U, 3.0, 6.0);
  for (StereoDepthReturn& stereo_return : aside) {
    stereo_return.point.y += 0.0;
  }
  const std::vector<SilentPatch> open =
      silentPatches(frameWithSquare(40U, 6.0, 8.0), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, {}, 0.0);
  EXPECT_TRUE(std::isnan(
      silentRangeAheadM(open, kGeometry, 4U, kWidth / 4U, kHeight / 4U, 0.26)));
}

TEST(SilentPatches, ARimBeyondReachOrRecedingLeavesThePatchOpen) {
  const std::vector<SilentPatch> far =
      silentPatches(frameWithSquare(40U, 6.0, 8.0), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(far.size(), 1U);
  EXPECT_FALSE(far[0].closed);
  EXPECT_STREQ(far[0].reason, "rim_beyond_reach");
  const std::vector<SilentPatch> corridor =
      silentPatches(frameWithSquare(40U, 2.0, 6.0, true), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(corridor.size(), 1U);
  EXPECT_FALSE(corridor[0].closed);
  EXPECT_STREQ(corridor[0].reason, "rim_recedes");
}

TEST(SilentPatches, TheDarkBeyondTheLitFloorIsBoundedBelowAloneAndStaysOpen) {
  // The upper half of the frame silent, the floor below it matched at 4 m
  // (its far edge flat across the frame): the rim covers the lower sectors
  // only, and the patch runs to the frame's border.
  std::vector<StereoDepthReturn> returns;
  for (std::size_t row = kHeight / 8U; row < kHeight / 4U; ++row) {
    for (std::size_t column = 0U; column < kWidth / 4U; ++column) {
      returns.push_back(returnAt(column, row, 4.0, true));
    }
  }
  const std::vector<SilentPatch> patches = silentPatches(
      returns, kGeometry, kReturns, kWidth, kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(patches.size(), 1U);
  EXPECT_FALSE(patches[0].closed);
  EXPECT_STREQ(patches[0].reason, "at_border");
  // The same silence held away from the border by a free-ray margin: the
  // rim's hits still lie below it alone.
  for (std::size_t column = 0U; column < kWidth / 4U; ++column) {
    returns.push_back(returnAt(column, 0U, 8.0, false));
  }
  for (std::size_t row = 0U; row < kHeight / 8U; ++row) {
    returns.push_back(returnAt(0U, row, 8.0, false));
    returns.push_back(returnAt(kWidth / 4U - 1U, row, 8.0, false));
  }
  const std::vector<SilentPatch> inset = silentPatches(
      returns, kGeometry, kReturns, kWidth, kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(inset.size(), 1U);
  EXPECT_FALSE(inset[0].closed);
  EXPECT_STREQ(inset[0].reason, "rim_not_round");
}

TEST(SilentPatches, ASmallPatchOrOneAnotherSensorReadsFreeThroughIsNotJudgedClosed) {
  const std::vector<SilentPatch> small =
      silentPatches(frameWithSquare(10U, 3.0, 6.0), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, {}, 0.0);
  EXPECT_TRUE(small.empty());
  // A time-of-flight zone looking through the patch's centre, free to 2.8 m,
  // past a rim at 2.0 m.
  const std::vector<Point3> through{Point3{1.0, 0.0, 0.0}};
  const std::vector<SilentPatch> open =
      silentPatches(frameWithSquare(40U, 2.0, 6.0), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, through, 2.8);
  ASSERT_EQ(open.size(), 1U);
  EXPECT_FALSE(open[0].closed);
  EXPECT_STREQ(open[0].reason, "free_through");
  // The same zone free only to 1.5 m says nothing of a rim at 2.0 m.
  const std::vector<SilentPatch> closed =
      silentPatches(frameWithSquare(40U, 2.0, 6.0), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, through, 1.5);
  ASSERT_EQ(closed.size(), 1U);
  EXPECT_TRUE(closed[0].closed);
}

TEST(SmokeFace, ALitClosedPatchThePairDoesNotSeeThroughIsAFace) {
  // The corridor rig: a dense face within 4 m is a closed patch whose cells
  // read 35 to 58 raw grey, with nothing matched through it.
  const std::vector<SilentPatch> patches =
      silentPatches(frameWithSquare(40U, 3.0, 6.0), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(patches.size(), 1U);
  EXPECT_EQ(patches[0].free_neighbours, 0U);
  EXPECT_TRUE(smokeFace(patches[0], 35.0, SmokeFaceConfig{}));
  // The dark beyond the lamp reads 0 to 10: a shadow, not a face (F13).
  EXPECT_FALSE(smokeFace(patches[0], 10.0, SmokeFaceConfig{}));
  // An open patch is never a face, however lit.
  const std::vector<SilentPatch> open =
      silentPatches(frameWithSquare(40U, 6.0, 8.0), kGeometry, kReturns, kWidth,
                    kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(open.size(), 1U);
  EXPECT_FALSE(smokeFace(open[0], 50.0, SmokeFaceConfig{}));
}

TEST(SmokeFace, ThinSmokeThePairMatchesThroughIsNoFace) {
  // The far end of a corridor of 0.1 per metre: a closed patch at the rim
  // where the veil kills the texture, with sparse matches deeper than the
  // confident depth scattered through it (1.6 to 3.8 percent of its cells
  // on the rig). Free rays inside the silence, one in every 20 cells.
  std::vector<StereoDepthReturn> frame = frameWithSquare(40U, 4.8, 6.0);
  const std::size_t columns = kWidth / 4U;
  const std::size_t rows = kHeight / 4U;
  const std::size_t c0 = columns / 2U - 20U;
  const std::size_t r0 = rows / 2U - 20U;
  std::size_t holes{0U};
  for (std::size_t row = r0 + 2U; row + 2U < r0 + 40U; row += 5U) {
    for (std::size_t column = c0 + 2U; column + 2U < c0 + 40U; column += 4U) {
      frame.push_back(returnAt(column, row, 5.2, false));
      ++holes;
    }
  }
  const std::vector<SilentPatch> patches = silentPatches(
      frame, kGeometry, kReturns, kWidth, kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(patches.size(), 1U);
  ASSERT_TRUE(patches[0].closed);
  EXPECT_EQ(patches[0].free_neighbours, holes);
  EXPECT_EQ(patches[0].cells, 1600U - holes);
  EXPECT_GT(static_cast<double>(holes) / static_cast<double>(patches[0].cells), 0.02);
  EXPECT_FALSE(smokeFace(patches[0], 38.0, SmokeFaceConfig{}));
  // One hole in a thousand cells, as 0.5 per metre leaves, is a face still.
  std::vector<StereoDepthReturn> dense = frameWithSquare(40U, 3.0, 6.0);
  dense.push_back(returnAt(columns / 2U, rows / 2U, 5.2, false));
  const std::vector<SilentPatch> one = silentPatches(
      dense, kGeometry, kReturns, kWidth, kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(one.size(), 1U);
  EXPECT_EQ(one[0].free_neighbours, 1U);
  EXPECT_TRUE(smokeFace(one[0], 38.0, SmokeFaceConfig{}));
}

} // namespace
} // namespace drone_city_nav
