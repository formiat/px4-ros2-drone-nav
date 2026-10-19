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

// Every stride cell lit, or none: the lit bitmap smokeFaces reads.
std::vector<std::uint8_t> litEverywhere(const bool lit) {
  return std::vector<std::uint8_t>((kWidth / 4U) * (kHeight / 4U), lit ? 1U : 0U);
}

TEST(SmokeFace, LitSilenceWithAFlatRimThePairDoesNotSeeThroughIsAFace) {
  // The corridor rig: a dense face within 4 m is lit silence (35 to 58 raw
  // grey) rimmed by the floor and the walls at one depth, nothing matched
  // through it. The same silence dark is a shadow or the dark beyond the
  // lamp (F13), and is not judged at all.
  const std::vector<StereoDepthReturn> frame = frameWithSquare(40U, 3.0, 6.0);
  const std::vector<SilentPatch> faces =
      smokeFaces(frame, kGeometry, kReturns, kWidth, kHeight, litEverywhere(true),
                 SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(faces.size(), 1U);
  EXPECT_TRUE(faces[0].closed);
  EXPECT_STREQ(faces[0].reason, "face");
  EXPECT_EQ(faces[0].cells, 1600U);
  EXPECT_EQ(faces[0].free_neighbours, 0U);
  EXPECT_NEAR(faces[0].rim_depth_m, 3.0, 1.0e-6);
  // A face bounds the motion into it as a closed patch does.
  EXPECT_NEAR(silentRangeAheadM(faces, kGeometry, 4U, kWidth / 4U, kHeight / 4U, 0.26),
              3.0, 1.0e-6);
  EXPECT_TRUE(smokeFaces(frame, kGeometry, kReturns, kWidth, kHeight,
                         litEverywhere(false), SilentPatchConfig{}, SmokeFaceConfig{})
                  .empty());
  // A rim beyond the lamp's reach, or one that recedes, is no face.
  const std::vector<SilentPatch> far =
      smokeFaces(frameWithSquare(40U, 6.0, 8.0), kGeometry, kReturns, kWidth, kHeight,
                 litEverywhere(true), SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(far.size(), 1U);
  EXPECT_FALSE(far[0].closed);
  EXPECT_STREQ(far[0].reason, "rim_beyond_reach");
  const std::vector<SilentPatch> corridor =
      smokeFaces(frameWithSquare(40U, 2.0, 6.0, true), kGeometry, kReturns, kWidth,
                 kHeight, litEverywhere(true), SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(corridor.size(), 1U);
  EXPECT_STREQ(corridor[0].reason, "rim_recedes");
}

TEST(SmokeFace, APlumeFillingTheCorridorToTheFrameTopIsAFaceOnItsFloorAndWalls) {
  // r1745: the plume fills the corridor to the top of the frame, the dark
  // above it gives no rim; the floor below and the walls beside end at the
  // face at one depth. Lit silence over the upper two thirds of the frame,
  // rimmed below and on both sides at 3 m, the floor matched at 3 m.
  std::vector<StereoDepthReturn> returns;
  const std::size_t columns = kWidth / 4U;
  const std::size_t rows = kHeight / 4U;
  const std::size_t floor_row = 2U * rows / 3U;
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t column = 0U; column < columns; ++column) {
      const bool side = column < 10U || column + 10U >= columns;
      if (row >= floor_row || side) {
        returns.push_back(returnAt(column, row, 3.0, true));
      }
    }
  }
  const std::vector<SilentPatch> faces =
      smokeFaces(returns, kGeometry, kReturns, kWidth, kHeight, litEverywhere(true),
                 SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(faces.size(), 1U);
  EXPECT_TRUE(faces[0].closed);
  EXPECT_EQ(faces[0].cells, floor_row * (columns - 20U));
  EXPECT_NEAR(faces[0].rim_depth_m, 3.0, 1.0e-6);
  // The same silence judged as a patch (K36) is at the border: not a patch.
  const std::vector<SilentPatch> patches = silentPatches(
      returns, kGeometry, kReturns, kWidth, kHeight, SilentPatchConfig{}, {}, 0.0);
  ASSERT_EQ(patches.size(), 1U);
  EXPECT_STREQ(patches[0].reason, "at_border");
}

TEST(SmokeFace, ThinSmokeThePairMatchesThroughIsNoFace) {
  // The far end of a corridor of 0.1 per metre: lit silence at the rim
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
  const std::vector<SilentPatch> thin =
      smokeFaces(frame, kGeometry, kReturns, kWidth, kHeight, litEverywhere(true),
                 SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(thin.size(), 1U);
  EXPECT_FALSE(thin[0].closed);
  EXPECT_STREQ(thin[0].reason, "seen_through");
  EXPECT_EQ(thin[0].free_neighbours, holes);
  EXPECT_EQ(thin[0].cells, 1600U - holes);
  // One hole in a thousand cells, as 0.5 per metre leaves, is a face still.
  std::vector<StereoDepthReturn> dense = frameWithSquare(40U, 3.0, 6.0);
  dense.push_back(returnAt(columns / 2U, rows / 2U, 5.2, false));
  const std::vector<SilentPatch> one =
      smokeFaces(dense, kGeometry, kReturns, kWidth, kHeight, litEverywhere(true),
                 SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(one.size(), 1U);
  EXPECT_EQ(one[0].free_neighbours, 1U);
  EXPECT_TRUE(one[0].closed);
}

} // namespace
} // namespace drone_city_nav
