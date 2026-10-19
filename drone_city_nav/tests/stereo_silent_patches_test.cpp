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
std::vector<StereoDepthReturn>
frameWithSquare(const std::size_t side, const double rim_m, const double background_m,
                const bool recede = false, const double recede_m = 2.0) {
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
        depth_m = rim_m + recede_m * static_cast<double>(column - c0 + 1U) /
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

// Every stride cell at one raw grey: the frame smokeFaces reads.
std::vector<std::uint8_t> rawEverywhere(const std::uint8_t grey) {
  return std::vector<std::uint8_t>((kWidth / 4U) * (kHeight / 4U), grey);
}

TEST(SmokeFace, LitSilenceWithAFlatRimThePairDoesNotSeeThroughIsAFace) {
  // The corridor rig: a dense face within 4 m is lit silence (35 to 58 raw
  // grey) rimmed by the floor and the walls at one depth, nothing matched
  // through it. The same silence dark is a shadow or the dark beyond the
  // lamp (F13), and is not judged at all.
  const std::vector<StereoDepthReturn> frame = frameWithSquare(40U, 3.0, 6.0);
  const std::vector<SilentPatch> faces =
      smokeFaces(frame, kGeometry, kReturns, kWidth, kHeight, rawEverywhere(36U),
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
  EXPECT_TRUE(smokeFaces(frame, kGeometry, kReturns, kWidth, kHeight, rawEverywhere(5U),
                         SilentPatchConfig{}, SmokeFaceConfig{})
                  .empty());
  // A rim beyond the lamp's reach, or one that recedes, is no face.
  const std::vector<SilentPatch> far =
      smokeFaces(frameWithSquare(40U, 6.0, 8.0), kGeometry, kReturns, kWidth, kHeight,
                 rawEverywhere(36U), SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(far.size(), 1U);
  EXPECT_FALSE(far[0].closed);
  EXPECT_STREQ(far[0].reason, "rim_beyond_reach");
  // A plume's rim spreads by up to a metre (its soft edge); a corridor's
  // walls running on recede by more.
  const std::vector<SilentPatch> soft =
      smokeFaces(frameWithSquare(40U, 2.0, 6.0, true, 0.8), kGeometry, kReturns, kWidth,
                 kHeight, rawEverywhere(36U), SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(soft.size(), 1U);
  EXPECT_TRUE(soft[0].closed);
  EXPECT_LT(soft[0].rim_depth_m, 2.6); // the nearest quartile of the rim
  const std::vector<SilentPatch> corridor =
      smokeFaces(frameWithSquare(40U, 2.0, 6.0, true, 4.0), kGeometry, kReturns, kWidth,
                 kHeight, rawEverywhere(36U), SilentPatchConfig{}, SmokeFaceConfig{});
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
      smokeFaces(returns, kGeometry, kReturns, kWidth, kHeight, rawEverywhere(36U),
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

TEST(SmokeFace, TheVeilOfThinSmokeInAHallRimmedByTheFloorAloneIsNoFace) {
  // r1769: in the hall of 0.1 per metre the veil beyond the lamp's reach is
  // lit silence whose only rim is the floor's fade below it, flat across the
  // frame at one depth, and nothing lies beyond it within the matcher's
  // range to see through: the rim does not go round it, as the dark beyond
  // the lamp's does not (K36).
  std::vector<StereoDepthReturn> returns;
  const std::size_t columns = kWidth / 4U;
  const std::size_t rows = kHeight / 4U;
  for (std::size_t row = 2U * rows / 3U; row < rows; ++row) {
    for (std::size_t column = 0U; column < columns; ++column) {
      returns.push_back(returnAt(column, row, 4.8, true));
    }
  }
  const std::vector<SilentPatch> veil =
      smokeFaces(returns, kGeometry, kReturns, kWidth, kHeight, rawEverywhere(36U),
                 SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(veil.size(), 1U);
  EXPECT_FALSE(veil[0].closed);
  EXPECT_STREQ(veil[0].reason, "rim_not_round");
}

TEST(SmokeFace, ALitBodyWithASharpEdgeInTheDarkIsAFaceAtItsBrightnessDepth) {
  // r1788: a plume floating above the horizon in a dark hall, no floor
  // under it within the lamp's reach: lit silence (raw 34 to 38 on the rig)
  // with dark silence all round and a sharp edge, the silence three cells
  // out at 0.13 to 0.26 of it. No return anywhere in the frame.
  std::vector<std::uint8_t> raw = rawEverywhere(4U);
  const std::size_t columns = kWidth / 4U;
  const std::size_t rows = kHeight / 4U;
  const std::size_t c0 = columns / 2U - 20U;
  const std::size_t r0 = rows / 2U - 20U;
  for (std::size_t row = r0; row < r0 + 40U; ++row) {
    for (std::size_t column = c0; column < c0 + 40U; ++column) {
      raw[row * columns + column] = 36U;
    }
  }
  const std::vector<SilentPatch> body =
      smokeFaces({}, kGeometry, kReturns, kWidth, kHeight, raw, SilentPatchConfig{},
                 SmokeFaceConfig{});
  ASSERT_EQ(body.size(), 1U);
  EXPECT_TRUE(body[0].closed);
  EXPECT_STREQ(body[0].reason, "body");
  EXPECT_EQ(body[0].cells, 1600U);
  EXPECT_NEAR(body[0].rim_depth_m, 2.5, 1.0e-6);
  // Brighter, nearer: at 49 to 58 the rig's face stood 2 m off.
  std::vector<std::uint8_t> bright = raw;
  for (std::uint8_t& grey : bright) {
    grey = grey == 36U ? 50U : grey;
  }
  const std::vector<SilentPatch> near =
      smokeFaces({}, kGeometry, kReturns, kWidth, kHeight, bright, SilentPatchConfig{},
                 SmokeFaceConfig{});
  ASSERT_EQ(near.size(), 1U);
  EXPECT_NEAR(near[0].rim_depth_m, 1.5, 1.0e-6);
  // The lamp's cone on the veil of a thin hall: the same body with a soft
  // edge, the silence round it fading over cells (the rig: the shell at 0.61
  // of the cone's middle).
  std::vector<std::uint8_t> soft = raw;
  for (std::size_t row = r0 - 6U; row < r0 + 46U; ++row) {
    for (std::size_t column = c0 - 6U; column < c0 + 46U; ++column) {
      const bool inside =
          row >= r0 && row < r0 + 40U && column >= c0 && column < c0 + 40U;
      if (!inside) {
        soft[row * columns + column] = 19U;
      }
    }
  }
  const std::vector<SilentPatch> veil =
      smokeFaces({}, kGeometry, kReturns, kWidth, kHeight, soft, SilentPatchConfig{},
                 SmokeFaceConfig{});
  ASSERT_EQ(veil.size(), 1U);
  EXPECT_FALSE(veil[0].closed);
  EXPECT_STREQ(veil[0].reason, "soft_edge");
  // At the frame's border it is not judged.
  std::vector<std::uint8_t> edge = rawEverywhere(4U);
  for (std::size_t row = 0U; row < 40U; ++row) {
    for (std::size_t column = c0; column < c0 + 40U; ++column) {
      edge[row * columns + column] = 36U;
    }
  }
  const std::vector<SilentPatch> border =
      smokeFaces({}, kGeometry, kReturns, kWidth, kHeight, edge, SilentPatchConfig{},
                 SmokeFaceConfig{});
  ASSERT_EQ(border.size(), 1U);
  EXPECT_FALSE(border[0].closed);
  EXPECT_STREQ(border[0].reason, "rim_unmatched");
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
      smokeFaces(frame, kGeometry, kReturns, kWidth, kHeight, rawEverywhere(36U),
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
      smokeFaces(dense, kGeometry, kReturns, kWidth, kHeight, rawEverywhere(36U),
                 SilentPatchConfig{}, SmokeFaceConfig{});
  ASSERT_EQ(one.size(), 1U);
  EXPECT_EQ(one[0].free_neighbours, 1U);
  EXPECT_TRUE(one[0].closed);
}

} // namespace
} // namespace drone_city_nav
