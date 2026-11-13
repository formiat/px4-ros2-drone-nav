#include "drone_city_nav/thermal_reading.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::size_t kWidth{160U};
constexpr std::size_t kHeight{120U};
constexpr double kFocalPx{80.0 / 0.542955}; // tan(57/2 degrees)

std::vector<std::uint16_t> frameOf(const std::uint16_t background) {
  return std::vector<std::uint16_t>(kWidth * kHeight, background);
}

// A block of `size` pixels a side with its top-left corner at (u, v).
void heat(std::vector<std::uint16_t>& frame, const std::size_t u, const std::size_t v,
          const std::size_t size, const std::uint16_t kelvin_hundredths) {
  for (std::size_t row = v; row < v + size; ++row) {
    for (std::size_t column = u; column < u + size; ++column) {
      frame[row * kWidth + column] = kelvin_hundredths;
    }
  }
}

double norm(const Point3& p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

TEST(ThermalReadingTest, ASourceIsReadWithItsDirectionAndWidth) {
  std::vector<std::uint16_t> frame = frameOf(28800U);
  heat(frame, 100U, 50U, 10U, 50000U);
  const ThermalReading reading =
      readThermalFrame(frame, kWidth, kHeight, ThermalReadingConfig{});
  ASSERT_TRUE(reading.hot());
  EXPECT_EQ(reading.hot_pixels, 100U);
  EXPECT_DOUBLE_EQ(reading.hot_share, 100.0 / (kWidth * kHeight));
  // The block's centre (104.5, 54.5) is right of the frame's centre (79.5) and above
  // (59.5).
  EXPECT_NEAR(norm(reading.direction), 1.0, 1e-12);
  EXPECT_NEAR(std::atan2(-reading.direction.y, reading.direction.x),
              std::atan(25.0 / kFocalPx), 1e-6);
  EXPECT_GT(reading.direction.z, 0.0);
  // A hundred hot pixels of a 0.3 m body: 8.4 m by its size (7000 pixel-metres
  // squared).
  EXPECT_NEAR(reading.range_by_size_m, std::sqrt(70.0), 1e-9);
  // A body cut by the frame's edge reads no size.
  std::vector<std::uint16_t> cut = frameOf(28800U);
  heat(cut, 150U, 50U, 10U, 50000U);
  EXPECT_EQ(
      readThermalFrame(cut, kWidth, kHeight, ThermalReadingConfig{}).range_by_size_m,
      0.0);
  // Half the block's diagonal, 6.4 px, plus half a pixel.
  EXPECT_NEAR(reading.half_angle_rad,
              std::atan(6.36 / kFocalPx) + std::atan(0.5 / kFocalPx), 0.003);
}

TEST(ThermalReadingTest, TheLocationsWarmSurfacesAndAFewPixelsAreNoSource) {
  std::vector<std::uint16_t> frame = frameOf(28800U);
  // A wall at 311 K over the upper half: 23 K over the median, under the threshold.
  heat(frame, 0U, 0U, kHeight / 2U, 31100U);
  EXPECT_FALSE(readThermalFrame(frame, kWidth, kHeight, ThermalReadingConfig{}).hot());
  frame = frameOf(28800U);
  frame[0] = 50000U;
  frame[1] = 50000U;
  frame[2] = 50000U;
  const ThermalReading three =
      readThermalFrame(frame, kWidth, kHeight, ThermalReadingConfig{});
  EXPECT_FALSE(three.hot());
  EXPECT_DOUBLE_EQ(three.hot_share, 3.0 / (kWidth * kHeight));
  EXPECT_FALSE(readThermalFrame(frame, 0U, 0U, ThermalReadingConfig{}).hot());
}

// A block `columns` wide and `rows` tall with its top-left corner at (u, v).
void warm(std::vector<std::uint16_t>& frame, const std::size_t u, const std::size_t v,
          const std::size_t columns, const std::size_t rows,
          const std::uint16_t kelvin_hundredths) {
  for (std::size_t row = v; row < v + rows; ++row) {
    for (std::size_t column = u; column < u + columns; ++column) {
      frame[row * kWidth + column] = kelvin_hundredths;
    }
  }
}

// The rig of 2026-11-06 (roadmap item 21): a person at 5 m is 14 x 53 pixels
// reading 305.83 K in a frame of 288 K.
constexpr std::size_t kPersonPixels{742U};

TEST(ThermalBodyTest, ABodyIsReadWithItsDirectionExtentAndRange) {
  std::vector<std::uint16_t> frame = frameOf(28800U);
  warm(frame, 73U, 40U, 14U, 53U, 30583U);
  const std::vector<ThermalBody> bodies = readThermalBodies(
      frame, kWidth, kHeight, ThermalReadingConfig{}, ThermalBodyConfig{});
  ASSERT_EQ(bodies.size(), 1U);
  const ThermalBody& body = bodies.front();
  EXPECT_EQ(body.warm_pixels, kPersonPixels);
  EXPECT_NEAR(norm(body.direction), 1.0, 1e-12);
  // The block's centre (79.5, 66) is on the frame's centre column and below
  // its centre row (59.5).
  EXPECT_NEAR(body.direction.y, 0.0, 1e-9);
  EXPECT_LT(body.direction.z, 0.0);
  EXPECT_NEAR(body.width_rad, 2.0 * std::atan(7.0 / kFocalPx), 1e-4);
  EXPECT_NEAR(body.height_rad, 2.0 * std::atan(26.5 / kFocalPx), 1e-4);
  EXPECT_NEAR(body.range_by_size_m, std::sqrt(18300.0 / 742.0), 1e-9);
  EXPECT_TRUE(bodySized(body, 5.0, ThermalBodyConfig{}));
  // Read at the range of a wall 25 m off it would be 2.4 m wide: no body.
  EXPECT_FALSE(bodySized(body, 25.0, ThermalBodyConfig{}));
  EXPECT_FALSE(bodySized(body, 0.0, ThermalBodyConfig{}));
  // The source's reader sees nothing of it: 17.8 K is under its 30.
  EXPECT_FALSE(readThermalFrame(frame, kWidth, kHeight, ThermalReadingConfig{}).hot());
  // A body the frame's edge cuts reads no size.
  std::vector<std::uint16_t> cut = frameOf(28800U);
  warm(cut, 0U, 40U, 14U, 53U, 30583U);
  EXPECT_EQ(readThermalBodies(cut, kWidth, kHeight, ThermalReadingConfig{},
                              ThermalBodyConfig{})
                .front()
                .range_by_size_m,
            0.0);
}

TEST(ThermalBodyTest, AWarmerWallBehindTheBodyIsARegionOfItsOwnAndNoBody) {
  std::vector<std::uint16_t> frame = frameOf(28800U);
  // The rig: a wall tagged 310 K 25 m off, a band 72 x 38 pixels reading
  // 310.97 K, and the person before it at 5 m.
  warm(frame, 44U, 40U, 72U, 38U, 31097U);
  warm(frame, 73U, 40U, 14U, 53U, 30583U);
  const std::vector<ThermalBody> bodies = readThermalBodies(
      frame, kWidth, kHeight, ThermalReadingConfig{}, ThermalBodyConfig{});
  // The wall's band either side of the body, and the body.
  ASSERT_EQ(bodies.size(), 3U);
  std::size_t of_a_body{0U};
  for (const ThermalBody& region : bodies) {
    if (region.warm_pixels == kPersonPixels) {
      EXPECT_TRUE(bodySized(region, 5.0, ThermalBodyConfig{}));
      ++of_a_body;
    } else {
      // The wall's halves at the wall's range: 5 m wide each.
      EXPECT_FALSE(bodySized(region, 24.7, ThermalBodyConfig{}));
    }
  }
  EXPECT_EQ(of_a_body, 1U);
}

TEST(ThermalBodyTest, ABodyIsRangedByTheHitsInItsSilhouetteAndItsReturnsAreItsOwn) {
  std::vector<std::uint16_t> frame = frameOf(28800U);
  warm(frame, 73U, 40U, 14U, 53U, 30583U);
  const std::vector<ThermalBody> regions = readThermalBodies(
      frame, kWidth, kHeight, ThermalReadingConfig{}, ThermalBodyConfig{});
  ASSERT_EQ(regions.size(), 1U);
  const Point3 towards = regions.front().direction;
  // The body's surface 5 m off, the wall behind it 12 m off beside the silhouette,
  // a free ray through the silhouette, which is no range.
  std::vector<StereoDepthReturn> returns;
  for (int i = -2; i <= 2; ++i) {
    returns.push_back(
        {.point = Point3{5.0 * towards.x, 5.0 * towards.y + 0.05 * i, 5.0 * towards.z},
         .hit = true});
  }
  returns.push_back({.point = Point3{12.0, 3.0, 0.0}, .hit = true});
  returns.push_back({.point = Point3{9.0 * towards.x, 9.0 * towards.y, 9.0 * towards.z},
                     .hit = false});
  const std::vector<ThermalBodySighting> bodies =
      sightBodies(regions, returns, 0.02, ThermalBodyConfig{});
  ASSERT_EQ(bodies.size(), 1U);
  EXPECT_NEAR(bodies.front().range_m, 5.0, 0.01);
  EXPECT_TRUE(onBody(returns.front().point, bodies.front(), 0.02, 0.75));
  EXPECT_FALSE(onBody(returns.at(5).point, bodies.front(), 0.02, 0.75));
  // The wall straight behind the body is in its silhouette and beyond its depth.
  EXPECT_FALSE(onBody(Point3{12.0 * towards.x, 12.0 * towards.y, 12.0 * towards.z},
                      bodies.front(), 0.02, 0.75));
  // Without a hit the region is ranged by its own size, 4.97 m.
  const std::vector<ThermalBodySighting> by_size =
      sightBodies(regions, {}, 0.02, ThermalBodyConfig{});
  ASSERT_EQ(by_size.size(), 1U);
  EXPECT_NEAR(by_size.front().range_m, std::sqrt(18300.0 / 742.0), 1e-9);
  // More of the silhouette's hits on the wall behind than on the body: it is
  // ranged by the hits that stand where its size puts it (r3479).
  std::vector<StereoDepthReturn> behind;
  behind.reserve(7U);
  for (int i = 0; i < 2; ++i) {
    behind.push_back(
        {.point = Point3{5.0 * towards.x, 5.0 * towards.y + 0.05 * i, 5.0 * towards.z},
         .hit = true});
  }
  for (int i = -2; i <= 2; ++i) {
    behind.push_back({.point = Point3{12.0 * towards.x, 12.0 * towards.y + 0.05 * i,
                                      12.0 * towards.z},
                      .hit = true});
  }
  const std::vector<ThermalBodySighting> before_the_wall =
      sightBodies(regions, behind, 0.02, ThermalBodyConfig{});
  ASSERT_EQ(before_the_wall.size(), 1U);
  EXPECT_NEAR(before_the_wall.front().range_m, 5.0, 0.01);
  // Hits that put the region 25 m off make it a wall's width: no body.
  std::vector<StereoDepthReturn> far;
  far.push_back({.point = Point3{25.0 * towards.x, 25.0 * towards.y, 25.0 * towards.z},
                 .hit = true});
  EXPECT_TRUE(sightBodies(regions, far, 0.02, ThermalBodyConfig{}).empty());
}

TEST(ThermalBodyTest, ASourceAWallThatFillsTheFrameAndAFewPixelsAreNoBody) {
  // The 500 K source of roadmap item 32: every pixel of it is over the
  // source's threshold, so none is warm.
  std::vector<std::uint16_t> frame = frameOf(28800U);
  heat(frame, 100U, 50U, 10U, 50115U);
  EXPECT_TRUE(readThermalBodies(frame, kWidth, kHeight, ThermalReadingConfig{},
                                ThermalBodyConfig{})
                  .empty());
  EXPECT_TRUE(readThermalFrame(frame, kWidth, kHeight, ThermalReadingConfig{}).hot());
  // A warm wall that fills the frame is the frame's median: nothing stands over it.
  frame = frameOf(31097U);
  warm(frame, 73U, 40U, 14U, 53U, 30583U);
  EXPECT_TRUE(readThermalBodies(frame, kWidth, kHeight, ThermalReadingConfig{},
                                ThermalBodyConfig{})
                  .empty());
  // Seven warm pixels are under the least.
  frame = frameOf(28800U);
  warm(frame, 10U, 10U, 7U, 1U, 30583U);
  EXPECT_TRUE(readThermalBodies(frame, kWidth, kHeight, ThermalReadingConfig{},
                                ThermalBodyConfig{})
                  .empty());
  EXPECT_TRUE(
      readThermalBodies(frame, 0U, 0U, ThermalReadingConfig{}, ThermalBodyConfig{})
          .empty());
}

TEST(ThermalReadingTest, TheRangeIsTheHitsInTheConeOrTheConservativeOne) {
  ThermalReading reading;
  reading.hot_pixels = 20U;
  reading.direction = Point3{1.0, 0.0, 0.0};
  reading.half_angle_rad = 0.05;
  std::vector<StereoDepthReturn> returns;
  returns.push_back({.point = {6.0, 0.1, 0.0}, .hit = true});
  returns.push_back({.point = {5.0, 0.0, -0.1}, .hit = true});
  returns.push_back({.point = {7.0, 0.0, 0.2}, .hit = true});
  returns.push_back({.point = {2.0, 1.5, 0.0}, .hit = true});  // outside the cone
  returns.push_back({.point = {3.0, 0.0, 0.0}, .hit = false}); // a free ray
  returns.push_back({.point = {2.5, 0.0, 0.0}, .hit = true, .unobservable = true});
  EXPECT_DOUBLE_EQ(heatSourceRangeM(reading, returns, 0.02, 12.0),
                   std::sqrt(36.0 + 0.01));
  // The hits beyond the ceiling are the smoke's chance matches (r2237).
  EXPECT_DOUBLE_EQ(heatSourceRangeM(reading, returns, 0.02, 5.2), 5.2);
  EXPECT_DOUBLE_EQ(
      heatSourceRangeM(reading, std::vector<StereoDepthReturn>{}, 0.02, 5.2), 5.2);
}

TEST(ThermalReadingTest, TheCapIsWholeAndTheBallDropsTheReturnsThroughIt) {
  ThermalReading reading;
  reading.hot_pixels = 20U;
  reading.direction = Point3{0.8, 0.6, 0.0};
  const std::vector<Point3> cap = heatStandoffCap(reading, 5.2, 4.0, 0.5);
  ASSERT_GT(cap.size(), 900U);
  const Point3 centre{5.2 * 0.8, 5.2 * 0.6, 0.0};
  double nearest{1e9};
  for (const Point3& point : cap) {
    const Point3 offset{point.x - centre.x, point.y - centre.y, point.z - centre.z};
    EXPECT_LE(norm(offset), 4.0 + 1e-9);
    nearest = std::min(nearest, norm(point));
  }
  // Filled to its apex, 1.2 m before the vehicle: no hole (r2237, r2239).
  EXPECT_LT(nearest, 1.6);
  // The whole ball, its far side too (r2321): as many marks behind the centre as
  // before.
  const auto behind = std::ranges::count_if(cap, [&](const Point3& point) {
    return (point.x - centre.x) * reading.direction.x +
               (point.y - centre.y) * reading.direction.y >
           0.0;
  });
  EXPECT_GT(static_cast<double>(behind), 0.4 * static_cast<double>(cap.size()));
  // Inside the ball (the range under the standoff) the same whole ball.
  const std::vector<Point3> ball = heatStandoffCap(reading, 3.0, 4.0, 0.5);
  EXPECT_EQ(ball.size(), cap.size());
  EXPECT_TRUE(heatStandoffCap(reading, 5.2, 0.0, 0.5).empty());
  // The returns: inside the ball, or beyond it along a ray through it, dropped;
  // beside it and before it kept.
  EXPECT_TRUE(crossesHeatStandoff(centre, reading, 5.2, 4.0));
  EXPECT_TRUE(
      crossesHeatStandoff(Point3{3.0 * 0.8, 3.0 * 0.6, 0.0}, reading, 5.2, 4.0));
  EXPECT_TRUE(
      crossesHeatStandoff(Point3{20.0 * 0.8, 20.0 * 0.6, 0.0}, reading, 5.2, 4.0));
  EXPECT_TRUE(
      crossesHeatStandoff(Point3{12.0 * 0.8, 12.0 * 0.6, 3.0}, reading, 5.2, 4.0));
  EXPECT_FALSE(
      crossesHeatStandoff(Point3{1.0 * 0.8, 1.0 * 0.6, 0.0}, reading, 5.2, 4.0));
  EXPECT_FALSE(crossesHeatStandoff(Point3{0.0, 0.0, 0.0}, reading, 5.2, 4.0));
  EXPECT_FALSE(crossesHeatStandoff(Point3{-6.0, 0.0, 0.0}, reading, 5.2, 4.0));
  EXPECT_FALSE(crossesHeatStandoff(Point3{0.0, 0.0, 10.0}, reading, 5.2, 4.0));
  EXPECT_FALSE(
      crossesHeatStandoff(Point3{6.0 * 0.6, -6.0 * 0.8, 0.0}, reading, 5.2, 4.0));
}

TEST(ThermalReadingTest, TheSourceIsFixedFromBearingsOverTheMotion) {
  // The vehicle flies along y past a source at (12, 22, 11), looking at it
  // from 8 m aside: a baseline of 2 m over 2 s fixes it within a decimetre.
  const Point3 source{12.0, 22.0, 11.0};
  HeatSourceFix fix;
  EXPECT_FALSE(fix.point().has_value());
  for (int step = 0; step <= 20; ++step) {
    const Point3 origin{4.0, 12.0 + 0.1 * step, 9.0};
    const Point3 offset{source.x - origin.x, source.y - origin.y, source.z - origin.z};
    const double length = norm(offset);
    fix.observe(origin, Point3{offset.x / length, offset.y / length, offset.z / length},
                0.1 * step);
    if (step < 10) {
      EXPECT_FALSE(fix.point().has_value()) << step; // under a metre of baseline
    }
  }
  EXPECT_NEAR(fix.baselineM(), 2.0 * 1.0 * std::sin(std::atan2(8.0, 9.0)), 0.15);
  const std::optional<Point3> point = fix.point();
  ASSERT_TRUE(point.has_value());
  EXPECT_LT(fix.residualRad(), 0.01);
  const Point3 fixed = point.value_or(Point3{});
  EXPECT_NEAR(fixed.x, source.x, 0.1);
  EXPECT_NEAR(fixed.y, source.y, 0.1);
  EXPECT_NEAR(fixed.z, source.z, 0.1);
  // A fan of bearings from one stand, turned by the yaw's errors over a short
  // climb, has a point near the stand within a metre of every ray (r2238):
  // by angle it is no fix.
  HeatSourceFix fan;
  for (int step = 0; step <= 20; ++step) {
    const double yaw = 0.3 * std::sin(0.7 * step);
    fan.observe({0.0, 0.0, 0.1 * step}, {std::cos(yaw), std::sin(yaw), 0.0},
                0.1 * step);
  }
  EXPECT_GE(fan.baselineM(), 1.0);
  EXPECT_FALSE(fan.point().has_value());
  EXPECT_GT(fan.residualRad(), 0.09);
  // Bearings that agree on nothing give no fix; the window drops the old ones.
  HeatSourceFix scatter;
  scatter.observe({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.0);
  scatter.observe({0.0, 3.0, 0.0}, {1.0, 0.0, 0.0}, 0.5);
  scatter.observe({0.0, -3.0, 0.0}, {0.0, 0.0, 1.0}, 1.0);
  EXPECT_FALSE(scatter.point().has_value());
  scatter.observe({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 20.0);
  EXPECT_EQ(scatter.bearings(), 1U);
  // Along the bearing the origins spread no baseline.
  HeatSourceFix along;
  along.observe({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.0);
  along.observe({3.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 1.0);
  EXPECT_DOUBLE_EQ(along.baselineM(), 0.0);
  EXPECT_FALSE(along.point().has_value());
}

} // namespace
} // namespace drone_city_nav
