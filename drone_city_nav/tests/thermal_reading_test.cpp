#include "drone_city_nav/thermal_reading.hpp"

#include <gtest/gtest.h>

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
  const std::vector<Point3> cap = heatStandoffCap(reading, 5.2, 4.0, 0.25);
  ASSERT_GT(cap.size(), 1000U);
  const Point3 centre{5.2 * 0.8, 5.2 * 0.6, 0.0};
  double nearest{1e9};
  for (const Point3& point : cap) {
    const Point3 offset{point.x - centre.x, point.y - centre.y, point.z - centre.z};
    EXPECT_NEAR(norm(offset), 4.0, 1e-9);
    EXPECT_LT(offset.x * reading.direction.x + offset.y * reading.direction.y, 0.0);
    nearest = std::min(nearest, norm(point));
  }
  // The apex, 1.2 m before the vehicle, is laid: no hole (r2237).
  EXPECT_LT(nearest, 1.3);
  // Inside the ball (the range under the standoff) the whole ball is laid.
  const std::vector<Point3> ball = heatStandoffCap(reading, 3.0, 4.0, 0.25);
  EXPECT_GT(ball.size(), 1.9 * static_cast<double>(cap.size()));
  EXPECT_TRUE(heatStandoffCap(reading, 5.2, 0.0, 0.25).empty());
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
