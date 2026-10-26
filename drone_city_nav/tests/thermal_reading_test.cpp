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
  EXPECT_DOUBLE_EQ(heatSourceRangeM(reading, returns, 0.02, 5.2),
                   std::sqrt(36.0 + 0.01));
  EXPECT_DOUBLE_EQ(
      heatSourceRangeM(reading, std::vector<StereoDepthReturn>{}, 0.02, 5.2), 5.2);
}

TEST(ThermalReadingTest, TheCapFacesTheVehicleAtTheStandoffAndKeepsOffIt) {
  ThermalReading reading;
  reading.hot_pixels = 20U;
  reading.direction = Point3{0.8, 0.6, 0.0};
  const std::vector<Point3> cap = heatStandoffCap(reading, 5.2, 4.0, 0.25, 1.5);
  ASSERT_GT(cap.size(), 1000U);
  const Point3 centre{5.2 * 0.8, 5.2 * 0.6, 0.0};
  for (const Point3& point : cap) {
    const Point3 offset{point.x - centre.x, point.y - centre.y, point.z - centre.z};
    EXPECT_NEAR(norm(offset), 4.0, 1e-9);
    EXPECT_LT(offset.x * reading.direction.x + offset.y * reading.direction.y, 0.0);
    EXPECT_GE(norm(point), 1.5);
    EXPECT_TRUE(
        withinHeatStandoff(Point3{0.99 * point.x, 0.99 * point.y, 0.99 * point.z},
                           reading, 5.2, 4.0) ||
        norm(Point3{0.99 * point.x - centre.x, 0.99 * point.y - centre.y,
                    0.99 * point.z - centre.z}) >= 4.0);
  }
  EXPECT_TRUE(withinHeatStandoff(centre, reading, 5.2, 4.0));
  EXPECT_FALSE(withinHeatStandoff(Point3{0.0, 0.0, 0.0}, reading, 5.2, 4.0));
  // Inside the ball (the range under the standoff) the half behind the vehicle is
  // dropped.
  const std::vector<Point3> near = heatStandoffCap(reading, 3.0, 4.0, 0.25, 1.5);
  for (const Point3& point : near) {
    EXPECT_GE(norm(point), 1.5);
  }
  EXPECT_LT(near.size(), cap.size());
  EXPECT_TRUE(heatStandoffCap(reading, 5.2, 0.0, 0.25, 1.5).empty());
}

} // namespace
} // namespace drone_city_nav
