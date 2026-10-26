#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "simulation_smoke_model.hpp"

namespace drone_city_nav::simulation_smoke {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// A 4 m cube of smoke centred 4 m ahead of the origin, 2 m off the floor.
SmokeField cube(double sigma, std::uint32_t seed = 1U) {
  return SmokeField{
      std::vector<SmokeZone>{SmokeZone{{4.0, 0.0, 2.0}, {4.0, 4.0, 4.0}, sigma}}, seed};
}

TEST(SimulationSmokeModelTest, ParsesZonesAndRefusesAnythingElse) {
  const auto zones = parseSmokeZones("1,2,3,4,5,6,0.5; -1, 0 ,2.5,1,1,1,2");
  ASSERT_EQ(zones.size(), 2U);
  EXPECT_EQ(zones[0].center, Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(zones[0].size, Eigen::Vector3d(4.0, 5.0, 6.0));
  EXPECT_DOUBLE_EQ(zones[1].extinction_per_m, 2.0);
  EXPECT_TRUE(parseSmokeZones("").empty());
  EXPECT_THROW(static_cast<void>(parseSmokeZones("1,2,3,4,5,6")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseSmokeZones("1,2,3,4,5,6,1,8")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseSmokeZones("1,2,3,0,5,6,1")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseSmokeZones("1,2,3,4,5,6,-1")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseSmokeZones("1,2,x,4,5,6,1")),
               std::invalid_argument);
}

TEST(SimulationSmokeModelTest, ParsesHeatSourcesAndRefusesAnythingElse) {
  const auto sources = parseHeatSources("12,22,11,0.3,500; -26.6, 24.5 ,5,0.1,350");
  ASSERT_EQ(sources.size(), 2U);
  EXPECT_EQ(sources[0].center, Eigen::Vector3d(12.0, 22.0, 11.0));
  EXPECT_DOUBLE_EQ(sources[0].radius_m, 0.3);
  EXPECT_DOUBLE_EQ(sources[1].kelvin, 350.0);
  EXPECT_TRUE(parseHeatSources("").empty());
  EXPECT_THROW(static_cast<void>(parseHeatSources("1,2,3,0.3")), std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseHeatSources("1,2,3,0.3,500,1")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseHeatSources("1,2,3,0,500")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseHeatSources("1,2,3,0.3,-5")),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(parseHeatSources("1,2,x,0.3,500")),
               std::invalid_argument);
}

TEST(SimulationSmokeModelTest, TheFieldHoldsTheZonesFigureOnAverageAndChurns) {
  const SmokeField field = cube(1.0);
  EXPECT_EQ(field.extinction({4.0, 0.0, 4.5}, 0.0), 0.0);
  EXPECT_EQ(field.extinction({-0.1, 0.0, 2.0}, 0.0), 0.0);
  double sum = 0.0;
  int count = 0;
  double low = kInfinity;
  double high = 0.0;
  for (int i = 0; i <= 20; ++i) {
    for (int j = 0; j <= 20; ++j) {
      const double sigma = field.extinction({3.0 + 0.1 * i, -1.0 + 0.1 * j, 2.0}, 0.0);
      sum += sigma;
      ++count;
      low = std::min(low, sigma);
      high = std::max(high, sigma);
    }
  }
  EXPECT_NEAR(sum / count, 1.0, 0.2);
  EXPECT_GE(low, 0.0);
  EXPECT_LE(high, 1.6);
  EXPECT_GT(high - low, 0.2);
  const Eigen::Vector3d point{4.0, 0.3, 2.0};
  EXPECT_NE(field.extinction(point, 0.0), field.extinction(point, 5.0));
  EXPECT_EQ(field.extinction(point, 5.0), cube(1.0).extinction(point, 5.0));
  EXPECT_NE(field.extinction(point, 5.0), cube(1.0, 2U).extinction(point, 5.0));
}

TEST(SimulationSmokeModelTest, SegmentsAreWhereTheRayCrossesTheZones) {
  const SmokeField field{
      std::vector<SmokeZone>{SmokeZone{{4.0, 0.0, 2.0}, {4.0, 4.0, 4.0}, 1.0},
                             SmokeZone{{6.5, 0.0, 2.0}, {3.0, 1.0, 1.0}, 1.0},
                             SmokeZone{{20.0, 0.0, 2.0}, {2.0, 2.0, 2.0}, 1.0}},
      0U};
  std::vector<RaySegment> segments;
  field.segments({0.0, 0.0, 2.0}, Eigen::Vector3d::UnitX(), 30.0, segments);
  ASSERT_EQ(segments.size(), 2U);
  EXPECT_NEAR(segments[0].entry_m, 2.0, 1e-9);
  EXPECT_NEAR(segments[0].exit_m, 8.0, 1e-9);
  EXPECT_NEAR(segments[1].entry_m, 19.0, 1e-9);
  field.segments({0.0, 0.0, 2.0}, Eigen::Vector3d::UnitX(), 3.0, segments);
  ASSERT_EQ(segments.size(), 1U);
  EXPECT_NEAR(segments[0].exit_m, 3.0, 1e-9);
  field.segments({0.0, 0.0, 2.0}, -Eigen::Vector3d::UnitX(), 30.0, segments);
  EXPECT_TRUE(segments.empty());
  field.segments({4.0, 0.0, 2.0}, Eigen::Vector3d::UnitY(), 30.0, segments);
  ASSERT_EQ(segments.size(), 1U);
  EXPECT_NEAR(segments[0].entry_m, 0.0, 1e-9);
  EXPECT_NEAR(segments[0].exit_m, 2.0, 1e-9);
  EXPECT_TRUE(field.within({0.0, 0.0, 2.0}, 2.0));
  EXPECT_FALSE(field.within({-3.0, 0.0, 2.0}, 4.0));
}

TEST(SimulationSmokeModelTest, ThePhaseIsNormalisedAndGivesASmokeLidarRatio) {
  double integral = 0.0;
  constexpr int kSteps = 20000;
  for (int index = 0; index < kSteps; ++index) {
    const double theta = std::numbers::pi * (index + 0.5) / kSteps;
    integral += phase(std::cos(theta)) * 2.0 * std::numbers::pi * std::sin(theta) *
                std::numbers::pi / kSteps;
  }
  EXPECT_NEAR(integral, 1.0, 1e-3);
  const double lidar_ratio_sr = 1.0 / (kSingleScatteringAlbedo * phase(-1.0));
  EXPECT_GT(lidar_ratio_sr, 40.0);
  EXPECT_LT(lidar_ratio_sr, 70.0);
}

TEST(SimulationSmokeModelTest, AnActiveSensorReadsThroughNoSmokeAsBefore) {
  const SmokeField clear{std::vector<SmokeZone>{}, 0U};
  const ActiveSensor lidar{0.2, 35.0, 1.0};
  const Eigen::Vector3d origin{0.0, 0.0, 2.0};
  EXPECT_EQ(activeRange(clear, lidar, origin, Eigen::Vector3d::UnitX(), 9.5, 0.0, 0.5),
            9.5);
  EXPECT_EQ(
      activeRange(clear, lidar, origin, Eigen::Vector3d::UnitX(), kInfinity, 0.0, 0.5),
      kInfinity);
  // A surface before the smoke is read as it is.
  EXPECT_EQ(
      activeRange(cube(3.0), lidar, origin, Eigen::Vector3d::UnitX(), 1.5, 0.0, 0.5),
      1.5);
  // Smoke of no density is no smoke.
  EXPECT_EQ(
      activeRange(cube(0.0), lidar, origin, Eigen::Vector3d::UnitX(), 9.5, 0.0, 0.5),
      9.5);
}

TEST(SimulationSmokeModelTest, ALastReturnSensorReportsTheWallBehindThinSmoke) {
  // The lidar's gate of a metre against a 10 percent surface at 35 m, inside
  // thin smoke (0.1/m) with a wall 8 m on: the strongest echo is the smoke's
  // near the sensor, the last the wall's.
  const ActiveSensor strongest{0.2, 35.0, 1.0, 1.0, 0.1, false};
  const ActiveSensor last{0.2, 35.0, 1.0, 1.0, 0.1, true};
  const Eigen::Vector3d origin{0.0, 0.0, 2.0};
  const SmokeField thin{
      std::vector<SmokeZone>{SmokeZone{{5.0, 0.0, 2.0}, {12.0, 6.0, 3.0}, 0.1}}, 0U};
  const double by_strongest =
      activeRange(thin, strongest, origin, Eigen::Vector3d::UnitX(), 8.0, 0.0, 0.5);
  const double by_last =
      activeRange(thin, last, origin, Eigen::Vector3d::UnitX(), 8.0, 0.0, 0.5);
  EXPECT_LT(by_strongest, 3.0);
  EXPECT_DOUBLE_EQ(by_last, 8.0);
  // Dense smoke (3/m) hides the wall from both.
  const SmokeField dense{
      std::vector<SmokeZone>{SmokeZone{{5.0, 0.0, 2.0}, {12.0, 6.0, 3.0}, 3.0}}, 0U};
  EXPECT_LT(activeRange(dense, last, origin, Eigen::Vector3d::UnitX(), 8.0, 0.0, 0.5),
            3.0);
}

TEST(SimulationSmokeModelTest, DenseSmokeReturnsTheBeamThinSmokeLetsItThrough) {
  const ActiveSensor lidar{0.2, 35.0, 1.0};
  const Eigen::Vector3d origin{0.0, 0.0, 2.0};
  const double dense =
      activeRange(cube(3.0), lidar, origin, Eigen::Vector3d::UnitX(), 9.5, 0.0, 0.5);
  // The echo relative to a matte surface's at the reported range: one for
  // the surface seen clear, a thin fraction for the smoke (its backscatter
  // over the lidar ratio against the surface's reflectance).
  const SmokeField clear{std::vector<SmokeZone>{}, 0U};
  double relative = -1.0;
  static_cast<void>(activeRange(clear, lidar, origin, Eigen::Vector3d::UnitX(), 9.5,
                                0.0, 0.5, &relative));
  EXPECT_DOUBLE_EQ(relative, 1.0);
  static_cast<void>(activeRange(cube(3.0), lidar, origin, Eigen::Vector3d::UnitX(), 9.5,
                                0.0, 0.5, &relative));
  EXPECT_GT(relative, 0.0);
  EXPECT_LT(relative, 0.5);
  static_cast<void>(activeRange(clear, lidar, origin, Eigen::Vector3d::UnitX(),
                                std::numeric_limits<double>::infinity(), 0.0, 0.5,
                                &relative));
  EXPECT_DOUBLE_EQ(relative, 0.0);
  EXPECT_GE(dense, 2.0);
  EXPECT_LT(dense, 3.0);
  const double open = activeRange(cube(3.0), lidar, origin, Eigen::Vector3d::UnitX(),
                                  kInfinity, 0.0, 0.5);
  EXPECT_GE(open, 2.0);
  EXPECT_LT(open, 3.0);
  EXPECT_EQ(
      activeRange(cube(0.02), lidar, origin, Eigen::Vector3d::UnitX(), 9.5, 0.0, 0.5),
      9.5);
  // A time-of-flight unit (2.8 m of range) inside dense smoke reads it at
  // once; a haze round it does not blind it.
  const ActiveSensor tof{0.02, 2.8, 0.25};
  const double inside = activeRange(cube(1.0), tof, {4.0, 0.0, 2.0},
                                    Eigen::Vector3d::UnitX(), kInfinity, 0.0, 0.5);
  EXPECT_LT(inside, 0.5);
  EXPECT_EQ(activeRange(cube(0.2), tof, {4.0, 0.0, 2.0}, Eigen::Vector3d::UnitX(), 1.5,
                        0.0, 0.5),
            1.5);
}

TEST(SimulationSmokeModelTest, TheLightsResponseFallsWithDistance) {
  EXPECT_DOUBLE_EQ(lightResponse(0.0), lightResponse(0.25));
  EXPECT_GT(lightResponse(1.0), lightResponse(4.0));
  EXPECT_GT(lightResponse(4.0), lightResponse(10.0));
  EXPECT_NEAR(lightResponse(5.0), 0.5 * (0.742 + 0.365), 1e-9);
  EXPECT_EQ(lightResponse(30.0), 0.0);
}

// The camera 2 m off the floor looking along +x, the light beside it.
struct CameraSmokeTest : ::testing::Test {
  static constexpr int kWidth = 64;
  static constexpr int kHeight = 48;
  static constexpr std::size_t kPixels = std::size_t{kWidth} * std::size_t{kHeight};
  static constexpr std::size_t kCentre =
      3U * (std::size_t{kHeight / 2} * std::size_t{kWidth} + std::size_t{kWidth / 2});
  CameraSmoke smoke{CameraIntrinsics{kWidth, kHeight, 2.0943951023931953}, 0.0, 7U, 2U};
  CameraPose pose{{0.0, 0.0, 2.0}, Eigen::Matrix3d::Identity()};
  CarriedLight light{{0.02, 0.0, 2.0}, Eigen::Vector3d::UnitX(), 1.0, 2.1, 2.4};
  std::vector<std::uint8_t> frame = std::vector<std::uint8_t>(kPixels * 3U, 128U);

  [[nodiscard]] static std::vector<float> wall_at(float planar_m) {
    return std::vector<float>(kPixels, planar_m);
  }

  [[nodiscard]] static double mean(const std::vector<std::uint8_t>& values) {
    double sum = 0.0;
    for (const auto value : values) {
      sum += value;
    }
    return sum / static_cast<double>(values.size());
  }
};

TEST_F(CameraSmokeTest, AFrameWithoutSmokeInViewIsUntouched) {
  const auto clean = frame;
  EXPECT_FALSE(smoke.apply(SmokeField{std::vector<SmokeZone>{}, 0U}, pose, light, 0.0,
                           frame, wall_at(9.0F)));
  EXPECT_EQ(frame, clean);
  // A surface before the smoke hides it.
  EXPECT_TRUE(smoke.apply(cube(2.0), pose, light, 0.0, frame, wall_at(1.0F)) ||
              frame == clean);
  EXPECT_EQ(frame, clean);
}

TEST_F(CameraSmokeTest, SmokeDimsTheWallBehindItAndVeilsItInTheLight) {
  const auto clean = frame;
  ASSERT_TRUE(smoke.apply(cube(2.0), pose, light, 0.0, frame, wall_at(9.0F)));
  // Two passes of four metres at two per metre leave nothing of the wall:
  // the centre shows the smoke's own veil.
  const std::size_t centre = kCentre;
  EXPECT_LT(frame[centre], clean[centre]);
  // In the dark the same smoke is black.
  std::vector<std::uint8_t> dark(frame.size(), 128U);
  CarriedLight off = light;
  off.share = 0.0;
  ASSERT_TRUE(smoke.apply(cube(2.0), pose, off, 0.0, dark, wall_at(9.0F)));
  EXPECT_LT(dark[centre], 3U);
  // A black wall through the lit smoke: what is left is the veil.
  std::vector<std::uint8_t> black(frame.size(), 0U);
  ASSERT_TRUE(smoke.apply(cube(2.0), pose, light, 0.0, black, wall_at(9.0F)));
  EXPECT_GT(mean(black), 10.0);
  EXPECT_EQ(black[centre], frame[centre]);
}

TEST_F(CameraSmokeTest, ThinSmokeLeavesTheWallVisible) {
  const auto clean = frame;
  ASSERT_TRUE(smoke.apply(cube(0.05), pose, light, 0.0, frame, wall_at(9.0F)));
  const std::size_t centre = kCentre;
  EXPECT_GT(frame[centre], 0.6 * clean[centre]);
  EXPECT_LE(frame[centre], clean[centre] + 5U);
}

TEST_F(CameraSmokeTest, TheFrameStaysAsNoisyAsTheCameraThroughDenseSmoke) {
  // Dense smoke takes the wall and the noise it came with; as much noise is
  // added again as the camera's own (0.05 gives some levels in the dark).
  CameraSmoke noisy{CameraIntrinsics{kWidth, kHeight, 2.0943951023931953}, 0.05, 7U,
                    2U};
  CarriedLight off = light;
  off.share = 0.0;
  std::vector<std::uint8_t> dark(frame.size(), 0U);
  ASSERT_TRUE(noisy.apply(cube(2.0), pose, off, 0.0, dark, wall_at(9.0F)));
  double sum = 0.0;
  double squares = 0.0;
  for (std::size_t pixel = 0U; pixel < dark.size(); pixel += 3U) {
    sum += dark[pixel];
    squares += static_cast<double>(dark[pixel]) * dark[pixel];
    EXPECT_EQ(dark[pixel], dark[pixel + 1U]);
  }
  const double count = static_cast<double>(dark.size()) / 3.0;
  const double spread = std::sqrt(squares / count - (sum / count) * (sum / count));
  EXPECT_GT(spread, 1.0);
}

} // namespace
} // namespace drone_city_nav::simulation_smoke
