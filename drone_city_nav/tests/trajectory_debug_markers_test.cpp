#include "drone_city_nav/trajectory_debug_markers.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std_msgs::msg::Header testHeader() {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  return header;
}

[[nodiscard]] std::vector<TrajectoryPointSample> testSamples() {
  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 3U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i);
    sample.point = Point2{static_cast<double>(i), 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.curvature_1pm = i == 1U ? 0.2 : 0.0;
    sample.z_m = 10.0 + static_cast<double>(i);
    samples.push_back(sample);
  }
  return samples;
}

[[nodiscard]] TrajectorySpeedProfile testProfile() {
  TrajectorySpeedProfile profile{};
  profile.valid = true;
  for (std::size_t i = 0U; i < 3U; ++i) {
    TrajectorySpeedSample sample{};
    sample.s_m = static_cast<double>(i);
    sample.profiled_limit_mps = 10.0 - static_cast<double>(i);
    sample.geometric_limit_mps = sample.profiled_limit_mps;
    profile.samples.push_back(sample);
  }
  return profile;
}

} // namespace

TEST(TrajectoryDebugMarkers, BuildsSpeedAndCurvatureColorMaps) {
  const visualization_msgs::msg::MarkerArray markers =
      buildTrajectoryDebugMarkers(testHeader(), testSamples(), testProfile());

  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_EQ(markers.markers[0].ns, "final_trajectory_speed_colormap");
  EXPECT_EQ(markers.markers[1].ns, "final_trajectory_curvature_colormap");
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(markers.markers[0].points.size(), 4U);
  EXPECT_EQ(markers.markers[0].colors.size(), markers.markers[0].points.size());
  EXPECT_EQ(markers.markers[1].colors.size(), markers.markers[1].points.size());
  EXPECT_DOUBLE_EQ(markers.markers[0].points[0].z, 10.04);
  EXPECT_DOUBLE_EQ(markers.markers[0].points[1].z, 11.04);
  EXPECT_DOUBLE_EQ(markers.markers[1].points[0].z, 10.08);
  EXPECT_DOUBLE_EQ(markers.markers[1].points[1].z, 11.08);
}

TEST(TrajectoryDebugMarkers, EmptyTrajectoryDeletesPreviousMarkers) {
  const visualization_msgs::msg::MarkerArray markers =
      buildTrajectoryDebugMarkers(testHeader(), {}, TrajectorySpeedProfile{});

  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(markers.markers[1].action, visualization_msgs::msg::Marker::DELETE);
}

} // namespace drone_city_nav
