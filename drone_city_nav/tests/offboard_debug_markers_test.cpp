#include "drone_city_nav/offboard_debug_markers.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std_msgs::msg::Header testHeader() {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 1;
  return header;
}

[[nodiscard]] std::vector<TrajectoryPointSample> testSamples() {
  return std::vector<TrajectoryPointSample>{
      TrajectoryPointSample{0.0, Point2{0.0, 0.0}, Point2{1.0, 0.0}, 0.0},
      TrajectoryPointSample{1.0, Point2{1.0, 0.0}, Point2{1.0, 0.0}, 0.0}};
}

[[nodiscard]] TrajectorySpeedProfile testSpeedProfile() {
  TrajectorySpeedProfile profile;
  profile.valid = true;
  profile.samples.push_back(TrajectorySpeedSample{0.0, 10.0, 10.0});
  profile.samples.push_back(TrajectorySpeedSample{1.0, 9.0, 9.0});
  return profile;
}

} // namespace

TEST(OffboardDebugMarkers, DeletesDroneMarkersWhenPoseIsStale) {
  const visualization_msgs::msg::MarkerArray markers =
      buildDroneDebugMarkers(testHeader(), DroneDebugMarkerState{}, 0.08);

  ASSERT_EQ(markers.markers.size(), 2U);
  EXPECT_EQ(markers.markers[0].ns, "drone_position");
  EXPECT_EQ(markers.markers[1].ns, "drone_heading");
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(markers.markers[1].action, visualization_msgs::msg::Marker::DELETE);
}

TEST(OffboardDebugMarkers, BuildsDroneAndTrajectoryMarkers) {
  const DroneDebugMarkerState state{true, Point2{2.0, 3.0}, 0.0};
  const visualization_msgs::msg::MarkerArray markers = buildOffboardDebugMarkers(
      testHeader(), state, testSamples(), testSpeedProfile(), 0.08);

  ASSERT_GE(markers.markers.size(), 4U);
  EXPECT_EQ(markers.markers[0].ns, "drone_position");
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(markers.markers[0].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(markers.markers[0].pose.position.y, 3.0);
  EXPECT_EQ(markers.markers[1].ns, "drone_heading");
  EXPECT_EQ(markers.markers[1].points.size(), 2U);
  EXPECT_EQ(markers.markers[2].ns, "final_trajectory_speed_colormap");
}

} // namespace drone_city_nav
