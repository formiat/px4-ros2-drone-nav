#include "drone_city_nav/directed_inflation_escape_debug_markers.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] std_msgs::msg::Header testHeader() {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  return header;
}

} // namespace

TEST(DirectedInflationEscapeDebugMarkers, ShowsTunnelCenterlineAndTarget) {
  DirectedInflationEscapeResult escape{};
  escape.applied = true;
  escape.target = Point2{4.0, 2.0};
  escape.centerline = {Point2{1.0, 1.0}, Point2{2.0, 1.5}, escape.target};

  const visualization_msgs::msg::MarkerArray markers =
      buildDirectedInflationEscapeDebugMarkers(testHeader(), escape, 5.0);

  ASSERT_EQ(markers.markers.size(), 3U);
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_DOUBLE_EQ(markers.markers[0].scale.x, 5.0);
  EXPECT_EQ(markers.markers[0].points.size(), 3U);
  EXPECT_EQ(markers.markers[1].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers[1].points.size(), 3U);
  EXPECT_EQ(markers.markers[2].type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_DOUBLE_EQ(markers.markers[2].pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(markers.markers[2].pose.position.y, 2.0);
}

TEST(DirectedInflationEscapeDebugMarkers, DeletesStaleEpisodeMarkers) {
  const visualization_msgs::msg::MarkerArray markers =
      buildDirectedInflationEscapeDebugMarkers(testHeader(),
                                               DirectedInflationEscapeResult{}, 5.0);

  ASSERT_EQ(markers.markers.size(), 3U);
  for (const visualization_msgs::msg::Marker& marker : markers.markers) {
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETE);
  }
}

} // namespace drone_city_nav
