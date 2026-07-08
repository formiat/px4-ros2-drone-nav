#include "drone_city_nav/known_passage_debug_markers.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] std_msgs::msg::Header testHeader() {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 4;
  return header;
}

[[nodiscard]] KnownPassageMap testMap() {
  PassageOpening opening;
  opening.id = "main";
  opening.structure_id = "arch";
  opening.center = Point3{10.0, 20.0, 8.0};
  opening.normal_xy = Point2{1.0, 0.0};
  opening.width_m = 4.0;
  opening.height_m = 4.0;
  opening.depth_m = 2.0;
  opening.min_z_m = 6.0;
  opening.max_z_m = 10.0;
  opening.approach_distance_m = 5.0;
  opening.exit_distance_m = 7.0;

  PassageStructure structure;
  structure.id = "arch";
  structure.center = Point2{10.0, 20.0};
  structure.size_x_m = 12.0;
  structure.size_y_m = 8.0;
  structure.z_min_m = 0.0;
  structure.z_max_m = 24.0;
  structure.openings.push_back(opening);

  KnownPassageMap map;
  map.frame_id = "map";
  map.structures.push_back(structure);
  return map;
}

} // namespace

TEST(KnownPassageDebugMarkers, BuildsStructureOpeningCenterAndDirectionMarkers) {
  const visualization_msgs::msg::MarkerArray markers =
      buildKnownPassageDebugMarkers(testHeader(), testMap());

  ASSERT_EQ(markers.markers.size(), 8U);
  EXPECT_EQ(markers.markers[0].ns, "known_passage_structure");
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_EQ(markers.markers[0].header.frame_id, "map");
  EXPECT_DOUBLE_EQ(markers.markers[0].pose.position.x, 10.0);
  EXPECT_DOUBLE_EQ(markers.markers[0].pose.position.y, 17.0);
  EXPECT_DOUBLE_EQ(markers.markers[0].pose.position.z, 12.0);
  EXPECT_DOUBLE_EQ(markers.markers[0].scale.x, 2.0);
  EXPECT_DOUBLE_EQ(markers.markers[0].scale.y, 2.0);
  EXPECT_DOUBLE_EQ(markers.markers[0].scale.z, 24.0);
  EXPECT_GT(markers.markers[0].color.a, 0.5F);

  EXPECT_EQ(markers.markers[1].ns, "known_passage_structure");
  EXPECT_DOUBLE_EQ(markers.markers[1].pose.position.y, 23.0);
  EXPECT_DOUBLE_EQ(markers.markers[2].scale.z, 6.0);
  EXPECT_DOUBLE_EQ(markers.markers[3].scale.z, 14.0);

  const visualization_msgs::msg::Marker& frame = markers.markers[4];
  EXPECT_EQ(frame.ns, "known_passage_opening_frame");
  EXPECT_EQ(frame.type, visualization_msgs::msg::Marker::LINE_LIST);
  ASSERT_EQ(frame.points.size(), 24U);
  EXPECT_DOUBLE_EQ(frame.points[0].x, 9.0);
  EXPECT_DOUBLE_EQ(frame.points[0].y, 18.0);
  EXPECT_DOUBLE_EQ(frame.points[0].z, 6.0);
  EXPECT_DOUBLE_EQ(frame.points[1].x, 9.0);
  EXPECT_DOUBLE_EQ(frame.points[1].y, 22.0);
  EXPECT_DOUBLE_EQ(frame.points[1].z, 6.0);
  EXPECT_DOUBLE_EQ(frame.points[8].z, 10.0);

  const visualization_msgs::msg::Marker& center = markers.markers[5];
  EXPECT_EQ(center.ns, "known_passage_gate_center");
  EXPECT_EQ(center.type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_DOUBLE_EQ(center.pose.position.x, 10.0);
  EXPECT_DOUBLE_EQ(center.pose.position.y, 20.0);
  EXPECT_DOUBLE_EQ(center.pose.position.z, 8.0);

  const visualization_msgs::msg::Marker& approach = markers.markers[6];
  EXPECT_EQ(approach.ns, "known_passage_approach");
  EXPECT_EQ(approach.type, visualization_msgs::msg::Marker::ARROW);
  ASSERT_EQ(approach.points.size(), 2U);
  EXPECT_DOUBLE_EQ(approach.points[0].x, 4.0);
  EXPECT_DOUBLE_EQ(approach.points[1].x, 9.0);
  EXPECT_DOUBLE_EQ(approach.points[0].y, 20.0);
  EXPECT_DOUBLE_EQ(approach.points[1].z, 8.0);

  const visualization_msgs::msg::Marker& exit = markers.markers[7];
  EXPECT_EQ(exit.ns, "known_passage_exit");
  ASSERT_EQ(exit.points.size(), 2U);
  EXPECT_DOUBLE_EQ(exit.points[0].x, 11.0);
  EXPECT_DOUBLE_EQ(exit.points[1].x, 18.0);
  EXPECT_DOUBLE_EQ(exit.points[0].y, 20.0);
  EXPECT_DOUBLE_EQ(exit.points[1].z, 8.0);
}

TEST(KnownPassageDebugMarkers, EmptyMapDeletesPreviousMarkers) {
  KnownPassageMap map;
  map.frame_id = "map";

  const visualization_msgs::msg::MarkerArray markers =
      buildKnownPassageDebugMarkers(testHeader(), map);

  ASSERT_EQ(markers.markers.size(), 1U);
  EXPECT_EQ(markers.markers.front().ns, "known_passage");
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
}

TEST(KnownPassageDebugMarkers, ExplicitDeleteMarkerUsesDeleteAll) {
  const visualization_msgs::msg::MarkerArray markers =
      buildKnownPassageDeleteMarkers(testHeader());

  ASSERT_EQ(markers.markers.size(), 1U);
  EXPECT_EQ(markers.markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(markers.markers.front().header.frame_id, "map");
}

} // namespace drone_city_nav
