#include "drone_city_nav/mppi_debug_markers.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string_view>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] const visualization_msgs::msg::Marker&
findMarker(const visualization_msgs::msg::MarkerArray& markers,
           const std::string_view marker_namespace, const int marker_id) {
  const auto found = std::ranges::find_if(
      markers.markers, [marker_namespace, marker_id](const auto& marker) {
        return marker.ns == marker_namespace && marker.id == marker_id;
      });
  EXPECT_NE(found, markers.markers.end());
  return *found;
}

[[nodiscard]] MppiDebugMarkerInput markerInput() {
  MppiDebugMarkerInput input;
  input.header.frame_id = "map";
  input.initial_state = mppi::State{10.0F, 20.0F, 18.0F};
  input.target = mppi::State{30.0F, 40.0F, 18.0F};
  input.mission_start = Point3{5.0, 6.0, 0.0};
  input.mission_goal = Point3{100.0, 120.0, 18.0};
  input.selected_tier = mppi::RiskTier::kPreferred;
  return input;
}

TEST(MppiDebugMarkers, PublishesCompleteGlobalLatticeGuide) {
  const std::vector<Point2> guide{Point2{10.0, 20.0}, Point2{14.0, 24.0},
                                  Point2{18.0, 28.0}};
  MppiDebugMarkerInput input = markerInput();
  input.global_guide = guide;

  const auto markers = buildMppiDebugMarkers(input);

  const auto& marker = findMarker(markers, "global_lattice_guide", 0);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  ASSERT_EQ(marker.points.size(), guide.size());
  EXPECT_DOUBLE_EQ(marker.points.front().x, 10.0);
  EXPECT_DOUBLE_EQ(marker.points.back().y, 28.0);
  EXPECT_DOUBLE_EQ(marker.points.front().z, -18.0);
}

TEST(MppiDebugMarkers, DeletesGlobalGuideWhenSnapshotHasNoGuide) {
  const auto markers = buildMppiDebugMarkers(markerInput());

  const auto& marker = findMarker(markers, "global_lattice_guide", 0);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETE);
}

TEST(MppiDebugMarkers, UsesSeparateCurrentTargetNamespace) {
  const auto markers = buildMppiDebugMarkers(markerInput());

  const auto& marker = findMarker(markers, "mppi_target", 0);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(marker.pose.position.x, 30.0);
  EXPECT_DOUBLE_EQ(marker.pose.position.y, 40.0);
  EXPECT_DOUBLE_EQ(marker.pose.position.z, -18.0);
}

TEST(MppiDebugMarkers, HighlightsSelectedPassageAndTraversalDirection) {
  MppiDebugMarkerInput input = markerInput();
  input.passage = mppi::PassageConstraint{
      25.0F, 30.0F, 1.0F, 0.0F, 2.0F,  4.0F,
      10.0F, 7.0F,  5.0F, 6.0F, 10.0F, mppi::PassagePhase::kApproach};

  const auto markers = buildMppiDebugMarkers(input);

  const auto& center = findMarker(markers, "selected_passage", 0);
  EXPECT_EQ(center.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(center.pose.position.x, 25.0);
  EXPECT_DOUBLE_EQ(center.pose.position.y, 30.0);
  EXPECT_DOUBLE_EQ(center.pose.position.z, -7.0);
  const auto& direction = findMarker(markers, "selected_passage", 1);
  EXPECT_EQ(direction.action, visualization_msgs::msg::Marker::ADD);
  ASSERT_EQ(direction.points.size(), 2U);
  EXPECT_DOUBLE_EQ(direction.points.front().x, 18.0);
  EXPECT_DOUBLE_EQ(direction.points.back().x, 33.0);
}

TEST(MppiDebugMarkers, DeletesSelectedPassageWhenNoneIsActive) {
  const auto markers = buildMppiDebugMarkers(markerInput());

  EXPECT_EQ(findMarker(markers, "selected_passage", 0).action,
            visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(findMarker(markers, "selected_passage", 1).action,
            visualization_msgs::msg::Marker::DELETE);
}

} // namespace
} // namespace drone_city_nav
