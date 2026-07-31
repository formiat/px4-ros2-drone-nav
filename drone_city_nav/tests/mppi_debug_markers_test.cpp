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

TEST(MppiDebugMarkers, SeparatesCandidateAndPublishedExecutionHorizons) {
  const std::vector<mppi::State> candidate{
      mppi::State{.x = 10.0F, .y = 20.0F, .z = 18.0F},
      mppi::State{.x = 30.0F, .y = 40.0F, .z = 18.0F},
  };
  const std::vector<mppi::State> execution{
      mppi::State{.x = 10.0F, .y = 20.0F, .z = 18.0F},
      mppi::State{.x = 12.0F, .y = 20.0F, .z = 18.0F},
  };
  MppiDebugMarkerInput input = markerInput();
  input.horizon = candidate;
  input.execution_horizon = execution;
  input.selected_tier = mppi::RiskTier::kCollision;

  const auto markers = buildMppiDebugMarkers(input);

  const auto& candidate_marker = findMarker(markers, "mppi_candidate", 0);
  const auto& execution_marker = findMarker(markers, "mppi_execution", 0);
  ASSERT_EQ(candidate_marker.points.size(), candidate.size());
  ASSERT_EQ(execution_marker.points.size(), execution.size());
  EXPECT_FLOAT_EQ(candidate_marker.color.r, 1.0F);
  EXPECT_FLOAT_EQ(candidate_marker.color.g, 0.0F);
  EXPECT_FLOAT_EQ(execution_marker.color.b, 1.0F);
  EXPECT_DOUBLE_EQ(execution_marker.points.back().x, 12.0);
}

TEST(MppiDebugMarkers, HighlightsSelectedPassageAndTraversalDirection) {
  MppiDebugMarkerInput input = markerInput();
  input.passage = mppi::PassageConstraint{
      .center_x_m = 25.0F,
      .center_y_m = 30.0F,
      .normal_x = 1.0F,
      .normal_y = 0.0F,
      .half_depth_m = 2.0F,
      .min_z_m = 4.0F,
      .max_z_m = 10.0F,
      .preferred_z_m = 7.0F,
      .normal_flight_z_m = 18.0F,
      .approach_station_m = 5.0F,
      .entry_station_m = 10.0F,
      .exit_station_m = 14.0F,
      .departure_station_m = 20.0F,
      .speed_limit_mps = 10.0F,
      .phase = mppi::PassagePhase::kUpcoming,
  };

  const auto markers = buildMppiDebugMarkers(input);

  const auto& center = findMarker(markers, "selected_passage", 0);
  EXPECT_EQ(center.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(center.pose.position.x, 25.0);
  EXPECT_DOUBLE_EQ(center.pose.position.y, 30.0);
  EXPECT_DOUBLE_EQ(center.pose.position.z, -7.0);
  const auto& direction = findMarker(markers, "selected_passage", 1);
  EXPECT_EQ(direction.action, visualization_msgs::msg::Marker::ADD);
  ASSERT_EQ(direction.points.size(), 2U);
  EXPECT_DOUBLE_EQ(direction.points.front().x, 23.0);
  EXPECT_DOUBLE_EQ(direction.points.back().x, 27.0);
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
