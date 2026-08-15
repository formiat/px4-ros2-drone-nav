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
  const std::vector<mppi::RouteSample3D> guide{
      mppi::RouteSample3D{.x_m = 10.0F, .y_m = 20.0F, .z_m = 18.0F},
      mppi::RouteSample3D{.x_m = 14.0F, .y_m = 24.0F, .z_m = 19.0F},
      mppi::RouteSample3D{.x_m = 18.0F, .y_m = 28.0F, .z_m = 20.0F}};
  MppiDebugMarkerInput input = markerInput();
  input.global_route = guide;

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

TEST(MppiDebugMarkers, ShowsObservedPredictedAndResolvedTrackingTargets) {
  MppiDebugMarkerInput input = markerInput();
  input.tracking_objective_active = true;
  input.observed_tracking_target = Point3{20.0, 30.0, 18.0};
  input.predicted_tracking_target = Point3{50.0, 30.0, 18.0};
  input.resolved_tracking_target = Point3{40.0, 30.0, 18.0};

  const auto markers = buildMppiDebugMarkers(input);

  const auto& observed = findMarker(markers, "tracking_target_observed", 0);
  const auto& predicted = findMarker(markers, "tracking_target_predicted", 0);
  const auto& resolved = findMarker(markers, "tracking_target_resolved", 0);
  EXPECT_EQ(observed.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(predicted.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(resolved.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(observed.pose.position.x, 20.0);
  EXPECT_DOUBLE_EQ(predicted.pose.position.x, 50.0);
  EXPECT_DOUBLE_EQ(resolved.pose.position.x, 40.0);
}

TEST(MppiDebugMarkers, DeletesTrackingTargetsForPositionObjective) {
  const auto markers = buildMppiDebugMarkers(markerInput());

  EXPECT_EQ(findMarker(markers, "tracking_target_observed", 0).action,
            visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(findMarker(markers, "tracking_target_predicted", 0).action,
            visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(findMarker(markers, "tracking_target_resolved", 0).action,
            visualization_msgs::msg::Marker::DELETE);
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

TEST(MppiDebugMarkers, DistinguishesCandidateAndSelectedPassageEdges) {
  const std::vector<PassageTraversalEdge> passages{
      PassageTraversalEdge{
          .id = "selected",
          .region_id = "selected_region",
          .entry_portal_id = "selected_entry",
          .exit_portal_id = "selected_exit",
          .centerline = sampleRoute3D(
              std::vector<Point3>{{1.0, 2.0, 5.0}, {3.0, 2.0, 5.0}}, 0.5, 10.0),
          .min_z_m = 1.5,
          .max_z_m = 8.5,
          .width_m = 24.0,
          .height_m = 7.0,
          .minimum_clearance_m = 3.5,
          .speed_limit_mps = 10.0},
      PassageTraversalEdge{
          .id = "candidate",
          .region_id = "candidate_region",
          .entry_portal_id = "candidate_entry",
          .exit_portal_id = "candidate_exit",
          .centerline = sampleRoute3D(
              std::vector<Point3>{{4.0, 2.0, 5.0}, {6.0, 2.0, 5.0}}, 0.5, 10.0),
          .min_z_m = 1.5,
          .max_z_m = 8.5,
          .width_m = 24.0,
          .height_m = 7.0,
          .minimum_clearance_m = 3.5,
          .speed_limit_mps = 10.0}};
  const std::vector<PassageTraversalId> selected{"selected"};
  MppiDebugMarkerInput input = markerInput();
  input.passage_traversals = passages;
  input.selected_passage_traversal_ids = selected;

  const auto markers = buildMppiDebugMarkers(input);

  const auto& candidate = findMarker(markers, "passage_candidate_traversals", 0);
  const auto& selected_marker = findMarker(markers, "selected_passage_traversals", 0);
  const auto& unselected_marker = findMarker(markers, "selected_passage_traversals", 1);
  EXPECT_EQ(candidate.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(selected_marker.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_GT(selected_marker.scale.x, candidate.scale.x);
  EXPECT_EQ(unselected_marker.action, visualization_msgs::msg::Marker::DELETE);
}

} // namespace
} // namespace drone_city_nav
