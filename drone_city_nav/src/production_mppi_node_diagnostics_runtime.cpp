#include "drone_city_nav/mppi_debug_markers.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <memory>
#include <span>
#include <string>

#include "production_mppi_diagnostics_snapshot.hpp"
#include "production_mppi_node.hpp"
#include "tracking_objective_diagnostics.hpp"

namespace drone_city_nav {

void ProductionMppiNode::publishRviz(
    const ProductionMppiDiagnosticsSnapshot& snapshot) {
  if (!snapshot.rviz.has_value()) {
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective>& objective =
      snapshot.objective;
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const ProductionMppiRvizSnapshot& rviz = *snapshot.rviz;
  const auto stamp = now();
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.world.frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(rviz.candidate_horizon.size());
  for (const mppi::State& state : rviz.candidate_horizon) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = gazeboAlignedRvizZ(state.z);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);

  const std::span<const mppi::State> previous_horizon{rviz.previous_horizon};
  const std::span<const mppi::State> execution_horizon{rviz.execution_horizon};
  const std::span<const mppi::RouteSample3D> persistent_route =
      rviz.route ? std::span<const mppi::RouteSample3D>{*rviz.route}
                 : std::span<const mppi::RouteSample3D>{};
  const std::span<const PassageTraversalEdge> passage_traversals =
      rviz.passage_traversals
          ? std::span<const PassageTraversalEdge>{*rviz.passage_traversals}
          : std::span<const PassageTraversalEdge>{};
  const std::span<const PassageTraversalId> selected_passage_traversal_ids =
      rviz.selected_passage_traversal_ids
          ? std::span<const PassageTraversalId>{*rviz.selected_passage_traversal_ids}
          : std::span<const PassageTraversalId>{};
  MppiDebugMarkerInput marker_input{
      .header = path.header,
      .horizon = rviz.candidate_horizon,
      .previous_horizon = previous_horizon,
      .execution_horizon = execution_horizon,
      .persistent_route = persistent_route,
      .passage_traversals = passage_traversals,
      .selected_passage_traversal_ids = selected_passage_traversal_ids,
      .initial_state = snapshot.input.initial_state,
      .target = snapshot.input.target,
      .mission_start = config_.planning.mission_start,
      .mission_goal = mission_goal,
      .selected_tier = snapshot.result.selected_tier,
  };
  detail::populateTrackingObjectiveMarkers(objective.get(), marker_input);
  const visualization_msgs::msg::MarkerArray markers =
      buildMppiDebugMarkers(marker_input);
  markers_pub_->publish(markers);
}

} // namespace drone_city_nav
