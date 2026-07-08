#include "drone_city_nav/offboard_debug_markers.hpp"

#include "drone_city_nav/trajectory_debug_markers.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <iterator>
#include <utility>

namespace drone_city_nav {

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildDroneDebugMarkers(const std_msgs::msg::Header& header,
                       const DroneDebugMarkerState& state) {
  visualization_msgs::msg::MarkerArray markers;
  auto position =
      makeMarker(header, "drone_position", 0, visualization_msgs::msg::Marker::SPHERE);
  position.scale.x = 2.5;
  position.scale.y = 2.5;
  position.scale.z = 2.5;
  position.color.r = 0.68F;
  position.color.g = 0.20F;
  position.color.b = 1.0F;
  position.color.a = 1.0F;

  auto heading =
      makeMarker(header, "drone_heading", 0, visualization_msgs::msg::Marker::ARROW);
  heading.scale.x = 0.25;
  heading.scale.y = 0.75;
  heading.scale.z = 1.0;
  heading.color = position.color;

  if (!state.pose_fresh || !state.altitude_valid || !std::isfinite(state.altitude_m)) {
    position.action = visualization_msgs::msg::Marker::DELETE;
    heading.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(position);
    markers.markers.push_back(heading);
    return markers;
  }

  position.pose.position =
      gazeboAlignedRvizMarkerPoint(state.position, state.altitude_m);
  const Point2 heading_end{state.position.x + std::cos(state.heading_rad) * 4.0,
                           state.position.y + std::sin(state.heading_rad) * 4.0};
  heading.points.push_back(
      gazeboAlignedRvizMarkerPoint(state.position, state.altitude_m));
  heading.points.push_back(gazeboAlignedRvizMarkerPoint(heading_end, state.altitude_m));
  markers.markers.push_back(position);
  markers.markers.push_back(heading);
  return markers;
}

[[nodiscard]] visualization_msgs::msg::MarkerArray buildOffboardDebugMarkers(
    const std_msgs::msg::Header& header, const DroneDebugMarkerState& state,
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile) {
  visualization_msgs::msg::MarkerArray markers = buildDroneDebugMarkers(header, state);
  visualization_msgs::msg::MarkerArray trajectory_markers =
      buildTrajectoryDebugMarkers(header, trajectory_samples, speed_profile);
  markers.markers.insert(markers.markers.end(),
                         std::make_move_iterator(trajectory_markers.markers.begin()),
                         std::make_move_iterator(trajectory_markers.markers.end()));
  return markers;
}

} // namespace drone_city_nav
