#include "drone_city_nav/offboard_debug_markers.hpp"

#include "drone_city_nav/trajectory_debug_markers.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <iterator>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] geometry_msgs::msg::Point markerPoint(const Point2 point,
                                                    const double z_m) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = z_m;
  return msg;
}

[[nodiscard]] visualization_msgs::msg::Marker
makeDebugMarker(const std_msgs::msg::Header& header, const char* marker_namespace,
                const int marker_id, const int marker_type) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = marker_type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

} // namespace

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildDroneDebugMarkers(const std_msgs::msg::Header& header,
                       const DroneDebugMarkerState& state, const double ground_z_m) {
  visualization_msgs::msg::MarkerArray markers;
  auto position = makeDebugMarker(header, "drone_position", 0,
                                  visualization_msgs::msg::Marker::SPHERE);
  position.scale.x = 2.5;
  position.scale.y = 2.5;
  position.scale.z = 0.25;
  position.color.r = 0.68F;
  position.color.g = 0.20F;
  position.color.b = 1.0F;
  position.color.a = 1.0F;

  auto heading = makeDebugMarker(header, "drone_heading", 0,
                                 visualization_msgs::msg::Marker::ARROW);
  heading.scale.x = 0.25;
  heading.scale.y = 0.75;
  heading.scale.z = 1.0;
  heading.color = position.color;

  if (!state.pose_fresh) {
    position.action = visualization_msgs::msg::Marker::DELETE;
    heading.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(position);
    markers.markers.push_back(heading);
    return markers;
  }

  position.pose.position = markerPoint(state.position, ground_z_m);
  const Point2 heading_end{state.position.x + std::cos(state.heading_rad) * 4.0,
                           state.position.y + std::sin(state.heading_rad) * 4.0};
  heading.points.push_back(markerPoint(state.position, ground_z_m + 0.06));
  heading.points.push_back(markerPoint(heading_end, ground_z_m + 0.06));
  markers.markers.push_back(position);
  markers.markers.push_back(heading);
  return markers;
}

[[nodiscard]] visualization_msgs::msg::MarkerArray buildOffboardDebugMarkers(
    const std_msgs::msg::Header& header, const DroneDebugMarkerState& state,
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile, const double ground_z_m) {
  visualization_msgs::msg::MarkerArray markers =
      buildDroneDebugMarkers(header, state, ground_z_m);
  visualization_msgs::msg::MarkerArray trajectory_markers = buildTrajectoryDebugMarkers(
      header, trajectory_samples, speed_profile, ground_z_m);
  markers.markers.insert(markers.markers.end(),
                         std::make_move_iterator(trajectory_markers.markers.begin()),
                         std::make_move_iterator(trajectory_markers.markers.end()));
  return markers;
}

} // namespace drone_city_nav
