#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <string>

namespace drone_city_nav {

geometry_msgs::msg::Point markerPoint(const Point3& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

geometry_msgs::msg::Point markerPoint(const Point2& point, const double z_m) {
  return markerPoint(Point3{point.x, point.y, z_m});
}

double gazeboAlignedRvizZ(const double map_z_m, const bool axes_swapped) noexcept {
  // The legacy `gazebo_map -> map` rotation that exchanges the horizontal axes
  // also maps map Z to negative gazebo_map Z, so map-frame overlays negate Z to
  // stay above the ground. With the identity transform the map Z is final.
  return axes_swapped ? -map_z_m : map_z_m;
}

geometry_msgs::msg::Point gazeboAlignedRvizMarkerPoint(const Point3& point,
                                                       const bool axes_swapped) {
  return markerPoint(
      Point3{point.x, point.y, gazeboAlignedRvizZ(point.z, axes_swapped)});
}

geometry_msgs::msg::Point gazeboAlignedRvizMarkerPoint(const Point2& point,
                                                       const double z_m,
                                                       const bool axes_swapped) {
  return gazeboAlignedRvizMarkerPoint(Point3{point.x, point.y, z_m}, axes_swapped);
}

Point3 gazeboAlignedRvizFramePosition(const Point3& map_position,
                                      const bool axes_swapped) noexcept {
  return axes_swapped ? Point3{map_position.y, map_position.x, map_position.z}
                      : map_position;
}

std_msgs::msg::ColorRGBA rgba(const float red, const float green, const float blue,
                              const float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

visualization_msgs::msg::Marker makeMarker(const std_msgs::msg::Header& header,
                                           const std::string_view marker_namespace,
                                           const int marker_id, const int marker_type) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = std::string{marker_namespace};
  marker.id = marker_id;
  marker.type = marker_type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

} // namespace drone_city_nav
