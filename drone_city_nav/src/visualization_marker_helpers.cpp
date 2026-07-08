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

double gazeboAlignedRvizZ(const double map_z_m) noexcept {
  // This sign flip is intentional and applies only to RViz/debug output. The
  // RViz config uses the legacy `gazebo_map` fixed frame and the
  // `gazebo_aligned_map_tf` transform, which deliberately swaps X/Y for visual
  // alignment with Gazebo. A proper TF rotation that swaps the horizontal axes
  // also maps map Z to negative gazebo_map Z, so positive map altitudes would be
  // displayed below the ground. We compensate visual Z here instead of changing
  // planner/control data. Do not remove this as a "negative altitude bug" unless
  // the Gazebo/RViz frame convention is migrated end-to-end.
  return -map_z_m;
}

geometry_msgs::msg::Point gazeboAlignedRvizMarkerPoint(const Point3& point) {
  return markerPoint(Point3{point.x, point.y, gazeboAlignedRvizZ(point.z)});
}

geometry_msgs::msg::Point gazeboAlignedRvizMarkerPoint(const Point2& point,
                                                       const double z_m) {
  return gazeboAlignedRvizMarkerPoint(Point3{point.x, point.y, z_m});
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
