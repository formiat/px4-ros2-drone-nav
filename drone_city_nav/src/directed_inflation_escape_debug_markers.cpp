#include "drone_city_nav/directed_inflation_escape_debug_markers.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr int kTunnelMarkerId = 0;
constexpr int kCenterlineMarkerId = 1;
constexpr int kTargetMarkerId = 2;
constexpr double kMarkerZ = 0.5;

[[nodiscard]] visualization_msgs::msg::Marker
makeDeleteMarker(const std_msgs::msg::Header& header, const int marker_id) {
  visualization_msgs::msg::Marker marker =
      makeMarker(header, "directed_inflation_escape", marker_id,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  marker.action = visualization_msgs::msg::Marker::DELETE;
  return marker;
}

} // namespace

visualization_msgs::msg::MarkerArray
buildDirectedInflationEscapeDebugMarkers(const std_msgs::msg::Header& header,
                                         const DirectedInflationEscapeResult& escape,
                                         const double tunnel_width_m) {
  visualization_msgs::msg::MarkerArray markers;
  if (!escape.applied || escape.centerline.size() < 2U) {
    markers.markers.push_back(makeDeleteMarker(header, kTunnelMarkerId));
    markers.markers.push_back(makeDeleteMarker(header, kCenterlineMarkerId));
    markers.markers.push_back(makeDeleteMarker(header, kTargetMarkerId));
    return markers;
  }

  visualization_msgs::msg::Marker tunnel =
      makeMarker(header, "directed_inflation_escape", kTunnelMarkerId,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  tunnel.scale.x = std::max(0.05, tunnel_width_m);
  tunnel.color = rgba(0.10F, 0.80F, 1.00F, 0.20F);

  visualization_msgs::msg::Marker centerline =
      makeMarker(header, "directed_inflation_escape", kCenterlineMarkerId,
                 visualization_msgs::msg::Marker::LINE_STRIP);
  centerline.scale.x = 0.25;
  centerline.color = rgba(0.00F, 0.95F, 1.00F, 1.00F);

  for (const Point2 point : escape.centerline) {
    const geometry_msgs::msg::Point marker_point = markerPoint(point, kMarkerZ);
    tunnel.points.push_back(marker_point);
    centerline.points.push_back(marker_point);
  }

  visualization_msgs::msg::Marker target =
      makeMarker(header, "directed_inflation_escape", kTargetMarkerId,
                 visualization_msgs::msg::Marker::SPHERE);
  target.pose.position = markerPoint(escape.target, kMarkerZ);
  target.scale.x = 1.0;
  target.scale.y = 1.0;
  target.scale.z = 1.0;
  target.color = rgba(0.10F, 1.00F, 0.30F, 1.00F);

  markers.markers.push_back(std::move(tunnel));
  markers.markers.push_back(std::move(centerline));
  markers.markers.push_back(std::move(target));
  return markers;
}

} // namespace drone_city_nav
