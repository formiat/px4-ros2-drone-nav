#include "drone_city_nav/lidar_radar_markers.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <numbers>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] geometry_msgs::msg::Point markerPoint(const Point2 point,
                                                    const double z) {
  geometry_msgs::msg::Point marker_point;
  marker_point.x = point.x;
  marker_point.y = point.y;
  marker_point.z = z;
  return marker_point;
}

[[nodiscard]] visualization_msgs::msg::Marker
baseMarker(const LidarRadarMarkerConfig& config, const std::string& ns, const int id,
           const int type) {
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = config.stamp;
  marker.header.frame_id = config.frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime.sec = 2;
  marker.lifetime.nanosec = 0U;
  return marker;
}

void setColor(visualization_msgs::msg::Marker& marker, const float red,
              const float green, const float blue, const float alpha) {
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;
}

void addRangeRing(visualization_msgs::msg::MarkerArray& markers,
                  const LidarRadarMarkerConfig& config, const double radius_m,
                  const int id, const double z) {
  constexpr int kSegments = 144;
  auto ring = baseMarker(config, "lidar_radar_range", id,
                         visualization_msgs::msg::Marker::LINE_STRIP);
  ring.scale.x = 0.08;
  setColor(ring, 0.25F, 0.70F, 0.95F, 0.45F);
  ring.points.reserve(static_cast<std::size_t>(kSegments) + 1U);
  for (int i = 0; i <= kSegments; ++i) {
    const double angle = 2.0 * std::numbers::pi * static_cast<double>(i) /
                         static_cast<double>(kSegments);
    ring.points.push_back(
        markerPoint(Point2{config.drone_position.x + radius_m * std::cos(angle),
                           config.drone_position.y + radius_m * std::sin(angle)},
                    z));
  }
  markers.markers.push_back(std::move(ring));
}

void addScanRayMarkers(visualization_msgs::msg::MarkerArray& markers,
                       const LidarRadarMarkerConfig& config,
                       const std::span<const LidarBeamProjection> projections,
                       const double z) {
  auto free_rays = baseMarker(config, "lidar_radar_free_rays", 0,
                              visualization_msgs::msg::Marker::LINE_LIST);
  free_rays.scale.x = 0.04;
  setColor(free_rays, 0.18F, 0.85F, 0.95F, 0.28F);
  auto hit_rays = baseMarker(config, "lidar_radar_hit_rays", 0,
                             visualization_msgs::msg::Marker::LINE_LIST);
  hit_rays.scale.x = 0.07;
  setColor(hit_rays, 1.0F, 0.22F, 0.18F, 0.70F);

  const auto origin = markerPoint(config.drone_position, z);
  for (const LidarBeamProjection& projection : projections) {
    if (projection.status != LidarBeamProjectionStatus::kAccepted) {
      continue;
    }

    const auto end = markerPoint(projection.endpoint, z);
    auto& ray_marker = projection.hit ? hit_rays : free_rays;
    ray_marker.points.push_back(origin);
    ray_marker.points.push_back(end);
  }

  markers.markers.push_back(std::move(free_rays));
  markers.markers.push_back(std::move(hit_rays));
}

void addDroneMarker(visualization_msgs::msg::MarkerArray& markers,
                    const LidarRadarMarkerConfig& config, const double z) {
  auto drone = baseMarker(config, "lidar_radar_drone", 0,
                          visualization_msgs::msg::Marker::ARROW);
  drone.scale.x = 0.9;
  drone.scale.y = 0.35;
  drone.scale.z = 0.35;
  setColor(drone, 0.30F, 0.55F, 1.0F, 1.0F);
  drone.points.push_back(markerPoint(config.drone_position, z));
  drone.points.push_back(
      markerPoint(Point2{config.drone_position.x + 3.0 * config.heading_direction.x,
                         config.drone_position.y + 3.0 * config.heading_direction.y},
                  z));
  markers.markers.push_back(std::move(drone));
}

} // namespace

visualization_msgs::msg::MarkerArray
buildLidarRadarMarkers(const LidarRadarMarkerConfig& config,
                       const std::span<const LidarBeamProjection> projections) {
  visualization_msgs::msg::MarkerArray markers;
  const double z = std::isfinite(config.marker_z_m) ? config.marker_z_m : 0.0;
  addRangeRing(markers, config, 10.0, 0, z);
  addRangeRing(markers, config, 20.0, 1, z);
  addRangeRing(markers, config, 30.0, 2, z);
  addRangeRing(markers, config, config.scan_range_max_m, 3, z);
  addScanRayMarkers(markers, config, projections, z);
  addDroneMarker(markers, config, z);
  return markers;
}

} // namespace drone_city_nav
