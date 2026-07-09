#include "drone_city_nav/known_passage_debug_markers.hpp"

#include "drone_city_nav/known_passage_solid_volumes.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr int kDeleteMarkerId = 0;
constexpr float kStructureAlpha = 0.62F;
constexpr float kFrameAlpha = 0.95F;
constexpr double kMinimumStructurePartM = 0.001;

[[nodiscard]] Point2 add(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 scale(const Point2 point, const double factor) noexcept {
  return Point2{point.x * factor, point.y * factor};
}

[[nodiscard]] Point2 openingCenter2D(const PassageOpening& opening) noexcept {
  return Point2{opening.center.x, opening.center.y};
}

[[nodiscard]] Point2 openingLateral(const PassageOpening& opening) noexcept {
  return Point2{-opening.normal_xy.y, opening.normal_xy.x};
}

[[nodiscard]] double markerYawForNormal(const Point2 normal) noexcept {
  return std::atan2(normal.y, normal.x);
}

void setYaw(visualization_msgs::msg::Marker& marker, const double yaw_rad) {
  marker.pose.orientation.z = std::sin(yaw_rad / 2.0);
  marker.pose.orientation.w = std::cos(yaw_rad / 2.0);
}

[[nodiscard]] Point3 openingCorner(const PassageOpening& opening,
                                   const double normal_sign, const double lateral_sign,
                                   const double z_m) {
  const Point2 center = openingCenter2D(opening);
  const Point2 normal_offset =
      scale(opening.normal_xy, normal_sign * opening.depth_m / 2.0);
  const Point2 lateral_offset =
      scale(openingLateral(opening), lateral_sign * opening.width_m / 2.0);
  const Point2 xy = add(add(center, normal_offset), lateral_offset);
  return Point3{xy.x, xy.y, z_m};
}

void appendEdge(visualization_msgs::msg::Marker& marker, const Point3& from,
                const Point3& to) {
  marker.points.push_back(gazeboAlignedRvizMarkerPoint(from));
  marker.points.push_back(gazeboAlignedRvizMarkerPoint(to));
}

void appendOpeningBox(visualization_msgs::msg::Marker& marker,
                      const PassageOpening& opening) {
  const std::array<Point3, 8U> corners{
      openingCorner(opening, -1.0, -1.0, opening.min_z_m),
      openingCorner(opening, -1.0, 1.0, opening.min_z_m),
      openingCorner(opening, 1.0, 1.0, opening.min_z_m),
      openingCorner(opening, 1.0, -1.0, opening.min_z_m),
      openingCorner(opening, -1.0, -1.0, opening.max_z_m),
      openingCorner(opening, -1.0, 1.0, opening.max_z_m),
      openingCorner(opening, 1.0, 1.0, opening.max_z_m),
      openingCorner(opening, 1.0, -1.0, opening.max_z_m),
  };
  appendEdge(marker, corners[0], corners[1]);
  appendEdge(marker, corners[1], corners[2]);
  appendEdge(marker, corners[2], corners[3]);
  appendEdge(marker, corners[3], corners[0]);
  appendEdge(marker, corners[4], corners[5]);
  appendEdge(marker, corners[5], corners[6]);
  appendEdge(marker, corners[6], corners[7]);
  appendEdge(marker, corners[7], corners[4]);
  appendEdge(marker, corners[0], corners[4]);
  appendEdge(marker, corners[1], corners[5]);
  appendEdge(marker, corners[2], corners[6]);
  appendEdge(marker, corners[3], corners[7]);
}

[[nodiscard]] visualization_msgs::msg::Marker
makeStructurePartMarker(const std_msgs::msg::Header& header,
                        const KnownPassageSolidVolume& volume, const int marker_id) {
  visualization_msgs::msg::Marker marker =
      makeMarker(header, "known_passage_structure", marker_id,
                 visualization_msgs::msg::Marker::CUBE);
  marker.pose.position.x = volume.center.x;
  marker.pose.position.y = volume.center.y;
  marker.pose.position.z = gazeboAlignedRvizZ((volume.min_z_m + volume.max_z_m) / 2.0);
  setYaw(marker, markerYawForNormal(volume.normal_xy));
  marker.scale.x = std::max(volume.depth_m, kMinimumStructurePartM);
  marker.scale.y = std::max(volume.width_m, kMinimumStructurePartM);
  marker.scale.z = std::max(volume.max_z_m - volume.min_z_m, kMinimumStructurePartM);
  marker.color = rgba(0.20F, 0.70F, 1.0F, kStructureAlpha);
  return marker;
}

void appendStructurePhysicalMarkers(visualization_msgs::msg::MarkerArray& markers,
                                    const std_msgs::msg::Header& header,
                                    const PassageStructure& structure, int& marker_id) {
  for (const KnownPassageSolidVolume& volume : knownPassageSolidVolumes(structure)) {
    markers.markers.push_back(makeStructurePartMarker(header, volume, marker_id++));
  }
}

[[nodiscard]] visualization_msgs::msg::Marker
makeOpeningFrameMarker(const std_msgs::msg::Header& header,
                       const PassageOpening& opening, const int marker_id) {
  visualization_msgs::msg::Marker marker =
      makeMarker(header, "known_passage_opening_frame", marker_id,
                 visualization_msgs::msg::Marker::LINE_LIST);
  marker.scale.x = 0.18;
  marker.color = rgba(0.10F, 1.0F, 0.45F, kFrameAlpha);
  appendOpeningBox(marker, opening);
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
makeOpeningCenterMarker(const std_msgs::msg::Header& header,
                        const PassageOpening& opening, const int marker_id) {
  visualization_msgs::msg::Marker marker =
      makeMarker(header, "known_passage_opening_center", marker_id,
                 visualization_msgs::msg::Marker::SPHERE);
  marker.pose.position = gazeboAlignedRvizMarkerPoint(opening.center);
  marker.scale.x = 0.9;
  marker.scale.y = 0.9;
  marker.scale.z = 0.9;
  marker.color = rgba(1.0F, 0.95F, 0.20F, 1.0F);
  return marker;
}

[[nodiscard]] visualization_msgs::msg::Marker
makeArrowMarker(const std_msgs::msg::Header& header, const PassageOpening& opening,
                const int marker_id, const bool exit_arrow) {
  const char* marker_namespace =
      exit_arrow ? "known_passage_exit" : "known_passage_approach";
  visualization_msgs::msg::Marker marker = makeMarker(
      header, marker_namespace, marker_id, visualization_msgs::msg::Marker::ARROW);
  marker.scale.x = 0.20;
  marker.scale.y = 0.55;
  marker.scale.z = 0.70;
  marker.color =
      exit_arrow ? rgba(1.0F, 0.48F, 0.20F, 0.95F) : rgba(0.45F, 0.95F, 1.0F, 0.95F);

  const Point2 center = openingCenter2D(opening);
  const Point2 entry = add(center, scale(opening.normal_xy, -opening.depth_m / 2.0));
  const Point2 exit = add(center, scale(opening.normal_xy, opening.depth_m / 2.0));
  const double z_m = opening.center.z;
  if (exit_arrow) {
    const Point2 arrow_end =
        add(exit, scale(opening.normal_xy, opening.exit_distance_m));
    marker.points.push_back(gazeboAlignedRvizMarkerPoint(exit, z_m));
    marker.points.push_back(gazeboAlignedRvizMarkerPoint(arrow_end, z_m));
  } else {
    const Point2 arrow_start =
        add(entry, scale(opening.normal_xy, -opening.approach_distance_m));
    marker.points.push_back(gazeboAlignedRvizMarkerPoint(arrow_start, z_m));
    marker.points.push_back(gazeboAlignedRvizMarkerPoint(entry, z_m));
  }
  return marker;
}

} // namespace

visualization_msgs::msg::MarkerArray
buildKnownPassageDeleteMarkers(const std_msgs::msg::Header& header) {
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker = makeMarker(
      header, "known_passage", kDeleteMarkerId, visualization_msgs::msg::Marker::CUBE);
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(marker);
  return markers;
}

visualization_msgs::msg::MarkerArray
buildKnownPassageDebugMarkers(const std_msgs::msg::Header& header,
                              const KnownPassageMap& map) {
  if (map.structures.empty()) {
    return buildKnownPassageDeleteMarkers(header);
  }

  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;
  for (const PassageStructure& structure : map.structures) {
    appendStructurePhysicalMarkers(markers, header, structure, marker_id);
    for (const PassageOpening& opening : structure.openings) {
      markers.markers.push_back(makeOpeningFrameMarker(header, opening, marker_id++));
      markers.markers.push_back(makeOpeningCenterMarker(header, opening, marker_id++));
      markers.markers.push_back(makeArrowMarker(header, opening, marker_id++, false));
      markers.markers.push_back(makeArrowMarker(header, opening, marker_id++, true));
    }
  }
  if (markers.markers.empty()) {
    return buildKnownPassageDeleteMarkers(header);
  }
  return markers;
}

} // namespace drone_city_nav
