#include "drone_city_nav/static_map_debug.hpp"

#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace drone_city_nav {
namespace {

constexpr int kStaticBuildingDeleteMarkerId = 0;
constexpr double kMinimumMarkerDimensionM = 0.001;

} // namespace

nav_msgs::msg::OccupancyGrid staticMapGridMessage(const OccupancyGrid2D& grid,
                                                  const StaticMapDebugConfig& config) {
  return rawOccupancyGridToRos(grid, RawOccupancyGridToRosConfig{config.header});
}

sensor_msgs::msg::PointCloud2 staticMapPointCloud(const OccupancyGrid2D& grid,
                                                  const StaticMapDebugConfig& config) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = config.header;
  cloud.height = 1U;
  cloud.width = 0U;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12U;
  cloud.row_step = 0U;
  cloud.fields.resize(3U);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0U;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1U;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4U;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1U;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8U;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1U;

  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (!grid.isOccupied(cell)) {
        continue;
      }

      const Point2 center = grid.cellCenter(cell);
      const float point_x = static_cast<float>(center.x);
      const float point_y = static_cast<float>(center.y);
      const std::size_t offset = cloud.data.size();
      cloud.data.resize(offset + static_cast<std::size_t>(cloud.point_step));
      std::memcpy(&cloud.data[offset], &point_x, sizeof(float));
      std::memcpy(&cloud.data[offset + 4U], &point_y, sizeof(float));
      const float point_z = static_cast<float>(gazeboAlignedRvizZ(config.point_z_m));
      std::memcpy(&cloud.data[offset + 8U], &point_z, sizeof(float));
      ++cloud.width;
    }
  }

  cloud.row_step = cloud.point_step * cloud.width;
  return cloud;
}

visualization_msgs::msg::MarkerArray
staticMapBuildingDeleteMarkers(const std_msgs::msg::Header& header) {
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker =
      makeMarker(header, "static_building", kStaticBuildingDeleteMarkerId,
                 visualization_msgs::msg::Marker::CUBE);
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(marker);
  return markers;
}

visualization_msgs::msg::MarkerArray
staticMapBuildingMarkers(const StaticCityMap& map, const StaticMapDebugConfig& config) {
  if (map.rectangles.empty()) {
    return staticMapBuildingDeleteMarkers(config.header);
  }

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(map.rectangles.size());
  int marker_id = 0;
  for (const StaticCityMapRect& rect : map.rectangles) {
    if (rect.size_x_m <= 0.0 || rect.size_y_m <= 0.0 || rect.height_m <= 0.0) {
      continue;
    }

    visualization_msgs::msg::Marker marker =
        makeMarker(config.header, "static_building", marker_id++,
                   visualization_msgs::msg::Marker::CUBE);
    marker.pose.position.x = rect.center.x;
    marker.pose.position.y = rect.center.y;
    marker.pose.position.z = gazeboAlignedRvizZ(rect.height_m / 2.0);
    marker.scale.x = std::max(rect.size_x_m, kMinimumMarkerDimensionM);
    marker.scale.y = std::max(rect.size_y_m, kMinimumMarkerDimensionM);
    marker.scale.z = std::max(rect.height_m, kMinimumMarkerDimensionM);
    marker.color = rgba(0.48F, 0.54F, 0.58F, config.building_alpha);
    markers.markers.push_back(marker);
  }

  if (markers.markers.empty()) {
    return staticMapBuildingDeleteMarkers(config.header);
  }
  return markers;
}

} // namespace drone_city_nav
