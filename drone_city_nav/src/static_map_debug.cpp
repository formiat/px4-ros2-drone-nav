#include "drone_city_nav/static_map_debug.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace drone_city_nav {
namespace {

constexpr int kStaticBuildingDeleteMarkerId = 0;

} // namespace

sensor_msgs::msg::PointCloud2
staticMapPointCloud3D(const OccupancyGrid3D& grid, const StaticMapDebugConfig& config) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = config.header;
  cloud.height = 1U;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12U;
  cloud.fields.resize(3U);
  constexpr std::array<const char*, 3U> kFieldNames{"x", "y", "z"};
  for (std::size_t index = 0U; index < cloud.fields.size(); ++index) {
    cloud.fields[index].name = kFieldNames.at(index);
    cloud.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
    cloud.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[index].count = 1U;
  }
  const GridBounds3D& bounds = grid.bounds();
  const std::size_t stride = std::max<std::size_t>(1U, config.voxel_stride);
  const std::size_t stride_volume = stride * stride * stride;
  cloud.data.reserve((grid.occupiedVoxelCount() / stride_volume + 1U) *
                     static_cast<std::size_t>(cloud.point_step));
  const int cell_stride = static_cast<int>(stride);
  for (int z = 0; z < bounds.depth_cells; z += cell_stride) {
    for (int y = 0; y < bounds.height_cells; y += cell_stride) {
      for (int x = 0; x < bounds.width_cells; x += cell_stride) {
        const GridIndex3D cell{x, y, z};
        if (!grid.isOccupied(cell)) {
          continue;
        }
        const Point3 center = grid.cellCenter(cell);
        const std::array<float, 3> point{
            static_cast<float>(center.x), static_cast<float>(center.y),
            static_cast<float>(gazeboAlignedRvizZ(center.z))};
        const std::size_t offset = cloud.data.size();
        cloud.data.resize(offset + static_cast<std::size_t>(cloud.point_step));
        std::memcpy(&cloud.data[offset], point.data(), cloud.point_step);
        ++cloud.width;
      }
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

} // namespace drone_city_nav
