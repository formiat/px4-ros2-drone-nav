#include "drone_city_nav/static_map_debug.hpp"

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace drone_city_nav {
namespace {

constexpr int kStaticBuildingDeleteMarkerId = 0;
constexpr double kBuildingGridCenterM = 27.0;
constexpr double kBuildingGridSpacingM = 54.0;

struct MutedRgb {
  std::uint8_t red;
  std::uint8_t green;
  std::uint8_t blue;
};

constexpr std::array<MutedRgb, 8U> kStaticMapPalette{
    MutedRgb{148U, 158U, 164U}, MutedRgb{126U, 151U, 168U}, MutedRgb{140U, 157U, 145U},
    MutedRgb{164U, 151U, 134U}, MutedRgb{154U, 142U, 159U}, MutedRgb{132U, 158U, 158U},
    MutedRgb{169U, 148U, 148U}, MutedRgb{146U, 148U, 170U},
};

[[nodiscard]] std::uint32_t staticMapColor(const Point3& point) noexcept {
  const std::int64_t building_x = static_cast<std::int64_t>(
      std::llround((point.x - kBuildingGridCenterM) / kBuildingGridSpacingM));
  const std::int64_t building_y = static_cast<std::int64_t>(
      std::llround((point.y - kBuildingGridCenterM) / kBuildingGridSpacingM));
  const std::int64_t palette_size = static_cast<std::int64_t>(kStaticMapPalette.size());
  const std::int64_t raw_index = building_x + 3 * building_y;
  const std::size_t palette_index = static_cast<std::size_t>(
      (raw_index % palette_size + palette_size) % palette_size);
  const MutedRgb color = kStaticMapPalette.at(palette_index);
  return static_cast<std::uint32_t>(color.red) << 16U |
         static_cast<std::uint32_t>(color.green) << 8U |
         static_cast<std::uint32_t>(color.blue);
}

} // namespace

sensor_msgs::msg::PointCloud2
staticMapPointCloud3D(const OccupancyGrid3D& grid, const StaticMapDebugConfig& config) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = config.header;
  cloud.height = 1U;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 16U;
  cloud.fields.resize(4U);
  constexpr std::array<const char*, 3U> kFieldNames{"x", "y", "z"};
  for (std::size_t index = 0U; index < kFieldNames.size(); ++index) {
    cloud.fields[index].name = kFieldNames.at(index);
    cloud.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
    cloud.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[index].count = 1U;
  }
  cloud.fields[3U].name = "rgb";
  cloud.fields[3U].offset = 12U;
  cloud.fields[3U].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud.fields[3U].count = 1U;
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
        std::memcpy(&cloud.data[offset], point.data(), sizeof(point));
        const std::uint32_t color = staticMapColor(center);
        std::memcpy(&cloud.data[offset + 12U], &color, sizeof(color));
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
