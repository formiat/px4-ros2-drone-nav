#include "drone_city_nav/static_map_debug.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] std_msgs::msg::Header header() {
  std_msgs::msg::Header result;
  result.frame_id = "map";
  return result;
}

TEST(StaticMapDebug3D, VisualizationStrideReducesPublishedVoxelCount) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 8, 8, 8}};
  for (int z = 0; z < 8; ++z) {
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        grid.setOccupied(GridIndex3D{x, y, z});
      }
    }
  }

  const sensor_msgs::msg::PointCloud2 full =
      staticMapPointCloud3D(grid, StaticMapDebugConfig{header(), 0.05F, 0.62F, 1U});
  const sensor_msgs::msg::PointCloud2 reduced =
      staticMapPointCloud3D(grid, StaticMapDebugConfig{header(), 0.05F, 0.62F, 4U});

  EXPECT_EQ(full.width, 512U);
  EXPECT_EQ(reduced.width, 8U);
  EXPECT_EQ(reduced.data.size(),
            static_cast<std::size_t>(reduced.width * reduced.point_step));
}

TEST(StaticMapDebug3D, ZeroStrideFallsBackToOne) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 2, 2, 2}};
  grid.setOccupied(GridIndex3D{1, 1, 1});

  const sensor_msgs::msg::PointCloud2 cloud =
      staticMapPointCloud3D(grid, StaticMapDebugConfig{header(), 0.05F, 0.62F, 0U});

  EXPECT_EQ(cloud.width, 1U);
}

} // namespace
} // namespace drone_city_nav
