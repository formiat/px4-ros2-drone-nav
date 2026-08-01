#include "drone_city_nav/static_map_debug.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstring>

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
      staticMapPointCloud3D(grid, StaticMapDebugConfig{header(), 1U});
  const sensor_msgs::msg::PointCloud2 reduced =
      staticMapPointCloud3D(grid, StaticMapDebugConfig{header(), 4U});

  EXPECT_EQ(full.width, 512U);
  EXPECT_EQ(reduced.width, 8U);
  EXPECT_EQ(reduced.data.size(),
            static_cast<std::size_t>(reduced.width * reduced.point_step));
}

TEST(StaticMapDebug3D, ZeroStrideFallsBackToOne) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 2, 2, 2}};
  grid.setOccupied(GridIndex3D{1, 1, 1});

  const sensor_msgs::msg::PointCloud2 cloud =
      staticMapPointCloud3D(grid, StaticMapDebugConfig{header(), 0U});

  EXPECT_EQ(cloud.width, 1U);
}

TEST(StaticMapDebug3D, CompensatesGazeboAlignedVisualizationZ) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 1, 1, 2}};
  grid.setOccupied(GridIndex3D{0, 0, 1});

  const sensor_msgs::msg::PointCloud2 cloud =
      staticMapPointCloud3D(grid, StaticMapDebugConfig{header(), 1U});

  ASSERT_EQ(cloud.width, 1U);
  std::array<float, 3> point{};
  ASSERT_GE(cloud.data.size(), sizeof(point));
  std::memcpy(point.data(), cloud.data.data(), sizeof(point));
  EXPECT_FLOAT_EQ(point[2], -0.75F);
}

} // namespace
} // namespace drone_city_nav
