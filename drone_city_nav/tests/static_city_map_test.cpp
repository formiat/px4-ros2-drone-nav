#include "drone_city_nav/static_city_map.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::filesystem::path writeStaticMapFixture(const std::string& name,
                                                          const std::string& content) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / ("drone_city_nav_" + name + ".map2d");
  std::ofstream output{path};
  output << content;
  return path;
}

} // namespace

TEST(StaticCityMap, LoadsValidMap2dFile) {
  const std::filesystem::path path =
      writeStaticMapFixture("valid", "drone_city_nav_static_map_v1\n"
                                     "frame_id map\n"
                                     "bounds 0.0 0.0 1.0 10.0 12.0\n"
                                     "rect building_a 5.0 6.0 2.0 4.0 8.0\n");

  const StaticCityMap map = loadStaticCityMap(path);

  EXPECT_EQ(map.frame_id, "map");
  EXPECT_EQ(map.bounds.width_cells, 10);
  EXPECT_EQ(map.bounds.height_cells, 12);
  ASSERT_EQ(map.rectangles.size(), 1U);
  EXPECT_EQ(map.rectangles.front().id, "building_a");
  EXPECT_DOUBLE_EQ(map.rectangles.front().height_m, 8.0);
  std::filesystem::remove(path);
}

TEST(StaticCityMap, RejectsInvalidVersion) {
  const std::filesystem::path path =
      writeStaticMapFixture("invalid_version", "wrong_version\n"
                                               "frame_id map\n"
                                               "bounds 0.0 0.0 1.0 10.0 12.0\n");

  EXPECT_THROW(loadStaticCityMap(path), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(StaticCityMap, RejectsRectangleOutsideBounds) {
  const std::filesystem::path path =
      writeStaticMapFixture("outside_bounds", "drone_city_nav_static_map_v1\n"
                                              "frame_id map\n"
                                              "bounds 0.0 0.0 1.0 10.0 10.0\n"
                                              "rect outside 9.0 9.0 4.0 4.0 5.0\n");

  EXPECT_THROW(loadStaticCityMap(path), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(StaticCityMap, RasterizesRectangleIntoOccupiedCells) {
  StaticCityMap map{};
  map.frame_id = "map";
  map.bounds = GridBounds{0.0, 0.0, 1.0, 10, 10};
  map.rectangles.push_back(
      StaticCityMapRect{"building", Point2{5.0, 5.0}, 2.0, 2.0, 6.0});

  const OccupancyGrid2D grid = rasterizeStaticCityMap(map, 0.0);

  EXPECT_TRUE(grid.isOccupied(GridIndex{4, 4}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{5, 4}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{4, 5}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{5, 5}));
  EXPECT_EQ(grid.state(GridIndex{3, 5}), CellState::kUnknown);
}

TEST(StaticCityMap, HeightThresholdCanExcludeLowBuildings) {
  StaticCityMap map{};
  map.frame_id = "map";
  map.bounds = GridBounds{0.0, 0.0, 1.0, 10, 10};
  map.rectangles.push_back(StaticCityMapRect{"low", Point2{5.0, 5.0}, 2.0, 2.0, 1.0});
  map.rectangles.push_back(StaticCityMapRect{"high", Point2{8.0, 8.0}, 2.0, 2.0, 5.0});

  const OccupancyGrid2D grid = rasterizeStaticCityMap(map, 2.0);

  EXPECT_FALSE(grid.isOccupied(GridIndex{4, 4}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{7, 7}));
}

} // namespace drone_city_nav
