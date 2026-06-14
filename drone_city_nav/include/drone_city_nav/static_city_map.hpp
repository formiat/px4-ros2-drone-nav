#pragma once

#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/types.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace drone_city_nav {

struct StaticCityMapRect {
  std::string id;
  Point2 center{};
  double size_x_m{0.0};
  double size_y_m{0.0};
  double height_m{0.0};
};

struct StaticCityMap {
  std::string frame_id;
  GridBounds bounds{};
  std::vector<StaticCityMapRect> rectangles;
};

[[nodiscard]] StaticCityMap loadStaticCityMap(const std::filesystem::path& path);

[[nodiscard]] OccupancyGrid2D rasterizeStaticCityMap(const StaticCityMap& map,
                                                     double min_blocking_height_m);

} // namespace drone_city_nav
