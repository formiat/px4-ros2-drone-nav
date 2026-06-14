#include "drone_city_nav/static_city_map.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr const char* kStaticMapVersion = "drone_city_nav_static_map_v1";

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] int positiveCellCount(const double length_m, const double resolution_m) {
  return std::max(1, static_cast<int>(std::ceil(length_m / resolution_m)));
}

[[nodiscard]] std::string stripComment(const std::string& line) {
  const std::size_t comment_position = line.find('#');
  if (comment_position == std::string::npos) {
    return line;
  }
  return line.substr(0, comment_position);
}

[[nodiscard]] double parseDouble(std::istringstream& stream, const char* field_name,
                                 const int line_number) {
  double value = 0.0;
  if (!(stream >> value) || !std::isfinite(value)) {
    throw std::runtime_error{"Invalid numeric field '" + std::string{field_name} +
                             "' at static map line " + std::to_string(line_number)};
  }
  return value;
}

void requireNoTrailingTokens(std::istringstream& stream, const int line_number) {
  std::string trailing;
  if (stream >> trailing) {
    throw std::runtime_error{"Unexpected trailing token '" + trailing +
                             "' at static map line " + std::to_string(line_number)};
  }
}

void validateRectangleInsideBounds(const StaticCityMapRect& rectangle,
                                   const GridBounds& bounds, const int line_number) {
  const double min_x = rectangle.center.x - rectangle.size_x_m / 2.0;
  const double max_x = rectangle.center.x + rectangle.size_x_m / 2.0;
  const double min_y = rectangle.center.y - rectangle.size_y_m / 2.0;
  const double max_y = rectangle.center.y + rectangle.size_y_m / 2.0;
  const double bounds_max_x =
      bounds.origin_x + static_cast<double>(bounds.width_cells) * bounds.resolution_m;
  const double bounds_max_y =
      bounds.origin_y + static_cast<double>(bounds.height_cells) * bounds.resolution_m;
  constexpr double kToleranceM = 1.0e-9;
  if (min_x < bounds.origin_x - kToleranceM || min_y < bounds.origin_y - kToleranceM ||
      max_x > bounds_max_x + kToleranceM || max_y > bounds_max_y + kToleranceM) {
    throw std::runtime_error{"Static map rectangle '" + rectangle.id +
                             "' is outside bounds at line " +
                             std::to_string(line_number)};
  }
}

[[nodiscard]] bool centerInsideRectangle(const Point2 center,
                                         const StaticCityMapRect& rectangle) noexcept {
  const double half_x = rectangle.size_x_m / 2.0;
  const double half_y = rectangle.size_y_m / 2.0;
  return center.x >= rectangle.center.x - half_x &&
         center.x <= rectangle.center.x + half_x &&
         center.y >= rectangle.center.y - half_y &&
         center.y <= rectangle.center.y + half_y;
}

} // namespace

StaticCityMap loadStaticCityMap(const std::filesystem::path& path) {
  std::ifstream input{path};
  if (!input) {
    throw std::runtime_error{"Unable to open static city map: " + path.string()};
  }

  StaticCityMap map{};
  std::unordered_set<std::string> ids;
  bool version_seen = false;
  bool bounds_seen = false;

  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    std::istringstream stream{stripComment(line)};
    std::string keyword;
    if (!(stream >> keyword)) {
      continue;
    }

    if (!version_seen) {
      if (keyword != kStaticMapVersion) {
        throw std::runtime_error{"Invalid static map version at line " +
                                 std::to_string(line_number)};
      }
      version_seen = true;
      requireNoTrailingTokens(stream, line_number);
      continue;
    }

    if (keyword == "frame_id") {
      if (map.frame_id.empty()) {
        if (!(stream >> map.frame_id) || map.frame_id.empty()) {
          throw std::runtime_error{"Missing frame_id value at static map line " +
                                   std::to_string(line_number)};
        }
        requireNoTrailingTokens(stream, line_number);
        continue;
      }
      throw std::runtime_error{"Duplicate frame_id at static map line " +
                               std::to_string(line_number)};
    }

    if (keyword == "bounds") {
      if (bounds_seen) {
        throw std::runtime_error{"Duplicate bounds at static map line " +
                                 std::to_string(line_number)};
      }
      const double origin_x = parseDouble(stream, "origin_x", line_number);
      const double origin_y = parseDouble(stream, "origin_y", line_number);
      const double resolution_m = parseDouble(stream, "resolution_m", line_number);
      const double width_m = parseDouble(stream, "width_m", line_number);
      const double height_m = parseDouble(stream, "height_m", line_number);
      requireNoTrailingTokens(stream, line_number);
      if (!finitePositive(resolution_m) || !finitePositive(width_m) ||
          !finitePositive(height_m)) {
        throw std::runtime_error{
            "Static map bounds require positive resolution, width, "
            "and height at line " +
            std::to_string(line_number)};
      }
      map.bounds = GridBounds{origin_x, origin_y, resolution_m,
                              positiveCellCount(width_m, resolution_m),
                              positiveCellCount(height_m, resolution_m)};
      bounds_seen = true;
      continue;
    }

    if (keyword == "rect") {
      if (!bounds_seen) {
        throw std::runtime_error{"Static map rect declared before bounds at line " +
                                 std::to_string(line_number)};
      }
      StaticCityMapRect rectangle{};
      if (!(stream >> rectangle.id) || rectangle.id.empty()) {
        throw std::runtime_error{"Missing rectangle id at static map line " +
                                 std::to_string(line_number)};
      }
      if (!ids.insert(rectangle.id).second) {
        throw std::runtime_error{"Duplicate static map rectangle id '" + rectangle.id +
                                 "' at line " + std::to_string(line_number)};
      }
      rectangle.center.x = parseDouble(stream, "center_x_m", line_number);
      rectangle.center.y = parseDouble(stream, "center_y_m", line_number);
      rectangle.size_x_m = parseDouble(stream, "size_x_m", line_number);
      rectangle.size_y_m = parseDouble(stream, "size_y_m", line_number);
      rectangle.height_m = parseDouble(stream, "height_m", line_number);
      requireNoTrailingTokens(stream, line_number);
      if (!finitePositive(rectangle.size_x_m) || !finitePositive(rectangle.size_y_m) ||
          rectangle.height_m < 0.0) {
        throw std::runtime_error{"Static map rectangle '" + rectangle.id +
                                 "' has invalid size or height at line " +
                                 std::to_string(line_number)};
      }
      validateRectangleInsideBounds(rectangle, map.bounds, line_number);
      map.rectangles.push_back(std::move(rectangle));
      continue;
    }

    throw std::runtime_error{"Unknown static map keyword '" + keyword + "' at line " +
                             std::to_string(line_number)};
  }

  if (!version_seen) {
    throw std::runtime_error{"Static map is empty or missing version"};
  }
  if (map.frame_id.empty()) {
    throw std::runtime_error{"Static map is missing frame_id"};
  }
  if (!bounds_seen) {
    throw std::runtime_error{"Static map is missing bounds"};
  }

  return map;
}

OccupancyGrid2D rasterizeStaticCityMap(const StaticCityMap& map,
                                       const double min_blocking_height_m) {
  OccupancyGrid2D grid{map.bounds};
  for (const StaticCityMapRect& rectangle : map.rectangles) {
    if (!(rectangle.height_m > min_blocking_height_m)) {
      continue;
    }
    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        const GridIndex cell{x, y};
        if (centerInsideRectangle(grid.cellCenter(cell), rectangle)) {
          grid.setOccupied(cell);
        }
      }
    }
  }
  return grid;
}

} // namespace drone_city_nav
