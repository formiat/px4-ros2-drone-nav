#include "free_space_topology_extractor_3d_geometry.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace drone_city_nav::topology_extractor_detail {
namespace {
constexpr double kEpsilon{1.0e-9};
}

std::size_t voxelCount(const GridBounds3D& bounds) {
  const auto width = static_cast<std::size_t>(bounds.width_cells);
  const auto height = static_cast<std::size_t>(bounds.height_cells);
  const auto depth = static_cast<std::size_t>(bounds.depth_cells);
  if (height != 0U && width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::overflow_error{"FreeSpaceTopology3D dimensions overflow"};
  }
  const std::size_t plane = width * height;
  if (depth != 0U && plane > std::numeric_limits<std::size_t>::max() / depth) {
    throw std::overflow_error{"FreeSpaceTopology3D dimensions overflow"};
  }
  return plane * depth;
}

std::size_t linearIndex(const GridBounds3D& bounds, const GridIndex3D index) noexcept {
  return (static_cast<std::size_t>(index.z) *
              static_cast<std::size_t>(bounds.height_cells) +
          static_cast<std::size_t>(index.y)) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(index.x);
}

GridIndex3D cellFor(const GridBounds3D& bounds, const std::size_t linear) noexcept {
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  const std::size_t plane = width * height;
  const auto z = static_cast<int>(linear / plane);
  const std::size_t remainder = linear % plane;
  return GridIndex3D{static_cast<int>(remainder % width),
                     static_cast<int>(remainder / width), z};
}

bool contains(const GridBounds3D& bounds, const GridIndex3D index) noexcept {
  return index.x >= 0 && index.y >= 0 && index.z >= 0 && index.x < bounds.width_cells &&
         index.y < bounds.height_cells && index.z < bounds.depth_cells;
}

GridIndex3D offset(const GridIndex3D cell, const GridIndex3D delta) noexcept {
  return GridIndex3D{cell.x + delta.x, cell.y + delta.y, cell.z + delta.z};
}

const std::vector<GridIndex3D>& neighbors26() {
  static const std::vector<GridIndex3D> directions = [] {
    std::vector<GridIndex3D> result;
    result.reserve(26U);
    for (int z = -1; z <= 1; ++z) {
      for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
          if (x != 0 || y != 0 || z != 0) {
            result.push_back(GridIndex3D{x, y, z});
          }
        }
      }
    }
    return result;
  }();
  return directions;
}

const std::vector<GridIndex3D>& antipodalNeighborDirections() {
  static const std::vector<GridIndex3D> directions = [] {
    std::vector<GridIndex3D> result;
    result.reserve(13U);
    for (const GridIndex3D direction : neighbors26()) {
      if (direction.z > 0 ||
          (direction.z == 0 &&
           (direction.y > 0 || (direction.y == 0 && direction.x > 0)))) {
        result.push_back(direction);
      }
    }
    return result;
  }();
  return directions;
}

bool sameBounds(const GridBounds3D& first, const GridBounds3D& second) noexcept {
  constexpr double tolerance = 1.0e-6;
  return std::abs(first.origin_x - second.origin_x) <= tolerance &&
         std::abs(first.origin_y - second.origin_y) <= tolerance &&
         std::abs(first.origin_z - second.origin_z) <= tolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= tolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

Vec3 normalized(const Vec3& value) noexcept {
  const double length = std::hypot(std::hypot(value.x, value.y), value.z);
  return length > kEpsilon ? Vec3{value.x / length, value.y / length, value.z / length}
                           : Vec3{};
}

Vec3 cross(const Vec3& first, const Vec3& second) noexcept {
  return Vec3{first.y * second.z - first.z * second.y,
              first.z * second.x - first.x * second.z,
              first.x * second.y - first.y * second.x};
}

double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

Point3 translated(const Point3& center, const Vec3& first, const double first_distance,
                  const Vec3& second, const double second_distance) noexcept {
  return Point3{center.x + first.x * first_distance + second.x * second_distance,
                center.y + first.y * first_distance + second.y * second_distance,
                center.z + first.z * first_distance + second.z * second_distance};
}

std::string indexedId(const char* const prefix, const std::size_t index) {
  std::ostringstream stream;
  stream << prefix << ':' << std::setw(6) << std::setfill('0') << index;
  return stream.str();
}

} // namespace drone_city_nav::topology_extractor_detail
