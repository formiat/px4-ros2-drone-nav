#include "drone_city_nav/collision_voxelizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

namespace drone_city_nav {
namespace {

[[nodiscard]] Vec3 subtract(const Point3& lhs, const Point3& rhs) noexcept {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

[[nodiscard]] Vec3 subtract(const Vec3& lhs, const Vec3& rhs) noexcept {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

[[nodiscard]] Vec3 cross(const Vec3& lhs, const Vec3& rhs) noexcept {
  return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z,
          lhs.x * rhs.y - lhs.y * rhs.x};
}

[[nodiscard]] double dot(const Vec3& lhs, const Vec3& rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

[[nodiscard]] bool separatedOnAxis(const std::array<Vec3, 3U>& vertices,
                                   const Vec3& axis, const Vec3& half_extent) noexcept {
  const double axis_squared = dot(axis, axis);
  if (axis_squared <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const std::array<double, 3U> projections{
      dot(vertices[0], axis), dot(vertices[1], axis), dot(vertices[2], axis)};
  const auto [minimum, maximum] = std::ranges::minmax(projections);
  const double radius = half_extent.x * std::abs(axis.x) +
                        half_extent.y * std::abs(axis.y) +
                        half_extent.z * std::abs(axis.z);
  return minimum > radius || maximum < -radius;
}

[[nodiscard]] int boundedCellIndex(const double coordinate, const double origin,
                                   const double resolution,
                                   const int upper_bound) noexcept {
  const double relative = (coordinate - origin) / resolution;
  return std::clamp(static_cast<int>(std::floor(relative)), 0, upper_bound - 1);
}

void updateFingerprint(std::uint64_t& hash, const std::string_view value) {
  for (const char byte : value) {
    hash ^= static_cast<std::uint8_t>(byte);
    hash *= 1099511628211ULL;
  }
}

void updateFingerprint(std::uint64_t& hash, const std::filesystem::path& path) {
  std::ifstream stream{path, std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"failed to fingerprint collision input: " + path.string()};
  }
  char byte{0};
  while (stream.get(byte)) {
    hash ^= static_cast<std::uint8_t>(byte);
    hash *= 1099511628211ULL;
  }
}

} // namespace

bool triangleIntersectsAxisAlignedBox(const CollisionTriangle3D& triangle,
                                      const Point3& box_center,
                                      const Vec3& box_half_extent) noexcept {
  const std::array<Vec3, 3U> vertices{subtract(triangle.a, box_center),
                                      subtract(triangle.b, box_center),
                                      subtract(triangle.c, box_center)};
  const std::array<Vec3, 3U> edges{subtract(vertices[1], vertices[0]),
                                   subtract(vertices[2], vertices[1]),
                                   subtract(vertices[0], vertices[2])};
  constexpr std::array<Vec3, 3U> box_axes{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0},
                                          Vec3{0.0, 0.0, 1.0}};

  for (const Vec3& axis : box_axes) {
    if (separatedOnAxis(vertices, axis, box_half_extent)) {
      return false;
    }
  }
  if (separatedOnAxis(vertices, cross(edges[0], edges[1]), box_half_extent)) {
    return false;
  }
  for (const Vec3& edge : edges) {
    for (const Vec3& box_axis : box_axes) {
      if (separatedOnAxis(vertices, cross(edge, box_axis), box_half_extent)) {
        return false;
      }
    }
  }
  return true;
}

CollisionVoxelizationStats
voxelizeCollisionTriangles(const std::vector<CollisionTriangle3D>& triangles,
                           OccupancyGrid3D& occupancy) {
  CollisionVoxelizationStats stats{.triangles = triangles.size()};
  const GridBounds3D& bounds = occupancy.bounds();
  const double half = 0.5 * bounds.resolution_m;
  const Vec3 half_extent{half, half, half};

  for (const CollisionTriangle3D& triangle : triangles) {
    const double minimum_x = std::min({triangle.a.x, triangle.b.x, triangle.c.x});
    const double minimum_y = std::min({triangle.a.y, triangle.b.y, triangle.c.y});
    const double minimum_z = std::min({triangle.a.z, triangle.b.z, triangle.c.z});
    const double maximum_x = std::max({triangle.a.x, triangle.b.x, triangle.c.x});
    const double maximum_y = std::max({triangle.a.y, triangle.b.y, triangle.c.y});
    const double maximum_z = std::max({triangle.a.z, triangle.b.z, triangle.c.z});
    const int begin_x = boundedCellIndex(minimum_x, bounds.origin_x,
                                         bounds.resolution_m, bounds.width_cells);
    const int begin_y = boundedCellIndex(minimum_y, bounds.origin_y,
                                         bounds.resolution_m, bounds.height_cells);
    const int begin_z = boundedCellIndex(minimum_z, bounds.origin_z,
                                         bounds.resolution_m, bounds.depth_cells);
    const int end_x = boundedCellIndex(maximum_x, bounds.origin_x, bounds.resolution_m,
                                       bounds.width_cells);
    const int end_y = boundedCellIndex(maximum_y, bounds.origin_y, bounds.resolution_m,
                                       bounds.height_cells);
    const int end_z = boundedCellIndex(maximum_z, bounds.origin_z, bounds.resolution_m,
                                       bounds.depth_cells);

    for (int z = begin_z; z <= end_z; ++z) {
      for (int y = begin_y; y <= end_y; ++y) {
        for (int x = begin_x; x <= end_x; ++x) {
          ++stats.tested_voxels;
          const GridIndex3D index{x, y, z};
          if (!triangleIntersectsAxisAlignedBox(triangle, occupancy.cellCenter(index),
                                                half_extent)) {
            continue;
          }
          const std::size_t before = occupancy.occupiedVoxelCount();
          occupancy.setOccupied(index);
          stats.newly_occupied_voxels +=
              occupancy.occupiedVoxelCount() == before ? 0U : 1U;
        }
      }
    }
  }
  return stats;
}

std::uint64_t
fingerprintCollisionInputs(const std::filesystem::path& sdf,
                           const std::vector<std::filesystem::path>& mesh_paths) {
  std::uint64_t hash{1469598103934665603ULL};
  const std::filesystem::path absolute_sdf = std::filesystem::weakly_canonical(sdf);
  updateFingerprint(hash, std::string_view{"sdf\0", 4U});
  updateFingerprint(hash, absolute_sdf);
  const std::set<std::filesystem::path> unique_mesh_paths{mesh_paths.begin(),
                                                          mesh_paths.end()};
  for (const std::filesystem::path& mesh_path : unique_mesh_paths) {
    updateFingerprint(hash, std::string_view{"mesh\0", 5U});
    const std::string path_text =
        mesh_path.lexically_relative(absolute_sdf.parent_path()).generic_string();
    if (path_text.empty()) {
      throw std::runtime_error{"failed to derive relative collision mesh identity"};
    }
    updateFingerprint(hash, std::string_view{path_text});
    updateFingerprint(hash, std::string_view{"\0", 1U});
    updateFingerprint(hash, mesh_path);
  }
  return hash;
}

} // namespace drone_city_nav
