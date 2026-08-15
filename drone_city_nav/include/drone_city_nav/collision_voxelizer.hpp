#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace drone_city_nav {

struct CollisionTriangle3D {
  Point3 a{};
  Point3 b{};
  Point3 c{};
};

struct CollisionVoxelizationStats {
  std::size_t triangles{0U};
  std::size_t tested_voxels{0U};
  std::size_t newly_occupied_voxels{0U};
};

[[nodiscard]] bool
triangleIntersectsAxisAlignedBox(const CollisionTriangle3D& triangle,
                                 const Point3& box_center,
                                 const Vec3& box_half_extent) noexcept;

[[nodiscard]] CollisionVoxelizationStats
voxelizeCollisionTriangles(const std::vector<CollisionTriangle3D>& triangles,
                           OccupancyGrid3D& occupancy);

[[nodiscard]] std::uint64_t
fingerprintCollisionInputs(const std::filesystem::path& sdf,
                           const std::vector<std::filesystem::path>& mesh_paths);

} // namespace drone_city_nav
