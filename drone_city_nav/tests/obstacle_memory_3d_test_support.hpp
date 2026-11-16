#pragma once

#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

// What the obstacle memory's tests share: the grid they scan and the beams
// they scan it with.

namespace drone_city_nav::memory_test {

inline constexpr GridBounds3D kBounds{0.0, 0.0, 0.0, 1.0, 32, 32, 16};
inline constexpr std::int64_t kSecondNs{1'000'000'000};

[[nodiscard]] inline LidarBeam3D hitAlongX(const double range_m,
                                           const bool warm_body = false) {
  return LidarBeam3D{.direction_map = {1.0, 0.0, 0.0},
                     .range_m = range_m,
                     .hit = true,
                     .valid = true,
                     .warm_body = warm_body};
}

[[nodiscard]] inline LidarBeam3D missAlongX(const double range_m) {
  return LidarBeam3D{.direction_map = {1.0, 0.0, 0.0},
                     .range_m = range_m,
                     .hit = false,
                     .valid = true};
}

// A hit on the cell whose centre is `end`, from `origin`.
[[nodiscard]] inline LidarBeam3D hitAt(const Point3& origin, const Point3& end,
                                       const bool warm_body = false) {
  const Vec3 along{end.x - origin.x, end.y - origin.y, end.z - origin.z};
  const double norm =
      std::sqrt(along.x * along.x + along.y * along.y + along.z * along.z);
  return LidarBeam3D{.direction_map = {along.x / norm, along.y / norm, along.z / norm},
                     .range_m = norm,
                     .hit = true,
                     .valid = true,
                     .warm_body = warm_body};
}

// A cold body of four voxels, two wide and two tall, its front face at x.
[[nodiscard]] inline std::vector<LidarBeam3D> coldBlockAt(const Point3& origin,
                                                          const double x) {
  std::vector<LidarBeam3D> block;
  for (const double y : {1.5, 2.5}) {
    for (const double z : {1.5, 2.5}) {
      block.push_back(hitAt(origin, {x, y, z}));
    }
  }
  return block;
}

} // namespace drone_city_nav::memory_test
