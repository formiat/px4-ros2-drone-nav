#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <string>

namespace drone_city_nav::topology_extractor_detail {

[[nodiscard]] std::size_t voxelCount(const GridBounds3D& bounds);
[[nodiscard]] std::size_t linearIndex(const GridBounds3D& bounds,
                                      GridIndex3D index) noexcept;
[[nodiscard]] GridIndex3D cellFor(const GridBounds3D& bounds,
                                  std::size_t linear) noexcept;
[[nodiscard]] bool contains(const GridBounds3D& bounds, GridIndex3D index) noexcept;
[[nodiscard]] GridIndex3D offset(GridIndex3D cell, GridIndex3D delta) noexcept;
[[nodiscard]] Vec3 normalized(const Vec3& value) noexcept;
[[nodiscard]] Vec3 cross(const Vec3& first, const Vec3& second) noexcept;
[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept;
[[nodiscard]] Point3 translated(const Point3& center, const Vec3& first,
                                double first_distance, const Vec3& second,
                                double second_distance) noexcept;
[[nodiscard]] std::string indexedId(const char* prefix, std::size_t index);

} // namespace drone_city_nav::topology_extractor_detail
