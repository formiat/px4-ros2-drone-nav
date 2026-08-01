#pragma once

#include <cmath>

namespace drone_city_nav {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

struct Point3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose2 {
  Point2 position{};
  double yaw_rad{0.0};
};

struct GridIndex {
  int x{0};
  int y{0};
};

struct GridIndex3D {
  int x{0};
  int y{0};
  int z{0};
};

struct GridBounds {
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution_m{1.0};
  int width_cells{1};
  int height_cells{1};
};

struct GridBounds3D {
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  double resolution_m{1.0};
  int width_cells{1};
  int height_cells{1};
  int depth_cells{1};

  [[nodiscard]] bool operator==(const GridBounds3D&) const noexcept = default;
};

[[nodiscard]] inline bool operator==(const GridIndex lhs,
                                     const GridIndex rhs) noexcept {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

[[nodiscard]] inline bool operator!=(const GridIndex lhs,
                                     const GridIndex rhs) noexcept {
  return !(lhs == rhs);
}

[[nodiscard]] inline bool operator==(const GridIndex3D lhs,
                                     const GridIndex3D rhs) noexcept {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

[[nodiscard]] inline double squaredDistance(const Point2 lhs,
                                            const Point2 rhs) noexcept {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

[[nodiscard]] inline double distance(const Point2 lhs, const Point2 rhs) noexcept {
  return std::sqrt(squaredDistance(lhs, rhs));
}

[[nodiscard]] inline double squaredDistance(const Point3& lhs,
                                            const Point3& rhs) noexcept {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return dx * dx + dy * dy + dz * dz;
}

[[nodiscard]] inline double distance3D(const Point3& lhs, const Point3& rhs) noexcept {
  return std::sqrt(squaredDistance(lhs, rhs));
}

} // namespace drone_city_nav
