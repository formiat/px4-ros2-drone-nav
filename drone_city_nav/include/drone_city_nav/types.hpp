#pragma once

#include <cmath>

namespace drone_city_nav {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

struct Pose2 {
  Point2 position{};
  double yaw_rad{0.0};
};

struct GridIndex {
  int x{0};
  int y{0};
};

struct GridBounds {
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution_m{1.0};
  int width_cells{1};
  int height_cells{1};
};

[[nodiscard]] inline bool operator==(const GridIndex lhs,
                                     const GridIndex rhs) noexcept {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

[[nodiscard]] inline bool operator!=(const GridIndex lhs,
                                     const GridIndex rhs) noexcept {
  return !(lhs == rhs);
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

} // namespace drone_city_nav
