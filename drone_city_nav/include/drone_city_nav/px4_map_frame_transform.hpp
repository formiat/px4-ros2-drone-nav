#pragma once

#include "drone_city_nav/types.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

struct Px4MapFrameTransform {
  Point3 map_origin{};
  double m00{1.0};
  double m01{0.0};
  double m10{0.0};
  double m11{1.0};

  void validate() const {
    if (!finiteUnit(m00) || !finiteUnit(m01) || !finiteUnit(m10) || !finiteUnit(m11)) {
      throw std::invalid_argument{
          "PX4-to-map matrix entries must be zero or signed unit values"};
    }
    const double row_zero_norm = m00 * m00 + m01 * m01;
    const double row_one_norm = m10 * m10 + m11 * m11;
    const double row_dot = m00 * m10 + m01 * m11;
    if (std::abs(row_zero_norm - 1.0) > 1.0e-9 ||
        std::abs(row_one_norm - 1.0) > 1.0e-9 || std::abs(row_dot) > 1.0e-9) {
      throw std::invalid_argument{"PX4-to-map matrix must be orthonormal"};
    }
    if (!std::isfinite(map_origin.x) || !std::isfinite(map_origin.y) ||
        !std::isfinite(map_origin.z)) {
      throw std::invalid_argument{"PX4 map origin must be finite"};
    }
  }

  [[nodiscard]] Point2 localVectorToMap(const Point2& local) const noexcept {
    return Point2{m00 * local.x + m01 * local.y, m10 * local.x + m11 * local.y};
  }

  [[nodiscard]] Point2 mapVectorToLocal(const Point2& map) const noexcept {
    return Point2{m00 * map.x + m10 * map.y, m01 * map.x + m11 * map.y};
  }

  [[nodiscard]] Point2 localPositionToMap(const Point2& local) const noexcept {
    const Point2 displacement = localVectorToMap(local);
    return Point2{map_origin.x + displacement.x, map_origin.y + displacement.y};
  }

  [[nodiscard]] Point2 mapPositionToLocal(const Point2& map) const noexcept {
    return mapVectorToLocal(Point2{map.x - map_origin.x, map.y - map_origin.y});
  }

  [[nodiscard]] double px4HeadingToMapYaw(const double heading_rad) const noexcept {
    const Point2 map_heading =
        localVectorToMap(Point2{std::cos(heading_rad), std::sin(heading_rad)});
    return std::atan2(map_heading.y, map_heading.x);
  }

  [[nodiscard]] double mapYawToPx4Heading(const double yaw_rad) const noexcept {
    const Point2 local_heading =
        mapVectorToLocal(Point2{std::cos(yaw_rad), std::sin(yaw_rad)});
    return std::atan2(local_heading.y, local_heading.x);
  }

  [[nodiscard]] double mapYawRateToPx4(const double yaw_rate_radps) const noexcept {
    return determinant() * yaw_rate_radps;
  }

private:
  [[nodiscard]] static bool finiteUnit(const double value) noexcept {
    return std::isfinite(value) &&
           (std::abs(value) <= 1.0e-9 || std::abs(std::abs(value) - 1.0) <= 1.0e-9);
  }

  [[nodiscard]] double determinant() const noexcept {
    return m00 * m11 - m01 * m10;
  }
};

} // namespace drone_city_nav
