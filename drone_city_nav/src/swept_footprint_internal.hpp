#pragma once

#include "drone_city_nav/swept_footprint.hpp"

#include <cmath>

namespace drone_city_nav::swept_footprint_detail {

[[nodiscard]] inline FootprintBodyAxis
normalized(const FootprintBodyAxis& axis) noexcept {
  const double length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  if (!(length > 1.0e-9) || !std::isfinite(length)) {
    return {};
  }
  return FootprintBodyAxis{axis.x / length, axis.y / length, axis.z / length};
}

[[nodiscard]] inline FootprintBodyAxis cross(const FootprintBodyAxis& first,
                                             const FootprintBodyAxis& second) noexcept {
  return FootprintBodyAxis{first.y * second.z - first.z * second.y,
                           first.z * second.x - first.x * second.z,
                           first.x * second.y - first.y * second.x};
}

[[nodiscard]] inline Point3
bodyPoint(const Point3& center, const FootprintBodyAxis& axis,
          const FootprintBodyAxis& radial_x, const FootprintBodyAxis& radial_y,
          const double axial_offset_m, const double radial_offset_m,
          const double angle_rad) noexcept {
  const double cos_angle = std::cos(angle_rad);
  const double sin_angle = std::sin(angle_rad);
  return Point3{
      center.x + axial_offset_m * axis.x +
          radial_offset_m * (cos_angle * radial_x.x + sin_angle * radial_y.x),
      center.y + axial_offset_m * axis.y +
          radial_offset_m * (cos_angle * radial_x.y + sin_angle * radial_y.y),
      center.z + axial_offset_m * axis.z +
          radial_offset_m * (cos_angle * radial_x.z + sin_angle * radial_y.z),
  };
}

} // namespace drone_city_nav::swept_footprint_detail
