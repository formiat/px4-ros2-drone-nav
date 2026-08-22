#pragma once

#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav::swept_footprint_detail {

[[nodiscard]] inline int statusPriority(const SweptFootprintStatus status) noexcept {
  switch (status) {
    case SweptFootprintStatus::kValid:
      return 0;
    case SweptFootprintStatus::kUnknownSpace:
      return 1;
    case SweptFootprintStatus::kOutsideGrid:
      return 2;
    case SweptFootprintStatus::kInvalidEsdf:
      return 3;
    case SweptFootprintStatus::kRawCollision:
      return 4;
  }
  return 3;
}

[[nodiscard]] inline SweptFootprintResult
makeStatusResult(const SweptFootprintStatus status,
                 const Point3& failure_point) noexcept {
  SweptFootprintResult result{.status = status, .failure_point = failure_point};
  switch (status) {
    case SweptFootprintStatus::kValid:
      break;
    case SweptFootprintStatus::kOutsideGrid:
      result.evidence.outside_grid_exposure = true;
      break;
    case SweptFootprintStatus::kUnknownSpace:
      result.evidence.unknown_exposure = true;
      break;
    case SweptFootprintStatus::kInvalidEsdf:
      result.evidence.invalid_esdf_exposure = true;
      break;
    case SweptFootprintStatus::kRawCollision:
      result.evidence.raw_collision = true;
      break;
  }
  return result;
}

[[nodiscard]] inline SweptFootprintResult
makeKnownClearanceResult(const double clearance_m) noexcept {
  SweptFootprintResult result{.status = SweptFootprintStatus::kValid};
  result.evidence.known_clearance_observed = true;
  result.evidence.minimum_known_clearance_m = clearance_m;
  return result;
}

inline void mergeEvidence(SweptFootprintResult& target,
                          const SweptFootprintResult& source) noexcept {
  target.evidence.raw_collision =
      target.evidence.raw_collision || source.evidence.raw_collision;
  target.evidence.outside_grid_exposure =
      target.evidence.outside_grid_exposure || source.evidence.outside_grid_exposure;
  target.evidence.unknown_exposure =
      target.evidence.unknown_exposure || source.evidence.unknown_exposure;
  target.evidence.invalid_esdf_exposure =
      target.evidence.invalid_esdf_exposure || source.evidence.invalid_esdf_exposure;
  if (source.evidence.known_clearance_observed) {
    target.evidence.known_clearance_observed = true;
    target.evidence.minimum_known_clearance_m =
        std::min(target.evidence.minimum_known_clearance_m,
                 source.evidence.minimum_known_clearance_m);
  }
  if (statusPriority(source.status) > statusPriority(target.status)) {
    target.status = source.status;
    target.failure_point = source.failure_point;
  }
}

inline void subtractKnownClearance(SweptFootprintResult& result,
                                   const double extent_m) noexcept {
  if (result.evidence.known_clearance_observed) {
    result.evidence.minimum_known_clearance_m =
        std::max(0.0, result.evidence.minimum_known_clearance_m - extent_m);
  }
}

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
