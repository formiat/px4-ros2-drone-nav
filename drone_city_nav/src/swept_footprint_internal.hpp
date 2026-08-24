#pragma once

#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

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
    return FootprintBodyAxis{.x = 0.0, .y = 0.0, .z = 0.0};
  }
  return FootprintBodyAxis{axis.x / length, axis.y / length, axis.z / length};
}

struct ConservativeSweepCover3D {
  Point3 first{};
  Point3 second{};
  FootprintBodyAxis first_axis{};
  FootprintBodyAxis second_axis{};
  Point3 half_interval_translation{};
  std::size_t interval_count{0U};
  double angle_rad{0.0};
  double rotation_inflation_m{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return interval_count > 0U;
  }
};

[[nodiscard]] inline bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] inline double axisLength(const FootprintBodyAxis& axis) noexcept {
  return std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
}

// Motion follows a linear center path and the shortest non-antipodal SLERP
// between body axes. A midpoint body enlarged by the interval translation and
// maximum rotational chord contains the complete body over that interval.
[[nodiscard]] inline ConservativeSweepCover3D makeConservativeSweepCover3D(
    const Point3& first, const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis,
    const SweptFootprintConfig& config) noexcept {
  ConservativeSweepCover3D cover{};
  if (!finitePoint(first) || !finitePoint(second) || !std::isfinite(config.radius_m) ||
      !std::isfinite(config.lower_extent_m) || !std::isfinite(config.upper_extent_m) ||
      !std::isfinite(config.sweep_step_m)) {
    return cover;
  }

  const double first_axis_length = axisLength(first_body_axis);
  const double second_axis_length = axisLength(second_body_axis);
  if (!(first_axis_length > 1.0e-9) || !std::isfinite(first_axis_length) ||
      !(second_axis_length > 1.0e-9) || !std::isfinite(second_axis_length)) {
    return cover;
  }
  cover.first = first;
  cover.second = second;
  cover.first_axis = FootprintBodyAxis{first_body_axis.x / first_axis_length,
                                       first_body_axis.y / first_axis_length,
                                       first_body_axis.z / first_axis_length};
  cover.second_axis = FootprintBodyAxis{second_body_axis.x / second_axis_length,
                                        second_body_axis.y / second_axis_length,
                                        second_body_axis.z / second_axis_length};
  const double axis_dot = cover.first_axis.x * cover.second_axis.x +
                          cover.first_axis.y * cover.second_axis.y +
                          cover.first_axis.z * cover.second_axis.z;
  const double axis_cross_x = cover.first_axis.y * cover.second_axis.z -
                              cover.first_axis.z * cover.second_axis.y;
  const double axis_cross_y = cover.first_axis.z * cover.second_axis.x -
                              cover.first_axis.x * cover.second_axis.z;
  const double axis_cross_z = cover.first_axis.x * cover.second_axis.y -
                              cover.first_axis.y * cover.second_axis.x;
  const double axis_cross_length = std::hypot(axis_cross_x, axis_cross_y, axis_cross_z);
  // End axes alone do not define which great-circle path an antipodal body took.
  // Reject that ambiguous motion instead of inventing an unsafe interpolation.
  constexpr double kAntipodalTolerance{1.0e-12};
  if (axis_dot <= -1.0 + kAntipodalTolerance) {
    return ConservativeSweepCover3D{};
  }
  cover.angle_rad = std::atan2(axis_cross_length, axis_dot);

  const double translation_length_m = distance3D(first, second);
  const double maximum_axial_extent_m = std::max(std::max(0.0, config.lower_extent_m),
                                                 std::max(0.0, config.upper_extent_m));
  const double body_radius_m =
      std::hypot(std::max(0.0, config.radius_m), maximum_axial_extent_m);
  const double total_body_motion_m =
      translation_length_m + body_radius_m * cover.angle_rad;
  const double step_m = std::max(1.0e-3, config.sweep_step_m);
  if (!std::isfinite(total_body_motion_m) || !std::isfinite(step_m)) {
    return ConservativeSweepCover3D{};
  }
  const double requested_interval_count =
      std::max(1.0, std::ceil(total_body_motion_m / step_m));
  if (requested_interval_count >
      static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    return ConservativeSweepCover3D{};
  }
  cover.interval_count = static_cast<std::size_t>(requested_interval_count);
  const double interval_count = static_cast<double>(cover.interval_count);
  cover.half_interval_translation =
      Point3{(second.x - first.x) / (2.0 * interval_count),
             (second.y - first.y) / (2.0 * interval_count),
             (second.z - first.z) / (2.0 * interval_count)};
  cover.rotation_inflation_m =
      2.0 * body_radius_m * std::sin(cover.angle_rad / (4.0 * interval_count));
  if (!finitePoint(cover.half_interval_translation) ||
      !std::isfinite(cover.rotation_inflation_m)) {
    return ConservativeSweepCover3D{};
  }
  return cover;
}

[[nodiscard]] inline FootprintBodyAxis
interpolateSweepAxis(const ConservativeSweepCover3D& cover,
                     const double ratio) noexcept {
  if (!(cover.angle_rad > 0.0)) {
    return cover.first_axis;
  }
  const double sin_angle = std::sin(cover.angle_rad);
  if (!(sin_angle > 0.0) || !std::isfinite(sin_angle)) {
    return FootprintBodyAxis{.x = 0.0, .y = 0.0, .z = 0.0};
  }
  const double first_weight = std::sin((1.0 - ratio) * cover.angle_rad) / sin_angle;
  const double second_weight = std::sin(ratio * cover.angle_rad) / sin_angle;
  return normalized(FootprintBodyAxis{
      first_weight * cover.first_axis.x + second_weight * cover.second_axis.x,
      first_weight * cover.first_axis.y + second_weight * cover.second_axis.y,
      first_weight * cover.first_axis.z + second_weight * cover.second_axis.z});
}

[[nodiscard]] inline Point3
interpolateSweepPosition(const ConservativeSweepCover3D& cover,
                         const double ratio) noexcept {
  return Point3{std::lerp(cover.first.x, cover.second.x, ratio),
                std::lerp(cover.first.y, cover.second.y, ratio),
                std::lerp(cover.first.z, cover.second.z, ratio)};
}

[[nodiscard]] inline SweptFootprintConfig
inflatedSweepFootprint(const SweptFootprintConfig& config,
                       const ConservativeSweepCover3D& cover,
                       const FootprintBodyAxis& midpoint_axis) noexcept {
  const double midpoint_axis_length = axisLength(midpoint_axis);
  const double axial_translation_m =
      std::abs(cover.half_interval_translation.x * midpoint_axis.x +
               cover.half_interval_translation.y * midpoint_axis.y +
               cover.half_interval_translation.z * midpoint_axis.z) /
      midpoint_axis_length;
  const double cross_x = cover.half_interval_translation.y * midpoint_axis.z -
                         cover.half_interval_translation.z * midpoint_axis.y;
  const double cross_y = cover.half_interval_translation.z * midpoint_axis.x -
                         cover.half_interval_translation.x * midpoint_axis.z;
  const double cross_z = cover.half_interval_translation.x * midpoint_axis.y -
                         cover.half_interval_translation.y * midpoint_axis.x;
  const double lateral_translation_m =
      std::hypot(cross_x, cross_y, cross_z) / midpoint_axis_length;
  const double radial_inflation_m = lateral_translation_m + cover.rotation_inflation_m;
  const double axial_inflation_m = axial_translation_m + cover.rotation_inflation_m;
  const double positive_infinity = std::numeric_limits<double>::infinity();
  const auto conservatively_expanded = [&](const double extent_m,
                                           const double inflation_m) noexcept {
    const double expanded_m = extent_m + inflation_m;
    return inflation_m > 0.0 ? std::nextafter(expanded_m, positive_infinity)
                             : expanded_m;
  };
  SweptFootprintConfig inflated = config;
  inflated.radius_m =
      conservatively_expanded(std::max(0.0, config.radius_m), radial_inflation_m);
  inflated.lower_extent_m =
      conservatively_expanded(std::max(0.0, config.lower_extent_m), axial_inflation_m);
  inflated.upper_extent_m =
      conservatively_expanded(std::max(0.0, config.upper_extent_m), axial_inflation_m);
  return inflated;
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
