#pragma once

#include "drone_city_nav/route_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <span>

namespace drone_city_nav::route_3d_detail {

// The route geometry route_3d.cpp and route_3d_prefix.cpp share.

[[nodiscard]] inline Vec3 normalized(const Point3& from, const Point3& to) noexcept {
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dz = to.z - from.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  return length > 1.0e-9 ? Vec3{dx / length, dy / length, dz / length} : Vec3{};
}

[[nodiscard]] inline double vectorNorm(const Vec3& value) noexcept {
  return std::hypot(std::hypot(value.x, value.y), value.z);
}

[[nodiscard]] inline Vec3 normalized(const Vec3& value) noexcept {
  const double length = vectorNorm(value);
  return length > 1.0e-9 ? Vec3{value.x / length, value.y / length, value.z / length}
                         : Vec3{};
}

[[nodiscard]] inline Point3 translated(const Point3& point, const Vec3& direction,
                                       const double distance_m) noexcept {
  return Point3{point.x + direction.x * distance_m, point.y + direction.y * distance_m,
                point.z + direction.z * distance_m};
}

struct CubicBezierCurve3D {
  Point3 start{};
  Point3 start_control{};
  Point3 end_control{};
  Point3 end{};
};

[[nodiscard]] inline Point3 cubicBezierPoint(const CubicBezierCurve3D& curve,
                                             const double parameter) noexcept {
  const double complement = 1.0 - parameter;
  const double first_weight = complement * complement * complement;
  const double second_weight = 3.0 * complement * complement * parameter;
  const double third_weight = 3.0 * complement * parameter * parameter;
  const double fourth_weight = parameter * parameter * parameter;
  return Point3{first_weight * curve.start.x + second_weight * curve.start_control.x +
                    third_weight * curve.end_control.x + fourth_weight * curve.end.x,
                first_weight * curve.start.y + second_weight * curve.start_control.y +
                    third_weight * curve.end_control.y + fourth_weight * curve.end.y,
                first_weight * curve.start.z + second_weight * curve.start_control.z +
                    third_weight * curve.end_control.z + fourth_weight * curve.end.z};
}

[[nodiscard]] inline RouteSample3D
sampleAtStation(const std::span<const RouteSample3D> route, const double station_m) {
  if (route.empty()) {
    return {};
  }
  const auto upper =
      std::lower_bound(route.begin(), route.end(), station_m,
                       [](const RouteSample3D& sample, const double station) {
                         return sample.station_m < station;
                       });
  if (upper == route.begin()) {
    return route.front();
  }
  if (upper == route.end()) {
    return route.back();
  }
  const RouteSample3D& first = *std::prev(upper);
  const RouteSample3D& second = *upper;
  const double span_m = second.station_m - first.station_m;
  const double ratio =
      span_m > 1.0e-9 ? std::clamp((station_m - first.station_m) / span_m, 0.0, 1.0)
                      : 0.0;
  Vec3 tangent{std::lerp(first.tangent.x, second.tangent.x, ratio),
               std::lerp(first.tangent.y, second.tangent.y, ratio),
               std::lerp(first.tangent.z, second.tangent.z, ratio)};
  const double tangent_norm = std::hypot(std::hypot(tangent.x, tangent.y), tangent.z);
  if (tangent_norm > 1.0e-9) {
    tangent.x /= tangent_norm;
    tangent.y /= tangent_norm;
    tangent.z /= tangent_norm;
  } else {
    tangent = normalized(first.position, second.position);
  }
  RouteKinematicTransition3D transition{RouteKinematicTransition3D::kContinuous};
  if (ratio >= 1.0 - 1.0e-9) {
    transition = second.transition;
  } else if (ratio <= 1.0e-9) {
    transition = first.transition;
  }
  return RouteSample3D{
      .position = Point3{std::lerp(first.position.x, second.position.x, ratio),
                         std::lerp(first.position.y, second.position.y, ratio),
                         std::lerp(first.position.z, second.position.z, ratio)},
      .tangent = tangent,
      .station_m = std::lerp(first.station_m, second.station_m, ratio),
      .reference_speed_mps =
          std::lerp(first.reference_speed_mps, second.reference_speed_mps, ratio),
      .required_risk_tier = static_cast<RouteRiskTier3D>(
          std::max(static_cast<std::uint8_t>(first.required_risk_tier),
                   static_cast<std::uint8_t>(second.required_risk_tier))),
      .transition = transition,
  };
}

[[nodiscard]] inline const RouteEnvelopeSample&
nearestEnvelopeSample(const ConstrainedRouteSpan& span, const double station_m) {
  const auto upper =
      std::lower_bound(span.envelope.begin(), span.envelope.end(), station_m,
                       [](const RouteEnvelopeSample& sample, const double station) {
                         return sample.station_m < station;
                       });
  if (upper == span.envelope.begin()) {
    return span.envelope.front();
  }
  if (upper == span.envelope.end()) {
    return span.envelope.back();
  }
  const RouteEnvelopeSample& previous = *std::prev(upper);
  return station_m - previous.station_m <= upper->station_m - station_m ? previous
                                                                        : *upper;
}

} // namespace drone_city_nav::route_3d_detail
