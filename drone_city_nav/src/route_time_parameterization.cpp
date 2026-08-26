#include "drone_city_nav/route_time_parameterization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kEpsilon{1.0e-6};

[[nodiscard]] double vectorNorm(const Vec3& value) noexcept {
  return std::hypot(std::hypot(value.x, value.y), value.z);
}

[[nodiscard]] double routeCurvature(const std::span<const RouteSample3D> route,
                                    const std::size_t index) noexcept {
  if (index == 0U || index + 1U >= route.size()) {
    return 0.0;
  }
  const Vec3& before = route[index - 1U].tangent;
  const Vec3& after = route[index].tangent;
  const double before_norm = vectorNorm(before);
  const double after_norm = vectorNorm(after);
  const double distance_m = route[index + 1U].station_m - route[index - 1U].station_m;
  if (!(before_norm > kEpsilon) || !(after_norm > kEpsilon) ||
      !(distance_m > kEpsilon)) {
    return 0.0;
  }
  const double cosine =
      std::clamp((before.x * after.x + before.y * after.y + before.z * after.z) /
                     (before_norm * after_norm),
                 -1.0, 1.0);
  return std::acos(cosine) / distance_m;
}

[[nodiscard]] double constrainedLimit(const RouteSample3D& sample,
                                      const std::span<const ConstrainedRouteSpan> spans,
                                      const double fallback_mps) noexcept {
  for (const ConstrainedRouteSpan& span : spans) {
    if (sample.station_m < span.begin_station_m ||
        sample.station_m > span.end_station_m) {
      continue;
    }
    return span.envelope.empty()
               ? fallback_mps
               : std::max(0.0, span.envelope.front().reference_speed_mps);
  }
  return std::numeric_limits<double>::infinity();
}

[[nodiscard]] double
segmentAccelerationLimit(const RouteSample3D& first, const RouteSample3D& second,
                         const mppi::DynamicsConfig& dynamics) noexcept {
  const double horizontal = std::hypot(second.position.x - first.position.x,
                                       second.position.y - first.position.y);
  const double vertical = std::abs(second.position.z - first.position.z);
  const double distance_m = second.station_m - first.station_m;
  if (!(distance_m > kEpsilon)) {
    return 0.0;
  }
  const double horizontal_share = horizontal / distance_m;
  const double vertical_share = vertical / distance_m;
  const double horizontal_limit =
      static_cast<double>(dynamics.maximum_horizontal_acceleration_mps2) /
      std::max(horizontal_share, kEpsilon);
  const double vertical_limit =
      static_cast<double>(dynamics.maximum_vertical_acceleration_mps2) /
      std::max(vertical_share, kEpsilon);
  return std::min(horizontal_limit, vertical_limit);
}

void enforceJerkEnvelope(const std::span<const RouteSample3D> route,
                         std::vector<double>& speeds,
                         const mppi::DynamicsConfig& dynamics) noexcept {
  const double jerk = static_cast<double>(dynamics.maximum_control_jerk_mps3);
  if (!(jerk > kEpsilon) || route.size() < 3U) {
    return;
  }
  for (std::size_t index = 1U; index + 1U < route.size(); ++index) {
    if (route[index].transition == RouteKinematicTransition3D::kStopAndTurn) {
      // A stop-and-turn separates two continuous-motion legs.  The vehicle can
      // settle its acceleration while stopped, so braking acceleration from the
      // incoming leg must not constrain departure along the new tangent.
      continue;
    }
    const double previous_distance =
        route[index].station_m - route[index - 1U].station_m;
    const double next_distance = route[index + 1U].station_m - route[index].station_m;
    const double previous_sum = speeds[index - 1U] + speeds[index];
    const double next_sum = speeds[index] + speeds[index + 1U];
    if (!(previous_distance > kEpsilon) || !(next_distance > kEpsilon) ||
        !(previous_sum > kEpsilon) || !(next_sum > kEpsilon)) {
      continue;
    }
    const double previous_acceleration =
        (speeds[index] * speeds[index] - speeds[index - 1U] * speeds[index - 1U]) /
        (2.0 * previous_distance);
    const double next_acceleration =
        (speeds[index + 1U] * speeds[index + 1U] - speeds[index] * speeds[index]) /
        (2.0 * next_distance);
    const double transition_time =
        0.5 * (2.0 * previous_distance / previous_sum + 2.0 * next_distance / next_sum);
    if (next_acceleration > previous_acceleration + jerk * transition_time) {
      const double capped_next_acceleration =
          previous_acceleration + jerk * transition_time;
      speeds[index + 1U] = std::min(
          speeds[index + 1U],
          std::sqrt(std::max(0.0, speeds[index] * speeds[index] +
                                      2.0 * capped_next_acceleration * next_distance)));
    }
  }
}

} // namespace

RouteTimeParameterization3D parameterizeRouteTime3D(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const double unconstrained_speed_mps, const double constrained_speed_mps,
    const RouteEndpointSemantics3D endpoint_semantics,
    const MppiSpeedPolicyConfig& speed_policy, const mppi::DynamicsConfig& dynamics) {
  RouteTimeParameterization3D result;
  if (route.size() < 2U || !(unconstrained_speed_mps > kEpsilon) ||
      !(constrained_speed_mps > kEpsilon) ||
      !(speed_policy.cruise_speed_mps > kEpsilon) ||
      !(speed_policy.absolute_speed_limit_mps > kEpsilon) ||
      !(speed_policy.maximum_lateral_acceleration_mps2 > kEpsilon) ||
      !(dynamics.maximum_horizontal_acceleration_mps2 > 0.0F) ||
      !(dynamics.maximum_vertical_acceleration_mps2 > 0.0F)) {
    return result;
  }
  result.reference_speeds_mps.resize(route.size());
  const double base_speed =
      std::min({unconstrained_speed_mps, speed_policy.cruise_speed_mps,
                speed_policy.absolute_speed_limit_mps,
                static_cast<double>(dynamics.maximum_horizontal_speed_mps)});
  for (std::size_t index = 0U; index < route.size(); ++index) {
    if (!std::isfinite(route[index].station_m) ||
        (index > 0U && !(route[index].station_m > route[index - 1U].station_m))) {
      return {};
    }
    const double curvature = routeCurvature(route, index);
    const double curvature_limit =
        curvature > kEpsilon
            ? std::sqrt(speed_policy.maximum_lateral_acceleration_mps2 / curvature)
            : std::numeric_limits<double>::infinity();
    const double tangent_norm = vectorNorm(route[index].tangent);
    const double vertical_limit =
        tangent_norm > kEpsilon
            ? static_cast<double>(dynamics.maximum_vertical_speed_mps) /
                  std::max(kEpsilon, std::abs(route[index].tangent.z) / tangent_norm)
            : std::numeric_limits<double>::infinity();
    result.reference_speeds_mps[index] = std::min(
        {base_speed,
         constrainedLimit(route[index], constrained_spans, constrained_speed_mps),
         curvature_limit, vertical_limit});
    if (route[index].transition == RouteKinematicTransition3D::kStopAndTurn) {
      result.reference_speeds_mps[index] = 0.0;
    }
  }
  if (routeEndpointHasTerminalStop3D(endpoint_semantics)) {
    result.reference_speeds_mps.back() = 0.0;
  }
  for (std::size_t index = 1U; index < route.size(); ++index) {
    const double distance_m = route[index].station_m - route[index - 1U].station_m;
    const double acceleration =
        segmentAccelerationLimit(route[index - 1U], route[index], dynamics);
    result.reference_speeds_mps[index] =
        std::min(result.reference_speeds_mps[index],
                 std::sqrt(result.reference_speeds_mps[index - 1U] *
                               result.reference_speeds_mps[index - 1U] +
                           2.0 * acceleration * distance_m));
  }
  for (std::size_t index = route.size() - 1U; index > 0U; --index) {
    const double distance_m = route[index].station_m - route[index - 1U].station_m;
    const double acceleration =
        segmentAccelerationLimit(route[index - 1U], route[index], dynamics);
    result.reference_speeds_mps[index - 1U] =
        std::min(result.reference_speeds_mps[index - 1U],
                 std::sqrt(result.reference_speeds_mps[index] *
                               result.reference_speeds_mps[index] +
                           2.0 * acceleration * distance_m));
  }
  enforceJerkEnvelope(route, result.reference_speeds_mps, dynamics);
  for (std::size_t index = 1U; index < route.size(); ++index) {
    const double distance_m = route[index].station_m - route[index - 1U].station_m;
    const double speed_sum =
        result.reference_speeds_mps[index - 1U] + result.reference_speeds_mps[index];
    if (!(speed_sum > kEpsilon)) {
      return {};
    }
    result.travel_time_s += 2.0 * distance_m / speed_sum;
  }
  result.valid = std::isfinite(result.travel_time_s) && result.travel_time_s > 0.0;
  return result;
}

} // namespace drone_city_nav
