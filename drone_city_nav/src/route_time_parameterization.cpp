#include "drone_city_nav/route_time_parameterization.hpp"

#include "drone_city_nav/flight_time_model_3d.hpp"

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

} // namespace

RouteTimeParameterization3D parameterizeRouteTime3D(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const double unconstrained_speed_mps, const double constrained_speed_mps,
    const RouteEndpointSemantics3D endpoint_semantics,
    const MppiSpeedPolicyConfig& speed_policy, const mppi::DynamicsConfig& dynamics,
    const std::optional<Vec3>& initial_velocity,
    const std::span<const double> tracking_speed_limits_mps) {
  RouteTimeParameterization3D result;
  const FlightTimeModel3D time_model{
      .maximum_horizontal_speed_mps =
          std::min({unconstrained_speed_mps, speed_policy.cruise_speed_mps,
                    speed_policy.absolute_speed_limit_mps,
                    static_cast<double>(dynamics.maximum_horizontal_speed_mps)}),
      .maximum_vertical_speed_mps =
          static_cast<double>(dynamics.maximum_vertical_speed_mps),
      .maximum_translational_speed_mps =
          static_cast<double>(dynamics.maximum_translational_speed_mps),
      .maximum_horizontal_acceleration_mps2 =
          static_cast<double>(dynamics.maximum_horizontal_acceleration_mps2),
      .maximum_vertical_acceleration_mps2 =
          static_cast<double>(dynamics.maximum_vertical_acceleration_mps2),
      .maximum_control_jerk_mps3 =
          static_cast<double>(dynamics.maximum_control_jerk_mps3),
      .maximum_yaw_acceleration_radps2 =
          static_cast<double>(dynamics.maximum_yaw_acceleration_radps2),
      .maximum_yaw_rate_radps = static_cast<double>(dynamics.maximum_yaw_rate_radps),
  };
  if (route.size() < 2U || !(unconstrained_speed_mps > kEpsilon) ||
      !(constrained_speed_mps > kEpsilon) ||
      !(speed_policy.cruise_speed_mps > kEpsilon) ||
      !(speed_policy.absolute_speed_limit_mps > kEpsilon) ||
      !(speed_policy.maximum_lateral_acceleration_mps2 > kEpsilon) ||
      (!tracking_speed_limits_mps.empty() &&
       tracking_speed_limits_mps.size() != route.size()) ||
      !time_model.valid()) {
    return result;
  }
  std::vector<Point3> points;
  std::vector<double> speed_limits;
  std::vector<std::uint8_t> stop_turn_flags;
  points.reserve(route.size());
  speed_limits.reserve(route.size());
  stop_turn_flags.reserve(route.size());
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
    const double tracking_limit = tracking_speed_limits_mps.empty()
                                      ? std::numeric_limits<double>::infinity()
                                      : tracking_speed_limits_mps[index];
    if (!tracking_speed_limits_mps.empty() &&
        (!std::isfinite(tracking_limit) || tracking_limit < 0.0)) {
      return {};
    }
    points.push_back(route[index].position);
    speed_limits.push_back(std::min(
        {time_model.maximum_horizontal_speed_mps,
         time_model.maximum_translational_speed_mps,
         constrainedLimit(route[index], constrained_spans, constrained_speed_mps),
         curvature_limit, tracking_limit}));
    stop_turn_flags.push_back(
        route[index].transition == RouteKinematicTransition3D::kStopAndTurn ? 1U : 0U);
  }
  const Vec3 effective_initial_velocity = initial_velocity.value_or(Vec3{
      .x = route.front().tangent.x * speed_limits.front(),
      .y = route.front().tangent.y * speed_limits.front(),
      .z = route.front().tangent.z * speed_limits.front(),
  });
  FlightPathTimeProfile3D profile = parameterizeFlightPathTime3D(
      points, speed_limits, stop_turn_flags, effective_initial_velocity,
      routeEndpointHasTerminalStop3D(endpoint_semantics), time_model);
  result.valid = profile.valid;
  result.travel_time_s = profile.travel_time_s;
  result.translation_time_s = profile.translation_time_s;
  result.stationary_turn_time_s = profile.stationary_turn_time_s;
  result.reference_speeds_mps = std::move(profile.reference_speeds_mps);
  return result;
}

} // namespace drone_city_nav
