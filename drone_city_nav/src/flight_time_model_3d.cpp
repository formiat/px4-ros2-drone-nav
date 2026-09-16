#include "drone_city_nav/flight_time_model_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kEpsilon{1.0e-9};

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] Vec3 direction(const Point3& first, const Point3& second) noexcept {
  const double length = distance3D(first, second);
  return length > kEpsilon
             ? Vec3{(second.x - first.x) / length, (second.y - first.y) / length,
                    (second.z - first.z) / length}
             : Vec3{};
}

[[nodiscard]] std::optional<Vec3> normalized(const Vec3& value) noexcept {
  const double norm = std::hypot(std::hypot(value.x, value.y), value.z);
  if (!(norm > kEpsilon) || !std::isfinite(norm)) {
    return std::nullopt;
  }
  return Vec3{value.x / norm, value.y / norm, value.z / norm};
}

[[nodiscard]] double scalarSpeedLimit(const Vec3& tangent,
                                      const FlightTimeModel3D& model) noexcept {
  const double horizontal_share = std::hypot(tangent.x, tangent.y);
  const double vertical_share = std::abs(tangent.z);
  const double horizontal_limit =
      horizontal_share > kEpsilon
          ? model.maximum_horizontal_speed_mps / horizontal_share
          : std::numeric_limits<double>::infinity();
  const double vertical_limit = vertical_share > kEpsilon
                                    ? model.maximum_vertical_speed_mps / vertical_share
                                    : std::numeric_limits<double>::infinity();
  return std::min(
      {horizontal_limit, vertical_limit, model.maximum_translational_speed_mps});
}

[[nodiscard]] double scalarAccelerationLimit(const Vec3& tangent,
                                             const FlightTimeModel3D& model) noexcept {
  const double horizontal_share = std::hypot(tangent.x, tangent.y);
  const double vertical_share = std::abs(tangent.z);
  const double horizontal_limit =
      horizontal_share > kEpsilon
          ? model.maximum_horizontal_acceleration_mps2 / horizontal_share
          : std::numeric_limits<double>::infinity();
  const double vertical_limit =
      vertical_share > kEpsilon
          ? model.maximum_vertical_acceleration_mps2 / vertical_share
          : std::numeric_limits<double>::infinity();
  return std::min(horizontal_limit, vertical_limit);
}

// Duration of one jerk-limited speed change of `delta_speed_mps` that starts
// and ends at zero acceleration: a triangular acceleration pulse when the
// change is too small to reach `acceleration_mps2`, an S-curve otherwise.
[[nodiscard]] double velocityTransitionTime(const double delta_speed_mps,
                                            const double acceleration_mps2,
                                            const double jerk_mps3) noexcept {
  if (!(delta_speed_mps > kEpsilon)) {
    return 0.0;
  }
  const double triangular_delta = acceleration_mps2 * acceleration_mps2 / jerk_mps3;
  if (delta_speed_mps <= triangular_delta) {
    return 2.0 * std::sqrt(delta_speed_mps / jerk_mps3);
  }
  return delta_speed_mps / acceleration_mps2 + acceleration_mps2 / jerk_mps3;
}

// Time a jerk-limited transition takes over the constant-acceleration one
// for the same speed change: the acceleration ramps the jerk limit adds.
[[nodiscard]] double jerkRampAllowanceS(const double delta_speed_mps,
                                        const double acceleration_mps2,
                                        const double jerk_mps3) noexcept {
  if (!(delta_speed_mps > kEpsilon) || !(acceleration_mps2 > kEpsilon)) {
    return 0.0;
  }
  return std::max(
      0.0, velocityTransitionTime(delta_speed_mps, acceleration_mps2, jerk_mps3) -
               delta_speed_mps / acceleration_mps2);
}

// Time along one segment under constant acceleration bounded by
// `acceleration_mps2`: the speed changes as early (accelerating) or as late
// (braking) as the bound allows and the segment cruises for the rest. The
// endpoint speeds are already reachable within the segment.
[[nodiscard]] double segmentTravelTime(const double distance_m,
                                       const double first_speed_mps,
                                       const double second_speed_mps,
                                       const double speed_limit_mps,
                                       const double acceleration_mps2) noexcept {
  if (!(distance_m > 0.0) || !(speed_limit_mps > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  const double slower = std::min(first_speed_mps, second_speed_mps);
  const double faster = std::max(first_speed_mps, second_speed_mps);
  if (faster - slower > kEpsilon) {
    const double transition_distance =
        (faster * faster - slower * slower) / (2.0 * acceleration_mps2);
    if (transition_distance > distance_m + 1.0e-7) {
      return std::numeric_limits<double>::infinity();
    }
    return (faster - slower) / acceleration_mps2 +
           std::max(0.0, distance_m - transition_distance) / faster;
  }
  if (faster > kEpsilon) {
    return distance_m / faster;
  }
  // A leg bounded by two full stops accelerates to a peak and brakes again.
  const double peak_speed_mps =
      std::min(speed_limit_mps, std::sqrt(acceleration_mps2 * distance_m));
  if (!(peak_speed_mps > kEpsilon)) {
    return std::numeric_limits<double>::infinity();
  }
  const double transition_distance =
      peak_speed_mps * peak_speed_mps / acceleration_mps2;
  return 2.0 * peak_speed_mps / acceleration_mps2 +
         std::max(0.0, distance_m - transition_distance) / peak_speed_mps;
}

[[nodiscard]] double stationaryYawTurnTime(const Vec3& incoming, const Vec3& outgoing,
                                           const FlightTimeModel3D& model) noexcept {
  const double incoming_horizontal = std::hypot(incoming.x, incoming.y);
  const double outgoing_horizontal = std::hypot(outgoing.x, outgoing.y);
  if (!(incoming_horizontal > kEpsilon) || !(outgoing_horizontal > kEpsilon)) {
    return 0.0;
  }
  const double cosine = std::clamp((incoming.x * outgoing.x + incoming.y * outgoing.y) /
                                       (incoming_horizontal * outgoing_horizontal),
                                   -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (!(angle > kEpsilon)) {
    return 0.0;
  }
  const double acceleration = model.maximum_yaw_acceleration_radps2;
  const double rate = model.maximum_yaw_rate_radps;
  const double acceleration_angle = rate * rate / acceleration;
  if (angle <= acceleration_angle) {
    return 2.0 * std::sqrt(angle / acceleration);
  }
  return 2.0 * rate / acceleration + (angle - acceleration_angle) / rate;
}

} // namespace

bool FlightTimeModel3D::valid() const noexcept {
  return finitePositive(maximum_horizontal_speed_mps) &&
         finitePositive(maximum_vertical_speed_mps) &&
         finitePositive(maximum_translational_speed_mps) &&
         finitePositive(maximum_horizontal_acceleration_mps2) &&
         finitePositive(maximum_vertical_acceleration_mps2) &&
         finitePositive(maximum_control_jerk_mps3) &&
         finitePositive(maximum_yaw_acceleration_radps2) &&
         finitePositive(maximum_yaw_rate_radps);
}

double minimumFlightTranslationTime3D(const Point3& first, const Point3& second,
                                      const FlightTimeModel3D& model) noexcept {
  if (!model.valid()) {
    return std::numeric_limits<double>::infinity();
  }
  const double horizontal = std::hypot(second.x - first.x, second.y - first.y);
  const double vertical = std::abs(second.z - first.z);
  const double translation = distance3D(first, second);
  return std::max({horizontal / model.maximum_horizontal_speed_mps,
                   vertical / model.maximum_vertical_speed_mps,
                   translation / model.maximum_translational_speed_mps});
}

bool requiresFlightStopAndTurn3D(const Vec3& incoming, const Vec3& outgoing,
                                 const double minimum_continuous_alignment) noexcept {
  const std::optional<Vec3> normalized_incoming = normalized(incoming);
  const std::optional<Vec3> normalized_outgoing = normalized(outgoing);
  if (!normalized_incoming.has_value() || !normalized_outgoing.has_value() ||
      !std::isfinite(minimum_continuous_alignment) ||
      minimum_continuous_alignment < -1.0 || minimum_continuous_alignment > 1.0) {
    return false;
  }
  const double alignment = normalized_incoming->x * normalized_outgoing->x +
                           normalized_incoming->y * normalized_outgoing->y +
                           normalized_incoming->z * normalized_outgoing->z;
  return alignment < minimum_continuous_alignment;
}

double estimatedFlightRestTransitionDelay3D(const Vec3& tangent,
                                            const FlightTimeModel3D& model) noexcept {
  const std::optional<Vec3> normalized_tangent = normalized(tangent);
  if (!model.valid() || !normalized_tangent.has_value()) {
    return std::numeric_limits<double>::infinity();
  }
  const double speed_limit = scalarSpeedLimit(*normalized_tangent, model);
  const double acceleration_limit = scalarAccelerationLimit(*normalized_tangent, model);
  if (!finitePositive(speed_limit) || !finitePositive(acceleration_limit)) {
    return std::numeric_limits<double>::infinity();
  }
  // The jerk-limited transition travels at the mean endpoint speed. Against
  // an instantaneous full-speed translation lower bound, half of its duration
  // is therefore the incremental delay.
  return 0.5 * velocityTransitionTime(speed_limit, acceleration_limit,
                                      model.maximum_control_jerk_mps3);
}

double estimatedFlightStopAndTurnDelay3D(const Vec3& incoming, const Vec3& outgoing,
                                         const FlightTimeModel3D& model) noexcept {
  const std::optional<Vec3> normalized_incoming = normalized(incoming);
  const std::optional<Vec3> normalized_outgoing = normalized(outgoing);
  if (!model.valid() || !normalized_incoming.has_value() ||
      !normalized_outgoing.has_value()) {
    return std::numeric_limits<double>::infinity();
  }
  const double braking_delay =
      estimatedFlightRestTransitionDelay3D(*normalized_incoming, model);
  const double restart_delay =
      estimatedFlightRestTransitionDelay3D(*normalized_outgoing, model);
  const double yaw_time =
      stationaryYawTurnTime(*normalized_incoming, *normalized_outgoing, model);
  return braking_delay + yaw_time + restart_delay;
}

FlightPathTimeProfile3D
parameterizeFlightPathTime3D(const std::span<const Point3> points,
                             const std::span<const double> speed_limits_mps,
                             const std::span<const std::uint8_t> stop_turn_flags,
                             const Vec3& initial_velocity, const bool terminal_stop,
                             const FlightTimeModel3D& model) {
  FlightPathTimeProfile3D result;
  if (!model.valid() || points.size() < 2U ||
      speed_limits_mps.size() != points.size() ||
      stop_turn_flags.size() != points.size() || !std::isfinite(initial_velocity.x) ||
      !std::isfinite(initial_velocity.y) || !std::isfinite(initial_velocity.z)) {
    return result;
  }

  const std::size_t segment_count = points.size() - 1U;
  if (std::ranges::any_of(speed_limits_mps, [](const double speed_limit_mps) {
        return !std::isfinite(speed_limit_mps) || speed_limit_mps < 0.0;
      })) {
    return result;
  }
  std::vector<double> lengths(segment_count);
  std::vector<Vec3> tangents(segment_count);
  std::vector<double> acceleration_limits(segment_count);
  std::vector<double> segment_speed_limits(segment_count);
  for (std::size_t index = 0U; index < segment_count; ++index) {
    lengths[index] = distance3D(points[index], points[index + 1U]);
    tangents[index] = direction(points[index], points[index + 1U]);
    acceleration_limits[index] = scalarAccelerationLimit(tangents[index], model);
    segment_speed_limits[index] =
        std::min({scalarSpeedLimit(tangents[index], model), speed_limits_mps[index],
                  speed_limits_mps[index + 1U]});
    if (!finitePositive(lengths[index]) ||
        !finitePositive(acceleration_limits[index]) ||
        !finitePositive(segment_speed_limits[index])) {
      return {};
    }
  }

  std::vector<double>& speeds = result.reference_speeds_mps;
  speeds.resize(points.size());
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const double incoming_limit = index > 0U ? segment_speed_limits[index - 1U]
                                             : std::numeric_limits<double>::infinity();
    const double outgoing_limit = index < segment_count
                                      ? segment_speed_limits[index]
                                      : std::numeric_limits<double>::infinity();
    speeds[index] = std::min({speed_limits_mps[index], incoming_limit, outgoing_limit});
    if (stop_turn_flags[index] != 0U) {
      speeds[index] = 0.0;
    }
  }
  const double initial_along_route =
      std::max(0.0, initial_velocity.x * tangents.front().x +
                        initial_velocity.y * tangents.front().y +
                        initial_velocity.z * tangents.front().z);
  speeds.front() = std::min(speeds.front(), initial_along_route);
  if (terminal_stop) {
    speeds.back() = 0.0;
  }

  // The profile carries its acceleration across points: a speed is bounded by
  // what the preceding point can accelerate to and by what the following
  // point can be braked to, over the whole path length between them. Treating
  // every sample as a transition that starts and ends at rest acceleration
  // priced a dense route by its sample spacing, not by its geometry — a
  // straight metre sampled every half metre cost a quarter more than the same
  // metre sampled once.
  for (std::size_t index = 0U; index < segment_count; ++index) {
    speeds[index + 1U] =
        std::min(speeds[index + 1U],
                 std::sqrt(speeds[index] * speeds[index] +
                           2.0 * acceleration_limits[index] * lengths[index]));
  }
  for (std::size_t index = segment_count; index > 0U; --index) {
    const std::size_t segment = index - 1U;
    speeds[segment] =
        std::min(speeds[segment],
                 std::sqrt(speeds[index] * speeds[index] +
                           2.0 * acceleration_limits[segment] * lengths[segment]));
  }

  // Segment times under the acceleration bound, then the jerk ramps once per
  // acceleration phase: a run of segments that keeps accelerating, or keeps
  // braking, is one jerk-limited transition however many samples it spans,
  // and its ramps are charged at the point where the phase ends.
  result.arrival_times_s.assign(points.size(), 0.0);
  result.departure_times_s.assign(points.size(), 0.0);
  int phase_sign{0};
  double phase_start_speed_mps{0.0};
  double phase_acceleration_mps2{0.0};
  const auto closePhase = [&](const double end_speed_mps) {
    const double allowance_s =
        jerkRampAllowanceS(std::abs(end_speed_mps - phase_start_speed_mps),
                           phase_acceleration_mps2, model.maximum_control_jerk_mps3);
    result.translation_time_s += allowance_s;
    phase_sign = 0;
    phase_acceleration_mps2 = 0.0;
    return allowance_s;
  };
  for (std::size_t index = 0U; index < segment_count; ++index) {
    const double delta_speed_mps = speeds[index + 1U] - speeds[index];
    const int sign = delta_speed_mps > kEpsilon    ? 1
                     : delta_speed_mps < -kEpsilon ? -1
                                                   : 0;
    if (sign != phase_sign) {
      if (phase_sign != 0) {
        result.arrival_times_s[index] += closePhase(speeds[index]);
      }
      if (sign != 0) {
        phase_sign = sign;
        phase_start_speed_mps = speeds[index];
      }
    }
    if (sign != 0) {
      // The transition within a segment runs at the segment's acceleration
      // limit and cruises for the rest, so the phase's ramps are those of the
      // steepest limit it touched.
      phase_acceleration_mps2 =
          std::max(phase_acceleration_mps2, acceleration_limits[index]);
    }
    double stationary_turn_time_s{0.0};
    if (index > 0U && stop_turn_flags[index] != 0U) {
      stationary_turn_time_s =
          stationaryYawTurnTime(tangents[index - 1U], tangents[index], model);
      if (!std::isfinite(stationary_turn_time_s) || stationary_turn_time_s < 0.0) {
        return {};
      }
    }
    result.stationary_turn_time_s += stationary_turn_time_s;
    result.departure_times_s[index] =
        result.arrival_times_s[index] + stationary_turn_time_s;
    const double segment_time =
        segmentTravelTime(lengths[index], speeds[index], speeds[index + 1U],
                          segment_speed_limits[index], acceleration_limits[index]);
    if (!std::isfinite(segment_time)) {
      return {};
    }
    result.translation_time_s += segment_time;
    result.arrival_times_s[index + 1U] = result.departure_times_s[index] + segment_time;
  }
  if (phase_sign != 0) {
    result.arrival_times_s.back() += closePhase(speeds.back());
  }
  result.departure_times_s.back() = result.arrival_times_s.back();
  result.travel_time_s = result.arrival_times_s.back();
  result.valid = finitePositive(result.travel_time_s);
  return result;
}

} // namespace drone_city_nav
