#include "drone_city_nav/flight_time_model_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
  return std::min(horizontal_limit, vertical_limit);
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

[[nodiscard]] double velocityTransitionDistance(const double lower_speed_mps,
                                                const double upper_speed_mps,
                                                const double acceleration_mps2,
                                                const double jerk_mps3) noexcept {
  return 0.5 * (lower_speed_mps + upper_speed_mps) *
         velocityTransitionTime(upper_speed_mps - lower_speed_mps, acceleration_mps2,
                                jerk_mps3);
}

[[nodiscard]] double reachableSpeed(const double lower_speed_mps,
                                    const double distance_m,
                                    const double upper_limit_mps,
                                    const double acceleration_mps2,
                                    const double jerk_mps3) noexcept {
  if (!(upper_limit_mps > lower_speed_mps) || !(distance_m > 0.0)) {
    return std::min(lower_speed_mps, upper_limit_mps);
  }
  if (velocityTransitionDistance(lower_speed_mps, upper_limit_mps, acceleration_mps2,
                                 jerk_mps3) <= distance_m) {
    return upper_limit_mps;
  }
  double lower = lower_speed_mps;
  double upper = upper_limit_mps;
  for (std::size_t iteration = 0U; iteration < 64U; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (velocityTransitionDistance(lower_speed_mps, middle, acceleration_mps2,
                                   jerk_mps3) <= distance_m) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  return lower;
}

[[nodiscard]] double
segmentTravelTime(const double distance_m, const double first_speed_mps,
                  const double second_speed_mps, const double speed_limit_mps,
                  const double acceleration_mps2, const double jerk_mps3) noexcept {
  if (!(distance_m > 0.0) || !(speed_limit_mps > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  const double slower = std::min(first_speed_mps, second_speed_mps);
  const double faster = std::max(first_speed_mps, second_speed_mps);
  if (faster - slower > kEpsilon) {
    const double transition_time =
        velocityTransitionTime(faster - slower, acceleration_mps2, jerk_mps3);
    const double transition_distance = 0.5 * (slower + faster) * transition_time;
    if (transition_distance > distance_m + 1.0e-7) {
      return std::numeric_limits<double>::infinity();
    }
    return transition_time + std::max(0.0, distance_m - transition_distance) / faster;
  }
  if (faster > kEpsilon) {
    return distance_m / faster;
  }

  // A short leg bounded by two full stops still has a valid accelerate/brake
  // profile even though both endpoint speeds are zero.
  double lower_peak = 0.0;
  double upper_peak = speed_limit_mps;
  for (std::size_t iteration = 0U; iteration < 64U; ++iteration) {
    const double middle = 0.5 * (lower_peak + upper_peak);
    const double transition_distance =
        2.0 * velocityTransitionDistance(0.0, middle, acceleration_mps2, jerk_mps3);
    if (transition_distance <= distance_m) {
      lower_peak = middle;
    } else {
      upper_peak = middle;
    }
  }
  if (!(lower_peak > kEpsilon)) {
    return std::numeric_limits<double>::infinity();
  }
  const double transition_time =
      velocityTransitionTime(lower_peak, acceleration_mps2, jerk_mps3);
  const double transition_distance =
      2.0 * velocityTransitionDistance(0.0, lower_peak, acceleration_mps2, jerk_mps3);
  return 2.0 * transition_time +
         std::max(0.0, distance_m - transition_distance) / lower_peak;
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
  return std::max(horizontal / model.maximum_horizontal_speed_mps,
                  vertical / model.maximum_vertical_speed_mps);
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

  result.reference_speeds_mps.resize(points.size());
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const double incoming_limit = index > 0U ? segment_speed_limits[index - 1U]
                                             : std::numeric_limits<double>::infinity();
    const double outgoing_limit = index < segment_count
                                      ? segment_speed_limits[index]
                                      : std::numeric_limits<double>::infinity();
    result.reference_speeds_mps[index] =
        std::min({speed_limits_mps[index], incoming_limit, outgoing_limit});
    if (stop_turn_flags[index] != 0U) {
      result.reference_speeds_mps[index] = 0.0;
    }
  }
  const double initial_along_route =
      std::max(0.0, initial_velocity.x * tangents.front().x +
                        initial_velocity.y * tangents.front().y +
                        initial_velocity.z * tangents.front().z);
  result.reference_speeds_mps.front() =
      std::min(result.reference_speeds_mps.front(), initial_along_route);
  if (terminal_stop) {
    result.reference_speeds_mps.back() = 0.0;
  }

  // Each transition uses a zero-acceleration S-curve at its boundaries. A few
  // symmetric passes propagate both acceleration and jerk reachability through
  // short adjacent segments and stop-turn boundaries.
  for (std::size_t pass = 0U; pass < 3U; ++pass) {
    for (std::size_t index = 0U; index < segment_count; ++index) {
      result.reference_speeds_mps[index + 1U] = std::min(
          result.reference_speeds_mps[index + 1U],
          reachableSpeed(result.reference_speeds_mps[index], lengths[index],
                         result.reference_speeds_mps[index + 1U],
                         acceleration_limits[index], model.maximum_control_jerk_mps3));
    }
    for (std::size_t index = segment_count; index > 0U; --index) {
      const std::size_t segment = index - 1U;
      result.reference_speeds_mps[segment] =
          std::min(result.reference_speeds_mps[segment],
                   reachableSpeed(result.reference_speeds_mps[index], lengths[segment],
                                  result.reference_speeds_mps[segment],
                                  acceleration_limits[segment],
                                  model.maximum_control_jerk_mps3));
    }
  }

  for (std::size_t index = 0U; index < segment_count; ++index) {
    const double segment_time = segmentTravelTime(
        lengths[index], result.reference_speeds_mps[index],
        result.reference_speeds_mps[index + 1U], segment_speed_limits[index],
        acceleration_limits[index], model.maximum_control_jerk_mps3);
    if (!std::isfinite(segment_time)) {
      return {};
    }
    result.translation_time_s += segment_time;
  }
  for (std::size_t index = 1U; index < segment_count; ++index) {
    if (stop_turn_flags[index] != 0U) {
      result.stationary_turn_time_s +=
          stationaryYawTurnTime(tangents[index - 1U], tangents[index], model);
    }
  }
  result.travel_time_s = result.translation_time_s + result.stationary_turn_time_s;
  result.valid = finitePositive(result.travel_time_s);
  return result;
}

} // namespace drone_city_nav
