#include "drone_city_nav/route_3d.hpp"

#include "drone_city_nav/flight_time_model_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <string_view>
#include <tuple>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kFrozenPrefixMaximumStitchSeparationM{0.05};
constexpr double kFrozenPrefixMinimumTangentAlignment{0.995};

enum class FutureStitchJoinPolicy : std::uint8_t {
  kRequireContinuousTangent,
  kAllowStopAndTurn,
};

void hashByte(std::uint64_t& hash, const std::uint8_t value) noexcept {
  hash ^= value;
  hash *= kFnvPrime;
}

void hashSigned(std::uint64_t& hash, const std::int64_t value) noexcept {
  const auto bits = static_cast<std::uint64_t>(value);
  for (unsigned shift = 0U; shift < 64U; shift += 8U) {
    hashByte(hash, static_cast<std::uint8_t>((bits >> shift) & 0xffU));
  }
}

void hashText(std::uint64_t& hash, const std::string_view text) noexcept {
  for (const char character : text) {
    hashByte(hash, static_cast<std::uint8_t>(character));
  }
  hashByte(hash, 0U);
}

[[nodiscard]] Vec3 normalized(const Point3& from, const Point3& to) noexcept {
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dz = to.z - from.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  return length > 1.0e-9 ? Vec3{dx / length, dy / length, dz / length} : Vec3{};
}

[[nodiscard]] double vectorNorm(const Vec3& value) noexcept {
  return std::hypot(std::hypot(value.x, value.y), value.z);
}

[[nodiscard]] Vec3 normalized(const Vec3& value) noexcept {
  const double length = vectorNorm(value);
  return length > 1.0e-9 ? Vec3{value.x / length, value.y / length, value.z / length}
                         : Vec3{};
}

[[nodiscard]] Point3 translated(const Point3& point, const Vec3& direction,
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

[[nodiscard]] Point3 cubicBezierPoint(const CubicBezierCurve3D& curve,
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

[[nodiscard]] RouteSample3D sampleAtStation(const std::span<const RouteSample3D> route,
                                            const double station_m) {
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

[[nodiscard]] const RouteEnvelopeSample&
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

void reportPrefixStatus(FrozenRoutePrefixStatus3D* const status,
                        const FrozenRoutePrefixStatus3D value) noexcept {
  if (status != nullptr) {
    *status = value;
  }
}

[[nodiscard]] std::optional<FrozenRoutePrefix3D> materializeFrozenRoutePrefixAtStations(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> successor_route,
    const RouteProjection3D& active_projection, const double active_stitch_station_m,
    const double successor_begin_station_m, const double successor_stitch_station_m,
    const FutureStitchJoinPolicy join_policy,
    FrozenRoutePrefixStatus3D* const status = nullptr) noexcept {
  if (!active_projection.valid) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kProjectionInvalid);
    return std::nullopt;
  }
  if (!std::isfinite(active_stitch_station_m) ||
      !std::isfinite(successor_begin_station_m) ||
      !std::isfinite(successor_stitch_station_m) ||
      successor_stitch_station_m < successor_begin_station_m) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kInvalidInput);
    return std::nullopt;
  }
  if (!(active_stitch_station_m > active_projection.station_m)) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kStitchNotAhead);
    return std::nullopt;
  }
  if (active_stitch_station_m > active_route.back().station_m) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kStitchBeyondRoute);
    return std::nullopt;
  }
  if (successor_stitch_station_m > successor_route.back().station_m) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kSuccessorTooShort);
    return std::nullopt;
  }
  const RouteSample3D active_stitch =
      sampleAtStation(active_route, active_stitch_station_m);
  const RouteSample3D successor_stitch =
      sampleAtStation(successor_route, successor_stitch_station_m);
  const double active_norm =
      std::hypot(std::hypot(active_stitch.tangent.x, active_stitch.tangent.y),
                 active_stitch.tangent.z);
  const double successor_norm =
      std::hypot(std::hypot(successor_stitch.tangent.x, successor_stitch.tangent.y),
                 successor_stitch.tangent.z);
  const double tangent_alignment =
      active_norm > 1.0e-9 && successor_norm > 1.0e-9
          ? (active_stitch.tangent.x * successor_stitch.tangent.x +
             active_stitch.tangent.y * successor_stitch.tangent.y +
             active_stitch.tangent.z * successor_stitch.tangent.z) /
                (active_norm * successor_norm)
          : -1.0;
  if (distance3D(active_stitch.position, successor_stitch.position) >
      kFrozenPrefixMaximumStitchSeparationM) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kStitchSeparation);
    return std::nullopt;
  }
  if (join_policy == FutureStitchJoinPolicy::kRequireContinuousTangent &&
      tangent_alignment < kFrozenPrefixMinimumTangentAlignment) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kTangentDiscontinuous);
    return std::nullopt;
  }

  FrozenRoutePrefix3D result{
      .route = {},
      .active_begin_station_m = active_projection.station_m,
      .stitch_station_m = active_stitch_station_m,
      .successor_begin_station_m = successor_begin_station_m,
      .successor_stitch_station_m = successor_stitch_station_m,
  };
  const auto append = [&result, &active_projection](RouteSample3D sample) noexcept {
    sample.station_m = std::max(0.0, sample.station_m - active_projection.station_m);
    if (result.route.empty() ||
        sample.station_m > result.route.back().station_m + 1.0e-6) {
      result.route.push_back(sample);
    }
  };
  append(sampleAtStation(active_route, active_projection.station_m));
  for (const RouteSample3D& sample : active_route) {
    if (sample.station_m > active_projection.station_m &&
        sample.station_m < active_stitch_station_m) {
      append(sample);
    }
  }
  append(active_stitch);
  const double successor_offset = active_stitch_station_m - successor_stitch_station_m;
  for (const RouteSample3D& source : successor_route) {
    if (source.station_m <= successor_stitch_station_m) {
      continue;
    }
    RouteSample3D sample = source;
    sample.station_m += successor_offset - active_projection.station_m;
    if (sample.station_m > result.route.back().station_m + 1.0e-6) {
      result.route.push_back(sample);
    }
  }
  if (!result.valid()) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kKinematicsInvalid);
    return std::nullopt;
  }
  reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kMaterialized);
  return result;
}

} // namespace

RouteSample3D sampleRoute3DAtStation(const std::span<const RouteSample3D> route,
                                     const double station_m) noexcept {
  return sampleAtStation(route, station_m);
}

bool FrozenRoutePrefix3D::valid() const noexcept {
  return route.size() >= 2U && std::isfinite(active_begin_station_m) &&
         std::isfinite(stitch_station_m) && std::isfinite(successor_begin_station_m) &&
         std::isfinite(successor_stitch_station_m) &&
         stitch_station_m > active_begin_station_m &&
         successor_stitch_station_m >= successor_begin_station_m;
}

bool futureRouteConnectorConfig3DValid(
    const FutureRouteConnectorConfig3D& config) noexcept {
  return std::isfinite(config.tangent_departure_length_m) &&
         config.tangent_departure_length_m > 0.0 &&
         std::isfinite(config.successor_join_station_m) &&
         config.successor_join_station_m > 0.0 &&
         std::isfinite(config.curve_control_distance_m) &&
         config.curve_control_distance_m > 0.0 && config.curve_samples >= 2U &&
         config.curve_samples <= 256U &&
         std::isfinite(config.minimum_continuous_turn_alignment) &&
         config.minimum_continuous_turn_alignment >= -1.0 &&
         config.minimum_continuous_turn_alignment <= 1.0;
}

std::optional<FrozenRoutePrefix3D>
materializeFrozenRoutePrefix3D(const std::span<const RouteSample3D> active_route,
                               const std::span<const RouteSample3D> successor_route,
                               const Point3& current_position,
                               const double frozen_prefix_length_m) noexcept {
  if (active_route.size() < 2U || successor_route.size() < 2U ||
      !std::isfinite(frozen_prefix_length_m) || !(frozen_prefix_length_m > 0.0)) {
    return std::nullopt;
  }
  const RouteProjection3D active_projection =
      projectOntoRoute3D(active_route, current_position);
  if (!active_projection.valid) {
    return std::nullopt;
  }
  const double stitch_station = active_projection.station_m + frozen_prefix_length_m;
  if (stitch_station > active_route.back().station_m) {
    return std::nullopt;
  }
  const RouteSample3D active_stitch = sampleAtStation(active_route, stitch_station);
  const RouteProjection3D successor_projection =
      projectOntoRoute3D(successor_route, current_position);
  const bool successor_starts_at_stitch =
      distance3D(successor_route.front().position, active_stitch.position) <=
      kFrozenPrefixMaximumStitchSeparationM;
  const double successor_stitch_station =
      successor_starts_at_stitch
          ? successor_route.front().station_m
          : successor_projection.station_m + frozen_prefix_length_m;
  if ((!successor_starts_at_stitch && !successor_projection.valid) ||
      successor_stitch_station > successor_route.back().station_m) {
    return std::nullopt;
  }
  return materializeFrozenRoutePrefixAtStations(
      active_route, successor_route, active_projection, stitch_station,
      successor_starts_at_stitch ? successor_route.front().station_m
                                 : successor_projection.station_m,
      successor_stitch_station, FutureStitchJoinPolicy::kRequireContinuousTangent);
}

std::optional<FrozenRoutePrefix3D> materializeFrozenRoutePrefixAtStation3D(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> successor_route,
    const Point3& current_position, const double active_stitch_station_m,
    FrozenRoutePrefixStatus3D* const status) noexcept {
  if (active_route.size() < 2U || successor_route.size() < 2U ||
      !std::isfinite(active_stitch_station_m)) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kInvalidInput);
    return std::nullopt;
  }
  const RouteProjection3D active_projection =
      projectOntoRoute3D(active_route, current_position);
  if (!active_projection.valid) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kProjectionInvalid);
    return std::nullopt;
  }
  if (active_stitch_station_m > active_route.back().station_m) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kStitchBeyondRoute);
    return std::nullopt;
  }
  return materializeFrozenRoutePrefixAtStations(
      active_route, successor_route, active_projection, active_stitch_station_m,
      successor_route.front().station_m, successor_route.front().station_m,
      FutureStitchJoinPolicy::kRequireContinuousTangent, status);
}

namespace {

// One attempt at the connector, joining the successor at the given station.
[[nodiscard]] std::optional<FrozenRoutePrefix3D> materializeTangentContinuousPrefix(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> successor_route,
    const Point3& current_position, const double active_stitch_station_m,
    const FutureRouteConnectorConfig3D& config, const double join_station_m,
    FrozenRoutePrefixStatus3D* const status) noexcept {
  if (active_route.size() < 2U || successor_route.size() < 2U ||
      !std::isfinite(active_stitch_station_m) ||
      !futureRouteConnectorConfig3DValid(config)) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kInvalidInput);
    return std::nullopt;
  }
  const RouteProjection3D active_projection =
      projectOntoRoute3D(active_route, current_position);
  if (!active_projection.valid) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kProjectionInvalid);
    return std::nullopt;
  }
  if (!(active_stitch_station_m > active_projection.station_m)) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kStitchNotAhead);
    return std::nullopt;
  }
  if (active_stitch_station_m > active_route.back().station_m) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kStitchBeyondRoute);
    return std::nullopt;
  }
  if (distance3D(sampleAtStation(active_route, active_stitch_station_m).position,
                 successor_route.front().position) >
      kFrozenPrefixMaximumStitchSeparationM) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kStitchSeparation);
    return std::nullopt;
  }
  if (successor_route.back().station_m < join_station_m) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kSuccessorTooShort);
    return std::nullopt;
  }

  const RouteSample3D active_stitch =
      sampleAtStation(active_route, active_stitch_station_m);
  const RouteSample3D successor_join = sampleAtStation(successor_route, join_station_m);
  const Vec3 active_tangent = normalized(active_stitch.tangent);
  const Vec3 successor_tangent = normalized(successor_join.tangent);
  if (!(vectorNorm(active_tangent) > 0.0) || !(vectorNorm(successor_tangent) > 0.0)) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kKinematicsInvalid);
    return std::nullopt;
  }

  FrozenRoutePrefix3D result{
      .route = {},
      .active_begin_station_m = active_projection.station_m,
      .stitch_station_m = active_stitch_station_m,
      .successor_begin_station_m = successor_route.front().station_m,
      .successor_stitch_station_m = join_station_m,
  };
  const auto append = [&result](RouteSample3D sample) noexcept {
    if (!result.route.empty()) {
      const double segment_length_m =
          distance3D(result.route.back().position, sample.position);
      if (!(segment_length_m > 1.0e-6)) {
        return;
      }
      sample.station_m = result.route.back().station_m + segment_length_m;
    } else {
      sample.station_m = 0.0;
    }
    sample.transition = RouteKinematicTransition3D::kContinuous;
    result.route.push_back(sample);
  };

  append(sampleAtStation(active_route, active_projection.station_m));
  for (const RouteSample3D& sample : active_route) {
    if (sample.station_m > active_projection.station_m &&
        sample.station_m < active_stitch_station_m) {
      append(sample);
    }
  }
  append(active_stitch);
  // The connector runs from the stitch through the departure and the curve to
  // the first successor sample past the join; only that stretch has to be
  // flown without a stop. A corner farther along the successor is the
  // successor's own, flown as a stop-and-turn like any route's corner, and
  // refusing the stitch for it refused four stitched replacements in five.
  const std::size_t connector_begin_index = result.route.size() - 1U;

  const double connector_speed_mps =
      std::min(active_stitch.reference_speed_mps, successor_join.reference_speed_mps);
  const auto connector_risk_tier = static_cast<RouteRiskTier3D>(
      std::max(static_cast<std::uint8_t>(active_stitch.required_risk_tier),
               static_cast<std::uint8_t>(successor_join.required_risk_tier)));
  const Point3 departure = translated(active_stitch.position, active_tangent,
                                      config.tangent_departure_length_m);
  append(RouteSample3D{.position = departure,
                       .tangent = active_tangent,
                       .reference_speed_mps = connector_speed_mps,
                       .required_risk_tier = connector_risk_tier});
  const Point3 first_control =
      translated(departure, active_tangent, config.curve_control_distance_m);
  const Point3 second_control = translated(successor_join.position, successor_tangent,
                                           -config.curve_control_distance_m);
  const CubicBezierCurve3D connector_curve{
      .start = departure,
      .start_control = first_control,
      .end_control = second_control,
      .end = successor_join.position,
  };
  for (std::size_t index = 1U; index <= config.curve_samples; ++index) {
    const double parameter =
        static_cast<double>(index) / static_cast<double>(config.curve_samples);
    append(RouteSample3D{
        .position = cubicBezierPoint(connector_curve, parameter),
        .tangent = {},
        .reference_speed_mps = connector_speed_mps,
        .required_risk_tier = connector_risk_tier,
    });
  }
  const std::size_t connector_end_index = result.route.size();
  for (const RouteSample3D& source : successor_route) {
    if (source.station_m > join_station_m + 1.0e-6) {
      append(source);
    }
  }
  if (!result.valid() || !canonicalizeRouteKinematics3D(
                             result.route, config.minimum_continuous_turn_alignment)) {
    reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kKinematicsInvalid);
    return std::nullopt;
  }
  const std::size_t connector_last_index =
      std::min(connector_end_index, result.route.size() - 1U);
  for (std::size_t index = connector_begin_index; index <= connector_last_index;
       ++index) {
    if (result.route[index].transition == RouteKinematicTransition3D::kStopAndTurn) {
      reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kConnectorStopTurn);
      return std::nullopt;
    }
  }
  reportPrefixStatus(status, FrozenRoutePrefixStatus3D::kMaterialized);
  return result;
}

} // namespace

const char*
frozenRoutePrefixStatus3DName(const FrozenRoutePrefixStatus3D status) noexcept {
  switch (status) {
    case FrozenRoutePrefixStatus3D::kNotAttempted:
      return "not_attempted";
    case FrozenRoutePrefixStatus3D::kMaterialized:
      return "materialized";
    case FrozenRoutePrefixStatus3D::kInvalidInput:
      return "invalid_input";
    case FrozenRoutePrefixStatus3D::kProjectionInvalid:
      return "projection_invalid";
    case FrozenRoutePrefixStatus3D::kStitchNotAhead:
      return "stitch_not_ahead";
    case FrozenRoutePrefixStatus3D::kStitchBeyondRoute:
      return "stitch_beyond_route";
    case FrozenRoutePrefixStatus3D::kStitchSeparation:
      return "stitch_separation";
    case FrozenRoutePrefixStatus3D::kTangentDiscontinuous:
      return "tangent_discontinuous";
    case FrozenRoutePrefixStatus3D::kSuccessorTooShort:
      return "successor_too_short";
    case FrozenRoutePrefixStatus3D::kKinematicsInvalid:
      return "kinematics_invalid";
    case FrozenRoutePrefixStatus3D::kConnectorStopTurn:
      return "connector_stop_turn";
    case FrozenRoutePrefixStatus3D::kSuccessorContractBeforeJoin:
      return "successor_contract_before_join";
  }
  return "unknown";
}

std::optional<FrozenRoutePrefix3D> materializeTangentContinuousRoutePrefixAtStation3D(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> successor_route,
    const Point3& current_position, const double active_stitch_station_m,
    const FutureRouteConnectorConfig3D& config,
    FrozenRoutePrefixStatus3D* const status) noexcept {
  // Where the connector meets the successor is a choice, not a constant. The
  // successor leaves the stitch on whatever heading its first lattice edge
  // takes, and a curve onto a heading two metres in that turns away again
  // cannot be flown without a stop; joining further along meets the successor
  // where it has settled onto its direction. Measured over one urban flight,
  // twelve of twenty refused stitched replacements were refused for exactly
  // that, each one retiring the search and searching the same stitch again
  // while the vehicle flew on toward the block. The configured station is
  // tried first, so an ordinary stitch keeps the shortest connector it has
  // always had.
  constexpr std::size_t kJoinStationAttempts{3U};
  FrozenRoutePrefixStatus3D attempt_status{FrozenRoutePrefixStatus3D::kNotAttempted};
  for (std::size_t attempt = 1U; attempt <= kJoinStationAttempts; ++attempt) {
    const double join_station_m =
        config.successor_join_station_m * static_cast<double>(attempt);
    if (successor_route.empty() || join_station_m > successor_route.back().station_m) {
      break;
    }
    if (std::optional<FrozenRoutePrefix3D> prefix = materializeTangentContinuousPrefix(
            active_route, successor_route, current_position, active_stitch_station_m,
            config, join_station_m, &attempt_status)) {
      reportPrefixStatus(status, attempt_status);
      return prefix;
    }
    // Only a connector the successor's own geometry refuses is worth another
    // join station; everything else is the stitch itself and will not change.
    if (attempt_status != FrozenRoutePrefixStatus3D::kConnectorStopTurn &&
        attempt_status != FrozenRoutePrefixStatus3D::kKinematicsInvalid) {
      break;
    }
  }
  reportPrefixStatus(status, attempt_status);
  return std::nullopt;
}

std::optional<FrozenRoutePrefix3D>
materializeRouteHandoffAtStation3D(const std::span<const RouteSample3D> active_route,
                                   const std::span<const RouteSample3D> successor_route,
                                   const Point3& current_position,
                                   const double active_stitch_station_m) noexcept {
  if (active_route.size() < 2U || successor_route.size() < 2U ||
      !std::isfinite(active_stitch_station_m)) {
    return std::nullopt;
  }
  const RouteProjection3D active_projection =
      projectOntoRoute3D(active_route, current_position);
  if (!active_projection.valid ||
      active_stitch_station_m > active_route.back().station_m) {
    return std::nullopt;
  }
  return materializeFrozenRoutePrefixAtStations(
      active_route, successor_route, active_projection, active_stitch_station_m,
      successor_route.front().station_m, successor_route.front().station_m,
      FutureStitchJoinPolicy::kAllowStopAndTurn);
}

std::uint64_t
routeFingerprint(const std::span<const RouteSample3D> route,
                 const std::span<const SelectedPassageTraversal> traversals) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  for (const RouteSample3D& sample : route) {
    hashSigned(hash,
               static_cast<std::int64_t>(std::llround(sample.position.x * 1000.0)));
    hashSigned(hash,
               static_cast<std::int64_t>(std::llround(sample.position.y * 1000.0)));
    hashSigned(hash,
               static_cast<std::int64_t>(std::llround(sample.position.z * 1000.0)));
    hashByte(hash, static_cast<std::uint8_t>(sample.required_risk_tier));
  }
  for (const SelectedPassageTraversal& traversal : traversals) {
    hashText(hash, traversal.passage_traversal_id.value());
    hashSigned(hash, static_cast<std::int64_t>(
                         std::llround(traversal.begin_station_m * 1000.0)));
    hashSigned(hash, static_cast<std::int64_t>(
                         std::llround(traversal.end_station_m * 1000.0)));
    for (const PassageTraversalSegmentSpan& segment : traversal.segment_spans) {
      hashText(hash, segment.passage_segment_id.value());
      hashSigned(hash, static_cast<std::int64_t>(
                           std::llround(segment.begin_station_m * 1000.0)));
      hashSigned(hash, static_cast<std::int64_t>(
                           std::llround(segment.end_station_m * 1000.0)));
    }
  }
  return hash;
}

std::uint64_t routeFingerprint(const std::span<const Point2> route) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  for (const Point2 point : route) {
    hashSigned(hash, static_cast<std::int64_t>(std::llround(point.x * 1000.0)));
    hashSigned(hash, static_cast<std::int64_t>(std::llround(point.y * 1000.0)));
  }
  return hash;
}

std::string_view constrainedRoutePhaseName(const ConstrainedRoutePhase phase) noexcept {
  switch (phase) {
    case ConstrainedRoutePhase::kUnavailable:
      return "unavailable";
    case ConstrainedRoutePhase::kUnconstrained:
      return "unconstrained";
    case ConstrainedRoutePhase::kApproach:
      return "approach";
    case ConstrainedRoutePhase::kTraversal:
      return "traversal";
    case ConstrainedRoutePhase::kDeparture:
      return "departure";
  }
  return "unknown";
}

std::string_view passageTraversalEvidenceStatusName(
    const PassageTraversalEvidenceStatus status) noexcept {
  switch (status) {
    case PassageTraversalEvidenceStatus::kEntered:
      return "entered";
    case PassageTraversalEvidenceStatus::kCompleted:
      return "completed";
    case PassageTraversalEvidenceStatus::kAborted:
      return "aborted";
  }
  return "unknown";
}

std::string_view passageTraversalEvidenceReasonName(
    const PassageTraversalEvidenceReason reason) noexcept {
  switch (reason) {
    case PassageTraversalEvidenceReason::kEntryBoundaryCrossed:
      return "entry_boundary_crossed";
    case PassageTraversalEvidenceReason::kExitBoundaryCrossed:
      return "exit_boundary_crossed";
    case PassageTraversalEvidenceReason::kRouteChanged:
      return "route_changed";
    case PassageTraversalEvidenceReason::kObservationLost:
      return "observation_lost";
  }
  return "unknown";
}

std::vector<RouteSample3D> sampleRoute3D(const std::span<const Point3> points,
                                         const double sample_step_m,
                                         const double reference_speed_mps) {
  std::vector<RouteSample3D> result;
  if (points.empty() || !(sample_step_m > 0.0)) {
    return result;
  }
  double station_m = 0.0;
  result.push_back(RouteSample3D{.position = points.front(),
                                 .reference_speed_mps = reference_speed_mps});
  for (std::size_t index = 0U; index + 1U < points.size(); ++index) {
    const Point3 first = points[index];
    const Point3 second = points[index + 1U];
    const Vec3 tangent = normalized(first, second);
    const double length = distance3D(first, second);
    if (!(length > 1.0e-9)) {
      continue;
    }
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(length / sample_step_m)));
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const double sample_station_m = station_m + ratio * length;
      result.push_back(RouteSample3D{
          .position = Point3{std::lerp(first.x, second.x, ratio),
                             std::lerp(first.y, second.y, ratio),
                             std::lerp(first.z, second.z, ratio)},
          .tangent = tangent,
          .station_m = sample_station_m,
          .reference_speed_mps = reference_speed_mps,
      });
    }
    station_m += length;
  }
  if (result.size() >= 2U) {
    constexpr double kDefaultMinimumContinuousTurnAlignment{0.7071067811865476};
    if (!canonicalizeRouteKinematics3D(result,
                                       kDefaultMinimumContinuousTurnAlignment)) {
      return {};
    }
  }
  return result;
}

bool canonicalizeRouteKinematics3D(const std::span<RouteSample3D> route,
                                   const double minimum_continuous_turn_alignment,
                                   std::size_t* const stop_turn_count) noexcept {
  if (stop_turn_count != nullptr) {
    *stop_turn_count = 0U;
  }
  if (route.size() < 2U || !std::isfinite(minimum_continuous_turn_alignment) ||
      minimum_continuous_turn_alignment < -1.0 ||
      minimum_continuous_turn_alignment > 1.0) {
    return false;
  }
  std::vector<Vec3> directions;
  directions.reserve(route.size() - 1U);
  for (std::size_t index = 1U; index < route.size(); ++index) {
    const double length_m =
        distance3D(route[index - 1U].position, route[index].position);
    if (!(length_m > 1.0e-9) || !std::isfinite(length_m)) {
      return false;
    }
    directions.push_back(Vec3{
        (route[index].position.x - route[index - 1U].position.x) / length_m,
        (route[index].position.y - route[index - 1U].position.y) / length_m,
        (route[index].position.z - route[index - 1U].position.z) / length_m,
    });
  }
  double station_m{0.0};
  route.front().station_m = 0.0;
  route.front().tangent = directions.front();
  route.front().transition = RouteKinematicTransition3D::kContinuous;
  for (std::size_t index = 1U; index < route.size(); ++index) {
    station_m += distance3D(route[index - 1U].position, route[index].position);
    route[index].station_m = station_m;
    route[index].tangent =
        index + 1U < route.size() ? directions[index] : directions.back();
    route[index].transition = RouteKinematicTransition3D::kContinuous;
    if (index + 1U >= route.size()) {
      continue;
    }
    const Vec3& incoming = directions[index - 1U];
    const Vec3& outgoing = directions[index];
    if (requiresFlightStopAndTurn3D(incoming, outgoing,
                                    minimum_continuous_turn_alignment)) {
      route[index].transition = RouteKinematicTransition3D::kStopAndTurn;
      route[index].reference_speed_mps = 0.0;
      if (stop_turn_count != nullptr) {
        ++*stop_turn_count;
      }
    }
  }
  return true;
}

RouteProjection3D projectOntoRoute3D(const std::span<const RouteSample3D> route,
                                     const Point3& position,
                                     const double minimum_station_m) noexcept {
  RouteProjection3D best;
  if (route.empty()) {
    return best;
  }
  best.distance_m = std::numeric_limits<double>::infinity();
  if (route.size() == 1U) {
    best.valid = true;
    best.station_m = route.front().station_m;
    best.point = route.front().position;
    best.distance_m = distance3D(position, best.point);
    return best;
  }
  for (std::size_t index = 0U; index + 1U < route.size(); ++index) {
    const RouteSample3D& first = route[index];
    const RouteSample3D& second = route[index + 1U];
    if (second.station_m + 2.0 < minimum_station_m) {
      continue;
    }
    const Vec3 segment{second.position.x - first.position.x,
                       second.position.y - first.position.y,
                       second.position.z - first.position.z};
    const double squared_length =
        segment.x * segment.x + segment.y * segment.y + segment.z * segment.z;
    const Vec3 offset{position.x - first.position.x, position.y - first.position.y,
                      position.z - first.position.z};
    const double ratio = squared_length > 1.0e-12
                             ? std::clamp((offset.x * segment.x + offset.y * segment.y +
                                           offset.z * segment.z) /
                                              squared_length,
                                          0.0, 1.0)
                             : 0.0;
    const Point3 projected{first.position.x + ratio * segment.x,
                           first.position.y + ratio * segment.y,
                           first.position.z + ratio * segment.z};
    const double distance_m = distance3D(position, projected);
    const double station_m = std::lerp(first.station_m, second.station_m, ratio);
    if (station_m + 2.0 < minimum_station_m || distance_m >= best.distance_m) {
      continue;
    }
    best.valid = true;
    best.station_m = station_m;
    best.distance_m = distance_m;
    best.point = projected;
  }
  if (best.valid) {
    best.remaining_m = std::max(0.0, route.back().station_m - best.station_m);
  }
  return best;
}

RouteProjection3D projectOntoRoute3DWithinStationWindow(
    const std::span<const RouteSample3D> route, const Point3& position,
    const double minimum_station_m, const double maximum_station_m) noexcept {
  RouteProjection3D best;
  if (route.empty() || !std::isfinite(minimum_station_m) ||
      !std::isfinite(maximum_station_m) || maximum_station_m < minimum_station_m) {
    return best;
  }
  best.distance_m = std::numeric_limits<double>::infinity();
  if (route.size() == 1U) {
    if (route.front().station_m < minimum_station_m ||
        route.front().station_m > maximum_station_m) {
      return best;
    }
    best.valid = true;
    best.station_m = route.front().station_m;
    best.point = route.front().position;
    best.distance_m = distance3D(position, best.point);
    return best;
  }

  for (std::size_t index = 0U; index + 1U < route.size(); ++index) {
    const RouteSample3D& first = route[index];
    const RouteSample3D& second = route[index + 1U];
    const double allowed_begin_station_m = std::max(first.station_m, minimum_station_m);
    const double allowed_end_station_m = std::min(second.station_m, maximum_station_m);
    const double station_length_m = second.station_m - first.station_m;
    if (!(station_length_m > 0.0) || allowed_end_station_m < allowed_begin_station_m) {
      continue;
    }

    const Vec3 segment{second.position.x - first.position.x,
                       second.position.y - first.position.y,
                       second.position.z - first.position.z};
    const double squared_length =
        segment.x * segment.x + segment.y * segment.y + segment.z * segment.z;
    const Vec3 offset{position.x - first.position.x, position.y - first.position.y,
                      position.z - first.position.z};
    const double unconstrained_ratio =
        squared_length > 1.0e-12
            ? (offset.x * segment.x + offset.y * segment.y + offset.z * segment.z) /
                  squared_length
            : 0.0;
    const double unconstrained_station_m =
        std::lerp(first.station_m, second.station_m, unconstrained_ratio);
    const double station_m = std::clamp(unconstrained_station_m,
                                        allowed_begin_station_m, allowed_end_station_m);
    const double ratio = (station_m - first.station_m) / station_length_m;
    const Point3 projected{first.position.x + ratio * segment.x,
                           first.position.y + ratio * segment.y,
                           first.position.z + ratio * segment.z};
    const double distance_m = distance3D(position, projected);
    if (distance_m >= best.distance_m) {
      continue;
    }
    best.valid = true;
    best.station_m = station_m;
    best.distance_m = distance_m;
    best.point = projected;
  }
  if (best.valid) {
    best.remaining_m = std::max(0.0, route.back().station_m - best.station_m);
  }
  return best;
}

bool validateConstrainedRouteSpans(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> spans) noexcept {
  // Clearance is a derived annotation, never a hard constrained-span gate.
  for (const ConstrainedRouteSpan& span : spans) {
    if (span.envelope.empty() || !(span.end_station_m > span.begin_station_m)) {
      return false;
    }
    for (const RouteSample3D& sample : route) {
      if (sample.station_m + 1.0e-6 < span.begin_station_m ||
          sample.station_m - 1.0e-6 > span.end_station_m) {
        continue;
      }
      const RouteEnvelopeSample& envelope =
          nearestEnvelopeSample(span, sample.station_m);
      if (sample.position.z < envelope.min_z_m ||
          sample.position.z > envelope.max_z_m) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Point3> departureChain3D(const std::span<const RouteSample3D> route,
                                     const double departure_end_station_m) {
  std::vector<Point3> chain;
  if (!std::isfinite(departure_end_station_m) || departure_end_station_m <= 0.0) {
    return chain;
  }
  constexpr double kStationToleranceM{1.0e-6};
  for (const RouteSample3D& sample : route) {
    if (sample.station_m > departure_end_station_m + kStationToleranceM) {
      break;
    }
    chain.push_back(sample.position);
  }
  return chain;
}

} // namespace drone_city_nav
