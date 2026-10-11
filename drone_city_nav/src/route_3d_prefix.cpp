#include "drone_city_nav/flight_time_model_3d.hpp"
#include "drone_city_nav/route_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <string_view>
#include <tuple>

#include "route_3d_internal.hpp"

namespace drone_city_nav {
namespace {

using route_3d_detail::CubicBezierCurve3D;
using route_3d_detail::cubicBezierPoint;
using route_3d_detail::nearestEnvelopeSample;
using route_3d_detail::normalized;
using route_3d_detail::sampleAtStation;
using route_3d_detail::translated;
using route_3d_detail::vectorNorm;

constexpr double kFrozenPrefixMaximumStitchSeparationM{0.05};
constexpr double kFrozenPrefixMinimumTangentAlignment{0.995};

enum class FutureStitchJoinPolicy : std::uint8_t {
  kRequireContinuousTangent,
  kAllowStopAndTurn,
};

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

double routeBlockFreeDistanceM(const std::span<const RouteSample3D> route,
                               const double projection_station_m,
                               const double blocked_station_m,
                               const Point3& position) noexcept {
  if (!std::isfinite(projection_station_m) || !std::isfinite(blocked_station_m)) {
    return 0.0;
  }
  const double along_route_m = std::max(0.0, blocked_station_m - projection_station_m);
  if (route.empty()) {
    return along_route_m;
  }
  const RouteSample3D blocked = sampleAtStation(route, blocked_station_m);
  const double straight_line_m = distance3D(position, blocked.position);
  if (!std::isfinite(straight_line_m)) {
    return along_route_m;
  }
  return std::min(along_route_m, straight_line_m);
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

} // namespace drone_city_nav
