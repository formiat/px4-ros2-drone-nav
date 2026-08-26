#include "drone_city_nav/route_3d.hpp"

#include "drone_city_nav/esdf_query.hpp"

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
      .required_risk_tier = static_cast<mppi::RiskTier>(
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

[[nodiscard]] std::optional<FrozenRoutePrefix3D> materializeFrozenRoutePrefixAtStations(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> successor_route,
    const RouteProjection3D& active_projection, const double active_stitch_station_m,
    const double successor_begin_station_m, const double successor_stitch_station_m,
    const FutureStitchJoinPolicy join_policy) noexcept {
  if (!active_projection.valid || !std::isfinite(active_stitch_station_m) ||
      !std::isfinite(successor_begin_station_m) ||
      !std::isfinite(successor_stitch_station_m) ||
      !(active_stitch_station_m > active_projection.station_m) ||
      active_stitch_station_m > active_route.back().station_m ||
      successor_stitch_station_m < successor_begin_station_m ||
      successor_stitch_station_m > successor_route.back().station_m) {
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
          kFrozenPrefixMaximumStitchSeparationM ||
      (join_policy == FutureStitchJoinPolicy::kRequireContinuousTangent &&
       tangent_alignment < kFrozenPrefixMinimumTangentAlignment)) {
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
  return result.valid() ? std::optional<FrozenRoutePrefix3D>{std::move(result)}
                        : std::nullopt;
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
    const Point3& current_position, const double active_stitch_station_m) noexcept {
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
      FutureStitchJoinPolicy::kRequireContinuousTangent);
}

std::optional<FrozenRoutePrefix3D> materializeTangentContinuousRoutePrefixAtStation3D(
    const std::span<const RouteSample3D> active_route,
    const std::span<const RouteSample3D> successor_route,
    const Point3& current_position, const double active_stitch_station_m,
    const FutureRouteConnectorConfig3D& config) noexcept {
  if (active_route.size() < 2U || successor_route.size() < 2U ||
      !std::isfinite(active_stitch_station_m) ||
      !futureRouteConnectorConfig3DValid(config)) {
    return std::nullopt;
  }
  const RouteProjection3D active_projection =
      projectOntoRoute3D(active_route, current_position);
  if (!active_projection.valid ||
      !(active_stitch_station_m > active_projection.station_m) ||
      active_stitch_station_m > active_route.back().station_m ||
      distance3D(sampleAtStation(active_route, active_stitch_station_m).position,
                 successor_route.front().position) >
          kFrozenPrefixMaximumStitchSeparationM ||
      successor_route.back().station_m < config.successor_join_station_m) {
    return std::nullopt;
  }

  const RouteSample3D active_stitch =
      sampleAtStation(active_route, active_stitch_station_m);
  const RouteSample3D successor_join =
      sampleAtStation(successor_route, config.successor_join_station_m);
  const Vec3 active_tangent = normalized(active_stitch.tangent);
  const Vec3 successor_tangent = normalized(successor_join.tangent);
  if (!(vectorNorm(active_tangent) > 0.0) || !(vectorNorm(successor_tangent) > 0.0)) {
    return std::nullopt;
  }

  FrozenRoutePrefix3D result{
      .route = {},
      .active_begin_station_m = active_projection.station_m,
      .stitch_station_m = active_stitch_station_m,
      .successor_begin_station_m = successor_route.front().station_m,
      .successor_stitch_station_m = config.successor_join_station_m,
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

  const double connector_speed_mps =
      std::min(active_stitch.reference_speed_mps, successor_join.reference_speed_mps);
  const auto connector_risk_tier = static_cast<mppi::RiskTier>(
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
  for (const RouteSample3D& source : successor_route) {
    if (source.station_m > config.successor_join_station_m + 1.0e-6) {
      append(source);
    }
  }
  std::size_t stop_turn_count{0U};
  if (!result.valid() ||
      !canonicalizeRouteKinematics3D(
          result.route, config.minimum_continuous_turn_alignment, &stop_turn_count) ||
      stop_turn_count != 0U) {
    return std::nullopt;
  }
  return result;
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

std::vector<PassageTraversalEvidenceEvent>
PassageTraversalEvidenceTracker::update(const ConstrainedRouteObservation& observation,
                                        const Point3& actual_position,
                                        const std::int64_t now_ns) {
  std::vector<PassageTraversalEvidenceEvent> events;
  const bool traversal_observed =
      observation.span_available &&
      observation.phase == ConstrainedRoutePhase::kTraversal &&
      !observation.passage_traversal_id.empty();
  const bool active_matches =
      active_.has_value() && observation.span_available &&
      active_->passage_traversal_id == observation.passage_traversal_id &&
      active_->route_generation == observation.route_generation &&
      active_->span_index == observation.span_index;
  bool entered_now = false;

  if (active_.has_value() && !active_matches) {
    const PassageTraversalEvidenceReason reason =
        observation.span_available ? PassageTraversalEvidenceReason::kRouteChanged
                                   : PassageTraversalEvidenceReason::kObservationLost;
    events.push_back(makeEvent(*active_, PassageTraversalEvidenceStatus::kAborted,
                               reason, observation, actual_position, now_ns));
    active_.reset();
  }

  if (traversal_observed && !active_.has_value()) {
    active_ = ActiveTraversal{
        .passage_traversal_id = observation.passage_traversal_id,
        .route_generation = observation.route_generation,
        .span_index = observation.span_index,
        .observation_count = 1U,
        .entry_stamp_ns = now_ns,
        .begin_station_m = observation.begin_station_m,
        .end_station_m = observation.end_station_m,
        .maximum_cross_track_error_m = std::abs(observation.cross_track_error_m),
        .maximum_absolute_vertical_error_m = std::abs(observation.vertical_error_m),
        .vertical_window_preserved = observation.within_vertical_window,
    };
    events.push_back(makeEvent(*active_, PassageTraversalEvidenceStatus::kEntered,
                               PassageTraversalEvidenceReason::kEntryBoundaryCrossed,
                               observation, actual_position, now_ns));
    entered_now = true;
  }

  if (!active_.has_value()) {
    return events;
  }

  if (traversal_observed && !entered_now) {
    ++active_->observation_count;
    active_->maximum_cross_track_error_m =
        std::max(active_->maximum_cross_track_error_m,
                 std::abs(observation.cross_track_error_m));
    active_->maximum_absolute_vertical_error_m =
        std::max(active_->maximum_absolute_vertical_error_m,
                 std::abs(observation.vertical_error_m));
    active_->vertical_window_preserved =
        active_->vertical_window_preserved && observation.within_vertical_window;
  }

  if (active_matches && observation.phase == ConstrainedRoutePhase::kDeparture) {
    events.push_back(makeEvent(*active_, PassageTraversalEvidenceStatus::kCompleted,
                               PassageTraversalEvidenceReason::kExitBoundaryCrossed,
                               observation, actual_position, now_ns));
    active_.reset();
  }
  return events;
}

void PassageTraversalEvidenceTracker::reset() noexcept {
  active_.reset();
}

PassageTraversalEvidenceEvent PassageTraversalEvidenceTracker::makeEvent(
    const ActiveTraversal& active, const PassageTraversalEvidenceStatus status,
    const PassageTraversalEvidenceReason reason,
    const ConstrainedRouteObservation& observation, const Point3& actual_position,
    const std::int64_t now_ns) {
  return PassageTraversalEvidenceEvent{
      .status = status,
      .reason = reason,
      .sequence = ++event_sequence_,
      .passage_traversal_id = active.passage_traversal_id,
      .route_generation = active.route_generation,
      .span_index = active.span_index,
      .traversal_observation_count = active.observation_count,
      .event_stamp_ns = now_ns,
      .duration_s = static_cast<double>(
                        std::max<std::int64_t>(0, now_ns - active.entry_stamp_ns)) /
                    1.0e9,
      .station_m = observation.station_m,
      .begin_station_m = active.begin_station_m,
      .end_station_m = active.end_station_m,
      .maximum_cross_track_error_m = active.maximum_cross_track_error_m,
      .maximum_absolute_vertical_error_m = active.maximum_absolute_vertical_error_m,
      .actual_position = actual_position,
      .vertical_window_preserved = active.vertical_window_preserved,
  };
}

std::vector<PassageGeometryEvidenceEvent> PassageGeometryEvidenceTracker::update(
    const std::span<const PassageGeometryObservation> observations,
    const Point3& actual_position, const std::int64_t now_ns,
    const PassageGeometryEvidenceConfig& config) {
  std::vector<PassageGeometryEvidenceEvent> events;
  if (!active_.has_value()) {
    const PassageGeometryObservation* selected = nullptr;
    for (const PassageGeometryObservation& observation : observations) {
      if (!observation.within_corridor ||
          observation.station_m > config.entry_capture_distance_m ||
          !(observation.traversal_length_m > 0.0)) {
        continue;
      }
      if (selected == nullptr ||
          std::tie(observation.traversal_length_m, observation.cross_track_error_m,
                   observation.passage_traversal_id) <
              std::tie(selected->traversal_length_m, selected->cross_track_error_m,
                       selected->passage_traversal_id)) {
        selected = &observation;
      }
    }
    if (selected == nullptr) {
      return events;
    }
    active_ = ActiveTraversal{
        .passage_traversal_id = selected->passage_traversal_id,
        .observation_count = 1U,
        .entry_stamp_ns = now_ns,
        .last_observation_stamp_ns = now_ns,
        .traversal_length_m = selected->traversal_length_m,
        .maximum_station_m = selected->station_m,
        .maximum_cross_track_error_m = selected->cross_track_error_m,
    };
    events.push_back(makeEvent(*active_, PassageTraversalEvidenceStatus::kEntered,
                               PassageTraversalEvidenceReason::kEntryBoundaryCrossed,
                               selected->station_m, actual_position, now_ns));
    return events;
  }

  const auto matching = std::ranges::find_if(
      observations, [&](const PassageGeometryObservation& observation) {
        return observation.passage_traversal_id == active_->passage_traversal_id;
      });
  if (matching != observations.end() && matching->within_corridor) {
    ++active_->observation_count;
    active_->last_observation_stamp_ns = now_ns;
    active_->maximum_station_m =
        std::max(active_->maximum_station_m, matching->station_m);
    active_->maximum_cross_track_error_m =
        std::max(active_->maximum_cross_track_error_m, matching->cross_track_error_m);
    const double required_station_m =
        config.minimum_progress_fraction * active_->traversal_length_m;
    const double exit_station_m =
        std::max(0.0, active_->traversal_length_m - config.exit_capture_distance_m);
    if (active_->maximum_station_m >= required_station_m &&
        matching->station_m >= exit_station_m) {
      events.push_back(makeEvent(*active_, PassageTraversalEvidenceStatus::kCompleted,
                                 PassageTraversalEvidenceReason::kExitBoundaryCrossed,
                                 matching->station_m, actual_position, now_ns));
      active_.reset();
    }
    return events;
  }

  const std::int64_t timeout_ns =
      static_cast<std::int64_t>(std::max(0.0, config.observation_timeout_s) * 1.0e9);
  if (now_ns - active_->last_observation_stamp_ns > timeout_ns) {
    events.push_back(makeEvent(*active_, PassageTraversalEvidenceStatus::kAborted,
                               PassageTraversalEvidenceReason::kObservationLost,
                               active_->maximum_station_m, actual_position, now_ns));
    active_.reset();
  }
  return events;
}

void PassageGeometryEvidenceTracker::reset() noexcept {
  active_.reset();
}

PassageGeometryEvidenceEvent PassageGeometryEvidenceTracker::makeEvent(
    const ActiveTraversal& active, const PassageTraversalEvidenceStatus status,
    const PassageTraversalEvidenceReason reason, const double station_m,
    const Point3& actual_position, const std::int64_t now_ns) {
  return PassageGeometryEvidenceEvent{
      .status = status,
      .reason = reason,
      .sequence = ++event_sequence_,
      .passage_traversal_id = active.passage_traversal_id,
      .observation_count = active.observation_count,
      .event_stamp_ns = now_ns,
      .duration_s = static_cast<double>(
                        std::max<std::int64_t>(0, now_ns - active.entry_stamp_ns)) /
                    1.0e9,
      .station_m = station_m,
      .traversal_length_m = active.traversal_length_m,
      .maximum_station_m = active.maximum_station_m,
      .maximum_cross_track_error_m = active.maximum_cross_track_error_m,
      .actual_position = actual_position,
  };
}

ConstrainedRouteObservation
observeConstrainedRoute(const std::span<const RouteSample3D> route,
                        const std::span<const ConstrainedRouteSpan> spans,
                        const std::uint64_t route_generation,
                        const double current_station_m, const Point3& actual_position,
                        const Vec3& actual_velocity, const RouteEnvelopeConfig& config,
                        const double event_distance_m) {
  ConstrainedRouteObservation observation{
      .phase = route.empty() ? ConstrainedRoutePhase::kUnavailable
                             : ConstrainedRoutePhase::kUnconstrained,
      .route_generation = route_generation,
      .span_count = spans.size(),
      .passage_traversal_id = {},
      .station_m = current_station_m,
      .actual_horizontal_speed_mps = std::hypot(actual_velocity.x, actual_velocity.y),
      .actual_vertical_speed_mps = actual_velocity.z,
      .actual_z_m = actual_position.z,
      .segment_spans = {},
  };
  if (route.empty() || spans.empty() || !(event_distance_m >= 0.0)) {
    return observation;
  }

  std::optional<std::size_t> selected_index;
  ConstrainedRoutePhase selected_phase = ConstrainedRoutePhase::kUnconstrained;
  const auto upcoming =
      std::find_if(spans.begin(), spans.end(),
                   [current_station_m](const ConstrainedRouteSpan& span) {
                     return current_station_m <= span.end_station_m;
                   });
  if (upcoming != spans.end()) {
    const std::size_t upcoming_index =
        static_cast<std::size_t>(std::distance(spans.begin(), upcoming));
    if (current_station_m >= upcoming->begin_station_m) {
      selected_index = upcoming_index;
      selected_phase = ConstrainedRoutePhase::kTraversal;
    } else if (upcoming->begin_station_m - current_station_m <= event_distance_m) {
      selected_index = upcoming_index;
      selected_phase = ConstrainedRoutePhase::kApproach;
    } else if (upcoming_index > 0U &&
               current_station_m - spans[upcoming_index - 1U].end_station_m <=
                   event_distance_m) {
      selected_index = upcoming_index - 1U;
      selected_phase = ConstrainedRoutePhase::kDeparture;
    }
  } else if (current_station_m - spans.back().end_station_m <= event_distance_m) {
    selected_index = spans.size() - 1U;
    selected_phase = ConstrainedRoutePhase::kDeparture;
  }
  if (!selected_index.has_value()) {
    return observation;
  }

  const ConstrainedRouteSpan& span = spans[*selected_index];
  if (span.envelope.empty()) {
    return observation;
  }
  const double envelope_station_m =
      std::clamp(current_station_m, span.begin_station_m, span.end_station_m);
  const RouteEnvelopeSample& envelope = nearestEnvelopeSample(span, envelope_station_m);
  const RouteSample3D route_sample = sampleAtStation(route, current_station_m);
  observation.phase = selected_phase;
  observation.span_index = *selected_index;
  observation.span_available = true;
  observation.passage_traversal_id = span.passage_traversal_id;
  observation.direction_sign = span.direction_sign;
  observation.begin_station_m = span.begin_station_m;
  observation.end_station_m = span.end_station_m;
  observation.distance_to_entry_m = span.begin_station_m - current_station_m;
  observation.distance_to_exit_m = span.end_station_m - current_station_m;
  observation.entry_position = sampleAtStation(route, span.begin_station_m).position;
  observation.exit_position = sampleAtStation(route, span.end_station_m).position;
  observation.reference_z_m = envelope.reference_z_m;
  observation.min_z_m = envelope.min_z_m;
  observation.max_z_m = envelope.max_z_m;
  observation.lateral_free_left_m = envelope.lateral_free_left_m;
  observation.lateral_free_right_m = envelope.lateral_free_right_m;
  observation.lateral_width_m =
      envelope.lateral_free_left_m + envelope.lateral_free_right_m;
  observation.vertical_height_m = envelope.max_z_m - envelope.min_z_m;
  observation.vertical_error_m = actual_position.z - envelope.reference_z_m;
  observation.cross_track_error_m =
      std::hypot(actual_position.x - route_sample.position.x,
                 actual_position.y - route_sample.position.y);
  observation.reference_speed_mps = envelope.reference_speed_mps;
  observation.within_vertical_window =
      actual_position.z >= envelope.min_z_m && actual_position.z <= envelope.max_z_m;
  observation.lateral_constrained =
      observation.lateral_width_m <= config.constrained_lateral_width_m;
  observation.vertical_constrained =
      observation.vertical_height_m <= config.constrained_vertical_height_m;
  observation.segment_spans = span.segment_spans;
  return observation;
}

ConstrainedRouteControl ConstrainedRouteCoordinator::update(
    const ConstrainedRouteObservation& observation,
    const double unconstrained_speed_mps,
    const ConstrainedRouteControlConfig& config) noexcept {
  if (!observation.span_available ||
      observation.phase == ConstrainedRoutePhase::kUnavailable ||
      observation.phase == ConstrainedRoutePhase::kUnconstrained ||
      observation.phase == ConstrainedRoutePhase::kDeparture) {
    reset();
    return {};
  }
  if (route_generation_ != observation.route_generation ||
      span_index_ != observation.span_index) {
    route_generation_ = observation.route_generation;
    span_index_ = observation.span_index;
    vertical_ready_latched_ = false;
  }
  const double capture_margin = std::max(0.0, config.vertical_capture_margin_m);
  const double capture_min = observation.min_z_m + capture_margin;
  const double capture_max = observation.max_z_m - capture_margin;
  const bool inside_capture_window = capture_max >= capture_min &&
                                     observation.actual_z_m >= capture_min &&
                                     observation.actual_z_m <= capture_max;
  if (inside_capture_window && std::abs(observation.actual_vertical_speed_mps) <=
                                   std::max(0.0, config.vertical_capture_speed_mps)) {
    vertical_ready_latched_ = true;
  }

  const double acceleration =
      std::max(1.0e-6, config.maximum_vertical_acceleration_mps2);
  const double maximum_speed = std::max(1.0e-6, config.maximum_vertical_speed_mps);
  // Align to the admissible capture interval, rather than only correcting a
  // positive error from the envelope reference.  The latter made an approach
  // from below appear to need no climb at all.
  const double capture_target_z =
      std::clamp(observation.reference_z_m, capture_min, capture_max);
  const double displacement_to_capture_m = capture_target_z - observation.actual_z_m;
  double direction_to_capture{0.0};
  if (displacement_to_capture_m > 0.0) {
    direction_to_capture = 1.0;
  } else if (displacement_to_capture_m < 0.0) {
    direction_to_capture = -1.0;
  }
  const double speed_toward_capture_mps =
      direction_to_capture * observation.actual_vertical_speed_mps;
  const double braking_distance_m =
      speed_toward_capture_mps < 0.0
          ? speed_toward_capture_mps * speed_toward_capture_mps / (2.0 * acceleration)
          : 0.0;
  const double distance = std::abs(displacement_to_capture_m) + braking_distance_m;
  const double acceleration_distance = maximum_speed * maximum_speed / acceleration;
  const double motion_time_s =
      distance <= acceleration_distance
          ? 2.0 * std::sqrt(distance / acceleration)
          : 2.0 * maximum_speed / acceleration +
                (distance - acceleration_distance) / maximum_speed;
  const double settle_time_s =
      std::abs(observation.actual_vertical_speed_mps) / acceleration;
  const double required_time_s = motion_time_s + settle_time_s;
  const double cruise_speed = std::max(0.0, unconstrained_speed_mps);
  const double traversal_speed =
      observation.reference_speed_mps > 0.0
          ? std::min(cruise_speed, observation.reference_speed_mps)
          : cruise_speed;
  const double alignment_start_distance_m =
      cruise_speed * required_time_s +
      std::max(0.0, config.alignment_distance_buffer_m);
  const double entry_distance_m = std::max(0.0, observation.distance_to_entry_m);
  const bool alignment_active =
      observation.phase == ConstrainedRoutePhase::kTraversal ||
      entry_distance_m <= alignment_start_distance_m;
  if (!alignment_active) {
    return {};
  }
  const double hold_distance = std::max(0.0, config.stationary_hold_distance_m);
  const bool hold_xy = !vertical_ready_latched_ && entry_distance_m <= hold_distance;
  double speed_limit_mps = observation.phase == ConstrainedRoutePhase::kTraversal
                               ? traversal_speed
                               : cruise_speed;
  if (!vertical_ready_latched_ && required_time_s > 1.0e-6) {
    speed_limit_mps = std::clamp((entry_distance_m - hold_distance) / required_time_s,
                                 0.0, cruise_speed);
  }
  return ConstrainedRouteControl{
      .active = true,
      .vertical_ready = vertical_ready_latched_,
      .hold_xy = hold_xy,
      .required_alignment_time_s = required_time_s,
      .alignment_start_distance_m = alignment_start_distance_m,
      .reference_z_m = observation.reference_z_m,
      .speed_limit_mps = speed_limit_mps,
  };
}

void ConstrainedRouteCoordinator::reset() noexcept {
  route_generation_ = 0U;
  span_index_ = 0U;
  vertical_ready_latched_ = false;
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
    const double alignment =
        incoming.x * outgoing.x + incoming.y * outgoing.y + incoming.z * outgoing.z;
    if (alignment < minimum_continuous_turn_alignment) {
      route[index].transition = RouteKinematicTransition3D::kStopAndTurn;
      route[index].reference_speed_mps = 0.0;
      if (stop_turn_count != nullptr) {
        ++*stop_turn_count;
      }
    }
  }
  return true;
}

RouteRiskTierAssignmentResult assignRouteRiskTiers(
    const std::span<RouteSample3D> route, const mppi::EsdfGrid& grid,
    const std::span<const float> esdf_m, const double critical_distance_m,
    const double preferred_distance_m, const bool require_known_free_space) noexcept {
  for (std::size_t index = 0U; index < route.size(); ++index) {
    RouteSample3D& sample = route[index];
    const EsdfQueryResult query = queryConservativeEsdf3D(
        grid, esdf_m, static_cast<float>(sample.position.x),
        static_cast<float>(sample.position.y), static_cast<float>(sample.position.z));
    if (query.raw_occupied) {
      return {.status = RouteRiskTierAssignmentStatus::kRawCollision,
              .failure_sample_index = index,
              .failure_point = sample.position};
    }
    if (query.status != EsdfQueryStatus::kValid && require_known_free_space) {
      const RouteRiskTierAssignmentStatus status = [&]() noexcept {
        switch (query.status) {
          case EsdfQueryStatus::kOutsideGrid:
            return RouteRiskTierAssignmentStatus::kOutsideGrid;
          case EsdfQueryStatus::kUnknownSpace:
            return RouteRiskTierAssignmentStatus::kUnknownSpace;
          case EsdfQueryStatus::kInvalidDistance:
            return RouteRiskTierAssignmentStatus::kInvalidEsdf;
          case EsdfQueryStatus::kValid:
            break;
        }
        return RouteRiskTierAssignmentStatus::kInvalidEsdf;
      }();
      return {.status = status,
              .failure_sample_index = index,
              .failure_point = sample.position};
    }
    if (query.status != EsdfQueryStatus::kValid) {
      sample.required_risk_tier = mppi::RiskTier::kPreferred;
      continue;
    }
    if (query.clearance_m < critical_distance_m) {
      sample.required_risk_tier = mppi::RiskTier::kCritical;
    } else if (query.clearance_m < preferred_distance_m) {
      sample.required_risk_tier = mppi::RiskTier::kPlanning;
    } else {
      sample.required_risk_tier = mppi::RiskTier::kPreferred;
    }
  }
  return {.status = RouteRiskTierAssignmentStatus::kAccepted};
}

std::string_view
routeRiskTierAssignmentStatusName(const RouteRiskTierAssignmentStatus status) noexcept {
  switch (status) {
    case RouteRiskTierAssignmentStatus::kAccepted:
      return "accepted";
    case RouteRiskTierAssignmentStatus::kOutsideGrid:
      return "outside_grid";
    case RouteRiskTierAssignmentStatus::kUnknownSpace:
      return "unknown_space";
    case RouteRiskTierAssignmentStatus::kInvalidEsdf:
      return "invalid_esdf";
    case RouteRiskTierAssignmentStatus::kRawCollision:
      return "raw_collision";
  }
  return "invalid_status";
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

bool validateConstrainedRouteSpans(const std::span<const RouteSample3D> route,
                                   const std::span<const ConstrainedRouteSpan> spans,
                                   const mppi::EsdfGrid& grid,
                                   const std::span<const float> esdf_m) noexcept {
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
      const EsdfQueryResult query = queryConservativeEsdf3D(
          grid, esdf_m, static_cast<float>(sample.position.x),
          static_cast<float>(sample.position.y), static_cast<float>(sample.position.z));
      if (query.status != EsdfQueryStatus::kValid || query.raw_occupied) {
        return false;
      }
    }
  }
  return true;
}

} // namespace drone_city_nav
