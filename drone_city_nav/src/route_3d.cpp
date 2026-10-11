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

#include "route_3d_internal.hpp"

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

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

using route_3d_detail::CubicBezierCurve3D;
using route_3d_detail::cubicBezierPoint;
using route_3d_detail::nearestEnvelopeSample;
using route_3d_detail::normalized;
using route_3d_detail::sampleAtStation;
using route_3d_detail::translated;
using route_3d_detail::vectorNorm;

} // namespace

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
