#include "drone_city_nav/route_3d.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] Vec3 normalized(const Point3& from, const Point3& to) noexcept {
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dz = to.z - from.z;
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  return length > 1.0e-9 ? Vec3{dx / length, dy / length, dz / length} : Vec3{};
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
  return RouteSample3D{
      .position = Point3{std::lerp(first.position.x, second.position.x, ratio),
                         std::lerp(first.position.y, second.position.y, ratio),
                         std::lerp(first.position.z, second.position.z, ratio)},
      .tangent = Vec3{std::lerp(first.tangent.x, second.tangent.x, ratio),
                      std::lerp(first.tangent.y, second.tangent.y, ratio),
                      std::lerp(first.tangent.z, second.tangent.z, ratio)},
      .station_m = std::lerp(first.station_m, second.station_m, ratio),
      .reference_speed_mps =
          std::lerp(first.reference_speed_mps, second.reference_speed_mps, ratio),
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

} // namespace

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
      .channel_id = {},
      .station_m = current_station_m,
      .actual_horizontal_speed_mps = std::hypot(actual_velocity.x, actual_velocity.y),
      .actual_vertical_speed_mps = actual_velocity.z,
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
  observation.channel_id = span.channel_id;
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
  return observation;
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
    result.front().tangent = result[1U].tangent;
  }
  return result;
}

std::vector<ConstrainedRouteSpan>
makeConstrainedRouteSpans(const std::span<const RouteSample3D> route,
                          const std::span<const SelectedChannelTraversal> traversals,
                          const std::uint64_t route_generation,
                          const RouteEnvelopeConfig& config) {
  std::vector<ConstrainedRouteSpan> spans;
  spans.reserve(traversals.size());
  for (const SelectedChannelTraversal& traversal : traversals) {
    if (traversal.end_station_m - traversal.begin_station_m <
        config.minimum_span_length_m) {
      continue;
    }
    ConstrainedRouteSpan span{.channel_id = traversal.channel_id,
                              .route_generation = route_generation,
                              .begin_station_m = traversal.begin_station_m,
                              .end_station_m = traversal.end_station_m,
                              .envelope = {}};
    for (const RouteSample3D& sample : route) {
      if (sample.station_m + 1.0e-6 < span.begin_station_m ||
          sample.station_m - 1.0e-6 > span.end_station_m) {
        continue;
      }
      span.envelope.push_back(RouteEnvelopeSample{
          .station_m = sample.station_m,
          .lateral_free_left_m = traversal.minimum_clearance_m,
          .lateral_free_right_m = traversal.minimum_clearance_m,
          .min_z_m = traversal.min_z_m,
          .max_z_m = traversal.max_z_m,
          .reference_z_m = sample.position.z,
          .reference_speed_mps = traversal.speed_limit_mps,
      });
    }
    if (span.envelope.empty()) {
      const RouteSample3D entry = sampleAtStation(route, span.begin_station_m);
      span.envelope.push_back(RouteEnvelopeSample{
          .station_m = span.begin_station_m,
          .lateral_free_left_m = traversal.minimum_clearance_m,
          .lateral_free_right_m = traversal.minimum_clearance_m,
          .min_z_m = traversal.min_z_m,
          .max_z_m = traversal.max_z_m,
          .reference_z_m = entry.position.z,
          .reference_speed_mps = traversal.speed_limit_mps,
      });
    }
    spans.push_back(std::move(span));
  }
  return spans;
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
