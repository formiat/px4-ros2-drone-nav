#include "drone_city_nav/route_3d.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>

namespace drone_city_nav {

std::vector<ConstrainedRouteSpan>
makeConstrainedRouteSpans(const std::span<const RouteSample3D> route,
                          const std::span<const SelectedPassageTraversal> traversals,
                          const std::uint64_t route_generation,
                          const RouteEnvelopeConfig& config) {
  std::vector<ConstrainedRouteSpan> spans;
  spans.reserve(traversals.size());
  for (const SelectedPassageTraversal& traversal : traversals) {
    if (traversal.end_station_m - traversal.begin_station_m <
        config.minimum_span_length_m) {
      continue;
    }
    ConstrainedRouteSpan span{.passage_traversal_id = traversal.passage_traversal_id,
                              .route_generation = route_generation,
                              .direction_sign = traversal.direction_sign,
                              .begin_station_m = traversal.begin_station_m,
                              .end_station_m = traversal.end_station_m,
                              .envelope = {},
                              .segment_spans = traversal.segment_spans};
    for (const RouteSample3D& sample : route) {
      if (sample.station_m + 1.0e-6 < span.begin_station_m ||
          sample.station_m - 1.0e-6 > span.end_station_m) {
        continue;
      }
      span.envelope.push_back(RouteEnvelopeSample{
          .station_m = sample.station_m,
          .lateral_free_left_m = 0.5 * traversal.width_m,
          .lateral_free_right_m = 0.5 * traversal.width_m,
          .min_z_m = traversal.min_z_m,
          .max_z_m = traversal.max_z_m,
          .minimum_clearance_m = traversal.minimum_clearance_m,
          .reference_z_m = sample.position.z,
          .reference_speed_mps = traversal.speed_limit_mps,
      });
    }
    if (span.envelope.empty()) {
      const RouteSample3D entry = sampleRoute3DAtStation(route, span.begin_station_m);
      span.envelope.push_back(RouteEnvelopeSample{
          .station_m = span.begin_station_m,
          .lateral_free_left_m = 0.5 * traversal.width_m,
          .lateral_free_right_m = 0.5 * traversal.width_m,
          .min_z_m = traversal.min_z_m,
          .max_z_m = traversal.max_z_m,
          .minimum_clearance_m = traversal.minimum_clearance_m,
          .reference_z_m = entry.position.z,
          .reference_speed_mps = traversal.speed_limit_mps,
      });
    }
    spans.push_back(std::move(span));
  }
  return spans;
}

std::vector<ConstrainedRouteSpan>
remapConstrainedRouteSpans(const std::span<const RouteSample3D> source_route,
                           const std::span<const ConstrainedRouteSpan> source_spans,
                           const std::span<const RouteSample3D> destination_route,
                           const RouteEnvelopeConfig& config) {
  std::vector<ConstrainedRouteSpan> result;
  if (source_route.size() < 2U || destination_route.size() < 2U) {
    return result;
  }
  result.reserve(source_spans.size());
  for (const ConstrainedRouteSpan& span : source_spans) {
    if (span.envelope.empty()) {
      continue;
    }
    const Point3 source_entry =
        sampleRoute3DAtStation(source_route, span.begin_station_m).position;
    const Point3 source_exit =
        sampleRoute3DAtStation(source_route, span.end_station_m).position;
    const RouteProjection3D destination_entry =
        projectOntoRoute3D(destination_route, source_entry);
    const RouteProjection3D destination_exit =
        projectOntoRoute3D(destination_route, source_exit, destination_entry.station_m);
    if (!destination_entry.valid || !destination_exit.valid ||
        destination_exit.station_m - destination_entry.station_m <
            config.minimum_span_length_m) {
      continue;
    }
    ConstrainedRouteSpan remapped = span;
    remapped.begin_station_m = destination_entry.station_m;
    remapped.end_station_m = destination_exit.station_m;
    remapped.segment_spans.clear();
    remapped.segment_spans.reserve(span.segment_spans.size());
    for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
      const Point3 source_segment_begin =
          sampleRoute3DAtStation(source_route, segment.begin_station_m).position;
      const Point3 source_segment_end =
          sampleRoute3DAtStation(source_route, segment.end_station_m).position;
      const RouteProjection3D destination_segment_begin = projectOntoRoute3D(
          destination_route, source_segment_begin, destination_entry.station_m);
      const RouteProjection3D destination_segment_end = projectOntoRoute3D(
          destination_route, source_segment_end,
          destination_segment_begin.valid ? destination_segment_begin.station_m
                                          : destination_entry.station_m);
      if (!destination_segment_begin.valid || !destination_segment_end.valid ||
          destination_segment_end.station_m <=
              destination_segment_begin.station_m + 1.0e-9) {
        continue;
      }
      remapped.segment_spans.push_back(PassageTraversalSegmentSpan{
          .passage_segment_id = segment.passage_segment_id,
          .begin_station_m =
              std::clamp(destination_segment_begin.station_m, remapped.begin_station_m,
                         remapped.end_station_m),
          .end_station_m = std::clamp(destination_segment_end.station_m,
                                      remapped.begin_station_m, remapped.end_station_m),
      });
    }
    remapped.envelope.clear();
    remapped.envelope.reserve(span.envelope.size());
    for (const RouteEnvelopeSample& envelope : span.envelope) {
      const Point3 source_position =
          sampleRoute3DAtStation(source_route, envelope.station_m).position;
      const RouteProjection3D destination = projectOntoRoute3D(
          destination_route, source_position, destination_entry.station_m);
      if (!destination.valid ||
          destination.station_m + 1.0e-6 < remapped.begin_station_m ||
          destination.station_m - 1.0e-6 > remapped.end_station_m) {
        continue;
      }
      RouteEnvelopeSample mapped_envelope = envelope;
      mapped_envelope.station_m = std::clamp(
          destination.station_m, remapped.begin_station_m, remapped.end_station_m);
      mapped_envelope.reference_z_m =
          sampleRoute3DAtStation(destination_route, mapped_envelope.station_m)
              .position.z;
      remapped.envelope.push_back(mapped_envelope);
    }
    std::ranges::sort(remapped.envelope, {}, &RouteEnvelopeSample::station_m);
    remapped.envelope.erase(
        std::unique(
            remapped.envelope.begin(), remapped.envelope.end(),
            [](const RouteEnvelopeSample& first, const RouteEnvelopeSample& second) {
              return std::abs(first.station_m - second.station_m) <= 1.0e-6;
            }),
        remapped.envelope.end());
    if (remapped.envelope.empty()) {
      RouteEnvelopeSample fallback = span.envelope.front();
      fallback.station_m = remapped.begin_station_m;
      fallback.reference_z_m =
          sampleRoute3DAtStation(destination_route, remapped.begin_station_m)
              .position.z;
      remapped.envelope.push_back(fallback);
    }
    result.push_back(std::move(remapped));
  }
  return result;
}

} // namespace drone_city_nav
