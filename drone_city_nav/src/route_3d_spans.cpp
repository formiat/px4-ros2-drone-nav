#include "drone_city_nav/route_3d.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <tuple>

namespace drone_city_nav {
namespace {

[[nodiscard]] RouteEnvelopeSample interpolateEnvelope(const RouteEnvelopeSample& first,
                                                      const RouteEnvelopeSample& second,
                                                      const double station_m) noexcept {
  const double distance_m = second.station_m - first.station_m;
  const double ratio =
      distance_m > 1.0e-9
          ? std::clamp((station_m - first.station_m) / distance_m, 0.0, 1.0)
          : 0.0;
  const auto interpolate = [ratio](const double left, const double right) noexcept {
    return std::lerp(left, right, ratio);
  };
  return RouteEnvelopeSample{
      .station_m = station_m,
      .lateral_free_left_m =
          interpolate(first.lateral_free_left_m, second.lateral_free_left_m),
      .lateral_free_right_m =
          interpolate(first.lateral_free_right_m, second.lateral_free_right_m),
      .min_z_m = interpolate(first.min_z_m, second.min_z_m),
      .max_z_m = interpolate(first.max_z_m, second.max_z_m),
      .minimum_clearance_m =
          interpolate(first.minimum_clearance_m, second.minimum_clearance_m),
      .reference_z_m = interpolate(first.reference_z_m, second.reference_z_m),
      .reference_speed_mps =
          interpolate(first.reference_speed_mps, second.reference_speed_mps),
  };
}

[[nodiscard]] RouteEnvelopeSample
envelopeAtStation(const std::span<const RouteEnvelopeSample> envelope,
                  const double station_m) noexcept {
  if (envelope.empty()) {
    return {.station_m = station_m};
  }
  if (station_m <= envelope.front().station_m) {
    RouteEnvelopeSample result = envelope.front();
    result.station_m = station_m;
    return result;
  }
  for (std::size_t index = 1U; index < envelope.size(); ++index) {
    if (station_m <= envelope[index].station_m) {
      return interpolateEnvelope(envelope[index - 1U], envelope[index], station_m);
    }
  }
  RouteEnvelopeSample result = envelope.back();
  result.station_m = station_m;
  return result;
}

void sortAndDeduplicateEnvelope(std::vector<RouteEnvelopeSample>& envelope) {
  std::ranges::sort(envelope, {}, &RouteEnvelopeSample::station_m);
  envelope.erase(std::unique(envelope.begin(), envelope.end(),
                             [](const RouteEnvelopeSample& first,
                                const RouteEnvelopeSample& second) {
                               return std::abs(first.station_m - second.station_m) <=
                                      1.0e-6;
                             }),
                 envelope.end());
}

} // namespace

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
    const auto make_boundary = [&](const double station_m) {
      return RouteEnvelopeSample{
          .station_m = station_m,
          .lateral_free_left_m = 0.5 * traversal.width_m,
          .lateral_free_right_m = 0.5 * traversal.width_m,
          .min_z_m = traversal.min_z_m,
          .max_z_m = traversal.max_z_m,
          .minimum_clearance_m = traversal.minimum_clearance_m,
          .reference_z_m = sampleRoute3DAtStation(route, station_m).position.z,
          .reference_speed_mps = traversal.speed_limit_mps,
      };
    };
    span.envelope.push_back(make_boundary(span.begin_station_m));
    span.envelope.push_back(make_boundary(span.end_station_m));
    sortAndDeduplicateEnvelope(span.envelope);
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
    sortAndDeduplicateEnvelope(remapped.envelope);
    if (remapped.envelope.empty()) {
      RouteEnvelopeSample fallback = span.envelope.front();
      fallback.station_m = remapped.begin_station_m;
      fallback.reference_z_m =
          sampleRoute3DAtStation(destination_route, remapped.begin_station_m)
              .position.z;
      remapped.envelope.push_back(fallback);
    }
    RouteEnvelopeSample begin_envelope =
        envelopeAtStation(remapped.envelope, remapped.begin_station_m);
    begin_envelope.reference_z_m =
        sampleRoute3DAtStation(destination_route, remapped.begin_station_m).position.z;
    RouteEnvelopeSample end_envelope =
        envelopeAtStation(remapped.envelope, remapped.end_station_m);
    end_envelope.reference_z_m =
        sampleRoute3DAtStation(destination_route, remapped.end_station_m).position.z;
    remapped.envelope.push_back(begin_envelope);
    remapped.envelope.push_back(end_envelope);
    sortAndDeduplicateEnvelope(remapped.envelope);
    result.push_back(std::move(remapped));
  }
  return result;
}

std::vector<ConstrainedRouteSpan>
clipConstrainedRouteSpans(const std::span<const ConstrainedRouteSpan> spans,
                          const double minimum_station_m,
                          const double maximum_station_m) {
  std::vector<ConstrainedRouteSpan> clipped;
  for (const ConstrainedRouteSpan& source : spans) {
    const double begin = std::max(source.begin_station_m, minimum_station_m);
    const double end = std::min(source.end_station_m, maximum_station_m);
    if (end <= begin + 1.0e-9 || source.envelope.empty()) {
      continue;
    }
    ConstrainedRouteSpan span = source;
    span.begin_station_m = begin;
    span.end_station_m = end;
    const RouteEnvelopeSample begin_envelope =
        envelopeAtStation(source.envelope, begin);
    const RouteEnvelopeSample end_envelope = envelopeAtStation(source.envelope, end);
    std::erase_if(span.envelope, [begin, end](const RouteEnvelopeSample& sample) {
      return sample.station_m + 1.0e-6 < begin || sample.station_m - 1.0e-6 > end;
    });
    span.envelope.push_back(begin_envelope);
    span.envelope.push_back(end_envelope);
    sortAndDeduplicateEnvelope(span.envelope);
    std::erase_if(span.segment_spans,
                  [begin, end](PassageTraversalSegmentSpan& segment) {
                    segment.begin_station_m = std::max(segment.begin_station_m, begin);
                    segment.end_station_m = std::min(segment.end_station_m, end);
                    return segment.end_station_m <= segment.begin_station_m + 1.0e-9;
                  });
    clipped.push_back(std::move(span));
  }
  return clipped;
}

void mergeAdjacentConstrainedRouteSpans(std::vector<ConstrainedRouteSpan>& spans) {
  std::ranges::sort(
      spans, [](const ConstrainedRouteSpan& first, const ConstrainedRouteSpan& second) {
        return std::tie(first.begin_station_m, first.end_station_m,
                        first.passage_traversal_id, first.direction_sign) <
               std::tie(second.begin_station_m, second.end_station_m,
                        second.passage_traversal_id, second.direction_sign);
      });
  std::vector<ConstrainedRouteSpan> merged;
  for (ConstrainedRouteSpan& span : spans) {
    if (!merged.empty() &&
        merged.back().passage_traversal_id == span.passage_traversal_id &&
        merged.back().direction_sign == span.direction_sign &&
        span.begin_station_m <= merged.back().end_station_m + 1.0e-6) {
      ConstrainedRouteSpan& previous = merged.back();
      previous.end_station_m = std::max(previous.end_station_m, span.end_station_m);
      previous.envelope.insert(previous.envelope.end(), span.envelope.begin(),
                               span.envelope.end());
      previous.segment_spans.insert(previous.segment_spans.end(),
                                    span.segment_spans.begin(),
                                    span.segment_spans.end());
      sortAndDeduplicateEnvelope(previous.envelope);
      continue;
    }
    merged.push_back(std::move(span));
  }
  spans = std::move(merged);
}

} // namespace drone_city_nav
