#include "drone_city_nav/static_route_geometry.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <optional>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool protectedStation(
    const double station_m, const std::span<const ConstrainedRouteSpan> spans,
    const std::optional<double> frozen_prefix_end_station_m = std::nullopt) noexcept {
  return (frozen_prefix_end_station_m &&
          station_m <= *frozen_prefix_end_station_m + 1.0e-9) ||
         std::ranges::any_of(spans, [station_m](const ConstrainedRouteSpan& span) {
           return station_m + 1.0e-9 >= span.begin_station_m &&
                  station_m <= span.end_station_m + 1.0e-9;
         });
}

[[nodiscard]] Point3 lerpPoint(const Point3& first, const Point3& second,
                               const double ratio) noexcept {
  return Point3{std::lerp(first.x, second.x, ratio),
                std::lerp(first.y, second.y, ratio),
                std::lerp(first.z, second.z, ratio)};
}

[[nodiscard]] Vec3 segmentVector(const Point3& first, const Point3& second) noexcept {
  return Vec3{second.x - first.x, second.y - first.y, second.z - first.z};
}

[[nodiscard]] double vectorNorm(const Vec3& vector) noexcept {
  return std::hypot(std::hypot(vector.x, vector.y), vector.z);
}

[[nodiscard]] double angleBetween(const Vec3& first, const Vec3& second) noexcept {
  const double first_norm = vectorNorm(first);
  const double second_norm = vectorNorm(second);
  if (!(first_norm > 1.0e-9) || !(second_norm > 1.0e-9)) {
    return 0.0;
  }
  const double cosine =
      std::clamp((first.x * second.x + first.y * second.y + first.z * second.z) /
                     (first_norm * second_norm),
                 -1.0, 1.0);
  return std::acos(cosine);
}

[[nodiscard]] bool
segmentValid(const Point3& first, const Point3& second, const mppi::EsdfGrid& grid,
             const std::span<const float> esdf_m,
             const SweptFootprintConfig& footprint_config,
             const StaticRouteGeometryRawValidation* const raw_validation) noexcept {
  if (raw_validation != nullptr && raw_validation->occupancy != nullptr) {
    return validateObservedSweptFootprint(
               *raw_validation->occupancy, first, FootprintBodyAxis{}, second,
               FootprintBodyAxis{}, footprint_config, raw_validation->policy,
               raw_validation->proprioceptive_free_space_seed,
               raw_validation->launch_support_contact)
        .accepted();
  }
  if (raw_validation != nullptr && raw_validation->static_occupancy != nullptr) {
    return validateKnownStaticSweptFootprint(*raw_validation->static_occupancy, first,
                                             FootprintBodyAxis{}, second,
                                             FootprintBodyAxis{}, footprint_config)
        .accepted();
  }
  return validateSweptFootprint(grid, esdf_m, first, second, footprint_config)
      .accepted();
}

[[nodiscard]] double pointSegmentDistance(const Point3& point, const Point3& first,
                                          const Point3& second) noexcept {
  const Vec3 segment = segmentVector(first, second);
  const double squared_length =
      segment.x * segment.x + segment.y * segment.y + segment.z * segment.z;
  if (!(squared_length > 1.0e-12)) {
    return distance3D(point, first);
  }
  const Vec3 offset = segmentVector(first, point);
  const double ratio =
      std::clamp((offset.x * segment.x + offset.y * segment.y + offset.z * segment.z) /
                     squared_length,
                 0.0, 1.0);
  return distance3D(point, lerpPoint(first, second, ratio));
}

[[nodiscard]] std::vector<std::size_t>
sparseRouteIndices(const std::span<const RouteSample3D> route,
                   const std::span<const ConstrainedRouteSpan> constrained_spans,
                   const double deviation_tolerance_m,
                   const std::optional<double> frozen_prefix_end_station_m) {
  std::vector<bool> retained(route.size(), false);
  retained.front() = true;
  retained.back() = true;
  for (std::size_t index = 0U; index < route.size(); ++index) {
    retained[index] =
        retained[index] || protectedStation(route[index].station_m, constrained_spans,
                                            frozen_prefix_end_station_m);
  }
  for (std::size_t index = 1U; index + 1U < route.size(); ++index) {
    const Vec3 incoming =
        segmentVector(route[index - 1U].position, route[index].position);
    const Vec3 outgoing =
        segmentVector(route[index].position, route[index + 1U].position);
    const double dot =
        incoming.x * outgoing.x + incoming.y * outgoing.y + incoming.z * outgoing.z;
    if (dot <= 0.0) {
      retained[index] = true;
    }
  }

  std::vector<std::size_t> mandatory;
  mandatory.reserve(route.size());
  for (std::size_t index = 0U; index < retained.size(); ++index) {
    if (retained[index]) {
      mandatory.push_back(index);
    }
  }
  for (std::size_t segment = 1U; segment < mandatory.size(); ++segment) {
    std::vector<std::pair<std::size_t, std::size_t>> pending{
        {mandatory[segment - 1U], mandatory[segment]}};
    while (!pending.empty()) {
      const auto [begin, end] = pending.back();
      pending.pop_back();
      if (end <= begin + 1U) {
        continue;
      }
      double maximum_deviation_m = 0.0;
      std::size_t maximum_index = begin;
      for (std::size_t index = begin + 1U; index < end; ++index) {
        const double deviation_m = pointSegmentDistance(
            route[index].position, route[begin].position, route[end].position);
        if (deviation_m > maximum_deviation_m) {
          maximum_deviation_m = deviation_m;
          maximum_index = index;
        }
      }
      if (maximum_deviation_m <= deviation_tolerance_m || maximum_index == begin) {
        continue;
      }
      retained[maximum_index] = true;
      pending.emplace_back(begin, maximum_index);
      pending.emplace_back(maximum_index, end);
    }
  }

  std::vector<std::size_t> sparse;
  sparse.reserve(route.size());
  for (std::size_t index = 0U; index < retained.size(); ++index) {
    if (retained[index]) {
      sparse.push_back(index);
    }
  }
  return sparse;
}

[[nodiscard]] double originalTurn(const std::span<const RouteSample3D> route,
                                  const std::size_t begin,
                                  const std::size_t end) noexcept {
  double result = 0.0;
  const std::size_t first_corner = begin == 0U ? 1U : begin;
  const std::size_t last_corner =
      std::min(end, route.size() > 1U ? route.size() - 2U : 0U);
  for (std::size_t corner = first_corner; corner <= last_corner; ++corner) {
    result += angleBetween(
        segmentVector(route[corner - 1U].position, route[corner].position),
        segmentVector(route[corner].position, route[corner + 1U].position));
  }
  return result;
}

[[nodiscard]] double shortcutTurn(const std::span<const RouteSample3D> route,
                                  const std::size_t begin,
                                  const std::size_t end) noexcept {
  const Vec3 shortcut = segmentVector(route[begin].position, route[end].position);
  double result = 0.0;
  if (begin > 0U) {
    result += angleBetween(
        segmentVector(route[begin - 1U].position, route[begin].position), shortcut);
  }
  if (end + 1U < route.size()) {
    result += angleBetween(
        shortcut, segmentVector(route[end].position, route[end + 1U].position));
  }
  return result;
}

[[nodiscard]] bool
shortcutWithinTurnBudget(const std::span<const RouteSample3D> route,
                         const std::size_t begin, const std::size_t end,
                         const double maximum_turn_increase_rad) noexcept {
  return shortcutTurn(route, begin, end) <=
         originalTurn(route, begin, end) + maximum_turn_increase_rad + 1.0e-9;
}

[[nodiscard]] std::size_t
maximumShortcutIndex(const std::span<const RouteSample3D> route,
                     const std::size_t current,
                     const std::span<const ConstrainedRouteSpan> constrained_spans,
                     const double maximum_shortcut_length_m,
                     const std::optional<double> frozen_prefix_end_station_m) noexcept {
  std::size_t maximum = current + 1U;
  while (maximum + 1U < route.size() &&
         route[maximum + 1U].station_m - route[current].station_m <=
             maximum_shortcut_length_m + 1.0e-9 &&
         !protectedStation(route[maximum + 1U].station_m, constrained_spans,
                           frozen_prefix_end_station_m)) {
    ++maximum;
  }
  return maximum;
}

[[nodiscard]] std::vector<std::size_t> shortcutCandidateIndices(
    const std::span<const RouteSample3D> route,
    const std::span<const std::size_t> sparse_indices, const std::size_t current,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const StaticRouteGeometryConfig& config, std::size_t& turn_budget_rejections) {
  std::vector<std::size_t> candidates;
  if (protectedStation(route[current].station_m, constrained_spans,
                       config.frozen_prefix_end_station_m)) {
    return candidates;
  }
  const std::size_t maximum = maximumShortcutIndex(route, current, constrained_spans,
                                                   config.maximum_shortcut_length_m,
                                                   config.frozen_prefix_end_station_m);
  if (maximum <= current + 1U) {
    return candidates;
  }
  candidates.push_back(maximum);
  for (const std::size_t index : sparse_indices) {
    if (index > current + 1U && index <= maximum) {
      candidates.push_back(index);
    }
  }
  std::ranges::sort(candidates, std::greater<>{});
  candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
  std::erase_if(candidates, [&](const std::size_t candidate) {
    const bool accepted = shortcutWithinTurnBudget(
        route, current, candidate, config.maximum_shortcut_turn_increase_rad);
    turn_budget_rejections += static_cast<std::size_t>(!accepted);
    return !accepted;
  });
  return candidates;
}

[[nodiscard]] std::optional<std::vector<Point3>>
smoothCorner(const Point3& previous, const Point3& corner, const Point3& next,
             const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
             const SweptFootprintConfig& footprint_config,
             const StaticRouteGeometryConfig& geometry_config,
             const StaticRouteGeometryRawValidation* const raw_validation) {
  const double incoming_length = distance3D(previous, corner);
  const double outgoing_length = distance3D(corner, next);
  const double smoothing_m = std::min({geometry_config.corner_smoothing_distance_m,
                                       incoming_length * 0.4, outgoing_length * 0.4});
  // The final route is resampled after corner materialization. A fillet must
  // therefore not be rejected merely because its radius is smaller than the
  // sampling interval; that rejected valid fillet is what leaves a raw-safe
  // Manhattan corner as an execution-time tangent discontinuity.
  if (!(smoothing_m > 1.0e-3)) {
    return std::nullopt;
  }
  const Point3 entry = lerpPoint(corner, previous, smoothing_m / incoming_length);
  const Point3 exit = lerpPoint(corner, next, smoothing_m / outgoing_length);
  const std::size_t samples =
      std::max<std::size_t>(2U, geometry_config.corner_curve_samples);
  std::vector<Point3> curve;
  curve.reserve(samples + 1U);
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double t = static_cast<double>(sample) / static_cast<double>(samples);
    const double one_minus_t = 1.0 - t;
    curve.push_back(Point3{
        one_minus_t * one_minus_t * entry.x + 2.0 * one_minus_t * t * corner.x +
            t * t * exit.x,
        one_minus_t * one_minus_t * entry.y + 2.0 * one_minus_t * t * corner.y +
            t * t * exit.y,
        one_minus_t * one_minus_t * entry.z + 2.0 * one_minus_t * t * corner.z +
            t * t * exit.z,
    });
  }
  for (std::size_t index = 1U; index < curve.size(); ++index) {
    if (!segmentValid(curve[index - 1U], curve[index], grid, esdf_m, footprint_config,
                      raw_validation)) {
      return std::nullopt;
    }
  }
  return curve;
}

} // namespace

StaticRouteGeometryResult optimizeStaticRouteGeometry(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const SweptFootprintConfig& footprint_config,
    const StaticRouteGeometryConfig& geometry_config,
    const RouteEnvelopeConfig& envelope_config, BoundedWorkerPool* const worker_pool,
    const StaticRouteGeometryRawValidation* const raw_validation) {
  StaticRouteGeometryResult result;
  if (!geometry_config.enabled) {
    result.route.assign(route.begin(), route.end());
    result.constrained_spans.assign(constrained_spans.begin(), constrained_spans.end());
    result.sparse_anchor_count = route.size();
    return result;
  }
  if (route.size() < 2U) {
    result.route.assign(route.begin(), route.end());
    result.sparse_anchor_count = route.size();
    return result;
  }

  const std::vector<std::size_t> sparse_indices = sparseRouteIndices(
      route, constrained_spans, geometry_config.sparse_deviation_tolerance_m,
      geometry_config.frozen_prefix_end_station_m);
  result.sparse_anchor_count = sparse_indices.size();
  result.sparse_samples_removed = route.size() - sparse_indices.size();
  std::vector<Point3> anchors;
  std::vector<double> anchor_stations_m;
  anchors.reserve(route.size());
  anchor_stations_m.reserve(route.size());
  const auto shortcut_validation_started = std::chrono::steady_clock::now();
  if (!geometry_config.shortcut_optimization_enabled) {
    for (const std::size_t index : sparse_indices) {
      anchors.push_back(route[index].position);
      anchor_stations_m.push_back(route[index].station_m);
    }
  } else {
    anchors.push_back(route.front().position);
    anchor_stations_m.push_back(route.front().station_m);
    std::size_t current = 0U;
    while (current + 1U < route.size()) {
      std::size_t selected = current + 1U;
      const std::vector<std::size_t> candidates = shortcutCandidateIndices(
          route, sparse_indices, current, constrained_spans, geometry_config,
          result.shortcut_turn_budget_rejections);
      const std::size_t batch_size =
          std::max<std::size_t>(1U, geometry_config.shortcut_validation_batch_size);
      for (std::size_t batch_begin = 0U; batch_begin < candidates.size();
           batch_begin += batch_size) {
        const std::size_t candidate_count =
            std::min(batch_size, candidates.size() - batch_begin);
        std::vector<std::uint8_t> accepted(candidate_count, 0U);
        const auto validate_candidate = [&](const std::size_t batch_index) {
          const std::size_t candidate = candidates[batch_begin + batch_index];
          accepted[batch_index] = static_cast<std::uint8_t>(
              segmentValid(route[current].position, route[candidate].position, grid,
                           esdf_m, footprint_config, raw_validation));
        };
        const bool parallel = worker_pool != nullptr &&
                              worker_pool->canParallelizeFromCurrentThread() &&
                              candidate_count > 1U;
        if (parallel) {
          worker_pool->parallelFor(candidate_count, WorkerTaskLane::kRouteCritical,
                                   validate_candidate);
          result.parallel_shortcut_candidates += candidate_count;
        } else {
          for (std::size_t batch_index = 0U; batch_index < candidate_count;
               ++batch_index) {
            validate_candidate(batch_index);
          }
        }
        ++result.shortcut_validation_batches;
        result.shortcut_candidates += candidate_count;
        const auto accepted_candidate = std::ranges::find(accepted, std::uint8_t{1U});
        if (accepted_candidate != accepted.end()) {
          const std::size_t batch_index = static_cast<std::size_t>(
              std::distance(accepted.begin(), accepted_candidate));
          selected = candidates[batch_begin + batch_index];
          break;
        }
      }
      if (selected > current + 1U) {
        ++result.shortcuts_applied;
      }
      anchors.push_back(route[selected].position);
      anchor_stations_m.push_back(route[selected].station_m);
      current = selected;
    }
  }
  result.shortcut_validation_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                shortcut_validation_started)
          .count();

  const auto corner_validation_started = std::chrono::steady_clock::now();
  std::vector<std::size_t> corner_candidates;
  for (std::size_t index = 1U; index + 1U < anchors.size(); ++index) {
    // Constrained spans retain their safety contract through the same raw
    // swept-footprint validation as every other candidate.  Excluding their
    // corners here left right-angle passages with a tangent discontinuity.
    if (!geometry_config.frozen_prefix_end_station_m ||
        anchor_stations_m[index] >
            *geometry_config.frozen_prefix_end_station_m + 1.0e-9) {
      corner_candidates.push_back(index);
    }
  }
  std::vector<std::optional<std::vector<Point3>>> curves(anchors.size());
  const auto validate_corner = [&](const std::size_t candidate_index) {
    const std::size_t index = corner_candidates[candidate_index];
    curves[index] =
        smoothCorner(anchors[index - 1U], anchors[index], anchors[index + 1U], grid,
                     esdf_m, footprint_config, geometry_config, raw_validation);
  };
  const bool corners_parallel = worker_pool != nullptr &&
                                worker_pool->canParallelizeFromCurrentThread() &&
                                corner_candidates.size() > 1U;
  if (corners_parallel) {
    worker_pool->parallelFor(corner_candidates.size(), WorkerTaskLane::kRouteCritical,
                             validate_corner);
    result.parallel_corner_candidates = corner_candidates.size();
  } else {
    for (std::size_t candidate_index = 0U; candidate_index < corner_candidates.size();
         ++candidate_index) {
      validate_corner(candidate_index);
    }
  }
  result.corner_candidates = corner_candidates.size();
  result.corner_validation_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                corner_validation_started)
          .count();
  std::vector<Point3> smoothed;
  smoothed.reserve(anchors.size() * 2U);
  smoothed.push_back(anchors.front());
  for (std::size_t index = 1U; index + 1U < anchors.size(); ++index) {
    const std::optional<std::vector<Point3>>& curve = curves[index];
    if (!curve.has_value()) {
      smoothed.push_back(anchors[index]);
      continue;
    }
    for (const Point3& point : *curve) {
      if (distance3D(smoothed.back(), point) > 1.0e-6) {
        smoothed.push_back(point);
      }
    }
    ++result.corners_smoothed;
  }
  if (distance3D(smoothed.back(), anchors.back()) > 1.0e-6) {
    smoothed.push_back(anchors.back());
  }
  result.route = sampleRoute3D(smoothed, geometry_config.sample_step_m,
                               route.front().reference_speed_mps);

  result.constrained_spans.clear();
  result.constrained_spans.reserve(constrained_spans.size());
  for (const ConstrainedRouteSpan& span : constrained_spans) {
    if (span.envelope.empty()) {
      continue;
    }
    const Point3 old_entry =
        sampleRoute3DAtStation(route, span.begin_station_m).position;
    const Point3 old_exit = sampleRoute3DAtStation(route, span.end_station_m).position;
    const RouteProjection3D new_entry = projectOntoRoute3D(result.route, old_entry);
    const RouteProjection3D new_exit = projectOntoRoute3D(result.route, old_exit);
    if (!new_entry.valid || !new_exit.valid ||
        new_exit.station_m - new_entry.station_m <
            envelope_config.minimum_span_length_m) {
      continue;
    }
    ConstrainedRouteSpan transformed = span;
    transformed.begin_station_m = new_entry.station_m;
    transformed.end_station_m = new_exit.station_m;
    transformed.segment_spans.clear();
    transformed.segment_spans.reserve(span.segment_spans.size());
    for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
      const Point3 old_segment_begin =
          sampleRoute3DAtStation(route, segment.begin_station_m).position;
      const Point3 old_segment_end =
          sampleRoute3DAtStation(route, segment.end_station_m).position;
      const RouteProjection3D new_segment_begin =
          projectOntoRoute3D(result.route, old_segment_begin, new_entry.station_m);
      const RouteProjection3D new_segment_end = projectOntoRoute3D(
          result.route, old_segment_end,
          new_segment_begin.valid ? new_segment_begin.station_m : new_entry.station_m);
      if (!new_segment_begin.valid || !new_segment_end.valid ||
          new_segment_end.station_m <= new_segment_begin.station_m + 1.0e-9) {
        continue;
      }
      transformed.segment_spans.push_back(PassageTraversalSegmentSpan{
          .passage_segment_id = segment.passage_segment_id,
          .begin_station_m =
              std::clamp(new_segment_begin.station_m, transformed.begin_station_m,
                         transformed.end_station_m),
          .end_station_m =
              std::clamp(new_segment_end.station_m, transformed.begin_station_m,
                         transformed.end_station_m),
      });
    }
    transformed.envelope.clear();
    transformed.envelope.reserve(span.envelope.size());
    for (const RouteEnvelopeSample& envelope : span.envelope) {
      const Point3 old_position =
          sampleRoute3DAtStation(route, envelope.station_m).position;
      const RouteProjection3D projected =
          projectOntoRoute3D(result.route, old_position, new_entry.station_m);
      if (!projected.valid ||
          projected.station_m + 1.0e-6 < transformed.begin_station_m ||
          projected.station_m - 1.0e-6 > transformed.end_station_m) {
        continue;
      }
      RouteEnvelopeSample remapped = envelope;
      remapped.station_m = std::clamp(projected.station_m, transformed.begin_station_m,
                                      transformed.end_station_m);
      remapped.reference_z_m =
          sampleRoute3DAtStation(result.route, remapped.station_m).position.z;
      transformed.envelope.push_back(remapped);
    }
    std::ranges::sort(transformed.envelope, {}, &RouteEnvelopeSample::station_m);
    transformed.envelope.erase(
        std::unique(
            transformed.envelope.begin(), transformed.envelope.end(),
            [](const RouteEnvelopeSample& first, const RouteEnvelopeSample& second) {
              return std::abs(first.station_m - second.station_m) <= 1.0e-6;
            }),
        transformed.envelope.end());
    if (transformed.envelope.empty()) {
      RouteEnvelopeSample fallback = span.envelope.front();
      fallback.station_m = transformed.begin_station_m;
      fallback.reference_z_m =
          sampleRoute3DAtStation(result.route, transformed.begin_station_m).position.z;
      transformed.envelope.push_back(fallback);
    }
    result.constrained_spans.push_back(std::move(transformed));
  }
  return result;
}

} // namespace drone_city_nav
