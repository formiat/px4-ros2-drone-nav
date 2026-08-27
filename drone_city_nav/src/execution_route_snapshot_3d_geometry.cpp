#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <numeric>
#include <ranges>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav::execution_route_snapshot_3d_internal {

} // namespace drone_city_nav::execution_route_snapshot_3d_internal

namespace drone_city_nav {

const char* executionRouteGeometryFailureReasonName3D(
    const ExecutionRouteGeometryFailureReason3D reason) noexcept {
  switch (reason) {
    case ExecutionRouteGeometryFailureReason3D::kNotAttempted:
      return "not_attempted";
    case ExecutionRouteGeometryFailureReason3D::kValid:
      return "valid";
    case ExecutionRouteGeometryFailureReason3D::kMissingRoute:
      return "missing_route";
    case ExecutionRouteGeometryFailureReason3D::kTooFewSamples:
      return "too_few_samples";
    case ExecutionRouteGeometryFailureReason3D::kNonFiniteSample:
      return "non_finite_sample";
    case ExecutionRouteGeometryFailureReason3D::kInvalidTangent:
      return "invalid_tangent";
    case ExecutionRouteGeometryFailureReason3D::kNonMonotonicStation:
      return "non_monotonic_station";
    case ExecutionRouteGeometryFailureReason3D::kSegmentStationMismatch:
      return "segment_station_mismatch";
    case ExecutionRouteGeometryFailureReason3D::kIncomingTangentMismatch:
      return "incoming_tangent_mismatch";
    case ExecutionRouteGeometryFailureReason3D::kTerminalTangentMismatch:
      return "terminal_tangent_mismatch";
    case ExecutionRouteGeometryFailureReason3D::kInvalidTimeProfile:
      return "invalid_time_profile";
    case ExecutionRouteGeometryFailureReason3D::kInvalidProjection:
      return "invalid_projection";
    case ExecutionRouteGeometryFailureReason3D::kInvalidConstrainedSpans:
      return "invalid_constrained_spans";
    case ExecutionRouteGeometryFailureReason3D::kInvalidPassageResources:
      return "invalid_passage_resources";
    case ExecutionRouteGeometryFailureReason3D::kInvalidFingerprint:
      return "invalid_fingerprint";
    case ExecutionRouteGeometryFailureReason3D::kDerivedResourceMismatch:
      return "derived_resource_mismatch";
  }
  return "unknown";
}

ExecutionRouteGeometryValidation3D validateExecutionRouteGeometrySamples3D(
    const std::span<const RouteSample3D> route) noexcept {
  using Failure = ExecutionRouteGeometryFailureReason3D;
  if (route.size() < 2U) {
    return {Failure::kTooFewSamples, route.size()};
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  Point3 previous_position{};
  Vec3 previous_tangent{};
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const RouteSample3D& sample = route[index];
    const double tangent_norm =
        std::hypot(std::hypot(sample.tangent.x, sample.tangent.y), sample.tangent.z);
    if (!execution_route_snapshot_3d_internal::finitePoint(sample.position) ||
        !execution_route_snapshot_3d_internal::finiteVector(sample.tangent) ||
        !std::isfinite(sample.station_m) ||
        !std::isfinite(sample.reference_speed_mps) || sample.station_m < 0.0 ||
        sample.reference_speed_mps < 0.0 ||
        (sample.transition == RouteKinematicTransition3D::kStopAndTurn &&
         sample.reference_speed_mps > 1.0e-6)) {
      return {Failure::kNonFiniteSample, index};
    }
    if (!execution_route_snapshot_3d_internal::nearlyEqual(tangent_norm, 1.0, 1.0e-3)) {
      return {Failure::kInvalidTangent, index};
    }
    if (sample.station_m <= previous_station_m) {
      return {Failure::kNonMonotonicStation, index};
    }
    if (index == 0U) {
      if (!execution_route_snapshot_3d_internal::nearlyEqual(
              sample.station_m, 0.0,
              execution_route_snapshot_3d_internal::kStationToleranceM)) {
        return {Failure::kSegmentStationMismatch, index};
      }
    } else {
      const double segment_length_m = distance3D(previous_position, sample.position);
      const double station_delta_m = sample.station_m - previous_station_m;
      if (segment_length_m <=
              execution_route_snapshot_3d_internal::kStationToleranceM ||
          !execution_route_snapshot_3d_internal::nearlyEqual(
              station_delta_m, segment_length_m, 1.0e-4)) {
        return {Failure::kSegmentStationMismatch, index};
      }
      const Vec3 segment_direction{
          (sample.position.x - previous_position.x) / segment_length_m,
          (sample.position.y - previous_position.y) / segment_length_m,
          (sample.position.z - previous_position.z) / segment_length_m};
      const auto dot = [](const Vec3& first, const Vec3& second) noexcept {
        return first.x * second.x + first.y * second.y + first.z * second.z;
      };
      if (dot(previous_tangent, segment_direction) <= 0.0) {
        return {Failure::kIncomingTangentMismatch, index};
      }
      if (index + 1U == route.size() && dot(sample.tangent, segment_direction) <= 0.0) {
        return {Failure::kTerminalTangentMismatch, index};
      }
    }
    previous_station_m = sample.station_m;
    previous_position = sample.position;
    previous_tangent = sample.tangent;
  }
  return {Failure::kValid, route.size() - 1U};
}

} // namespace drone_city_nav

namespace drone_city_nav::execution_route_snapshot_3d_internal {

[[nodiscard]] bool
validRouteSamples(const std::span<const RouteSample3D> route) noexcept {
  return validateExecutionRouteGeometrySamples3D(route).valid();
}

[[nodiscard]] double vectorNorm(const Vec3& vector) noexcept {
  return std::hypot(std::hypot(vector.x, vector.y), vector.z);
}

[[nodiscard]] double vectorDot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] bool validEnvelopeSamples(const ConstrainedRouteSpan& span) noexcept {
  if (span.envelope.empty()) {
    return false;
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  for (const RouteEnvelopeSample& sample : span.envelope) {
    if (!std::isfinite(sample.station_m) ||
        !std::isfinite(sample.lateral_free_left_m) ||
        !std::isfinite(sample.lateral_free_right_m) || !std::isfinite(sample.min_z_m) ||
        !std::isfinite(sample.max_z_m) || !std::isfinite(sample.minimum_clearance_m) ||
        !std::isfinite(sample.reference_z_m) ||
        !std::isfinite(sample.reference_speed_mps) ||
        sample.station_m <= previous_station_m ||
        sample.station_m + kStationToleranceM < span.begin_station_m ||
        sample.station_m > span.end_station_m + kStationToleranceM ||
        sample.lateral_free_left_m < 0.0 || sample.lateral_free_right_m < 0.0 ||
        sample.min_z_m > sample.max_z_m ||
        sample.reference_z_m < sample.min_z_m - kGeometryTolerance ||
        sample.reference_z_m > sample.max_z_m + kGeometryTolerance ||
        sample.minimum_clearance_m < 0.0 || sample.reference_speed_mps < 0.0) {
      return false;
    }
    previous_station_m = sample.station_m;
  }
  return span.envelope.front().station_m <= span.begin_station_m + kStationToleranceM &&
         span.envelope.back().station_m + kStationToleranceM >= span.end_station_m;
}

[[nodiscard]] bool
validTraversalSegmentSpans(const ConstrainedRouteSpan& span) noexcept {
  double previous_end_station_m{span.begin_station_m};
  for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
    if (segment.passage_segment_id.empty() || !std::isfinite(segment.begin_station_m) ||
        !std::isfinite(segment.end_station_m) ||
        segment.begin_station_m + kStationToleranceM < span.begin_station_m ||
        segment.end_station_m > span.end_station_m + kStationToleranceM ||
        segment.end_station_m <= segment.begin_station_m ||
        segment.begin_station_m + kStationToleranceM < previous_end_station_m) {
      return false;
    }
    previous_end_station_m = segment.end_station_m;
  }
  return true;
}

[[nodiscard]] bool
validPassageCrossSection(const PassageCrossSection& section) noexcept {
  const double tangent_norm = vectorNorm(section.tangent);
  const double lateral_norm = vectorNorm(section.lateral_axis);
  const double secondary_norm = vectorNorm(section.secondary_axis);
  const Vec3 tangent_cross_lateral{section.tangent.y * section.lateral_axis.z -
                                       section.tangent.z * section.lateral_axis.y,
                                   section.tangent.z * section.lateral_axis.x -
                                       section.tangent.x * section.lateral_axis.z,
                                   section.tangent.x * section.lateral_axis.y -
                                       section.tangent.y * section.lateral_axis.x};
  return std::isfinite(section.station_m) && finitePoint(section.center) &&
         finiteVector(section.tangent) && finiteVector(section.lateral_axis) &&
         finiteVector(section.secondary_axis) &&
         nearlyEqual(tangent_norm, 1.0, 1.0e-3) &&
         nearlyEqual(lateral_norm, 1.0, 1.0e-3) &&
         nearlyEqual(secondary_norm, 1.0, 1.0e-3) &&
         std::abs(vectorDot(section.tangent, section.lateral_axis)) <= 1.0e-3 &&
         std::abs(vectorDot(section.tangent, section.secondary_axis)) <= 1.0e-3 &&
         std::abs(vectorDot(section.lateral_axis, section.secondary_axis)) <= 1.0e-3 &&
         vectorDot(tangent_cross_lateral, section.secondary_axis) >= 0.999 &&
         std::isfinite(section.minimum_lateral_offset_m) &&
         std::isfinite(section.maximum_lateral_offset_m) &&
         std::isfinite(section.minimum_secondary_offset_m) &&
         std::isfinite(section.maximum_secondary_offset_m) &&
         section.minimum_lateral_offset_m <= section.maximum_lateral_offset_m &&
         section.minimum_secondary_offset_m <= section.maximum_secondary_offset_m &&
         section.raw_validated;
}

[[nodiscard]] bool validPassageVolume(const PassageVolume& volume,
                                      const ConstrainedRouteSpan& span,
                                      const std::size_t span_index) noexcept {
  if (volume.span_index != span_index ||
      volume.passage_traversal_id != span.passage_traversal_id ||
      !volume.raw_validated || volume.cross_sections.empty() ||
      !std::isfinite(volume.begin_station_m) || !std::isfinite(volume.end_station_m) ||
      !nearlyEqual(volume.begin_station_m, span.begin_station_m, kStationToleranceM) ||
      !nearlyEqual(volume.end_station_m, span.end_station_m, kStationToleranceM) ||
      !std::isfinite(volume.minimum_lateral_offset_m) ||
      !std::isfinite(volume.maximum_lateral_offset_m) ||
      !std::isfinite(volume.minimum_secondary_offset_m) ||
      !std::isfinite(volume.maximum_secondary_offset_m) ||
      volume.minimum_lateral_offset_m > volume.maximum_lateral_offset_m ||
      volume.minimum_secondary_offset_m > volume.maximum_secondary_offset_m ||
      !std::isfinite(volume.minimum_physical_width_m) ||
      !std::isfinite(volume.minimum_physical_secondary_extent_m) ||
      volume.minimum_physical_width_m < 0.0 ||
      volume.minimum_physical_secondary_extent_m < 0.0 ||
      volume.segment_spans != span.segment_spans) {
    return false;
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  const PassageCrossSection* previous_section{nullptr};
  for (const PassageCrossSection& section : volume.cross_sections) {
    if (!validPassageCrossSection(section) || section.station_m <= previous_station_m ||
        section.station_m + kStationToleranceM < volume.begin_station_m ||
        section.station_m > volume.end_station_m + kStationToleranceM ||
        (previous_section != nullptr &&
         (vectorDot(previous_section->lateral_axis, section.lateral_axis) <= 0.0 ||
          vectorDot(previous_section->secondary_axis, section.secondary_axis) <=
              0.0))) {
      return false;
    }
    previous_station_m = section.station_m;
    previous_section = &section;
  }
  return volume.cross_sections.front().station_m <=
             volume.begin_station_m + kStationToleranceM &&
         volume.cross_sections.back().station_m + kStationToleranceM >=
             volume.end_station_m;
}

[[nodiscard]] bool samePointExact(const Point3& first, const Point3& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

[[nodiscard]] bool sameVectorExact(const Vec3& first, const Vec3& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

[[nodiscard]] bool
samePassageCrossSectionExact(const PassageCrossSection& first,
                             const PassageCrossSection& second) noexcept {
  return first.station_m == second.station_m &&
         samePointExact(first.center, second.center) &&
         sameVectorExact(first.tangent, second.tangent) &&
         sameVectorExact(first.lateral_axis, second.lateral_axis) &&
         sameVectorExact(first.secondary_axis, second.secondary_axis) &&
         first.minimum_lateral_offset_m == second.minimum_lateral_offset_m &&
         first.maximum_lateral_offset_m == second.maximum_lateral_offset_m &&
         first.minimum_secondary_offset_m == second.minimum_secondary_offset_m &&
         first.maximum_secondary_offset_m == second.maximum_secondary_offset_m &&
         first.raw_validated == second.raw_validated;
}

[[nodiscard]] bool samePassageVolumeExact(const PassageVolume& first,
                                          const PassageVolume& second) noexcept {
  return first.passage_traversal_id == second.passage_traversal_id &&
         first.span_index == second.span_index &&
         first.begin_station_m == second.begin_station_m &&
         first.end_station_m == second.end_station_m &&
         first.minimum_lateral_offset_m == second.minimum_lateral_offset_m &&
         first.maximum_lateral_offset_m == second.maximum_lateral_offset_m &&
         first.minimum_secondary_offset_m == second.minimum_secondary_offset_m &&
         first.maximum_secondary_offset_m == second.maximum_secondary_offset_m &&
         first.minimum_physical_width_m == second.minimum_physical_width_m &&
         first.minimum_physical_secondary_extent_m ==
             second.minimum_physical_secondary_extent_m &&
         first.raw_validated == second.raw_validated &&
         first.segment_spans == second.segment_spans &&
         std::ranges::equal(first.cross_sections, second.cross_sections,
                            samePassageCrossSectionExact);
}

[[nodiscard]] bool
sameRouteEnvelopeSampleExact(const RouteEnvelopeSample& first,
                             const RouteEnvelopeSample& second) noexcept {
  return first.station_m == second.station_m &&
         first.lateral_free_left_m == second.lateral_free_left_m &&
         first.lateral_free_right_m == second.lateral_free_right_m &&
         first.min_z_m == second.min_z_m && first.max_z_m == second.max_z_m &&
         first.minimum_clearance_m == second.minimum_clearance_m &&
         first.reference_z_m == second.reference_z_m &&
         first.reference_speed_mps == second.reference_speed_mps;
}

[[nodiscard]] bool
canonicalPassageGeometryMatchesWorld(const ExecutionRouteGeometry3D& geometry,
                                     const OccupancyGrid3D& occupancy,
                                     const PassageVolumeConfig& expected_config) {
  if (!samePassageVolumeConfig(geometry.passage_volume_config, expected_config)) {
    return false;
  }
  const std::vector<ConstrainedRouteSpan>& spans = *geometry.constrained_spans;
  const std::vector<PassageVolume>& claimed_volumes = *geometry.passage_volumes;
  if (spans.empty()) {
    return claimed_volumes.empty();
  }
  const std::vector<PassageVolume> canonical_volumes =
      derivePassageVolumes(*geometry.route, spans, occupancy, expected_config);
  if (canonical_volumes.size() != claimed_volumes.size() ||
      !std::ranges::equal(canonical_volumes, claimed_volumes, samePassageVolumeExact)) {
    return false;
  }
  std::vector<ConstrainedRouteSpan> canonical_spans = spans;
  if (projectPassageVolumeEnvelopes(canonical_spans, canonical_volumes,
                                    expected_config.footprint) != spans.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < spans.size(); ++index) {
    if (!std::ranges::equal(canonical_spans[index].envelope, spans[index].envelope,
                            sameRouteEnvelopeSampleExact)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool canonicalPassageGeometryMatchesObservedWorld(
    const ExecutionRouteGeometry3D& geometry, const VersionedObservedRawWorld3D& world,
    const PassageVolumeConfig& expected_config) {
  if (geometry.constrained_spans->empty()) {
    return geometry.passage_volumes->empty() &&
           samePassageVolumeConfig(geometry.passage_volume_config, expected_config);
  }
  const std::shared_ptr<const OccupancyGrid3D> occupied_snapshot =
      world.occupiedSnapshot();
  return world.valid() && occupied_snapshot != nullptr &&
         canonicalPassageGeometryMatchesWorld(geometry, *occupied_snapshot,
                                              expected_config);
}

[[nodiscard]] Vec3 normalizedVector(const Vec3& vector) noexcept {
  const double norm = vectorNorm(vector);
  return norm > kStationToleranceM
             ? Vec3{vector.x / norm, vector.y / norm, vector.z / norm}
             : Vec3{};
}

[[nodiscard]] PassageFrame3D
passageFrameBetweenSections(const PassageCrossSection& lower_section,
                            const PassageCrossSection& upper_section,
                            const double station_m) noexcept {
  const double station_interval_m = upper_section.station_m - lower_section.station_m;
  const double ratio =
      station_interval_m > 0.0
          ? std::clamp((station_m - lower_section.station_m) / station_interval_m, 0.0,
                       1.0)
          : 0.0;
  const auto interpolate = [ratio](const double first, const double second) noexcept {
    return std::lerp(first, second, ratio);
  };
  const Vec3 lateral = normalizedVector(
      Vec3{interpolate(lower_section.lateral_axis.x, upper_section.lateral_axis.x),
           interpolate(lower_section.lateral_axis.y, upper_section.lateral_axis.y),
           interpolate(lower_section.lateral_axis.z, upper_section.lateral_axis.z)});
  const Vec3 secondary = normalizedVector(Vec3{
      interpolate(lower_section.secondary_axis.x, upper_section.secondary_axis.x),
      interpolate(lower_section.secondary_axis.y, upper_section.secondary_axis.y),
      interpolate(lower_section.secondary_axis.z, upper_section.secondary_axis.z)});
  const Vec3 tangent = normalizedVector(
      Vec3{interpolate(lower_section.tangent.x, upper_section.tangent.x),
           interpolate(lower_section.tangent.y, upper_section.tangent.y),
           interpolate(lower_section.tangent.z, upper_section.tangent.z)});
  const double minimum_lateral_offset_m = std::max(
      lower_section.minimum_lateral_offset_m, upper_section.minimum_lateral_offset_m);
  const double maximum_lateral_offset_m = std::min(
      lower_section.maximum_lateral_offset_m, upper_section.maximum_lateral_offset_m);
  const double minimum_secondary_offset_m =
      std::max(lower_section.minimum_secondary_offset_m,
               upper_section.minimum_secondary_offset_m);
  const double maximum_secondary_offset_m =
      std::min(lower_section.maximum_secondary_offset_m,
               upper_section.maximum_secondary_offset_m);
  return PassageFrame3D{
      .center = Point3{interpolate(lower_section.center.x, upper_section.center.x),
                       interpolate(lower_section.center.y, upper_section.center.y),
                       interpolate(lower_section.center.z, upper_section.center.z)},
      .tangent = tangent,
      .lateral_axis = lateral,
      .secondary_axis = secondary,
      .minimum_lateral_offset_m = minimum_lateral_offset_m,
      .maximum_lateral_offset_m = maximum_lateral_offset_m,
      .minimum_secondary_offset_m = minimum_secondary_offset_m,
      .maximum_secondary_offset_m = maximum_secondary_offset_m,
      .valid = vectorNorm(tangent) > 0.5 && vectorNorm(lateral) > 0.5 &&
               vectorNorm(secondary) > 0.5 &&
               minimum_lateral_offset_m <= maximum_lateral_offset_m &&
               minimum_secondary_offset_m <= maximum_secondary_offset_m,
  };
}

[[nodiscard]] PassageFrame3D passageFrameAtStation(const PassageVolume& volume,
                                                   const double station_m) noexcept {
  if (volume.cross_sections.empty() || !std::isfinite(station_m) ||
      station_m + kStationToleranceM < volume.begin_station_m ||
      station_m > volume.end_station_m + kStationToleranceM ||
      station_m + kStationToleranceM < volume.cross_sections.front().station_m ||
      station_m > volume.cross_sections.back().station_m + kStationToleranceM) {
    return {};
  }
  const auto upper = std::ranges::lower_bound(volume.cross_sections, station_m, {},
                                              &PassageCrossSection::station_m);
  const PassageCrossSection* lower_section{nullptr};
  const PassageCrossSection* upper_section{nullptr};
  if (upper == volume.cross_sections.begin()) {
    lower_section = &volume.cross_sections.front();
    upper_section = lower_section;
  } else if (upper == volume.cross_sections.end()) {
    lower_section = &volume.cross_sections.back();
    upper_section = lower_section;
  } else {
    lower_section = &*std::prev(upper);
    upper_section = &*upper;
  }
  return passageFrameBetweenSections(*lower_section, *upper_section, station_m);
}

[[nodiscard]] bool pointInsidePassageVolume(const PassageVolume& volume,
                                            const Point3& point,
                                            const double station_m) noexcept {
  const PassageFrame3D frame = passageFrameAtStation(volume, station_m);
  if (!frame.valid || !finitePoint(point)) {
    return false;
  }
  const Vec3 offset{point.x - frame.center.x, point.y - frame.center.y,
                    point.z - frame.center.z};
  const double lateral_offset_m = vectorDot(offset, frame.lateral_axis);
  const double secondary_offset_m = vectorDot(offset, frame.secondary_axis);
  return lateral_offset_m + kGeometryTolerance >= frame.minimum_lateral_offset_m &&
         lateral_offset_m <= frame.maximum_lateral_offset_m + kGeometryTolerance &&
         secondary_offset_m + kGeometryTolerance >= frame.minimum_secondary_offset_m &&
         secondary_offset_m <= frame.maximum_secondary_offset_m + kGeometryTolerance;
}

[[nodiscard]] bool constrainedPointAccepted(const ExecutionRouteGeometry3D& geometry,
                                            const Point3& point,
                                            const double station_m) noexcept {
  const std::vector<ConstrainedRouteSpan>& spans = *geometry.constrained_spans;
  const std::vector<PassageVolume>& volumes = *geometry.passage_volumes;
  for (std::size_t index = 0U; index < spans.size(); ++index) {
    const ConstrainedRouteSpan& span = spans[index];
    if (station_m + kStationToleranceM < span.begin_station_m ||
        station_m > span.end_station_m + kStationToleranceM) {
      continue;
    }
    if (index >= volumes.size() ||
        !pointInsidePassageVolume(volumes[index], point, station_m)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::vector<double>
constrainedStationEvents(const ExecutionRouteGeometry3D& geometry,
                         const double begin_station_m, const double end_station_m) {
  std::vector<double> events;
  const auto add_event = [&](const double station_m) {
    if (station_m > begin_station_m + kStationToleranceM &&
        station_m < end_station_m - kStationToleranceM) {
      events.push_back(station_m);
    }
  };
  for (const ConstrainedRouteSpan& span : *geometry.constrained_spans) {
    add_event(span.begin_station_m);
    add_event(span.end_station_m);
    for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
      add_event(segment.begin_station_m);
      add_event(segment.end_station_m);
    }
  }
  for (const PassageVolume& volume : *geometry.passage_volumes) {
    for (const PassageCrossSection& section : volume.cross_sections) {
      add_event(section.station_m);
    }
  }
  std::ranges::sort(events);
  const auto unique_end =
      std::ranges::unique(events, [](const double first, const double second) {
        return nearlyEqual(first, second, kStationToleranceM);
      });
  events.erase(unique_end.begin(), unique_end.end());
  return events;
}

[[nodiscard]] bool pointInsidePassageFrame(const PassageFrame3D& frame,
                                           const Point3& point) noexcept {
  if (!frame.valid || !finitePoint(point)) {
    return false;
  }
  const Vec3 offset{point.x - frame.center.x, point.y - frame.center.y,
                    point.z - frame.center.z};
  const double lateral_offset_m = vectorDot(offset, frame.lateral_axis);
  const double secondary_offset_m = vectorDot(offset, frame.secondary_axis);
  return lateral_offset_m + kGeometryTolerance >= frame.minimum_lateral_offset_m &&
         lateral_offset_m <= frame.maximum_lateral_offset_m + kGeometryTolerance &&
         secondary_offset_m + kGeometryTolerance >= frame.minimum_secondary_offset_m &&
         secondary_offset_m <= frame.maximum_secondary_offset_m + kGeometryTolerance;
}

[[nodiscard]] Point3 interpolatePoint(const Point3& first, const Point3& second,
                                      const double ratio) noexcept {
  return Point3{std::lerp(first.x, second.x, ratio),
                std::lerp(first.y, second.y, ratio),
                std::lerp(first.z, second.z, ratio)};
}

[[nodiscard]] Vec3 pointOffset(const Point3& point, const Point3& center) noexcept {
  return Vec3{point.x - center.x, point.y - center.y, point.z - center.z};
}

[[nodiscard]] bool projectionContinuouslyInsideBounds(
    const Vec3& begin_offset, const Vec3& end_offset, const Vec3& begin_axis,
    const Vec3& end_axis, const double axis_derivative_bound,
    const double minimum_offset_m, const double maximum_offset_m) noexcept {
  const Vec3 offset_delta{end_offset.x - begin_offset.x, end_offset.y - begin_offset.y,
                          end_offset.z - begin_offset.z};
  const double projection_lipschitz =
      vectorNorm(offset_delta) +
      std::max(vectorNorm(begin_offset), vectorNorm(end_offset)) *
          axis_derivative_bound;
  const double begin_projection = vectorDot(begin_offset, begin_axis);
  const double end_projection = vectorDot(end_offset, end_axis);
  const double conservative_minimum =
      std::min(begin_projection, end_projection) - 0.5 * projection_lipschitz;
  const double conservative_maximum =
      std::max(begin_projection, end_projection) + 0.5 * projection_lipschitz;
  return conservative_minimum + kGeometryTolerance >= minimum_offset_m &&
         conservative_maximum <= maximum_offset_m + kGeometryTolerance;
}

[[nodiscard]] bool passageSectionIntervalAccepted(
    const PassageCrossSection& lower_section, const PassageCrossSection& upper_section,
    const Point3& begin, const double begin_station_m, const Point3& end,
    const double end_station_m, const std::size_t depth) noexcept {
  constexpr std::size_t kMaximumProofDepth{20U};
  const PassageFrame3D begin_frame =
      passageFrameBetweenSections(lower_section, upper_section, begin_station_m);
  const PassageFrame3D end_frame =
      passageFrameBetweenSections(lower_section, upper_section, end_station_m);
  if (!pointInsidePassageFrame(begin_frame, begin) ||
      !pointInsidePassageFrame(end_frame, end)) {
    return false;
  }
  const double section_station_delta_m =
      upper_section.station_m - lower_section.station_m;
  if (!(section_station_delta_m > 0.0)) {
    return false;
  }
  const double axis_ratio_delta =
      std::clamp((end_station_m - begin_station_m) / section_station_delta_m, 0.0, 1.0);
  const auto axis_derivative_bound =
      [axis_ratio_delta](const Vec3& lower_axis, const Vec3& upper_axis) noexcept {
        const double axis_dot =
            std::clamp(vectorDot(lower_axis, upper_axis), -1.0, 1.0);
        const double minimum_axis_norm =
            std::sqrt(std::max(kStationToleranceM, 0.5 * (1.0 + axis_dot)));
        const Vec3 axis_delta{upper_axis.x - lower_axis.x, upper_axis.y - lower_axis.y,
                              upper_axis.z - lower_axis.z};
        return axis_ratio_delta * vectorNorm(axis_delta) / minimum_axis_norm;
      };
  const Vec3 begin_offset = pointOffset(begin, begin_frame.center);
  const Vec3 end_offset = pointOffset(end, end_frame.center);
  const bool lateral_proven = projectionContinuouslyInsideBounds(
      begin_offset, end_offset, begin_frame.lateral_axis, end_frame.lateral_axis,
      axis_derivative_bound(lower_section.lateral_axis, upper_section.lateral_axis),
      begin_frame.minimum_lateral_offset_m, begin_frame.maximum_lateral_offset_m);
  const bool secondary_proven = projectionContinuouslyInsideBounds(
      begin_offset, end_offset, begin_frame.secondary_axis, end_frame.secondary_axis,
      axis_derivative_bound(lower_section.secondary_axis, upper_section.secondary_axis),
      begin_frame.minimum_secondary_offset_m, begin_frame.maximum_secondary_offset_m);
  if (lateral_proven && secondary_proven) {
    return true;
  }
  if (depth >= kMaximumProofDepth) {
    return false;
  }
  const double middle_station_m = std::midpoint(begin_station_m, end_station_m);
  const Point3 middle = interpolatePoint(begin, end, 0.5);
  return passageSectionIntervalAccepted(lower_section, upper_section, begin,
                                        begin_station_m, middle, middle_station_m,
                                        depth + 1U) &&
         passageSectionIntervalAccepted(lower_section, upper_section, middle,
                                        middle_station_m, end, end_station_m,
                                        depth + 1U);
}

[[nodiscard]] bool constrainedSegmentAccepted(const ExecutionRouteGeometry3D& geometry,
                                              const Point3& begin,
                                              const double begin_station_m,
                                              const Point3& end,
                                              const double end_station_m) noexcept {
  if (end_station_m + kStationToleranceM < begin_station_m) {
    return false;
  }
  const std::vector<ConstrainedRouteSpan>& spans = *geometry.constrained_spans;
  const std::vector<PassageVolume>& volumes = *geometry.passage_volumes;
  for (std::size_t index = 0U; index < spans.size(); ++index) {
    const ConstrainedRouteSpan& span = spans[index];
    const double overlap_begin_station_m =
        std::max(begin_station_m, span.begin_station_m);
    const double overlap_end_station_m = std::min(end_station_m, span.end_station_m);
    if (overlap_end_station_m + kStationToleranceM < overlap_begin_station_m) {
      continue;
    }
    if (index >= volumes.size()) {
      return false;
    }
    const double station_delta_m = end_station_m - begin_station_m;
    const auto point_at_station = [&](const double station_m) noexcept {
      const double ratio =
          station_delta_m > 0.0
              ? std::clamp((station_m - begin_station_m) / station_delta_m, 0.0, 1.0)
              : 0.0;
      return interpolatePoint(begin, end, ratio);
    };
    const PassageVolume& volume = volumes[index];
    if (overlap_end_station_m <= overlap_begin_station_m + kStationToleranceM) {
      const double boundary_station_m =
          std::clamp(std::midpoint(overlap_begin_station_m, overlap_end_station_m),
                     span.begin_station_m, span.end_station_m);
      const PassageFrame3D frame = passageFrameAtStation(volume, boundary_station_m);
      if (!pointInsidePassageFrame(frame, point_at_station(boundary_station_m))) {
        return false;
      }
      continue;
    }
    std::vector<double> interval_stations{overlap_begin_station_m};
    for (const PassageCrossSection& section : volume.cross_sections) {
      if (section.station_m > overlap_begin_station_m &&
          section.station_m < overlap_end_station_m) {
        interval_stations.push_back(section.station_m);
      }
    }
    interval_stations.push_back(overlap_end_station_m);
    for (std::size_t interval_index = 1U; interval_index < interval_stations.size();
         ++interval_index) {
      const double interval_begin_station_m = interval_stations[interval_index - 1U];
      const double interval_end_station_m = interval_stations[interval_index];
      const double middle_station_m =
          std::midpoint(interval_begin_station_m, interval_end_station_m);
      const auto upper = std::ranges::upper_bound(
          volume.cross_sections, middle_station_m, {}, &PassageCrossSection::station_m);
      if (upper == volume.cross_sections.begin() || volume.cross_sections.size() < 2U) {
        return false;
      }
      const auto clamped_upper = upper == volume.cross_sections.end()
                                     ? std::prev(volume.cross_sections.end())
                                     : upper;
      const PassageCrossSection& lower_section = *std::prev(clamped_upper);
      const PassageCrossSection& upper_section = *clamped_upper;
      if (!passageSectionIntervalAccepted(
              lower_section, upper_section, point_at_station(interval_begin_station_m),
              interval_begin_station_m, point_at_station(interval_end_station_m),
              interval_end_station_m)) {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] Point3 statePoint(const mppi::State& state) noexcept {
  return Point3{state.x, state.y, state.z};
}

[[nodiscard]] bool
validateOrderedPassageCrossings(const ExecutionRouteGeometry3D& geometry,
                                const std::span<const StationedRoutePoint3D> path,
                                const double begin_station_m,
                                const double end_station_m) noexcept {
  if (path.size() < 2U) {
    return false;
  }
  const std::vector<ConstrainedRouteSpan>& spans = *geometry.constrained_spans;
  const std::vector<PassageVolume>& volumes = *geometry.passage_volumes;
  for (std::size_t span_index = 0U; span_index < spans.size(); ++span_index) {
    const ConstrainedRouteSpan& span = spans[span_index];
    if (end_station_m <= span.begin_station_m + kStationToleranceM ||
        begin_station_m >= span.end_station_m - kStationToleranceM) {
      continue;
    }
    if (span_index >= volumes.size()) {
      return false;
    }
    const PassageVolume& volume = volumes[span_index];
    std::vector<double> plane_stations;
    plane_stations.reserve(volume.cross_sections.size() +
                           2U * span.segment_spans.size() + 2U);
    plane_stations.push_back(span.begin_station_m);
    plane_stations.push_back(span.end_station_m);
    for (const PassageCrossSection& section : volume.cross_sections) {
      plane_stations.push_back(section.station_m);
    }
    for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
      plane_stations.push_back(segment.begin_station_m);
      plane_stations.push_back(segment.end_station_m);
    }
    std::ranges::sort(plane_stations);
    const auto unique_end = std::ranges::unique(
        plane_stations, [](const double first, const double second) {
          return nearlyEqual(first, second, kStationToleranceM);
        });
    plane_stations.erase(unique_end.begin(), unique_end.end());

    std::size_t path_segment_index{0U};
    double path_segment_begin_ratio{0.0};
    for (const double plane_station_m : plane_stations) {
      if (plane_station_m <= begin_station_m + kStationToleranceM ||
          plane_station_m > end_station_m + kStationToleranceM ||
          plane_station_m + kStationToleranceM < span.begin_station_m ||
          plane_station_m > span.end_station_m + kStationToleranceM) {
        continue;
      }
      const PassageFrame3D frame = passageFrameAtStation(volume, plane_station_m);
      if (!frame.valid) {
        return false;
      }
      bool crossed{false};
      for (; path_segment_index + 1U < path.size(); ++path_segment_index) {
        const StationedRoutePoint3D& path_start = path[path_segment_index];
        const StationedRoutePoint3D& path_end = path[path_segment_index + 1U];
        const Point3& segment_start = path_start.point;
        const Point3& segment_end = path_end.point;
        const Point3 search_start =
            interpolatePoint(segment_start, segment_end, path_segment_begin_ratio);
        const double search_start_station_m = std::lerp(
            path_start.station_m, path_end.station_m, path_segment_begin_ratio);
        if (plane_station_m + kStationToleranceM < search_start_station_m ||
            plane_station_m > path_end.station_m + kStationToleranceM) {
          path_segment_begin_ratio = 0.0;
          continue;
        }
        const Vec3 search_direction{segment_end.x - search_start.x,
                                    segment_end.y - search_start.y,
                                    segment_end.z - search_start.z};
        const Vec3 from_plane_start{search_start.x - frame.center.x,
                                    search_start.y - frame.center.y,
                                    search_start.z - frame.center.z};
        const Vec3 from_plane_end{segment_end.x - frame.center.x,
                                  segment_end.y - frame.center.y,
                                  segment_end.z - frame.center.z};
        const double signed_start = vectorDot(from_plane_start, frame.tangent);
        const double signed_end = vectorDot(from_plane_end, frame.tangent);
        const double forward_motion = vectorDot(search_direction, frame.tangent);
        if (forward_motion > kStationToleranceM && signed_start <= kGeometryTolerance &&
            signed_end >= -kGeometryTolerance) {
          const double denominator = signed_end - signed_start;
          const double local_ratio =
              std::abs(denominator) > kStationToleranceM
                  ? std::clamp(-signed_start / denominator, 0.0, 1.0)
                  : 0.0;
          const Point3 crossing =
              interpolatePoint(search_start, segment_end, local_ratio);
          const double crossing_station_m =
              std::lerp(search_start_station_m, path_end.station_m, local_ratio);
          if (std::abs(crossing_station_m - plane_station_m) >
                  distance3D(search_start, segment_end) + kStationToleranceM ||
              !pointInsidePassageFrame(frame, crossing)) {
            return false;
          }
          const double remaining_ratio = 1.0 - path_segment_begin_ratio;
          path_segment_begin_ratio = std::min(
              1.0, path_segment_begin_ratio + remaining_ratio * local_ratio + 1.0e-9);
          if (path_segment_begin_ratio >= 1.0 - 1.0e-9) {
            ++path_segment_index;
            path_segment_begin_ratio = 0.0;
          }
          crossed = true;
          break;
        }
        path_segment_begin_ratio = 0.0;
      }
      if (!crossed) {
        return false;
      }
    }
  }
  return true;
}

[[nodiscard]] RouteAdherenceAssessment3D validateFiniteRouteAdherence(
    const ExecutionRouteGeometry3D& geometry, const std::span<const mppi::State> states,
    const double initial_station_m, const double minimum_station_m,
    const double maximum_station_m, const std::optional<double> maximum_cross_track_m,
    const std::optional<double> terminal_cross_track_tolerance_m,
    const double requested_sweep_step_m, const bool allow_initial_handoff) {
  RouteAdherenceAssessment3D result;
  if (states.empty() || !std::isfinite(initial_station_m) ||
      !std::isfinite(minimum_station_m) || !std::isfinite(maximum_station_m) ||
      (maximum_cross_track_m.has_value() &&
       (!std::isfinite(*maximum_cross_track_m) || *maximum_cross_track_m <= 0.0)) ||
      (terminal_cross_track_tolerance_m.has_value() &&
       (!std::isfinite(*terminal_cross_track_tolerance_m) ||
        *terminal_cross_track_tolerance_m <= 0.0)) ||
      !std::isfinite(requested_sweep_step_m) || requested_sweep_step_m <= 0.0) {
    result.status = FiniteExecutionRouteAdherenceStatus3D::kInvalidInput;
    return result;
  }
  constexpr double kMaximumAdherenceSweepStepM{0.1};
  const double sweep_step_m =
      std::min(requested_sweep_step_m, kMaximumAdherenceSweepStepM);
  const std::span<const RouteSample3D> route{*geometry.route};
  const std::vector<double> station_events =
      constrainedStationEvents(geometry, minimum_station_m, maximum_station_m);
  const Point3 initial_point = statePoint(states.front());
  RouteProjection3D previous_projection = projectOntoRoute3DWithinStationWindow(
      route, initial_point,
      std::max(minimum_station_m, initial_station_m - kExecutionBindingToleranceM),
      std::min(maximum_station_m, initial_station_m + kExecutionBindingToleranceM));
  result.begin = previous_projection;
  if (!previous_projection.valid) {
    result.status = FiniteExecutionRouteAdherenceStatus3D::kInitialProjectionInvalid;
    return result;
  }
  // Initial activation may begin on the separately certified handoff connector.
  // Preserve that measured envelope when the optional cross-track policy is active.
  const std::optional<double> effective_maximum_cross_track_m =
      maximum_cross_track_m.has_value() && allow_initial_handoff
          ? std::optional<double>{std::max(*maximum_cross_track_m,
                                           previous_projection.distance_m +
                                               0.5 * sweep_step_m)}
          : maximum_cross_track_m;
  if (effective_maximum_cross_track_m.has_value() &&
      previous_projection.distance_m > *effective_maximum_cross_track_m) {
    result.status = FiniteExecutionRouteAdherenceStatus3D::kInitialCrossTrackExceeded;
    result.failure_distance_m = previous_projection.distance_m;
    return result;
  }
  const double initial_station_error_m =
      std::abs(previous_projection.station_m - initial_station_m);
  if (initial_station_error_m > kExecutionBindingToleranceM) {
    result.status = FiniteExecutionRouteAdherenceStatus3D::kInitialStationMismatch;
    result.failure_distance_m = initial_station_error_m;
    return result;
  }
  if (!constrainedPointAccepted(geometry, initial_point,
                                previous_projection.station_m)) {
    result.status = FiniteExecutionRouteAdherenceStatus3D::kInitialConstraintRejected;
    result.failure_distance_m = previous_projection.distance_m;
    return result;
  }
  std::vector<StationedRoutePoint3D> stationed_path{StationedRoutePoint3D{
      .point = initial_point, .station_m = previous_projection.station_m}};
  Point3 previous_point = initial_point;
  for (std::size_t state_index = 1U; state_index < states.size(); ++state_index) {
    const Point3 state_position = statePoint(states[state_index]);
    const double state_segment_length_m = distance3D(previous_point, state_position);
    if (!std::isfinite(state_segment_length_m)) {
      result.status = FiniteExecutionRouteAdherenceStatus3D::kNonFiniteSegment;
      result.failure_state_index = state_index;
      return result;
    }
    const std::size_t subdivision_count = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(state_segment_length_m / sweep_step_m)));
    const double physical_increment_m =
        state_segment_length_m / static_cast<double>(subdivision_count);
    Point3 sample_begin = previous_point;
    for (std::size_t subdivision = 1U; subdivision <= subdivision_count;
         ++subdivision) {
      const double ratio =
          static_cast<double>(subdivision) / static_cast<double>(subdivision_count);
      const Point3 sample = interpolatePoint(previous_point, state_position, ratio);
      const double allowed_end_station_m =
          std::min(maximum_station_m, previous_projection.station_m +
                                          physical_increment_m + kStationToleranceM);
      const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
          route, sample,
          std::max(minimum_station_m,
                   previous_projection.station_m - kStationToleranceM),
          allowed_end_station_m);
      const double continuous_margin_m = 0.5 * physical_increment_m;
      if (!projection.valid) {
        result.status = FiniteExecutionRouteAdherenceStatus3D::kProjectionInvalid;
        result.failure_state_index = state_index;
        return result;
      }
      if (projection.station_m + kStationToleranceM < previous_projection.station_m) {
        result.status = FiniteExecutionRouteAdherenceStatus3D::kStationRegression;
        result.failure_state_index = state_index;
        result.failure_distance_m =
            previous_projection.station_m - projection.station_m;
        return result;
      }
      const double cross_track_with_margin_m =
          std::max(previous_projection.distance_m, projection.distance_m) +
          continuous_margin_m;
      if (effective_maximum_cross_track_m.has_value() &&
          cross_track_with_margin_m > *effective_maximum_cross_track_m) {
        result.status = FiniteExecutionRouteAdherenceStatus3D::kCrossTrackExceeded;
        result.failure_state_index = state_index;
        result.failure_distance_m = cross_track_with_margin_m;
        return result;
      }
      if (!constrainedPointAccepted(geometry, sample, projection.station_m) ||
          !constrainedSegmentAccepted(geometry, sample_begin,
                                      previous_projection.station_m, sample,
                                      projection.station_m)) {
        result.status = FiniteExecutionRouteAdherenceStatus3D::kConstraintRejected;
        result.failure_state_index = state_index;
        result.failure_distance_m = projection.distance_m;
        return result;
      }
      if (projection.station_m > previous_projection.station_m + kStationToleranceM) {
        const auto event_begin = std::ranges::upper_bound(
            station_events, previous_projection.station_m + kStationToleranceM);
        const auto event_end = std::ranges::lower_bound(
            station_events, projection.station_m - kStationToleranceM);
        for (auto event = event_begin; event != event_end; ++event) {
          const double station_ratio =
              (*event - previous_projection.station_m) /
              (projection.station_m - previous_projection.station_m);
          const Point3 event_point =
              interpolatePoint(sample_begin, sample, station_ratio);
          if (!constrainedPointAccepted(geometry, event_point, *event)) {
            result.status = FiniteExecutionRouteAdherenceStatus3D::kConstraintRejected;
            result.failure_state_index = state_index;
            result.failure_distance_m = projection.distance_m;
            return result;
          }
        }
      }
      sample_begin = sample;
      previous_projection = projection;
      stationed_path.push_back(
          StationedRoutePoint3D{.point = sample, .station_m = projection.station_m});
    }
    previous_point = state_position;
  }
  result.stop = previous_projection;
  if (terminal_cross_track_tolerance_m.has_value() &&
      previous_projection.distance_m > *terminal_cross_track_tolerance_m) {
    result.status = FiniteExecutionRouteAdherenceStatus3D::kTerminalCrossTrackExceeded;
    result.failure_state_index = states.size() - 1U;
    result.failure_distance_m = previous_projection.distance_m;
    return result;
  }
  if (!validateOrderedPassageCrossings(geometry, stationed_path, result.begin.station_m,
                                       previous_projection.station_m)) {
    result.status = FiniteExecutionRouteAdherenceStatus3D::kPassageCrossingRejected;
    result.failure_state_index = states.size() - 1U;
    return result;
  }
  result.status = FiniteExecutionRouteAdherenceStatus3D::kAccepted;
  result.accepted = true;
  return result;
}

[[nodiscard]] bool validMppiRoute(const std::span<const mppi::RouteSample3D> mppi_route,
                                  const std::span<const RouteSample3D> route) noexcept {
  if (mppi_route.size() != route.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const RouteSample3D& source = route[index];
    const mppi::RouteSample3D& executable = mppi_route[index];
    if (!nearlyEqual(executable.x_m, source.position.x) ||
        !nearlyEqual(executable.y_m, source.position.y) ||
        !nearlyEqual(executable.z_m, source.position.z) ||
        !nearlyEqual(executable.tangent_x, source.tangent.x) ||
        !nearlyEqual(executable.tangent_y, source.tangent.y) ||
        !nearlyEqual(executable.tangent_z, source.tangent.z) ||
        !nearlyEqual(executable.station_m, source.station_m) ||
        !std::isfinite(executable.reference_speed_mps) ||
        executable.reference_speed_mps < 0.0F ||
        executable.required_risk_tier != source.required_risk_tier) {
      return false;
    }
  }
  return true;
}

} // namespace drone_city_nav::execution_route_snapshot_3d_internal
