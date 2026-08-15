#include "drone_city_nav/cooperative_passage_route.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

struct OffsetApplication {
  std::size_t span_index{0U};
  double transition_before_m{0.0};
  double transition_after_m{0.0};
  std::vector<RouteSample3D> offset_route;
};

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] double smoothStep(const double value) noexcept {
  const double bounded = std::clamp(value, 0.0, 1.0);
  return bounded * bounded * (3.0 - 2.0 * bounded);
}

[[nodiscard]] Vec3 difference(const Point3& first, const Point3& second) noexcept {
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

[[nodiscard]] Point3 translated(const Point3& point, const Vec3& offset,
                                const double scale) noexcept {
  return Point3{point.x + scale * offset.x, point.y + scale * offset.y,
                point.z + scale * offset.z};
}

[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] double norm(const Vec3& value) noexcept {
  return std::hypot(std::hypot(value.x, value.y), value.z);
}

[[nodiscard]] Vec3 lateralOffsetVector(const PassageVolume& volume,
                                       const double station_m,
                                       const double offset_m) noexcept {
  const auto upper = std::ranges::lower_bound(volume.cross_sections, station_m, {},
                                              &PassageCrossSection::station_m);
  std::size_t previous_index = 0U;
  std::size_t next_index = 0U;
  if (upper == volume.cross_sections.end()) {
    previous_index = volume.cross_sections.size() - 1U;
    next_index = previous_index;
  } else {
    next_index =
        static_cast<std::size_t>(std::distance(volume.cross_sections.begin(), upper));
    previous_index = next_index == 0U ? 0U : next_index - 1U;
    if (std::abs(upper->station_m - station_m) <= 1.0e-9 &&
        next_index + 1U < volume.cross_sections.size()) {
      ++next_index;
    }
  }
  const Vec3& previous = volume.cross_sections[previous_index].lateral_axis;
  const Vec3& next = volume.cross_sections[next_index].lateral_axis;
  const Vec3 sum{previous.x + next.x, previous.y + next.y, previous.z + next.z};
  const double sum_length = norm(sum);
  if (sum_length <= 1.0e-9) {
    return Vec3{offset_m * previous.x, offset_m * previous.y, offset_m * previous.z};
  }
  const Vec3 miter{sum.x / sum_length, sum.y / sum_length, sum.z / sum_length};
  const double denominator = dot(miter, previous);
  if (denominator <= 0.25) {
    return Vec3{offset_m * previous.x, offset_m * previous.y, offset_m * previous.z};
  }
  const double scale = offset_m / denominator;
  return Vec3{scale * miter.x, scale * miter.y, scale * miter.z};
}

[[nodiscard]] std::vector<RouteSample3D>
offsetRouteWithinPassage(const std::span<const RouteSample3D> route,
                         const PassageVolume& volume, const double lateral_offset_m) {
  if (route.size() < 2U || volume.cross_sections.empty() ||
      !std::isfinite(lateral_offset_m)) {
    return {};
  }
  std::vector<RouteSample3D> result(route.begin(), route.end());
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const Vec3 offset =
        lateralOffsetVector(volume, route[index].station_m, lateral_offset_m);
    result[index].position = translated(route[index].position, offset, 1.0);
  }
  return result;
}

[[nodiscard]] const PassageVolume*
findPassageVolume(const std::span<const PassageVolume> volumes,
                  const ConstrainedRouteSpan& span,
                  const std::size_t span_index) noexcept {
  const auto match = std::ranges::find_if(volumes, [&](const PassageVolume& volume) {
    return volume.span_index == span_index &&
           volume.passage_traversal_id == span.passage_traversal_id;
  });
  return match == volumes.end() ? nullptr : &*match;
}

[[nodiscard]] double mapStation(const std::span<const RouteSample3D> original,
                                const std::span<const RouteSample3D> transformed,
                                const double station_m) noexcept {
  if (original.empty() || transformed.size() != original.size()) {
    return 0.0;
  }
  const auto upper =
      std::ranges::lower_bound(original, station_m, {}, &RouteSample3D::station_m);
  if (upper == original.begin()) {
    return transformed.front().station_m;
  }
  if (upper == original.end()) {
    return transformed.back().station_m;
  }
  const std::size_t upper_index =
      static_cast<std::size_t>(std::distance(original.begin(), upper));
  const RouteSample3D& lower_sample = original[upper_index - 1U];
  const double interval_m = upper->station_m - lower_sample.station_m;
  const double ratio =
      interval_m > 1.0e-9 ? (station_m - lower_sample.station_m) / interval_m : 0.0;
  return std::lerp(transformed[upper_index - 1U].station_m,
                   transformed[upper_index].station_m, ratio);
}

void recomputeRouteGeometry(std::vector<RouteSample3D>& route) noexcept {
  if (route.empty()) {
    return;
  }
  route.front().station_m = 0.0;
  for (std::size_t index = 1U; index < route.size(); ++index) {
    route[index].station_m =
        route[index - 1U].station_m +
        distance3D(route[index - 1U].position, route[index].position);
  }
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const std::size_t first = index == 0U ? index : index - 1U;
    const std::size_t second = index + 1U < route.size() ? index + 1U : index;
    const Vec3 delta = difference(route[second].position, route[first].position);
    const double length = std::hypot(std::hypot(delta.x, delta.y), delta.z);
    if (length > 1.0e-9) {
      route[index].tangent = Vec3{delta.x / length, delta.y / length, delta.z / length};
    }
  }
}

[[nodiscard]] bool rawRouteAccepted(const std::span<const RouteSample3D> route,
                                    const OccupancyGrid3D& occupancy,
                                    const SweptFootprintConfig& footprint) noexcept {
  const FootprintBodyAxis body_axis{};
  for (std::size_t index = 1U; index < route.size(); ++index) {
    if (!validateRawSweptFootprint(occupancy, route[index - 1U].position, body_axis,
                                   route[index].position, body_axis, footprint)
             .accepted()) {
      return false;
    }
  }
  return true;
}

void makeAssignmentsCentered(std::vector<CooperativePassageAssignment>& assignments,
                             const CooperativePassageRouteStatus status) noexcept {
  for (CooperativePassageAssignment& assignment : assignments) {
    assignment.applied_lateral_offset_m = 0.0;
    assignment.status = status;
  }
}

[[nodiscard]] double
preferredDirectionalOffset(const PassageVolume& volume, const int direction_sign,
                           const CooperativePassageRouteConfig& config) noexcept {
  if (!volume.raw_validated || direction_sign == 0) {
    return 0.0;
  }
  const double available_route_right_m =
      std::max(0.0, -volume.minimum_lateral_offset_m);
  const double desired_route_offset_m =
      std::min(available_route_right_m,
               std::max(0.5 * config.desired_center_separation_m,
                        config.directional_offset_fraction * available_route_right_m));
  return -desired_route_offset_m * static_cast<double>(direction_sign);
}

[[nodiscard]] const RouteEnvelopeSample*
nearestOriginalEnvelope(const ConstrainedRouteSpan& span,
                        const double station_m) noexcept {
  if (span.envelope.empty()) {
    return nullptr;
  }
  const auto upper = std::ranges::lower_bound(span.envelope, station_m, {},
                                              &RouteEnvelopeSample::station_m);
  if (upper == span.envelope.begin()) {
    return &span.envelope.front();
  }
  if (upper == span.envelope.end()) {
    return &span.envelope.back();
  }
  const RouteEnvelopeSample& previous = *std::prev(upper);
  return station_m - previous.station_m <= upper->station_m - station_m ? &previous
                                                                        : &*upper;
}

} // namespace

CooperativePassageRouteResult applyCooperativePassageCorridors(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const std::span<const PassageVolume> passage_volumes,
    const OccupancyGrid3D& occupancy, const CooperativePassageRouteConfig& config) {
  CooperativePassageRouteResult result{
      .route = std::vector<RouteSample3D>{route.begin(), route.end()},
      .constrained_spans = std::vector<ConstrainedRouteSpan>{constrained_spans.begin(),
                                                             constrained_spans.end()},
      .assignments = {},
      .applied_offset_count = 0U,
      .valid = false,
  };
  result.assignments.reserve(constrained_spans.size());
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    result.assignments.push_back(CooperativePassageAssignment{
        .passage_traversal_id = constrained_spans[index].passage_traversal_id,
        .route_generation = constrained_spans[index].route_generation,
        .span_index = index,
        .desired_center_separation_m = config.desired_center_separation_m,
    });
  }
  if (route.size() < 2U || !finitePositive(config.preferred_transition_length_m) ||
      !finitePositive(config.minimum_transition_length_m) ||
      !finitePositive(config.desired_center_separation_m) ||
      !(config.directional_offset_fraction >= 0.0) ||
      !(config.directional_offset_fraction <= 1.0) ||
      config.minimum_transition_length_m > config.preferred_transition_length_m ||
      !std::ranges::is_sorted(route, {}, &RouteSample3D::station_m) ||
      !std::ranges::is_sorted(constrained_spans, {},
                              &ConstrainedRouteSpan::begin_station_m)) {
    makeAssignmentsCentered(result.assignments,
                            CooperativePassageRouteStatus::kInvalidInput);
    return result;
  }

  std::vector<OffsetApplication> applications;
  applications.reserve(constrained_spans.size());
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    const ConstrainedRouteSpan& span = constrained_spans[index];
    CooperativePassageAssignment& assignment = result.assignments[index];
    const PassageVolume* const volume = findPassageVolume(passage_volumes, span, index);
    if (volume == nullptr || !volume->raw_validated) {
      assignment.status = CooperativePassageRouteStatus::kMissingCorridorGeometry;
      continue;
    }
    assignment.physical_width_m = volume->minimum_physical_width_m;
    const double first_bound_m =
        volume->minimum_lateral_offset_m * static_cast<double>(span.direction_sign);
    const double second_bound_m =
        volume->maximum_lateral_offset_m * static_cast<double>(span.direction_sign);
    assignment.minimum_lateral_offset_m = std::min(first_bound_m, second_bound_m);
    assignment.maximum_lateral_offset_m = std::max(first_bound_m, second_bound_m);
    assignment.minimum_secondary_offset_m = volume->minimum_secondary_offset_m;
    assignment.maximum_secondary_offset_m = volume->maximum_secondary_offset_m;
    assignment.passage_cross_section_count = volume->cross_sections.size();
    assignment.passage_volume_raw_validated = volume->raw_validated;
    assignment.requested_lateral_offset_m =
        preferredDirectionalOffset(*volume, span.direction_sign, config);
    if (volume->exclusive(config.desired_center_separation_m) ||
        std::abs(assignment.requested_lateral_offset_m) <= 1.0e-9) {
      assignment.status = CooperativePassageRouteStatus::kCentered;
      continue;
    }

    const double previous_boundary_m =
        index == 0U ? route.front().station_m
                    : constrained_spans[index - 1U].end_station_m;
    const double next_boundary_m = index + 1U == constrained_spans.size()
                                       ? route.back().station_m
                                       : constrained_spans[index + 1U].begin_station_m;
    const double before_share = index == 0U ? 1.0 : 0.5;
    const double after_share = index + 1U == constrained_spans.size() ? 1.0 : 0.5;
    const double transition_before_m = std::min(
        config.preferred_transition_length_m,
        before_share * std::max(0.0, span.begin_station_m - previous_boundary_m));
    const double transition_after_m =
        std::min(config.preferred_transition_length_m,
                 after_share * std::max(0.0, next_boundary_m - span.end_station_m));
    if (transition_before_m + 1.0e-9 < config.minimum_transition_length_m ||
        transition_after_m + 1.0e-9 < config.minimum_transition_length_m) {
      assignment.status = CooperativePassageRouteStatus::kInsufficientTransition;
      continue;
    }
    const double route_relative_offset_m = assignment.requested_lateral_offset_m *
                                           static_cast<double>(span.direction_sign);
    std::vector<RouteSample3D> offset_route =
        offsetRouteWithinPassage(route, *volume, route_relative_offset_m);
    if (offset_route.size() != route.size()) {
      assignment.status = CooperativePassageRouteStatus::kMissingCorridorGeometry;
      continue;
    }
    applications.push_back(OffsetApplication{
        .span_index = index,
        .transition_before_m = transition_before_m,
        .transition_after_m = transition_after_m,
        .offset_route = std::move(offset_route),
    });
    assignment.applied_lateral_offset_m = assignment.requested_lateral_offset_m;
    assignment.status = CooperativePassageRouteStatus::kApplied;
  }

  for (const OffsetApplication& application : applications) {
    const ConstrainedRouteSpan& span = constrained_spans[application.span_index];
    const double influence_begin_m =
        span.begin_station_m - application.transition_before_m;
    const double influence_end_m = span.end_station_m + application.transition_after_m;
    for (std::size_t route_index = 0U; route_index < route.size(); ++route_index) {
      const double station_m = route[route_index].station_m;
      if (station_m + 1.0e-9 < influence_begin_m ||
          station_m - 1.0e-9 > influence_end_m) {
        continue;
      }
      const Vec3 offset = difference(application.offset_route[route_index].position,
                                     route[route_index].position);
      double ratio = 1.0;
      if (station_m < span.begin_station_m) {
        ratio = smoothStep((station_m - influence_begin_m) /
                           application.transition_before_m);
      } else if (station_m > span.end_station_m) {
        ratio =
            smoothStep((influence_end_m - station_m) / application.transition_after_m);
      }
      result.route[route_index].position =
          translated(route[route_index].position, offset, ratio);
    }
  }
  recomputeRouteGeometry(result.route);
  if (!rawRouteAccepted(result.route, occupancy, config.footprint)) {
    result.route.assign(route.begin(), route.end());
    result.constrained_spans.assign(constrained_spans.begin(), constrained_spans.end());
    makeAssignmentsCentered(result.assignments,
                            CooperativePassageRouteStatus::kRawValidationRejected);
    result.valid = true;
    return result;
  }

  for (std::size_t index = 0U; index < result.constrained_spans.size(); ++index) {
    ConstrainedRouteSpan& transformed = result.constrained_spans[index];
    const ConstrainedRouteSpan& original = constrained_spans[index];
    transformed.begin_station_m =
        mapStation(route, result.route, original.begin_station_m);
    transformed.end_station_m = mapStation(route, result.route, original.end_station_m);
    const CooperativePassageAssignment& assignment = result.assignments[index];
    const PassageVolume* const volume =
        findPassageVolume(passage_volumes, original, index);
    if (volume == nullptr || !volume->raw_validated) {
      for (RouteEnvelopeSample& envelope : transformed.envelope) {
        envelope.station_m = mapStation(route, result.route, envelope.station_m);
      }
      continue;
    }
    const double route_relative_offset_m = assignment.applied_lateral_offset_m *
                                           static_cast<double>(original.direction_sign);
    transformed.envelope.clear();
    transformed.envelope.reserve(volume->cross_sections.size());
    for (const PassageCrossSection& section : volume->cross_sections) {
      const RouteEnvelopeSample* const original_envelope =
          nearestOriginalEnvelope(original, section.station_m);
      const double mapped_station_m =
          mapStation(route, result.route, section.station_m);
      const RouteSample3D transformed_sample =
          sampleRoute3DAtStation(result.route, mapped_station_m);
      const Point3 first_secondary_boundary{
          section.center.x +
              section.secondary_axis.x * section.minimum_secondary_offset_m,
          section.center.y +
              section.secondary_axis.y * section.minimum_secondary_offset_m,
          section.center.z +
              section.secondary_axis.z * section.minimum_secondary_offset_m,
      };
      const Point3 second_secondary_boundary{
          section.center.x +
              section.secondary_axis.x * section.maximum_secondary_offset_m,
          section.center.y +
              section.secondary_axis.y * section.maximum_secondary_offset_m,
          section.center.z +
              section.secondary_axis.z * section.maximum_secondary_offset_m,
      };
      const double lateral_left_m =
          config.footprint.radius_m +
          std::max(0.0, section.maximum_lateral_offset_m - route_relative_offset_m);
      const double lateral_right_m =
          config.footprint.radius_m +
          std::max(0.0, route_relative_offset_m - section.minimum_lateral_offset_m);
      const double secondary_clearance_m =
          std::max(0.0, std::min(-section.minimum_secondary_offset_m,
                                 section.maximum_secondary_offset_m));
      const double minimum_center_z_m =
          std::min(first_secondary_boundary.z, second_secondary_boundary.z);
      const double maximum_center_z_m =
          std::max(first_secondary_boundary.z, second_secondary_boundary.z);
      transformed.envelope.push_back(RouteEnvelopeSample{
          .station_m = mapped_station_m,
          .lateral_free_left_m = lateral_left_m,
          .lateral_free_right_m = lateral_right_m,
          .min_z_m = minimum_center_z_m - config.footprint.lower_extent_m,
          .max_z_m = maximum_center_z_m + config.footprint.upper_extent_m,
          .minimum_clearance_m =
              config.footprint.radius_m +
              std::min({lateral_left_m, lateral_right_m, secondary_clearance_m}),
          .reference_z_m = transformed_sample.position.z,
          .reference_speed_mps = original_envelope != nullptr
                                     ? original_envelope->reference_speed_mps
                                     : transformed_sample.reference_speed_mps,
      });
    }
  }
  result.applied_offset_count = static_cast<std::size_t>(std::ranges::count_if(
      result.assignments, &CooperativePassageAssignment::applied));
  result.valid = true;
  return result;
}

const char*
cooperativePassageRouteStatusName(const CooperativePassageRouteStatus status) noexcept {
  switch (status) {
    case CooperativePassageRouteStatus::kCentered:
      return "centered";
    case CooperativePassageRouteStatus::kApplied:
      return "applied";
    case CooperativePassageRouteStatus::kMissingCorridorGeometry:
      return "missing_corridor_geometry";
    case CooperativePassageRouteStatus::kInsufficientTransition:
      return "insufficient_transition";
    case CooperativePassageRouteStatus::kRawValidationRejected:
      return "raw_validation_rejected";
    case CooperativePassageRouteStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

} // namespace drone_city_nav
