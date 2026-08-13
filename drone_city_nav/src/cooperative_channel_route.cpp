#include "drone_city_nav/cooperative_channel_route.hpp"

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

[[nodiscard]] const ChannelCorridor*
findCorridor(const std::span<const ChannelCorridor> corridors,
             const std::string& channel_id) noexcept {
  const auto match =
      std::ranges::find(corridors, channel_id, &ChannelCorridor::channel_id);
  return match == corridors.end() ? nullptr : &*match;
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

void makeAssignmentsCentered(std::vector<CooperativeChannelAssignment>& assignments,
                             const CooperativeChannelRouteStatus status) noexcept {
  for (CooperativeChannelAssignment& assignment : assignments) {
    assignment.applied_lateral_offset_m = 0.0;
    assignment.status = status;
  }
}

[[nodiscard]] ChannelCorridorConfig
corridorConfig(const CooperativeChannelRouteConfig& config) noexcept {
  return ChannelCorridorConfig{
      .desired_center_separation_m = config.desired_center_separation_m,
      .minimum_wall_clearance_m = 0.0,
      .lateral_probe_step_m = 0.5,
      .directional_offset_fraction = config.directional_offset_fraction,
      .footprint = config.footprint,
  };
}

} // namespace

CooperativeChannelRouteResult applyCooperativeChannelCorridors(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const std::span<const ChannelCorridor> corridors, const OccupancyGrid3D& occupancy,
    const CooperativeChannelRouteConfig& config) {
  CooperativeChannelRouteResult result{
      .route = std::vector<RouteSample3D>{route.begin(), route.end()},
      .constrained_spans = std::vector<ConstrainedRouteSpan>{constrained_spans.begin(),
                                                             constrained_spans.end()},
      .assignments = {},
      .applied_offset_count = 0U,
      .valid = false,
  };
  result.assignments.reserve(constrained_spans.size());
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    result.assignments.push_back(CooperativeChannelAssignment{
        .channel_id = constrained_spans[index].channel_id,
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
                            CooperativeChannelRouteStatus::kInvalidInput);
    return result;
  }

  std::vector<OffsetApplication> applications;
  applications.reserve(constrained_spans.size());
  const ChannelCorridorConfig lateral_config = corridorConfig(config);
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    const ConstrainedRouteSpan& span = constrained_spans[index];
    CooperativeChannelAssignment& assignment = result.assignments[index];
    const ChannelCorridor* const corridor = findCorridor(corridors, span.channel_id);
    if (corridor == nullptr || !corridor->raw_validated) {
      assignment.status = CooperativeChannelRouteStatus::kMissingCorridorGeometry;
      continue;
    }
    assignment.physical_width_m = corridor->physical_width_m;
    assignment.minimum_lateral_offset_m = corridor->minimum_lateral_offset_m;
    assignment.maximum_lateral_offset_m = corridor->maximum_lateral_offset_m;
    assignment.requested_lateral_offset_m = preferredDirectionalChannelOffset(
        *corridor, span.direction_sign, lateral_config);
    if (corridor->exclusive(config.desired_center_separation_m) ||
        std::abs(assignment.requested_lateral_offset_m) <= 1.0e-9) {
      assignment.status = CooperativeChannelRouteStatus::kCentered;
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
      assignment.status = CooperativeChannelRouteStatus::kInsufficientTransition;
      continue;
    }
    const double route_relative_offset_m = assignment.requested_lateral_offset_m *
                                           static_cast<double>(span.direction_sign);
    std::vector<RouteSample3D> offset_route =
        offsetChannelCenterline(route, route_relative_offset_m);
    if (offset_route.size() != route.size()) {
      assignment.status = CooperativeChannelRouteStatus::kMissingCorridorGeometry;
      continue;
    }
    applications.push_back(OffsetApplication{
        .span_index = index,
        .transition_before_m = transition_before_m,
        .transition_after_m = transition_after_m,
        .offset_route = std::move(offset_route),
    });
    assignment.applied_lateral_offset_m = assignment.requested_lateral_offset_m;
    assignment.status = CooperativeChannelRouteStatus::kApplied;
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
                            CooperativeChannelRouteStatus::kRawValidationRejected);
    result.valid = true;
    return result;
  }

  for (std::size_t index = 0U; index < result.constrained_spans.size(); ++index) {
    ConstrainedRouteSpan& transformed = result.constrained_spans[index];
    const ConstrainedRouteSpan& original = constrained_spans[index];
    transformed.begin_station_m =
        mapStation(route, result.route, original.begin_station_m);
    transformed.end_station_m = mapStation(route, result.route, original.end_station_m);
    const CooperativeChannelAssignment& assignment = result.assignments[index];
    for (RouteEnvelopeSample& envelope : transformed.envelope) {
      envelope.station_m = mapStation(route, result.route, envelope.station_m);
      if (!assignment.applied()) {
        continue;
      }
      const double route_relative_offset_m =
          assignment.applied_lateral_offset_m *
          static_cast<double>(original.direction_sign);
      const double half_width_m = 0.5 * assignment.physical_width_m;
      envelope.lateral_free_left_m =
          std::max(0.0, half_width_m - route_relative_offset_m);
      envelope.lateral_free_right_m =
          std::max(0.0, half_width_m + route_relative_offset_m);
    }
  }
  result.applied_offset_count = static_cast<std::size_t>(std::ranges::count_if(
      result.assignments, &CooperativeChannelAssignment::applied));
  result.valid = true;
  return result;
}

const char*
cooperativeChannelRouteStatusName(const CooperativeChannelRouteStatus status) noexcept {
  switch (status) {
    case CooperativeChannelRouteStatus::kCentered:
      return "centered";
    case CooperativeChannelRouteStatus::kApplied:
      return "applied";
    case CooperativeChannelRouteStatus::kMissingCorridorGeometry:
      return "missing_corridor_geometry";
    case CooperativeChannelRouteStatus::kInsufficientTransition:
      return "insufficient_transition";
    case CooperativeChannelRouteStatus::kRawValidationRejected:
      return "raw_validation_rejected";
    case CooperativeChannelRouteStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

} // namespace drone_city_nav
