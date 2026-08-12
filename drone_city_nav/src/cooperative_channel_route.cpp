#include "drone_city_nav/cooperative_channel_route.hpp"

#include "drone_city_nav/cooperative_channel_coordination.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

struct LaneApplication {
  std::size_t span_index{0U};
  const ChannelLaneSet* lane_set{nullptr};
  const ChannelLane* lane{nullptr};
  double transition_before_m{0.0};
  double transition_after_m{0.0};
  bool reverse_lane{false};
};

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] double smoothStep(const double value) noexcept {
  const double bounded = std::clamp(value, 0.0, 1.0);
  return bounded * bounded * (3.0 - 2.0 * bounded);
}

[[nodiscard]] Point3 interpolatePoint(const Point3& first, const Point3& second,
                                      const double ratio) noexcept {
  return Point3{std::lerp(first.x, second.x, ratio),
                std::lerp(first.y, second.y, ratio),
                std::lerp(first.z, second.z, ratio)};
}

[[nodiscard]] Vec3 difference(const Point3& first, const Point3& second) noexcept {
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

[[nodiscard]] Point3 translated(const Point3& point, const Vec3& offset,
                                const double scale) noexcept {
  return Point3{point.x + scale * offset.x, point.y + scale * offset.y,
                point.z + scale * offset.z};
}

[[nodiscard]] const ChannelLaneSet*
findLaneSet(const std::span<const ChannelLaneSet> lane_sets,
            const std::string& channel_id) noexcept {
  const auto match =
      std::ranges::find(lane_sets, channel_id, &ChannelLaneSet::channel_id);
  return match == lane_sets.end() ? nullptr : &*match;
}

[[nodiscard]] std::vector<Point3> orientedLanePoints(const ChannelLane& lane,
                                                     const bool reverse) {
  std::vector<Point3> points;
  points.reserve(lane.centerline.size());
  if (reverse) {
    for (const RouteSample3D& sample : std::views::reverse(lane.centerline)) {
      points.push_back(sample.position);
    }
  } else {
    for (const RouteSample3D& sample : lane.centerline) {
      points.push_back(sample.position);
    }
  }
  return points;
}

[[nodiscard]] Point3 samplePolylineFraction(const std::span<const Point3> points,
                                            const double fraction) noexcept {
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1U) {
    return points.front();
  }
  double total_length_m = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    total_length_m += distance3D(points[index - 1U], points[index]);
  }
  if (!(total_length_m > 1.0e-9)) {
    return points.front();
  }
  const double target_m = std::clamp(fraction, 0.0, 1.0) * total_length_m;
  double station_m = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double segment_m = distance3D(points[index - 1U], points[index]);
    if (target_m <= station_m + segment_m || index + 1U == points.size()) {
      const double ratio =
          segment_m > 1.0e-9 ? (target_m - station_m) / segment_m : 0.0;
      return interpolatePoint(points[index - 1U], points[index], ratio);
    }
    station_m += segment_m;
  }
  return points.back();
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

void makeAssignmentsExclusive(
    std::vector<CooperativeChannelLaneAssignment>& assignments,
    const CooperativeChannelLaneRouteStatus status) noexcept {
  for (CooperativeChannelLaneAssignment& assignment : assignments) {
    assignment.lane_index = 0U;
    assignment.lane_count = 1U;
    assignment.lateral_offset_m = 0.0;
    assignment.status = status;
  }
}

} // namespace

CooperativeChannelRouteResult applyCooperativeChannelLanes(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const std::span<const ChannelLaneSet> lane_sets, const OccupancyGrid3D& occupancy,
    const CooperativeChannelRouteConfig& config) {
  CooperativeChannelRouteResult result{
      .route = std::vector<RouteSample3D>{route.begin(), route.end()},
      .constrained_spans = std::vector<ConstrainedRouteSpan>{constrained_spans.begin(),
                                                             constrained_spans.end()},
      .assignments = {},
      .applied_lane_count = 0U,
      .valid = false,
  };
  result.assignments.reserve(constrained_spans.size());
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    result.assignments.push_back(CooperativeChannelLaneAssignment{
        .channel_id = constrained_spans[index].channel_id,
        .route_generation = constrained_spans[index].route_generation,
        .span_index = index,
    });
  }
  if (route.size() < 2U || !finitePositive(config.preferred_transition_length_m) ||
      !finitePositive(config.minimum_transition_length_m) ||
      config.minimum_transition_length_m > config.preferred_transition_length_m ||
      !std::ranges::is_sorted(route, {}, &RouteSample3D::station_m) ||
      !std::ranges::is_sorted(constrained_spans, {},
                              &ConstrainedRouteSpan::begin_station_m)) {
    makeAssignmentsExclusive(result.assignments,
                             CooperativeChannelLaneRouteStatus::kInvalidInput);
    return result;
  }

  std::vector<LaneApplication> applications;
  applications.reserve(constrained_spans.size());
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    const ConstrainedRouteSpan& span = constrained_spans[index];
    CooperativeChannelLaneAssignment& assignment = result.assignments[index];
    const ChannelLaneSet* const lane_set = findLaneSet(lane_sets, span.channel_id);
    if (lane_set == nullptr || lane_set->lanes.empty()) {
      assignment.status = CooperativeChannelLaneRouteStatus::kMissingLaneGeometry;
      continue;
    }
    const std::size_t lane_index =
        assignCooperativeChannelLane(span.direction_sign, lane_set->lanes.size());
    const ChannelLane& lane = lane_set->lanes[lane_index];
    if (lane.centerline.size() < 2U) {
      assignment.status = CooperativeChannelLaneRouteStatus::kMissingLaneGeometry;
      continue;
    }
    assignment.lane_index = lane_index;
    assignment.lane_count = lane_set->lanes.size();
    assignment.lateral_offset_m = lane.lateral_offset_m;
    if (lane_set->exclusive() || std::abs(lane.lateral_offset_m) <= 1.0e-9) {
      assignment.status = CooperativeChannelLaneRouteStatus::kExclusive;
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
      assignment.lane_index = 0U;
      assignment.lane_count = 1U;
      assignment.lateral_offset_m = 0.0;
      assignment.status = CooperativeChannelLaneRouteStatus::kInsufficientTransition;
      continue;
    }
    const Point3 route_entry =
        sampleRoute3DAtStation(route, span.begin_station_m).position;
    const bool reverse_lane = distance3D(route_entry, lane.centerline.back().position) <
                              distance3D(route_entry, lane.centerline.front().position);
    applications.push_back(LaneApplication{
        .span_index = index,
        .lane_set = lane_set,
        .lane = &lane,
        .transition_before_m = transition_before_m,
        .transition_after_m = transition_after_m,
        .reverse_lane = reverse_lane,
    });
    assignment.status = CooperativeChannelLaneRouteStatus::kApplied;
  }

  for (const LaneApplication& application : applications) {
    const ConstrainedRouteSpan& span = constrained_spans[application.span_index];
    const std::vector<Point3> lane_points =
        orientedLanePoints(*application.lane, application.reverse_lane);
    if (lane_points.size() < 2U) {
      CooperativeChannelLaneAssignment& assignment =
          result.assignments[application.span_index];
      assignment.lane_index = 0U;
      assignment.lane_count = 1U;
      assignment.lateral_offset_m = 0.0;
      assignment.status = CooperativeChannelLaneRouteStatus::kMissingLaneGeometry;
      continue;
    }
    const Point3 original_entry =
        sampleRoute3DAtStation(route, span.begin_station_m).position;
    const Point3 original_exit =
        sampleRoute3DAtStation(route, span.end_station_m).position;
    const Vec3 entry_offset = difference(lane_points.front(), original_entry);
    const Vec3 exit_offset = difference(lane_points.back(), original_exit);
    const double influence_begin_m =
        span.begin_station_m - application.transition_before_m;
    const double influence_end_m = span.end_station_m + application.transition_after_m;
    for (std::size_t route_index = 0U; route_index < route.size(); ++route_index) {
      const double station_m = route[route_index].station_m;
      if (station_m + 1.0e-9 < influence_begin_m ||
          station_m - 1.0e-9 > influence_end_m) {
        continue;
      }
      if (station_m < span.begin_station_m) {
        const double ratio = smoothStep((station_m - influence_begin_m) /
                                        application.transition_before_m);
        result.route[route_index].position =
            translated(route[route_index].position, entry_offset, ratio);
      } else if (station_m <= span.end_station_m) {
        const double fraction =
            (station_m - span.begin_station_m) /
            std::max(1.0e-9, span.end_station_m - span.begin_station_m);
        result.route[route_index].position =
            samplePolylineFraction(lane_points, fraction);
      } else {
        const double ratio =
            smoothStep((influence_end_m - station_m) / application.transition_after_m);
        result.route[route_index].position =
            translated(route[route_index].position, exit_offset, ratio);
      }
    }
  }
  recomputeRouteGeometry(result.route);
  if (!rawRouteAccepted(result.route, occupancy, config.footprint)) {
    result.route.assign(route.begin(), route.end());
    result.constrained_spans.assign(constrained_spans.begin(), constrained_spans.end());
    makeAssignmentsExclusive(result.assignments,
                             CooperativeChannelLaneRouteStatus::kRawValidationRejected);
    result.valid = true;
    return result;
  }

  for (std::size_t index = 0U; index < result.constrained_spans.size(); ++index) {
    ConstrainedRouteSpan& transformed = result.constrained_spans[index];
    const ConstrainedRouteSpan& original = constrained_spans[index];
    transformed.begin_station_m =
        mapStation(route, result.route, original.begin_station_m);
    transformed.end_station_m = mapStation(route, result.route, original.end_station_m);
    const CooperativeChannelLaneAssignment& assignment = result.assignments[index];
    const ChannelLaneSet* const lane_set =
        assignment.applied() ? findLaneSet(lane_sets, assignment.channel_id) : nullptr;
    for (RouteEnvelopeSample& envelope : transformed.envelope) {
      envelope.station_m = mapStation(route, result.route, envelope.station_m);
      if (lane_set == nullptr) {
        continue;
      }
      const double route_relative_offset_m =
          assignment.lateral_offset_m * static_cast<double>(original.direction_sign);
      const double half_width_m = 0.5 * lane_set->physical_width_m;
      envelope.lateral_free_left_m =
          std::max(0.0, half_width_m - route_relative_offset_m);
      envelope.lateral_free_right_m =
          std::max(0.0, half_width_m + route_relative_offset_m);
    }
  }
  result.applied_lane_count = static_cast<std::size_t>(std::ranges::count_if(
      result.assignments, &CooperativeChannelLaneAssignment::applied));
  result.valid = true;
  return result;
}

const char* cooperativeChannelLaneRouteStatusName(
    const CooperativeChannelLaneRouteStatus status) noexcept {
  switch (status) {
    case CooperativeChannelLaneRouteStatus::kExclusive:
      return "exclusive";
    case CooperativeChannelLaneRouteStatus::kApplied:
      return "applied";
    case CooperativeChannelLaneRouteStatus::kMissingLaneGeometry:
      return "missing_lane_geometry";
    case CooperativeChannelLaneRouteStatus::kInsufficientTransition:
      return "insufficient_transition";
    case CooperativeChannelLaneRouteStatus::kRawValidationRejected:
      return "raw_validation_rejected";
    case CooperativeChannelLaneRouteStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

} // namespace drone_city_nav
