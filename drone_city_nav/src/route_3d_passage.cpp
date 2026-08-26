#include "drone_city_nav/route_3d.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <tuple>

namespace drone_city_nav {
namespace {

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
  const RouteSample3D route_sample = sampleRoute3DAtStation(route, current_station_m);
  observation.phase = selected_phase;
  observation.span_index = *selected_index;
  observation.span_available = true;
  observation.passage_traversal_id = span.passage_traversal_id;
  observation.direction_sign = span.direction_sign;
  observation.begin_station_m = span.begin_station_m;
  observation.end_station_m = span.end_station_m;
  observation.distance_to_entry_m = span.begin_station_m - current_station_m;
  observation.distance_to_exit_m = span.end_station_m - current_station_m;
  observation.entry_position =
      sampleRoute3DAtStation(route, span.begin_station_m).position;
  observation.exit_position =
      sampleRoute3DAtStation(route, span.end_station_m).position;
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

} // namespace drone_city_nav
