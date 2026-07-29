#include "drone_city_nav/passage_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool inside(const double value, const double lower,
                          const double upper) noexcept {
  return value >= lower && value <= upper;
}

[[nodiscard]] double distanceToInterval(const double value, const double lower,
                                        const double upper) noexcept {
  if (value < lower) {
    return lower - value;
  }
  if (value > upper) {
    return value - upper;
  }
  return 0.0;
}

[[nodiscard]] double signedPlaneDistance(const Point2 position,
                                         const PortalPlane& plane) noexcept {
  return (position.x - plane.point.x) * plane.normal.x +
         (position.y - plane.point.y) * plane.normal.y;
}

[[nodiscard]] double portalLateralDistance(const Point2 position,
                                           const Portal& portal) noexcept {
  const Point2 lateral{-portal.normal_xy.y, portal.normal_xy.x};
  return (position.x - portal.center.x) * lateral.x +
         (position.y - portal.center.y) * lateral.y;
}

[[nodiscard]] double minimumRestToRestTime(const double distance_m,
                                           const double acceleration_mps2,
                                           const double speed_mps) noexcept {
  if (!(distance_m > 0.0)) {
    return 0.0;
  }
  const double acceleration_distance = speed_mps * speed_mps / acceleration_mps2;
  if (distance_m <= acceleration_distance) {
    return 2.0 * std::sqrt(distance_m / acceleration_mps2);
  }
  return 2.0 * speed_mps / acceleration_mps2 +
         (distance_m - acceleration_distance) / speed_mps;
}

[[nodiscard]] mppi::PassagePhase
toMppiPassagePhase(const PassageCoordinatorPhase phase) noexcept {
  switch (phase) {
    case PassageCoordinatorPhase::kVerticalAlignment:
      return mppi::PassagePhase::kVerticalAlignment;
    case PassageCoordinatorPhase::kReady:
      return mppi::PassagePhase::kReady;
    case PassageCoordinatorPhase::kTraversal:
      return mppi::PassagePhase::kTraversal;
    case PassageCoordinatorPhase::kInactive:
    case PassageCoordinatorPhase::kUpcoming:
    case PassageCoordinatorPhase::kCleared:
    case PassageCoordinatorPhase::kInvalidRouteEvent:
      return mppi::PassagePhase::kUpcoming;
  }
  return mppi::PassagePhase::kUpcoming;
}

[[nodiscard]] bool routeEventIsValid(const RoutePassageEvent& event) noexcept {
  return event.traversal_direction != 0 &&
         event.entry_station_m >= event.approach_station_m &&
         event.exit_station_m > event.entry_station_m &&
         event.departure_station_m >= event.exit_station_m &&
         event.portal.max_z_m > event.portal.min_z_m && event.portal.depth_m > 0.0 &&
         event.portal.width_m > 0.0 && event.speed_limit_mps >= 0.0;
}

} // namespace

PassageCoordinator::PassageCoordinator(const PassageCoordinatorConfig& config)
    : config_{config} {
  if (!(config_.vertical_clearance_margin_m >= 0.0) ||
      !(config_.vertical_capture_hysteresis_m >= 0.0) ||
      !(config_.preferred_z_capture_tolerance_m > 0.0) ||
      !(config_.maximum_capture_vertical_speed_mps >= 0.0) ||
      config_.capture_stable_cycles == 0U || config_.retention_violation_cycles == 0U ||
      !(config_.alignment_time_margin_s >= 0.0) ||
      !(config_.minimum_stationary_trigger_distance_m >= 0.0) ||
      !(config_.maximum_vertical_acceleration_mps2 > 0.0) ||
      !(config_.maximum_vertical_speed_mps > 0.0) ||
      !(config_.maximum_horizontal_braking_acceleration_mps2 > 0.0) ||
      !(config_.reaction_latency_s >= 0.0) ||
      !(config_.exit_station_hysteresis_m >= 0.0)) {
    throw std::invalid_argument{"invalid passage coordinator configuration"};
  }
}

PassageCoordinatorResult
PassageCoordinator::update(const PassageCoordinatorInput& input) {
  if (!input.route || input.route->passage_events.empty()) {
    reset();
    return {};
  }
  if (!route_seen_ || route_generation_ != input.route->generation) {
    resetForRoute(input.route->generation);
  }

  if (next_event_index_ >= input.route->passage_events.size()) {
    return {};
  }

  const RoutePassageEvent& event = input.route->passage_events[next_event_index_];
  if (!routeEventIsValid(event)) {
    return PassageCoordinatorResult{
        .phase = PassageCoordinatorPhase::kInvalidRouteEvent,
        .constraint = std::nullopt,
        .portal_id = event.portal.id,
        .route_generation = route_generation_,
        .route_event_index = next_event_index_,
    };
  }

  const double safe_min_z = event.portal.min_z_m + config_.vertical_clearance_margin_m;
  const double safe_max_z = event.portal.max_z_m - config_.vertical_clearance_margin_m;
  if (!(safe_max_z > safe_min_z)) {
    return PassageCoordinatorResult{
        .phase = PassageCoordinatorPhase::kInvalidRouteEvent,
        .constraint = std::nullopt,
        .portal_id = event.portal.id,
        .route_generation = route_generation_,
        .route_event_index = next_event_index_,
    };
  }

  const Point2 position{input.state.x, input.state.y};
  const double entry_plane_distance_m =
      signedPlaneDistance(position, event.portal.entry_plane);
  const double exit_plane_distance_m =
      signedPlaneDistance(position, event.portal.exit_plane);
  const bool inside_portal_width =
      std::abs(portalLateralDistance(position, event.portal)) <=
      0.5 * event.portal.width_m;

  if (!event_initialized_) {
    event_initialized_ = true;
    preferred_z_m_ = event.preferred_z_m;
    const bool starts_inside = inside_portal_width && entry_plane_distance_m >= 0.0 &&
                               exit_plane_distance_m <= 0.0;
    if (starts_inside) {
      entry_plane_crossed_ = true;
    }
    if (starts_inside && inside(input.state.z, safe_min_z, safe_max_z)) {
      preserve_inside_altitude_ = true;
      vertical_ready_latched_ = true;
      preferred_z_m_ = input.state.z;
    }
    if (inside_portal_width &&
        exit_plane_distance_m >= config_.exit_station_hysteresis_m &&
        input.route_station_m >= event.exit_station_m) {
      entry_plane_crossed_ = true;
      exit_plane_crossed_ = true;
    }
  } else if (position_seen_) {
    if (!entry_plane_crossed_ && inside_portal_width &&
        previous_entry_plane_distance_m_ < 0.0 && entry_plane_distance_m >= 0.0) {
      entry_plane_crossed_ = true;
    }
    if (entry_plane_crossed_ && inside_portal_width &&
        previous_exit_plane_distance_m_ < config_.exit_station_hysteresis_m &&
        exit_plane_distance_m >= config_.exit_station_hysteresis_m) {
      exit_plane_crossed_ = true;
    }
  }
  position_seen_ = true;
  previous_entry_plane_distance_m_ = entry_plane_distance_m;
  previous_exit_plane_distance_m_ = exit_plane_distance_m;

  if (exit_plane_crossed_) {
    const std::size_t cleared_event_index = next_event_index_;
    ++next_event_index_;
    event_initialized_ = false;
    position_seen_ = false;
    entry_plane_crossed_ = false;
    exit_plane_crossed_ = false;
    vertical_alignment_active_ = false;
    vertical_ready_latched_ = false;
    preserve_inside_altitude_ = false;
    capture_stable_cycles_ = 0U;
    retention_violation_cycles_ = 0U;
    preferred_z_m_ = 0.0;
    previous_entry_plane_distance_m_ = 0.0;
    previous_exit_plane_distance_m_ = 0.0;
    return PassageCoordinatorResult{
        .phase = PassageCoordinatorPhase::kCleared,
        .constraint = std::nullopt,
        .portal_id = event.portal.id,
        .route_generation = route_generation_,
        .route_event_index = cleared_event_index,
        .entry_plane_crossed = true,
        .exit_plane_crossed = true,
        .preferred_z_m = event.preferred_z_m,
        .z_reference_m = semanticRouteZReference(*input.route, input.route_station_m,
                                                 input.normal_flight_z_m),
        .entry_plane_signed_distance_m = entry_plane_distance_m,
        .exit_plane_signed_distance_m = exit_plane_distance_m,
    };
  }

  const double capture_min_z =
      std::max(safe_min_z + config_.vertical_capture_hysteresis_m,
               preferred_z_m_ - config_.preferred_z_capture_tolerance_m);
  const double capture_max_z =
      std::min(safe_max_z - config_.vertical_capture_hysteresis_m,
               preferred_z_m_ + config_.preferred_z_capture_tolerance_m);
  if (!(capture_max_z > capture_min_z)) {
    return PassageCoordinatorResult{
        .phase = PassageCoordinatorPhase::kInvalidRouteEvent,
        .constraint = std::nullopt,
        .portal_id = event.portal.id,
        .route_generation = route_generation_,
        .route_event_index = next_event_index_,
    };
  }

  const double z_m = input.state.z;
  const bool capture_velocity_safe = std::abs(static_cast<double>(input.state.vz)) <=
                                     config_.maximum_capture_vertical_speed_mps;
  if (inside(z_m, capture_min_z, capture_max_z) && capture_velocity_safe) {
    capture_stable_cycles_ =
        std::min(capture_stable_cycles_ + 1U, config_.capture_stable_cycles);
  } else if (!vertical_ready_latched_) {
    capture_stable_cycles_ = 0U;
  }
  if (capture_stable_cycles_ >= config_.capture_stable_cycles) {
    vertical_ready_latched_ = true;
  }

  if (vertical_ready_latched_) {
    if (inside(z_m, safe_min_z, safe_max_z)) {
      retention_violation_cycles_ = 0U;
    } else {
      retention_violation_cycles_ = std::min(retention_violation_cycles_ + 1U,
                                             config_.retention_violation_cycles);
    }
    if (retention_violation_cycles_ >= config_.retention_violation_cycles) {
      vertical_ready_latched_ = false;
      vertical_alignment_active_ = true;
      capture_stable_cycles_ = 0U;
      retention_violation_cycles_ = 0U;
      hold_position_ = Point2{input.state.x, input.state.y};
    }
  }

  const double vertical_error_m = distanceToInterval(z_m, capture_min_z, capture_max_z);
  const double braking_time_s = std::abs(static_cast<double>(input.state.vz)) /
                                config_.maximum_vertical_acceleration_mps2;
  const double alignment_time_s =
      braking_time_s + minimumRestToRestTime(vertical_error_m,
                                             config_.maximum_vertical_acceleration_mps2,
                                             config_.maximum_vertical_speed_mps);
  const double approach_speed_mps =
      std::max(0.0, std::min(input.approach_speed_mps, event.speed_limit_mps));
  const double vertical_alignment_distance_m =
      approach_speed_mps * (alignment_time_s + config_.alignment_time_margin_s);
  const double horizontal_speed_mps = std::hypot(static_cast<double>(input.state.vx),
                                                 static_cast<double>(input.state.vy));
  const double stopping_distance_m =
      horizontal_speed_mps * config_.reaction_latency_s +
      horizontal_speed_mps * horizontal_speed_mps /
          (2.0 * config_.maximum_horizontal_braking_acceleration_mps2);
  const double required_distance_m =
      std::max(vertical_alignment_distance_m, stopping_distance_m);
  const double distance_to_entry_m =
      std::max(0.0, event.entry_station_m - input.route_station_m);
  const bool in_approach = input.route_station_m >= event.approach_station_m &&
                           input.route_station_m < event.entry_station_m;
  const bool in_traversal = entry_plane_crossed_;

  if (vertical_alignment_active_ && vertical_ready_latched_) {
    vertical_alignment_active_ = false;
  }
  if (in_approach && !vertical_ready_latched_ &&
      distance_to_entry_m <= std::max(config_.minimum_stationary_trigger_distance_m,
                                      required_distance_m) &&
      !vertical_alignment_active_) {
    vertical_alignment_active_ = true;
    hold_position_ = Point2{input.state.x, input.state.y};
  }

  PassageCoordinatorPhase phase = PassageCoordinatorPhase::kUpcoming;
  if (vertical_alignment_active_) {
    phase = PassageCoordinatorPhase::kVerticalAlignment;
  } else if (in_traversal) {
    phase = PassageCoordinatorPhase::kTraversal;
  } else if (in_approach && vertical_ready_latched_) {
    phase = PassageCoordinatorPhase::kReady;
  }

  const double z_reference_m =
      phase == PassageCoordinatorPhase::kUpcoming
          ? semanticRouteZReference(*input.route, input.route_station_m,
                                    input.normal_flight_z_m)
          : preferred_z_m_;
  const mppi::PassageConstraint constraint{
      .center_x_m = static_cast<float>(event.portal.center.x),
      .center_y_m = static_cast<float>(event.portal.center.y),
      .normal_x = static_cast<float>(event.portal.normal_xy.x),
      .normal_y = static_cast<float>(event.portal.normal_xy.y),
      .half_depth_m = static_cast<float>(0.5 * event.portal.depth_m),
      .min_z_m = static_cast<float>(safe_min_z),
      .max_z_m = static_cast<float>(safe_max_z),
      .preferred_z_m = static_cast<float>(preferred_z_m_),
      .normal_flight_z_m = static_cast<float>(input.normal_flight_z_m),
      .approach_station_m = static_cast<float>(event.approach_station_m),
      .entry_station_m = static_cast<float>(event.entry_station_m),
      .exit_station_m = static_cast<float>(event.exit_station_m),
      .departure_station_m = static_cast<float>(event.departure_station_m),
      .speed_limit_mps = static_cast<float>(event.speed_limit_mps),
      .phase = toMppiPassagePhase(phase),
  };
  return PassageCoordinatorResult{
      .phase = phase,
      .constraint = constraint,
      .portal_id = event.portal.id,
      .route_generation = route_generation_,
      .route_event_index = next_event_index_,
      .active = true,
      .hold_xy = vertical_alignment_active_,
      .vertical_ready = preserve_inside_altitude_ || vertical_ready_latched_,
      .speed_limit_active =
          input.route_station_m >= event.approach_station_m && !exit_plane_crossed_,
      .entry_plane_crossed = entry_plane_crossed_,
      .exit_plane_crossed = exit_plane_crossed_,
      .hold_position = hold_position_,
      .preferred_z_m = preferred_z_m_,
      .z_reference_m = z_reference_m,
      .vertical_error_m = vertical_error_m,
      .distance_to_entry_m = distance_to_entry_m,
      .capture_stable_cycles = capture_stable_cycles_,
      .retention_violation_cycles = retention_violation_cycles_,
      .required_alignment_time_s = alignment_time_s,
      .required_stopping_distance_m = stopping_distance_m,
      .required_alignment_distance_m = required_distance_m,
      .entry_plane_signed_distance_m = entry_plane_distance_m,
      .exit_plane_signed_distance_m = exit_plane_distance_m,
  };
}

void PassageCoordinator::resetForRoute(const std::uint64_t generation) noexcept {
  reset();
  route_generation_ = generation;
  route_seen_ = true;
}

void PassageCoordinator::reset() noexcept {
  route_generation_ = 0U;
  next_event_index_ = 0U;
  route_seen_ = false;
  event_initialized_ = false;
  position_seen_ = false;
  entry_plane_crossed_ = false;
  exit_plane_crossed_ = false;
  vertical_alignment_active_ = false;
  vertical_ready_latched_ = false;
  preserve_inside_altitude_ = false;
  capture_stable_cycles_ = 0U;
  retention_violation_cycles_ = 0U;
  hold_position_ = {};
  preferred_z_m_ = 0.0;
  previous_entry_plane_distance_m_ = 0.0;
  previous_exit_plane_distance_m_ = 0.0;
}

const char* passageCoordinatorPhaseName(const PassageCoordinatorPhase phase) noexcept {
  switch (phase) {
    case PassageCoordinatorPhase::kInactive:
      return "inactive";
    case PassageCoordinatorPhase::kUpcoming:
      return "upcoming";
    case PassageCoordinatorPhase::kVerticalAlignment:
      return "vertical_alignment";
    case PassageCoordinatorPhase::kReady:
      return "ready";
    case PassageCoordinatorPhase::kTraversal:
      return "traversal";
    case PassageCoordinatorPhase::kCleared:
      return "cleared";
    case PassageCoordinatorPhase::kInvalidRouteEvent:
      return "invalid_route_event";
  }
  return "unknown";
}

} // namespace drone_city_nav
