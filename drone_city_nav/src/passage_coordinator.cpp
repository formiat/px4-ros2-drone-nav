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

[[nodiscard]] bool insidePortalFootprint(const Point2 position,
                                         const Portal& portal) noexcept {
  return std::abs(portalLateralDistance(position, portal)) <= 0.5 * portal.width_m &&
         signedPlaneDistance(position, portal.entry_plane) >= 0.0 &&
         signedPlaneDistance(position, portal.exit_plane) <= 0.0;
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

struct VerticalPrediction {
  bool safe{false};
  double entry_z_m{0.0};
  double entry_vz_mps{0.0};
  double exit_z_m{0.0};
  double exit_vz_mps{0.0};
  double minimum_z_m{0.0};
  double maximum_z_m{0.0};
};

[[nodiscard]] double timeToCoverDistance(const double distance_m,
                                         const double initial_speed_mps,
                                         const double speed_limit_mps,
                                         const double braking_acceleration_mps2,
                                         const double reaction_latency_s) noexcept {
  if (!(distance_m > 0.0)) {
    return 0.0;
  }
  const double speed_limit = std::max(speed_limit_mps, 1.0e-3);
  const double initial_speed = std::max(initial_speed_mps, speed_limit);
  if (initial_speed <= speed_limit + 1.0e-9) {
    return distance_m / speed_limit;
  }

  const double latency_distance = initial_speed * reaction_latency_s;
  if (distance_m <= latency_distance) {
    return distance_m / initial_speed;
  }
  const double remaining_distance = distance_m - latency_distance;
  const double braking_time = (initial_speed - speed_limit) / braking_acceleration_mps2;
  const double braking_distance = 0.5 * (initial_speed + speed_limit) * braking_time;
  if (remaining_distance <= braking_distance) {
    const double discriminant =
        std::max(0.0, initial_speed * initial_speed -
                          2.0 * braking_acceleration_mps2 * remaining_distance);
    return reaction_latency_s +
           (initial_speed - std::sqrt(discriminant)) / braking_acceleration_mps2;
  }
  return reaction_latency_s + braking_time +
         (remaining_distance - braking_distance) / speed_limit;
}

[[nodiscard]] VerticalPrediction
predictVerticalTraversal(const mppi::State& state, const double preferred_z_m,
                         const double safe_min_z_m, const double safe_max_z_m,
                         const double time_to_entry_s, const double time_to_exit_s,
                         const PassageCoordinatorConfig& config) noexcept {
  VerticalPrediction prediction{
      .entry_z_m = state.z,
      .entry_vz_mps = state.vz,
      .exit_z_m = state.z,
      .exit_vz_mps = state.vz,
      .minimum_z_m = state.z,
      .maximum_z_m = state.z,
  };
  double z_m = state.z;
  double vz_mps = state.vz;
  double elapsed_s = 0.0;
  bool entry_recorded = time_to_entry_s <= 1.0e-9;
  bool safe = !entry_recorded || inside(z_m, safe_min_z_m, safe_max_z_m);
  if (entry_recorded) {
    prediction.minimum_z_m = z_m;
    prediction.maximum_z_m = z_m;
  }

  while (elapsed_s < time_to_exit_s - 1.0e-9) {
    const double step_s =
        std::min(config.prediction_time_step_s, time_to_exit_s - elapsed_s);
    const double error_m = preferred_z_m - z_m;
    const double desired_speed_mps = std::copysign(
        std::min(config.maximum_vertical_speed_mps,
                 std::sqrt(2.0 * config.maximum_vertical_acceleration_mps2 *
                           std::abs(error_m))),
        error_m);
    const double acceleration_mps2 =
        std::clamp((desired_speed_mps - vz_mps) / step_s,
                   -config.maximum_vertical_acceleration_mps2,
                   config.maximum_vertical_acceleration_mps2);
    const double next_z_m =
        z_m + vz_mps * step_s + 0.5 * acceleration_mps2 * step_s * step_s;
    const double next_vz_mps = std::clamp(vz_mps + acceleration_mps2 * step_s,
                                          -config.maximum_vertical_speed_mps,
                                          config.maximum_vertical_speed_mps);
    const double next_elapsed_s = elapsed_s + step_s;

    if (!entry_recorded && next_elapsed_s >= time_to_entry_s) {
      const double ratio = std::clamp((time_to_entry_s - elapsed_s) / step_s, 0.0, 1.0);
      prediction.entry_z_m = std::lerp(z_m, next_z_m, ratio);
      prediction.entry_vz_mps = std::lerp(vz_mps, next_vz_mps, ratio);
      prediction.minimum_z_m = prediction.entry_z_m;
      prediction.maximum_z_m = prediction.entry_z_m;
      safe = inside(prediction.entry_z_m, safe_min_z_m, safe_max_z_m);
      entry_recorded = true;
    }
    if (entry_recorded) {
      prediction.minimum_z_m = std::min(prediction.minimum_z_m, next_z_m);
      prediction.maximum_z_m = std::max(prediction.maximum_z_m, next_z_m);
      safe = safe && inside(next_z_m, safe_min_z_m, safe_max_z_m);
    }

    z_m = next_z_m;
    vz_mps = next_vz_mps;
    elapsed_s = next_elapsed_s;
  }

  prediction.exit_z_m = z_m;
  prediction.exit_vz_mps = vz_mps;
  prediction.safe =
      entry_recorded && safe && inside(prediction.exit_z_m, safe_min_z_m, safe_max_z_m);
  return prediction;
}

struct PassageSpeedPrediction {
  bool safe{false};
  double speed_limit_mps{0.0};
  VerticalPrediction vertical{};
};

[[nodiscard]] PassageSpeedPrediction
predictAtSpeed(const PassageCoordinatorInput& input, const RoutePassageEvent& event,
               const double safe_min_z_m, const double safe_max_z_m,
               const double speed_limit_mps,
               const PassageCoordinatorConfig& config) noexcept {
  const double horizontal_speed_mps = std::hypot(static_cast<double>(input.state.vx),
                                                 static_cast<double>(input.state.vy));
  const double distance_to_entry_m =
      std::max(0.0, event.entry_station_m - input.route_station_m);
  const double distance_to_exit_m =
      std::max(0.0, event.exit_station_m - input.route_station_m);
  const double time_to_entry_s = timeToCoverDistance(
      distance_to_entry_m, horizontal_speed_mps, speed_limit_mps,
      config.maximum_horizontal_braking_acceleration_mps2, config.reaction_latency_s);
  const double time_to_exit_s = timeToCoverDistance(
      distance_to_exit_m, horizontal_speed_mps, speed_limit_mps,
      config.maximum_horizontal_braking_acceleration_mps2, config.reaction_latency_s);
  VerticalPrediction vertical =
      predictVerticalTraversal(input.state, event.preferred_z_m, safe_min_z_m,
                               safe_max_z_m, time_to_entry_s, time_to_exit_s, config);
  return PassageSpeedPrediction{
      .safe = vertical.safe,
      .speed_limit_mps = speed_limit_mps,
      .vertical = vertical,
  };
}

[[nodiscard]] PassageSpeedPrediction
maximumSafePassageSpeed(const PassageCoordinatorInput& input,
                        const RoutePassageEvent& event, const double safe_min_z_m,
                        const double safe_max_z_m,
                        const PassageCoordinatorConfig& config) noexcept {
  PassageSpeedPrediction upper = predictAtSpeed(
      input, event, safe_min_z_m, safe_max_z_m, event.speed_limit_mps, config);
  if (upper.safe) {
    return upper;
  }

  const double minimum_speed_mps =
      std::min(config.minimum_continuous_speed_mps, event.speed_limit_mps);
  PassageSpeedPrediction lower = predictAtSpeed(
      input, event, safe_min_z_m, safe_max_z_m, minimum_speed_mps, config);
  if (!lower.safe) {
    return lower;
  }

  double lower_speed_mps = minimum_speed_mps;
  double upper_speed_mps = event.speed_limit_mps;
  for (std::size_t iteration = 0U; iteration < 16U; ++iteration) {
    const double candidate_speed_mps = 0.5 * (lower_speed_mps + upper_speed_mps);
    PassageSpeedPrediction candidate = predictAtSpeed(
        input, event, safe_min_z_m, safe_max_z_m, candidate_speed_mps, config);
    if (candidate.safe) {
      lower = candidate;
      lower_speed_mps = candidate_speed_mps;
    } else {
      upper_speed_mps = candidate_speed_mps;
    }
  }
  return lower;
}

[[nodiscard]] Point2 pointAtRouteStation(const SemanticPortalRoute& route,
                                         const double station_m) noexcept {
  if (!route.polyline || route.polyline->empty() ||
      route.point_stations_m.size() != route.polyline->size()) {
    return {};
  }
  const double station = std::clamp(station_m, 0.0, route.total_length_m);
  const auto upper = std::upper_bound(route.point_stations_m.begin(),
                                      route.point_stations_m.end(), station);
  if (upper == route.point_stations_m.begin()) {
    return route.polyline->front();
  }
  if (upper == route.point_stations_m.end()) {
    return route.polyline->back();
  }
  const std::size_t second_index =
      static_cast<std::size_t>(std::distance(route.point_stations_m.begin(), upper));
  const std::size_t first_index = second_index - 1U;
  const double length =
      route.point_stations_m[second_index] - route.point_stations_m[first_index];
  const double ratio =
      length > 1.0e-9 ? (station - route.point_stations_m[first_index]) / length : 0.0;
  return Point2{
      std::lerp((*route.polyline)[first_index].x, (*route.polyline)[second_index].x,
                ratio),
      std::lerp((*route.polyline)[first_index].y, (*route.polyline)[second_index].y,
                ratio),
  };
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
         event.portal.width_m > 0.0 && event.speed_limit_mps > 0.0;
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
      !(config_.stationary_hold_clearance_m > 0.0) ||
      !(config_.minimum_continuous_speed_mps > 0.0) ||
      !(config_.prediction_time_step_s > 0.0) ||
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
    if (!continueTraversalForRoute(input)) {
      resetForRoute(input.route->generation);
    }
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
  const double capture_interior_min_z =
      safe_min_z + config_.vertical_capture_hysteresis_m;
  const double capture_interior_max_z =
      safe_max_z - config_.vertical_capture_hysteresis_m;
  if (!(capture_interior_max_z > capture_interior_min_z)) {
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
    active_portal_id_ = event.portal.id;
    active_traversal_direction_ = event.traversal_direction;
    vertical_state_ = VerticalState::kUncaptured;
    preferred_z_m_ = event.preferred_z_m;
    effective_approach_station_m_ = event.approach_station_m;
    alignment_completion_station_m_ = event.entry_station_m;
    stationary_hold_station_m_ =
        std::max(0.0, event.entry_station_m - config_.stationary_hold_clearance_m);
    hold_position_ = pointAtRouteStation(*input.route, stationary_hold_station_m_);
    const bool starts_inside = insidePortalFootprint(position, event.portal);
    if (starts_inside) {
      entry_plane_crossed_ = true;
    }
    if (starts_inside && inside(input.state.z, safe_min_z, safe_max_z)) {
      vertical_state_ = VerticalState::kReady;
      capture_stable_cycles_ = config_.capture_stable_cycles;
      preferred_z_m_ = std::clamp(static_cast<double>(input.state.z),
                                  capture_interior_min_z, capture_interior_max_z);
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
  preferred_z_m_ =
      std::clamp(preferred_z_m_, capture_interior_min_z, capture_interior_max_z);
  position_seen_ = true;
  previous_entry_plane_distance_m_ = entry_plane_distance_m;
  previous_exit_plane_distance_m_ = exit_plane_distance_m;

  if (exit_plane_crossed_) {
    const std::size_t cleared_event_index = next_event_index_;
    ++next_event_index_;
    resetEventState();
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

  const double capture_min_z = std::max(
      capture_interior_min_z, preferred_z_m_ - config_.preferred_z_capture_tolerance_m);
  const double capture_max_z = std::min(
      capture_interior_max_z, preferred_z_m_ + config_.preferred_z_capture_tolerance_m);
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
  } else if (vertical_state_ != VerticalState::kReady) {
    capture_stable_cycles_ = 0U;
  }
  if (capture_stable_cycles_ >= config_.capture_stable_cycles) {
    vertical_state_ = VerticalState::kReady;
  }

  if (vertical_state_ == VerticalState::kReady) {
    if (inside(z_m, safe_min_z, safe_max_z)) {
      retention_violation_cycles_ = 0U;
    } else {
      retention_violation_cycles_ = std::min(retention_violation_cycles_ + 1U,
                                             config_.retention_violation_cycles);
    }
    if (retention_violation_cycles_ >= config_.retention_violation_cycles) {
      vertical_state_ = VerticalState::kUncaptured;
      capture_stable_cycles_ = 0U;
      retention_violation_cycles_ = 0U;
    }
  }

  const double vertical_error_m = distanceToInterval(z_m, capture_min_z, capture_max_z);
  const double braking_time_s = std::abs(static_cast<double>(input.state.vz)) /
                                config_.maximum_vertical_acceleration_mps2;
  const double alignment_time_s =
      braking_time_s + minimumRestToRestTime(vertical_error_m,
                                             config_.maximum_vertical_acceleration_mps2,
                                             config_.maximum_vertical_speed_mps);
  const double horizontal_speed_mps = std::hypot(static_cast<double>(input.state.vx),
                                                 static_cast<double>(input.state.vy));
  const double approach_speed_mps = std::max(
      horizontal_speed_mps,
      std::max(0.0, std::min(input.approach_speed_mps, event.speed_limit_mps)));
  const double vertical_alignment_distance_m =
      approach_speed_mps * (alignment_time_s + config_.alignment_time_margin_s);
  const double stopping_distance_m =
      horizontal_speed_mps * config_.reaction_latency_s +
      horizontal_speed_mps * horizontal_speed_mps /
          (2.0 * config_.maximum_horizontal_braking_acceleration_mps2);
  const double required_distance_m =
      std::max(vertical_alignment_distance_m, stopping_distance_m);
  effective_approach_station_m_ =
      std::min(effective_approach_station_m_,
               std::max(0.0, event.entry_station_m - required_distance_m));
  alignment_completion_station_m_ =
      std::min(alignment_completion_station_m_,
               std::max(effective_approach_station_m_,
                        event.entry_station_m -
                            approach_speed_mps * config_.alignment_time_margin_s));
  const double distance_to_entry_m =
      std::max(0.0, event.entry_station_m - input.route_station_m);
  const bool in_approach = input.route_station_m >= effective_approach_station_m_ &&
                           input.route_station_m < event.entry_station_m;
  const bool in_traversal = entry_plane_crossed_;

  PassageSpeedPrediction speed_prediction{
      .safe = vertical_state_ == VerticalState::kReady,
      .speed_limit_mps = event.speed_limit_mps,
      .vertical =
          VerticalPrediction{
              .safe = vertical_state_ == VerticalState::kReady,
              .entry_z_m = z_m,
              .entry_vz_mps = input.state.vz,
              .exit_z_m = z_m,
              .exit_vz_mps = input.state.vz,
              .minimum_z_m = z_m,
              .maximum_z_m = z_m,
          },
  };
  if (in_approach || in_traversal) {
    speed_prediction =
        maximumSafePassageSpeed(input, event, safe_min_z, safe_max_z, config_);
  }
  const bool traversal_predicted_safe = speed_prediction.safe;
  const bool movement_ready =
      vertical_state_ != VerticalState::kAlignment &&
      (in_approach || in_traversal ? traversal_predicted_safe
                                   : vertical_state_ == VerticalState::kReady);

  const double distance_to_hold_station_m =
      std::max(0.0, stationary_hold_station_m_ - input.route_station_m);
  const bool must_start_stationary_fallback =
      in_approach && !entry_plane_crossed_ && !movement_ready &&
      speed_prediction.speed_limit_mps <=
          config_.minimum_continuous_speed_mps + 1.0e-6 &&
      !speed_prediction.safe && distance_to_hold_station_m <= stopping_distance_m;
  if (must_start_stationary_fallback && vertical_state_ != VerticalState::kAlignment) {
    vertical_state_ = VerticalState::kAlignment;
    stationary_hold_station_m_ =
        std::min(event.entry_station_m - 0.25,
                 std::max(stationary_hold_station_m_, input.route_station_m));
    hold_position_ = pointAtRouteStation(*input.route, stationary_hold_station_m_);
  }

  PassageCoordinatorPhase phase = PassageCoordinatorPhase::kUpcoming;
  if (vertical_state_ == VerticalState::kAlignment) {
    phase = PassageCoordinatorPhase::kVerticalAlignment;
  } else if (in_traversal) {
    phase = PassageCoordinatorPhase::kTraversal;
  } else if (in_approach && movement_ready) {
    phase = PassageCoordinatorPhase::kReady;
  }

  const double z_reference_m =
      phase == PassageCoordinatorPhase::kVerticalAlignment
          ? preferred_z_m_
          : routePassageZReference(
                event, input.route_station_m, input.normal_flight_z_m,
                effective_approach_station_m_, alignment_completion_station_m_);
  double effective_speed_limit_mps = event.speed_limit_mps;
  if (vertical_state_ == VerticalState::kAlignment) {
    effective_speed_limit_mps = 0.0;
  } else if (in_approach || in_traversal) {
    effective_speed_limit_mps = speed_prediction.speed_limit_mps;
  }
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
      .approach_station_m = static_cast<float>(effective_approach_station_m_),
      .alignment_station_m = static_cast<float>(alignment_completion_station_m_),
      .entry_station_m = static_cast<float>(event.entry_station_m),
      .exit_station_m = static_cast<float>(event.exit_station_m),
      .departure_station_m = static_cast<float>(event.departure_station_m),
      .speed_limit_mps = static_cast<float>(effective_speed_limit_mps),
      .phase = toMppiPassagePhase(phase),
  };
  return PassageCoordinatorResult{
      .phase = phase,
      .constraint = constraint,
      .portal_id = event.portal.id,
      .route_generation = route_generation_,
      .route_event_index = next_event_index_,
      .active = true,
      .hold_xy = vertical_state_ == VerticalState::kAlignment,
      .vertical_ready = movement_ready,
      .traversal_predicted_safe = traversal_predicted_safe,
      .speed_limit_active = input.route_station_m >= effective_approach_station_m_ &&
                            !exit_plane_crossed_,
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
      .effective_approach_station_m = effective_approach_station_m_,
      .alignment_completion_station_m = alignment_completion_station_m_,
      .effective_speed_limit_mps = effective_speed_limit_mps,
      .stationary_hold_station_m = stationary_hold_station_m_,
      .predicted_entry_z_m = speed_prediction.vertical.entry_z_m,
      .predicted_entry_vz_mps = speed_prediction.vertical.entry_vz_mps,
      .predicted_exit_z_m = speed_prediction.vertical.exit_z_m,
      .predicted_exit_vz_mps = speed_prediction.vertical.exit_vz_mps,
      .predicted_minimum_z_m = speed_prediction.vertical.minimum_z_m,
      .predicted_maximum_z_m = speed_prediction.vertical.maximum_z_m,
      .entry_plane_signed_distance_m = entry_plane_distance_m,
      .exit_plane_signed_distance_m = exit_plane_distance_m,
  };
}

bool PassageCoordinator::continueTraversalForRoute(
    const PassageCoordinatorInput& input) noexcept {
  if (!input.route || !event_initialized_ || !entry_plane_crossed_ ||
      exit_plane_crossed_ || active_portal_id_.empty()) {
    return false;
  }

  const Point2 position{input.state.x, input.state.y};
  for (std::size_t index = 0U; index < input.route->passage_events.size(); ++index) {
    const RoutePassageEvent& event = input.route->passage_events[index];
    const double entry_distance_m =
        signedPlaneDistance(position, event.portal.entry_plane);
    const double exit_distance_m =
        signedPlaneDistance(position, event.portal.exit_plane);
    const bool within_traversal_region =
        std::abs(portalLateralDistance(position, event.portal)) <=
            0.5 * event.portal.width_m &&
        entry_distance_m >= -config_.exit_station_hysteresis_m &&
        exit_distance_m < config_.exit_station_hysteresis_m;
    if (event.portal.id != active_portal_id_ ||
        event.traversal_direction != active_traversal_direction_ ||
        !routeEventIsValid(event) || !within_traversal_region) {
      continue;
    }

    route_generation_ = input.route->generation;
    next_event_index_ = index;
    route_seen_ = true;
    position_seen_ = true;
    previous_entry_plane_distance_m_ = entry_distance_m;
    previous_exit_plane_distance_m_ = exit_distance_m;
    return true;
  }
  return false;
}

void PassageCoordinator::resetForRoute(const std::uint64_t generation) noexcept {
  reset();
  route_generation_ = generation;
  route_seen_ = true;
}

void PassageCoordinator::resetEventState() noexcept {
  event_initialized_ = false;
  position_seen_ = false;
  entry_plane_crossed_ = false;
  exit_plane_crossed_ = false;
  vertical_state_ = VerticalState::kUncaptured;
  capture_stable_cycles_ = 0U;
  retention_violation_cycles_ = 0U;
  active_portal_id_.clear();
  active_traversal_direction_ = 0;
  hold_position_ = {};
  preferred_z_m_ = 0.0;
  effective_approach_station_m_ = 0.0;
  alignment_completion_station_m_ = 0.0;
  stationary_hold_station_m_ = 0.0;
  previous_entry_plane_distance_m_ = 0.0;
  previous_exit_plane_distance_m_ = 0.0;
}

void PassageCoordinator::reset() noexcept {
  route_generation_ = 0U;
  next_event_index_ = 0U;
  route_seen_ = false;
  resetEventState();
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
