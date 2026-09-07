#include "drone_city_nav/rolling_route_telemetry_3d.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {

RouteEndpointSemantics3D
effectiveRouteEndpointSemantics3D(const RouteEndpointSemantics3D planned_semantics,
                                  const bool raw_invalidation_active,
                                  const bool finite_braking_tail_active) noexcept {
  return raw_invalidation_active && finite_braking_tail_active
             ? RouteEndpointSemantics3D::kEmergencyBrakeTail
             : planned_semantics;
}

double routeSpeed3D(const Vec3& velocity) noexcept {
  if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
      !std::isfinite(velocity.z)) {
    return 0.0;
  }
  return std::hypot(velocity.x, velocity.y, velocity.z);
}

bool RollingRouteTelemetrySnapshot3D::regressionFree() const noexcept {
  return continuation_zero_speed_ticks == 0U &&
         continuation_endpoint_limited_ticks == 0U && ownership_gap_ticks == 0U &&
         continuity_transition_zero_speed_ticks == 0U &&
         moving_raw_invalidation_without_braking_tail_ticks == 0U &&
         continuity_preserving_reseed_ticks == 0U;
}

double
RollingRouteTelemetrySnapshot3D::postBootstrapRouteAvailabilityRatio() const noexcept {
  return post_bootstrap_observations == 0U
             ? 0.0
             : static_cast<double>(post_bootstrap_route_available_ticks) /
                   static_cast<double>(post_bootstrap_observations);
}

double
RollingRouteTelemetrySnapshot3D::postBootstrapRouteExecutableRatio() const noexcept {
  return post_bootstrap_observations == 0U
             ? 0.0
             : static_cast<double>(post_bootstrap_route_executable_ticks) /
                   static_cast<double>(post_bootstrap_observations);
}

RollingRouteTelemetry3D::RollingRouteTelemetry3D(RollingRouteTelemetryConfig3D config)
    : config_{config} {
  if (!std::isfinite(config_.continuation_boundary_distance_m) ||
      config_.continuation_boundary_distance_m < 0.0) {
    config_.continuation_boundary_distance_m = 1.0;
  }
  if (!std::isfinite(config_.stationary_speed_tolerance_mps) ||
      config_.stationary_speed_tolerance_mps < 0.0) {
    config_.stationary_speed_tolerance_mps = 0.25;
  }
}

void RollingRouteTelemetry3D::observe(
    const RollingRouteTelemetryObservation3D& observation) noexcept {
  ++snapshot_.observations;
  const bool route_generation_changed =
      previous_available_ && observation.route_generation != 0U &&
      previous_.route_generation != 0U &&
      observation.route_generation != previous_.route_generation;
  const bool geometry_revision_changed =
      previous_available_ && observation.geometry_revision != 0U &&
      previous_.geometry_revision != 0U &&
      observation.geometry_revision != previous_.geometry_revision;
  const bool same_continuity = previous_available_ && observation.continuity_id != 0U &&
                               observation.continuity_id == previous_.continuity_id;
  const bool continuity_preserving_update =
      observation.continuity_preserving_update ||
      (same_continuity && (route_generation_changed || geometry_revision_changed));
  const double speed_mps =
      std::isfinite(observation.speed_mps) ? std::max(0.0, observation.speed_mps) : 0.0;
  const double previous_speed_mps =
      std::isfinite(previous_.speed_mps) ? std::max(0.0, previous_.speed_mps) : 0.0;
  const bool continuation_boundary =
      observation.endpoint_semantics == RouteEndpointSemantics3D::kContinuation &&
      std::isfinite(observation.route_remaining_m) &&
      observation.route_remaining_m >= 0.0 &&
      observation.route_remaining_m <= config_.continuation_boundary_distance_m;
  if (continuation_boundary) {
    ++snapshot_.continuation_boundary_ticks;
    snapshot_.minimum_continuation_boundary_speed_mps =
        std::min(snapshot_.minimum_continuation_boundary_speed_mps, speed_mps);
    snapshot_.continuation_zero_speed_ticks +=
        speed_mps <= config_.stationary_speed_tolerance_mps ? 1U : 0U;
  }
  if (observation.endpoint_semantics == RouteEndpointSemantics3D::kContinuation &&
      observation.endpoint_limiter_active) {
    ++snapshot_.continuation_endpoint_limited_ticks;
  }
  if (same_continuity && (route_generation_changed || geometry_revision_changed)) {
    ++snapshot_.continuity_transition_ticks;
    snapshot_.minimum_continuity_transition_speed_mps =
        std::min(snapshot_.minimum_continuity_transition_speed_mps, speed_mps);
    snapshot_.maximum_continuity_transition_speed_drop_mps =
        std::max(snapshot_.maximum_continuity_transition_speed_drop_mps,
                 std::max(0.0, previous_speed_mps - speed_mps));
    snapshot_.continuity_transition_zero_speed_ticks +=
        speed_mps <= config_.stationary_speed_tolerance_mps ? 1U : 0U;
  }

  const bool ownership_gap =
      observation.resident_route_available && !observation.execution_owner_available;
  if (ownership_gap) {
    ++snapshot_.ownership_gap_ticks;
    ++consecutive_ownership_gap_ticks_;
    snapshot_.maximum_consecutive_ownership_gap_ticks =
        std::max(snapshot_.maximum_consecutive_ownership_gap_ticks,
                 consecutive_ownership_gap_ticks_);
    if (!previous_ownership_gap_) {
      ++snapshot_.ownership_gap_episodes;
    }
  } else {
    consecutive_ownership_gap_ticks_ = 0U;
  }

  const bool moving_raw_invalidation =
      observation.raw_invalidation_active &&
      speed_mps > config_.stationary_speed_tolerance_mps;
  if (moving_raw_invalidation) {
    ++snapshot_.moving_raw_invalidation_ticks;
    snapshot_.moving_raw_invalidation_without_braking_tail_ticks +=
        observation.finite_braking_tail_active ? 0U : 1U;
  }
  if (observation.finite_braking_tail_active && !previous_braking_tail_active_) {
    ++snapshot_.finite_braking_tail_activations;
  }

  const bool route_available =
      observation.resident_route_available && observation.execution_owner_available;
  route_bootstrapped_ = route_bootstrapped_ || route_available;
  if (route_bootstrapped_) {
    ++snapshot_.post_bootstrap_observations;
    snapshot_.post_bootstrap_route_available_ticks += route_available ? 1U : 0U;
    snapshot_.post_bootstrap_route_executable_ticks +=
        observation.resident_route_available &&
                observation.route_execution_owner_available
            ? 1U
            : 0U;
    snapshot_.post_bootstrap_no_executable_route_hold_ticks +=
        observation.no_executable_route_hold ? 1U : 0U;
  }

  snapshot_.nominal_reseed_ticks += observation.nominal_reseeded ? 1U : 0U;
  if (observation.nominal_reseeded && continuity_preserving_update) {
    ++snapshot_.continuity_preserving_reseed_ticks;
  }
  if (route_generation_changed) {
    ++snapshot_.route_generation_changes;
    if (same_continuity) {
      ++snapshot_.continuity_preserving_generation_changes;
    }
  }
  if (geometry_revision_changed) {
    ++snapshot_.geometry_revision_changes;
  }

  previous_ = observation;
  previous_available_ = true;
  previous_ownership_gap_ = ownership_gap;
  previous_braking_tail_active_ = observation.finite_braking_tail_active;
}

const RollingRouteTelemetrySnapshot3D&
RollingRouteTelemetry3D::snapshot() const noexcept {
  return snapshot_;
}

void RollingRouteTelemetry3D::reset() noexcept {
  snapshot_ = {};
  previous_ = {};
  consecutive_ownership_gap_ticks_ = 0U;
  previous_available_ = false;
  previous_ownership_gap_ = false;
  previous_braking_tail_active_ = false;
  route_bootstrapped_ = false;
}

} // namespace drone_city_nav
