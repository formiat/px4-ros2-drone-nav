#include "drone_city_nav/mppi_liveness.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance3(const mppi::State& first,
                               const mppi::State& second) noexcept {
  return std::hypot(std::hypot(static_cast<double>(first.x - second.x),
                               static_cast<double>(first.y - second.y)),
                    static_cast<double>(first.z - second.z));
}

// Displacement from `anchor` to `state` measured along `tangent`, which the
// caller guarantees is a unit vector.
[[nodiscard]] double projectOnTangent(const mppi::State& state,
                                      const mppi::State& anchor,
                                      const Vec3& tangent) noexcept {
  return (static_cast<double>(state.x) - static_cast<double>(anchor.x)) * tangent.x +
         (static_cast<double>(state.y) - static_cast<double>(anchor.y)) * tangent.y +
         (static_cast<double>(state.z) - static_cast<double>(anchor.z)) * tangent.z;
}

[[nodiscard]] double speed3(const mppi::State& state) noexcept {
  return std::hypot(
      std::hypot(static_cast<double>(state.vx), static_cast<double>(state.vy)),
      static_cast<double>(state.vz));
}

} // namespace

MppiLivenessSupervisor::MppiLivenessSupervisor(const MppiLivenessConfig& config)
    : config_{config} {
  if (!(config_.observation_window_s > 0.0) ||
      !(config_.minimum_actual_displacement_m >= 0.0) ||
      !(config_.minimum_offroute_displacement_m >= 0.0) ||
      config_.stalled_windows_before_reseed == 0U) {
    throw std::invalid_argument{"invalid MPPI liveness configuration"};
  }
}

MppiLivenessSupervisor::Anchor
MppiLivenessSupervisor::anchorFrom(const MppiLivenessObservation& observation,
                                   const bool route_progress_available) const noexcept {
  return Anchor{
      .stamp_ns = observation.stamp_ns,
      .state = observation.actual_state,
      .route_generation = observation.route_generation,
      .route_station_m = observation.route_station_m,
      .route_station_valid = route_progress_available,
      .route_tangent = observation.route_tangent,
      .route_tangent_valid = observation.route_tangent_valid,
  };
}

MppiLivenessResult
MppiLivenessSupervisor::evaluate(const MppiLivenessObservation& observation) {
  MppiLivenessResult result;
  result.actual_speed_mps = speed3(observation.actual_state);
  result.predicted_head_progress_m = observation.predicted_head_progress_m;
  result.reseed_generation = reseed_generation_;
  result.recovery_active = recovery_active_;

  result.stalled_windows = stalled_windows_;

  if (!config_.enabled) {
    recovery_active_ = false;
    stalled_windows_ = 0U;
    anchor_.reset();
    result.recovery_active = false;
    result.state = MppiLivenessState::kInactive;
    return result;
  }
  if (observation.stamp_ns <= 0) {
    anchor_.reset();
    result.state = MppiLivenessState::kInactive;
    return result;
  }
  if (!observation.controller_active) {
    // A tick without a published horizon does not restart the observation
    // window: the vehicle is still not progressing. The anchor only lapses
    // once the controller has been inactive for a whole window.
    if (anchor_.has_value() &&
        static_cast<double>(observation.stamp_ns - anchor_->stamp_ns) / 1.0e9 >
            2.0 * config_.observation_window_s &&
        !recovery_active_) {
      anchor_.reset();
    }
    result.state = MppiLivenessState::kInactive;
    return result;
  }
  const bool route_progress_available = observation.route_station_valid &&
                                        observation.route_generation != 0U &&
                                        std::isfinite(observation.route_station_m);
  if (!anchor_.has_value() || observation.stamp_ns <= anchor_->stamp_ns ||
      (route_progress_available &&
       (!anchor_->route_station_valid ||
        anchor_->route_generation != observation.route_generation))) {
    // The window restarts against the route the vehicle is on now. A route
    // replaced mid-window carries no comparable station, and the displacement
    // measured across the swap belongs to neither geometry.
    anchor_ = anchorFrom(observation, route_progress_available);
    stalled_windows_ = 0U;
    result.stalled_windows = 0U;
    result.state = MppiLivenessState::kMonitoring;
    return result;
  }

  result.observation_age_s =
      static_cast<double>(observation.stamp_ns - anchor_->stamp_ns) / 1.0e9;
  result.actual_displacement_m = distance3(observation.actual_state, anchor_->state);
  result.used_route_progress = route_progress_available && anchor_->route_station_valid;
  if (result.used_route_progress) {
    result.actual_route_progress_m =
        std::max(0.0, observation.route_station_m - anchor_->route_station_m);
  }
  result.tangential_progress_m =
      anchor_->route_tangent_valid
          ? std::max(0.0, projectOnTangent(observation.actual_state, anchor_->state,
                                           anchor_->route_tangent))
          : 0.0;
  if (result.observation_age_s < config_.observation_window_s) {
    result.state = MppiLivenessState::kMonitoring;
    return result;
  }
  // Progress is the best evidence of it, not the station coordinate alone: a
  // vehicle rounding a corner or crossing to a route that has just moved
  // covers ground along the route without its station keeping up.
  result.useful_progress_m =
      result.used_route_progress
          ? std::max(result.actual_route_progress_m, result.tangential_progress_m)
          : result.actual_displacement_m;
  if (result.useful_progress_m >= config_.minimum_actual_displacement_m) {
    recovery_active_ = false;
    stalled_windows_ = 0U;
    anchor_ = anchorFrom(observation, route_progress_available);
    result.recovery_active = false;
    result.stalled_windows = 0U;
    result.state = MppiLivenessState::kMoving;
    return result;
  }

  // A body that covered real ground without gaining route station is flying a
  // manoeuvre, not standing still. Only motion that goes nowhere at all — the
  // loops a collapsed sampler makes in place — leaves the window stalled.
  if (result.used_route_progress &&
      result.actual_displacement_m >= config_.minimum_offroute_displacement_m) {
    recovery_active_ = false;
    stalled_windows_ = 0U;
    anchor_ = anchorFrom(observation, route_progress_available);
    result.recovery_active = false;
    result.stalled_windows = 0U;
    result.state = MppiLivenessState::kMoving;
    return result;
  }

  ++stalled_windows_;
  result.stalled_windows = stalled_windows_;
  anchor_ = anchorFrom(observation, route_progress_available);
  // One window without progress is a measurement; a stall has to show twice
  // before an optimised sequence is replaced by a route connector.
  if (stalled_windows_ < config_.stalled_windows_before_reseed) {
    result.state = MppiLivenessState::kMonitoring;
    return result;
  }

  ++reseed_generation_;
  recovery_active_ = true;
  stalled_windows_ = 0U;
  result.stalled_windows = 0U;
  result.state = MppiLivenessState::kReseedRequested;
  result.reseed_requested = true;
  result.recovery_active = true;
  result.reseed_generation = reseed_generation_;
  return result;
}

void MppiLivenessSupervisor::reset() noexcept {
  anchor_.reset();
  stalled_windows_ = 0U;
  recovery_active_ = false;
}

const char* mppiLivenessStateName(const MppiLivenessState state) noexcept {
  switch (state) {
    case MppiLivenessState::kInactive:
      return "inactive";
    case MppiLivenessState::kMonitoring:
      return "monitoring";
    case MppiLivenessState::kMoving:
      return "moving";
    case MppiLivenessState::kReseedRequested:
      return "reseed_requested";
  }
  return "unknown";
}

} // namespace drone_city_nav
