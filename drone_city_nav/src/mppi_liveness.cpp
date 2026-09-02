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
      !(config_.minimum_predicted_terminal_progress_m >= 0.0)) {
    throw std::invalid_argument{"invalid MPPI liveness configuration"};
  }
}

MppiLivenessResult
MppiLivenessSupervisor::evaluate(const MppiLivenessObservation& observation) {
  MppiLivenessResult result;
  result.actual_speed_mps = speed3(observation.actual_state);
  result.predicted_head_progress_m = observation.predicted_head_progress_m;
  result.predicted_terminal_progress_m = observation.predicted_terminal_progress_m;
  result.reseed_generation = reseed_generation_;
  result.recovery_active = recovery_active_;

  if (!config_.enabled) {
    recovery_active_ = false;
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
    anchor_ = Anchor{observation.stamp_ns, observation.actual_state,
                     observation.route_generation, observation.route_station_m,
                     route_progress_available};
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
  if (result.observation_age_s < config_.observation_window_s) {
    result.state = MppiLivenessState::kMonitoring;
    return result;
  }
  const double useful_progress_m = result.used_route_progress
                                       ? result.actual_route_progress_m
                                       : result.actual_displacement_m;
  if (useful_progress_m >= config_.minimum_actual_displacement_m) {
    recovery_active_ = false;
    anchor_ = Anchor{observation.stamp_ns, observation.actual_state,
                     observation.route_generation, observation.route_station_m,
                     route_progress_available};
    result.recovery_active = false;
    result.state = MppiLivenessState::kMoving;
    return result;
  }

  ++reseed_generation_;
  recovery_active_ = true;
  anchor_ = Anchor{observation.stamp_ns, observation.actual_state,
                   observation.route_generation, observation.route_station_m,
                   route_progress_available};
  result.state = MppiLivenessState::kReseedRequested;
  result.reseed_requested = true;
  result.recovery_active = true;
  result.reseed_generation = reseed_generation_;
  if (observation.predicted_terminal_progress_m <
      config_.minimum_predicted_terminal_progress_m) {
    result.state = MppiLivenessState::kReseedRequested;
  }
  return result;
}

void MppiLivenessSupervisor::reset() noexcept {
  anchor_.reset();
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
