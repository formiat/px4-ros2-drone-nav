#include "drone_city_nav/mppi_liveness.hpp"

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

  if (!config_.enabled || !observation.controller_active || observation.stamp_ns <= 0) {
    anchor_.reset();
    emergency_braking_started_ns_.reset();
    result.state = MppiLivenessState::kInactive;
    return result;
  }
  if (observation.emergency_braking) {
    if (!emergency_braking_started_ns_.has_value() ||
        observation.stamp_ns < *emergency_braking_started_ns_) {
      emergency_braking_started_ns_ = observation.stamp_ns;
    }
    result.emergency_braking_duration_s =
        static_cast<double>(observation.stamp_ns - *emergency_braking_started_ns_) /
        1.0e9;
    result.state = MppiLivenessState::kEmergencyBraking;
    return result;
  }
  emergency_braking_started_ns_.reset();
  if (observation.predicted_terminal_progress_m <
      config_.minimum_predicted_terminal_progress_m) {
    anchor_.reset();
    result.state = MppiLivenessState::kInsufficientPrediction;
    return result;
  }
  if (!anchor_.has_value() || observation.stamp_ns <= anchor_->stamp_ns) {
    anchor_ = Anchor{observation.stamp_ns, observation.actual_state};
    result.state = MppiLivenessState::kMonitoring;
    return result;
  }

  result.observation_age_s =
      static_cast<double>(observation.stamp_ns - anchor_->stamp_ns) / 1.0e9;
  result.actual_displacement_m = distance3(observation.actual_state, anchor_->state);
  if (result.observation_age_s < config_.observation_window_s) {
    result.state = MppiLivenessState::kMonitoring;
    return result;
  }
  if (result.actual_displacement_m >= config_.minimum_actual_displacement_m) {
    anchor_ = Anchor{observation.stamp_ns, observation.actual_state};
    result.state = MppiLivenessState::kMoving;
    return result;
  }

  ++reseed_generation_;
  anchor_ = Anchor{observation.stamp_ns, observation.actual_state};
  result.state = MppiLivenessState::kReseedRequested;
  result.reseed_requested = true;
  result.reseed_generation = reseed_generation_;
  return result;
}

void MppiLivenessSupervisor::reset() noexcept {
  anchor_.reset();
  emergency_braking_started_ns_.reset();
}

const char* mppiLivenessStateName(const MppiLivenessState state) noexcept {
  switch (state) {
    case MppiLivenessState::kInactive:
      return "inactive";
    case MppiLivenessState::kEmergencyBraking:
      return "emergency_braking";
    case MppiLivenessState::kInsufficientPrediction:
      return "insufficient_prediction";
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
