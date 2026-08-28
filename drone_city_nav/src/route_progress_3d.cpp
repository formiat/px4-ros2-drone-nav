#include "drone_city_nav/route_progress_3d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

RouteProgressTracker3D::RouteProgressTracker3D(const RouteProgressConfig3D& config)
    : config_{config} {
  if (!(config_.observation_window_s > 0.0) || !(config_.minimum_progress_m >= 0.0) ||
      !(config_.minimum_predicted_head_progress_m >= 0.0)) {
    throw std::invalid_argument{"invalid route progress configuration"};
  }
}

RouteProgressUpdate3D
RouteProgressTracker3D::evaluate(const RouteProgressObservation3D& observation) {
  RouteProgressUpdate3D update{
      .stall_generation = stall_generation_,
      .local_reseed_generation = local_reseed_generation_,
      .predicted_head_progress_m = observation.predicted_head_progress_m,
  };
  if (observation.stamp_ns <= 0 || observation.route_generation == 0U ||
      !std::isfinite(observation.station_m) ||
      !std::isfinite(observation.predicted_head_progress_m) ||
      !observation.controller_active) {
    anchor_valid_ = false;
    local_reseed_pending_ = false;
    return update;
  }
  if (!anchor_valid_ || observation.route_generation != anchor_route_generation_ ||
      observation.stamp_ns < anchor_stamp_ns_) {
    local_reseed_pending_ = false;
    resetAnchor(observation);
    return update;
  }

  update.observation_age_s =
      static_cast<double>(observation.stamp_ns - anchor_stamp_ns_) / 1.0e9;
  const double station_progress_m = observation.station_m - anchor_station_m_;
  const double cross_track_reduction_m =
      observation.recovery_active && std::isfinite(observation.cross_track_m) &&
              std::isfinite(anchor_cross_track_m_)
          ? anchor_cross_track_m_ - observation.cross_track_m
          : 0.0;
  update.progress_m = std::max(station_progress_m, cross_track_reduction_m);
  if (update.progress_m >= config_.minimum_progress_m) {
    local_reseed_pending_ = false;
    resetAnchor(observation);
    return update;
  }
  if (update.observation_age_s < config_.observation_window_s) {
    return update;
  }

  if (observation.predicted_head_progress_m <
      config_.minimum_predicted_head_progress_m) {
    if (!local_reseed_pending_) {
      ++local_reseed_generation_;
      update.action = RouteProgressAction3D::kReseedLocalMppi;
      update.local_reseed_requested = true;
      update.local_reseed_generation = local_reseed_generation_;
      local_reseed_pending_ = true;
      resetAnchor(observation);
      return update;
    }
    ++stall_generation_;
    update.action = RouteProgressAction3D::kReleaseLowPredictedProgress;
    update.stalled = true;
    update.stall_generation = stall_generation_;
    local_reseed_pending_ = false;
    resetAnchor(observation);
    return update;
  }

  if (!local_reseed_pending_) {
    ++local_reseed_generation_;
    update.action = RouteProgressAction3D::kReseedLocalMppi;
    update.local_reseed_requested = true;
    update.local_reseed_generation = local_reseed_generation_;
    local_reseed_pending_ = true;
    resetAnchor(observation);
    return update;
  }

  ++stall_generation_;
  update.action = RouteProgressAction3D::kReleasePredictionMismatch;
  update.stalled = true;
  update.stall_generation = stall_generation_;
  local_reseed_pending_ = false;
  resetAnchor(observation);
  return update;
}

void RouteProgressTracker3D::resetAnchor(
    const RouteProgressObservation3D& observation) noexcept {
  anchor_valid_ = true;
  anchor_stamp_ns_ = observation.stamp_ns;
  anchor_route_generation_ = observation.route_generation;
  anchor_station_m_ = observation.station_m;
  anchor_cross_track_m_ = observation.cross_track_m;
}

const char* routeReleaseReason3DName(const RouteReleaseReason3D reason) noexcept {
  switch (reason) {
    case RouteReleaseReason3D::kNone:
      return "none";
    case RouteReleaseReason3D::kNoActiveRoute:
      return "no_active_route";
    case RouteReleaseReason3D::kBlocked:
      return "blocked";
    case RouteReleaseReason3D::kExhausted:
      return "exhausted";
    case RouteReleaseReason3D::kStalled:
      return "stalled";
    case RouteReleaseReason3D::kNoEligibleRollouts:
      return "no_eligible_rollouts";
    case RouteReleaseReason3D::kDiverged:
      return "diverged";
    case RouteReleaseReason3D::kObjectiveChanged:
      return "objective_changed";
  }
  return "unknown";
}

const char* routeProgressAction3DName(const RouteProgressAction3D action) noexcept {
  switch (action) {
    case RouteProgressAction3D::kNone:
      return "none";
    case RouteProgressAction3D::kReseedLocalMppi:
      return "reseed_local_mppi";
    case RouteProgressAction3D::kReleaseLowPredictedProgress:
      return "release_low_predicted_progress";
    case RouteProgressAction3D::kReleasePredictionMismatch:
      return "release_prediction_mismatch";
  }
  return "unknown";
}

} // namespace drone_city_nav
