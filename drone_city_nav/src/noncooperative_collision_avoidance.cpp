#include "drone_city_nav/noncooperative_collision_avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] double dotProduct(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] double magnitude(const Vec3& vector) noexcept {
  return std::sqrt(dotProduct(vector, vector));
}

[[nodiscard]] NonCooperativeThreatReason
classifyThreat(const double current_range_m, const double closing_speed_mps,
               const double closest_approach_distance_m,
               const NonCooperativeAvoidanceConfig& config) noexcept {
  if (current_range_m < config.strong_separation_m) {
    return NonCooperativeThreatReason::kStrongCurrentSeparation;
  }
  if (closing_speed_mps > 0.0 &&
      closest_approach_distance_m < config.strong_separation_m) {
    return NonCooperativeThreatReason::kStrongPredictedClosestApproach;
  }
  if (current_range_m < config.anticipation_separation_m) {
    return NonCooperativeThreatReason::kNearby;
  }
  if (closing_speed_mps > 0.0 &&
      closest_approach_distance_m < config.anticipation_separation_m) {
    return NonCooperativeThreatReason::kConverging;
  }
  return NonCooperativeThreatReason::kNone;
}

[[nodiscard]] bool strongThreat(const NonCooperativeThreatReason reason) noexcept {
  return reason == NonCooperativeThreatReason::kStrongCurrentSeparation ||
         reason == NonCooperativeThreatReason::kStrongPredictedClosestApproach;
}

[[nodiscard]] bool betterThreat(const NonCooperativeClosestApproach& candidate,
                                const NonCooperativeClosestApproach& selected) {
  const bool candidate_relevant = candidate.reason != NonCooperativeThreatReason::kNone;
  const bool selected_relevant = selected.reason != NonCooperativeThreatReason::kNone;
  return std::tuple{candidate_relevant, strongThreat(candidate.reason),
                    -candidate.closest_approach_distance_m, -candidate.current_range_m,
                    std::numeric_limits<std::uint64_t>::max() -
                        candidate.local_track_id} >
         std::tuple{selected_relevant, strongThreat(selected.reason),
                    -selected.closest_approach_distance_m, -selected.current_range_m,
                    std::numeric_limits<std::uint64_t>::max() -
                        selected.local_track_id};
}

} // namespace

const char*
nonCooperativeThreatReasonName(const NonCooperativeThreatReason reason) noexcept {
  switch (reason) {
    case NonCooperativeThreatReason::kNone:
      return "none";
    case NonCooperativeThreatReason::kNearby:
      return "nearby";
    case NonCooperativeThreatReason::kConverging:
      return "converging";
    case NonCooperativeThreatReason::kStrongCurrentSeparation:
      return "strong_current_separation";
    case NonCooperativeThreatReason::kStrongPredictedClosestApproach:
      return "strong_predicted_closest_approach";
  }
  return "unknown";
}

const char* nonCooperativeAvoidanceLifecycleStateName(
    const NonCooperativeAvoidanceLifecycleState state) noexcept {
  switch (state) {
    case NonCooperativeAvoidanceLifecycleState::kInactive:
      return "inactive";
    case NonCooperativeAvoidanceLifecycleState::kEntered:
      return "entered";
    case NonCooperativeAvoidanceLifecycleState::kHolding:
      return "holding";
    case NonCooperativeAvoidanceLifecycleState::kReleased:
      return "released";
  }
  return "unknown";
}

NonCooperativeCollisionAvoidance::NonCooperativeCollisionAvoidance(
    const NonCooperativeAvoidanceConfig& config)
    : config_{config} {
  if (!(config_.prediction_horizon_s > 0.0) || !(config_.strong_separation_m > 0.0) ||
      !(config_.anticipation_separation_m > config_.strong_separation_m) ||
      !(config_.release_separation_m > config_.strong_separation_m) ||
      config_.release_separation_m > config_.anticipation_separation_m ||
      !(config_.release_hysteresis_s >= 0.0) || !(config_.maximum_track_age_s > 0.0) ||
      !(config_.tracked_aircraft_radius_m >= 0.0) ||
      !(config_.minimum_relative_speed_mps > 0.0) ||
      !(config_.candidate_acceleration_fraction > 0.0) ||
      config_.candidate_acceleration_fraction > 1.0 ||
      !(config_.candidate_duration_s > 0.0) || !(config_.strong_cost_weight > 0.0) ||
      !(config_.anticipation_cost_weight >= 0.0) ||
      !(config_.time_to_collision_gain_s >= 0.0) ||
      !(config_.maximum_time_to_collision_multiplier >= 1.0)) {
    throw std::invalid_argument{"invalid non-cooperative avoidance configuration"};
  }
}

NonCooperativeAvoidanceUpdate
NonCooperativeCollisionAvoidance::update(const NonCooperativeAvoidanceInput& input) {
  NonCooperativeAvoidanceUpdate result;
  result.received_track_count = input.tracks.size();
  result.lifecycle_generation = lifecycle_generation_;
  result.cost_policy = mppi::DynamicAircraftCostPolicy{
      .strong_separation_m = static_cast<float>(config_.strong_separation_m),
      .anticipation_separation_m =
          static_cast<float>(config_.anticipation_separation_m),
      .strong_weight = static_cast<float>(config_.strong_cost_weight),
      .anticipation_weight = static_cast<float>(config_.anticipation_cost_weight),
      .time_to_collision_gain_s = static_cast<float>(config_.time_to_collision_gain_s),
      .maximum_time_to_collision_multiplier =
          static_cast<float>(config_.maximum_time_to_collision_multiplier),
  };
  if (input.now_ns <= 0 || input.horizon_steps == 0U || !(input.step_s > 0.0)) {
    result.active = active_;
    return result;
  }

  std::vector<NonCooperativeAircraftTrack> tracks = input.tracks;
  std::ranges::sort(tracks, {}, &NonCooperativeAircraftTrack::local_track_id);
  bool strong_threat_present = false;
  bool release_clear = true;
  for (const NonCooperativeAircraftTrack& track : tracks) {
    if (track.local_track_id == 0U || !track.position_valid ||
        track.measurement_stamp_ns <= 0 || !finite(track.position) ||
        (track.velocity_valid && !finite(track.velocity))) {
      continue;
    }
    const double track_age_s =
        static_cast<double>(input.now_ns - track.measurement_stamp_ns) * 1.0e-9;
    if (track_age_s < 0.0 || track_age_s > config_.maximum_track_age_s) {
      continue;
    }
    const Vec3 velocity = track.velocity_valid ? track.velocity : Vec3{};
    const Point3 current_position{
        track.position.x + velocity.x * track_age_s,
        track.position.y + velocity.y * track_age_s,
        track.position.z + velocity.z * track_age_s,
    };
    const Vec3 relative_position{
        current_position.x - static_cast<double>(input.ownship.x),
        current_position.y - static_cast<double>(input.ownship.y),
        current_position.z - static_cast<double>(input.ownship.z),
    };
    const Vec3 relative_velocity{
        velocity.x - static_cast<double>(input.ownship.vx),
        velocity.y - static_cast<double>(input.ownship.vy),
        velocity.z - static_cast<double>(input.ownship.vz),
    };
    const double current_range_m = magnitude(relative_position);
    if (!std::isfinite(current_range_m)) {
      continue;
    }
    const double relative_speed_squared =
        dotProduct(relative_velocity, relative_velocity);
    const double minimum_relative_speed_squared =
        config_.minimum_relative_speed_mps * config_.minimum_relative_speed_mps;
    const double time_to_closest_approach_s =
        relative_speed_squared > minimum_relative_speed_squared
            ? std::clamp(-dotProduct(relative_position, relative_velocity) /
                             relative_speed_squared,
                         0.0, config_.prediction_horizon_s)
            : 0.0;
    const Vec3 closest_relative_position{
        relative_position.x + relative_velocity.x * time_to_closest_approach_s,
        relative_position.y + relative_velocity.y * time_to_closest_approach_s,
        relative_position.z + relative_velocity.z * time_to_closest_approach_s,
    };
    const double closest_approach_distance_m = magnitude(closest_relative_position);
    const double closing_speed_mps =
        current_range_m > 1.0e-6
            ? -dotProduct(relative_position, relative_velocity) / current_range_m
            : 0.0;
    const NonCooperativeThreatReason reason = classifyThreat(
        current_range_m, closing_speed_mps, closest_approach_distance_m, config_);
    NonCooperativeClosestApproach closest_approach{
        .local_track_id = track.local_track_id,
        .track_age_s = track_age_s,
        .current_range_m = current_range_m,
        .closing_speed_mps = closing_speed_mps,
        .time_to_closest_approach_s = time_to_closest_approach_s,
        .closest_approach_distance_m = closest_approach_distance_m,
        .current_position = current_position,
        .estimated_velocity = velocity,
        .velocity_valid = track.velocity_valid,
        .reason = reason,
    };
    if (!result.primary_threat ||
        betterThreat(closest_approach, *result.primary_threat)) {
      result.primary_threat = closest_approach;
    }
    strong_threat_present = strong_threat_present || strongThreat(reason);
    release_clear = release_clear && current_range_m > config_.release_separation_m &&
                    (closing_speed_mps <= 0.0 ||
                     closest_approach_distance_m > config_.release_separation_m);
    auto samples = std::make_shared<std::vector<mppi::DynamicAircraftSample>>();
    samples->reserve(input.horizon_steps);
    for (std::size_t step = 0U; step < input.horizon_steps; ++step) {
      const double elapsed_s = static_cast<double>(step + 1U) * input.step_s;
      samples->push_back(mppi::DynamicAircraftSample{
          .x = static_cast<float>(current_position.x + velocity.x * elapsed_s),
          .y = static_cast<float>(current_position.y + velocity.y * elapsed_s),
          .z = static_cast<float>(current_position.z + velocity.z * elapsed_s),
      });
    }
    result.trajectories.push_back(mppi::DynamicAircraftTrajectory{
        .samples = std::move(samples),
        .footprint_radius_m = static_cast<float>(config_.tracked_aircraft_radius_m),
        .active_steps = input.horizon_steps,
    });
    ++result.fresh_track_count;
    result.maximum_radar_age_s = std::max(result.maximum_radar_age_s, track_age_s);
  }

  const bool entered = !active_ && strong_threat_present;
  bool released = false;
  if (entered) {
    active_ = true;
    clear_since_ns_ = 0;
    ++lifecycle_generation_;
  } else if (active_) {
    if (strong_threat_present || !release_clear) {
      clear_since_ns_ = 0;
    } else if (clear_since_ns_ == 0) {
      clear_since_ns_ = input.now_ns;
    } else if (static_cast<double>(input.now_ns - clear_since_ns_) * 1.0e-9 >=
               config_.release_hysteresis_s) {
      active_ = false;
      released = true;
      clear_since_ns_ = 0;
      ++lifecycle_generation_;
    }
  }

  result.active = active_;
  result.lifecycle_generation = lifecycle_generation_;
  if (entered) {
    result.lifecycle_state = NonCooperativeAvoidanceLifecycleState::kEntered;
  } else if (released) {
    result.lifecycle_state = NonCooperativeAvoidanceLifecycleState::kReleased;
  } else if (active_) {
    result.lifecycle_state = NonCooperativeAvoidanceLifecycleState::kHolding;
  } else {
    result.lifecycle_state = NonCooperativeAvoidanceLifecycleState::kInactive;
  }
  if (active_ && result.primary_threat.has_value()) {
    const Vec3 threat_direction{
        result.primary_threat->current_position.x -
            static_cast<double>(input.ownship.x),
        result.primary_threat->current_position.y -
            static_cast<double>(input.ownship.y),
        result.primary_threat->current_position.z -
            static_cast<double>(input.ownship.z),
    };
    const double threat_distance_m = magnitude(threat_direction);
    if (threat_distance_m > 1.0e-6) {
      result.acquisition = mppi::NonCooperativeSeparationAcquisition{
          .threat_direction_x =
              static_cast<float>(threat_direction.x / threat_distance_m),
          .threat_direction_y =
              static_cast<float>(threat_direction.y / threat_distance_m),
          .threat_direction_z =
              static_cast<float>(threat_direction.z / threat_distance_m),
          .candidate_acceleration_fraction =
              static_cast<float>(config_.candidate_acceleration_fraction),
          .candidate_duration_s = static_cast<float>(config_.candidate_duration_s),
          .generation = lifecycle_generation_,
      };
    }
  }
  return result;
}

} // namespace drone_city_nav
