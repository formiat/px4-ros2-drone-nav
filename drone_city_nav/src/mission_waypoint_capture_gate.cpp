#include "drone_city_nav/mission_waypoint_capture_gate.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] std::int64_t durationNanoseconds(const double seconds,
                                               const char* const name,
                                               const bool allow_zero) {
  const long double nanoseconds = static_cast<long double>(seconds) * 1'000'000'000.0L;
  const long double first_unrepresentable_rounding_input =
      static_cast<long double>(std::numeric_limits<std::int64_t>::max()) + 0.5L;
  if (!std::isfinite(seconds) || (allow_zero ? seconds < 0.0 : !(seconds > 0.0)) ||
      nanoseconds >= first_unrepresentable_rounding_input) {
    throw std::invalid_argument{std::string{name} + " is invalid"};
  }
  const std::int64_t duration_ns = static_cast<std::int64_t>(std::llround(nanoseconds));
  if (!allow_zero && duration_ns <= 0) {
    throw std::invalid_argument{std::string{name} + " rounds to zero"};
  }
  return duration_ns;
}

[[nodiscard]] bool timestampFresh(const std::int64_t source_stamp_ns,
                                  const std::int64_t now_ns,
                                  const std::int64_t maximum_age_ns) noexcept {
  return source_stamp_ns > 0 && now_ns >= source_stamp_ns && maximum_age_ns > 0 &&
         now_ns - source_stamp_ns <= maximum_age_ns;
}

[[nodiscard]] bool finiteNonnegative(const double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] bool samePoint(const Point3& first, const Point3& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

} // namespace

bool missionWaypointStationaryRearmEligible(
    const MissionWaypointStationaryRearmGateConfig& config,
    const MissionWaypointStationaryRearmObservation& observation) noexcept {
  const bool config_valid = std::isfinite(config.maximum_pose_age_s) &&
                            config.maximum_pose_age_s > 0.0 &&
                            std::isfinite(config.maximum_vehicle_status_age_s) &&
                            config.maximum_vehicle_status_age_s > 0.0 &&
                            std::isfinite(config.maximum_offboard_session_age_s) &&
                            config.maximum_offboard_session_age_s > 0.0 &&
                            finiteNonnegative(config.position_tolerance_m) &&
                            finiteNonnegative(config.speed_tolerance_mps) &&
                            finiteNonnegative(config.yaw_rate_tolerance_radps);
  if (!config_valid || observation.stamp_ns <= 0 ||
      observation.offboard_instance_id == 0U || !observation.objective_eligible ||
      !observation.goal_capture_latched ||
      !observation.execution_input_state_authoritative ||
      !observation.position_velocity_authoritative ||
      !observation.yaw_rate_authoritative || !observation.vehicle_status_valid ||
      !observation.vehicle_status_epoch_stable || !observation.armed ||
      !observation.offboard_session_valid || !observation.applied_control_empty ||
      !observation.horizon_owner_empty ||
      !observation.execution_snapshot_revoked_empty ||
      !observation.validation_policy_current || !observation.world_evidence_current ||
      !observation.lidar_evidence_current || !finitePoint(observation.mission_goal) ||
      !finitePoint(observation.active_waypoint_goal) ||
      !finitePoint(observation.position) || !finitePoint(observation.velocity) ||
      !std::isfinite(observation.yaw_rate_radps) ||
      !samePoint(observation.mission_goal, observation.active_waypoint_goal)) {
    return false;
  }

  const auto maximum_age_ns = [](const double seconds) noexcept {
    return static_cast<long double>(seconds) * 1'000'000'000.0L;
  };
  const auto fresh = [&](const std::int64_t source_stamp_ns,
                         const double maximum_age_s) noexcept {
    return source_stamp_ns > 0 && source_stamp_ns <= observation.stamp_ns &&
           static_cast<long double>(observation.stamp_ns - source_stamp_ns) <=
               maximum_age_ns(maximum_age_s);
  };
  if (!fresh(observation.pose_receive_stamp_ns, config.maximum_pose_age_s) ||
      !fresh(observation.vehicle_status_receive_stamp_ns,
             config.maximum_vehicle_status_age_s) ||
      !fresh(observation.offboard_session_source_stamp_ns,
             config.maximum_offboard_session_age_s) ||
      !fresh(observation.offboard_session_receive_stamp_ns,
             config.maximum_offboard_session_age_s)) {
    return false;
  }

  const double total_speed_mps =
      std::hypot(std::hypot(observation.velocity.x, observation.velocity.y),
                 observation.velocity.z);
  return distance3D(observation.position, observation.mission_goal) <=
             config.position_tolerance_m &&
         total_speed_mps <= config.speed_tolerance_mps &&
         std::abs(observation.yaw_rate_radps) <= config.yaw_rate_tolerance_radps;
}

MissionWaypointCaptureGate::MissionWaypointCaptureGate(
    const MissionWaypointCaptureGateConfig& config)
    : config_{config},
      required_stop_hold_ns_{
          durationNanoseconds(config.stop_hold_s, "stop_hold_s", true)},
      maximum_pose_age_ns_{
          durationNanoseconds(config.maximum_pose_age_s, "maximum_pose_age_s", false)},
      maximum_vehicle_status_age_ns_{durationNanoseconds(
          config.maximum_vehicle_status_age_s, "maximum_vehicle_status_age_s", false)},
      maximum_feedback_age_ns_{durationNanoseconds(config.maximum_feedback_age_s,
                                                   "maximum_feedback_age_s", false)} {
  if (!std::isfinite(config_.goal_radius_m) || !(config_.goal_radius_m > 0.0) ||
      !std::isfinite(config_.target_match_tolerance_m) ||
      !(config_.target_match_tolerance_m >= 0.0) ||
      !std::isfinite(config_.stop_speed_mps) || !(config_.stop_speed_mps >= 0.0)) {
    throw std::invalid_argument{"mission waypoint capture gate geometry is invalid"};
  }
}

void MissionWaypointCaptureGate::beginTick(const std::int64_t stamp_ns) noexcept {
  continuity_broken_on_begin_ = false;
  if (tick_open_ || stamp_ns <= 0 ||
      (open_tick_stamp_ns_ > 0 && stamp_ns <= open_tick_stamp_ns_)) {
    continuity_broken_on_begin_ = continuous_since_ns_ != 0;
    resetContinuity();
  }
  open_tick_stamp_ns_ = stamp_ns;
  tick_open_ = stamp_ns > 0;
}

MissionWaypointCaptureGateResult MissionWaypointCaptureGate::update(
    const MissionWaypointCaptureObservation& observation) noexcept {
  MissionWaypointCaptureGateResult result;
  result.continuity_broken = continuity_broken_on_begin_;
  continuity_broken_on_begin_ = false;
  if (!tick_open_ || observation.stamp_ns != open_tick_stamp_ns_) {
    result.continuity_broken = result.continuity_broken || continuous_since_ns_ != 0;
    tick_open_ = false;
    resetContinuity();
    return result;
  }
  tick_open_ = false;
  if (!evidenceValid(observation)) {
    result.continuity_broken = result.continuity_broken || continuous_since_ns_ != 0;
    resetContinuity();
    return result;
  }

  result.evidence_valid = true;
  const bool identity_changed =
      continuous_since_ns_ != 0 &&
      (continuous_horizon_producer_instance_id_ !=
           observation.horizon_producer_instance_id ||
       continuous_horizon_sequence_ != observation.horizon_sequence ||
       continuous_offboard_instance_id_ != observation.target_offboard_instance_id ||
       continuous_feedback_generation_ != observation.feedback_continuity_generation);
  if (identity_changed) {
    result.continuity_broken = true;
    resetContinuity();
  }
  if (continuous_since_ns_ == 0) {
    continuous_since_ns_ = observation.stamp_ns;
    continuous_horizon_producer_instance_id_ = observation.horizon_producer_instance_id;
    continuous_horizon_sequence_ = observation.horizon_sequence;
    continuous_offboard_instance_id_ = observation.target_offboard_instance_id;
    continuous_feedback_generation_ = observation.feedback_continuity_generation;
    result.continuity_started = true;
  }
  result.continuous_since_ns = continuous_since_ns_;
  result.continuous_duration_ns = observation.stamp_ns - continuous_since_ns_;
  result.ready = result.continuous_duration_ns >= required_stop_hold_ns_;
  return result;
}

void MissionWaypointCaptureGate::reset() noexcept {
  tick_open_ = false;
  open_tick_stamp_ns_ = 0;
  continuity_broken_on_begin_ = false;
  resetContinuity();
}

bool MissionWaypointCaptureGate::evidenceValid(
    const MissionWaypointCaptureObservation& observation) const noexcept {
  if (observation.stamp_ns <= 0 || !observation.goal_capture_latched ||
      !observation.position_velocity_authoritative ||
      !observation.vehicle_status_valid || !observation.vehicle_status_epoch_stable ||
      !observation.armed || !observation.horizon_valid ||
      !observation.horizon_position_hold || !observation.horizon_goal_capture ||
      !observation.horizon_stationary_position_hold || !observation.feedback_valid ||
      !observation.feedback_position_hold ||
      observation.feedback_control_authoritative ||
      !observation.feedback_continuity_generation_valid ||
      !finitePoint(observation.goal) || !finitePoint(observation.position) ||
      !finitePoint(observation.velocity) || !finitePoint(observation.route_target) ||
      !finitePoint(observation.stationary_hold_position)) {
    return false;
  }
  if (!timestampFresh(observation.pose_receive_stamp_ns, observation.stamp_ns,
                      maximum_pose_age_ns_) ||
      !timestampFresh(observation.vehicle_status_receive_stamp_ns, observation.stamp_ns,
                      maximum_vehicle_status_age_ns_) ||
      !timestampFresh(observation.feedback_source_stamp_ns, observation.stamp_ns,
                      maximum_feedback_age_ns_) ||
      !timestampFresh(observation.feedback_receive_stamp_ns, observation.stamp_ns,
                      maximum_feedback_age_ns_)) {
    return false;
  }
  if (observation.horizon_valid_from_ns <= 0 ||
      observation.horizon_valid_until_ns <= observation.horizon_valid_from_ns ||
      observation.stamp_ns < observation.horizon_valid_from_ns ||
      observation.stamp_ns >= observation.horizon_valid_until_ns ||
      observation.feedback_source_stamp_ns < observation.horizon_valid_from_ns ||
      observation.feedback_source_stamp_ns >= observation.horizon_valid_until_ns) {
    return false;
  }
  if (observation.horizon_producer_instance_id == 0U ||
      observation.horizon_sequence == 0U ||
      observation.target_offboard_instance_id == 0U ||
      observation.feedback_horizon_producer_instance_id !=
          observation.horizon_producer_instance_id ||
      observation.feedback_horizon_sequence != observation.horizon_sequence ||
      observation.feedback_offboard_instance_id !=
          observation.target_offboard_instance_id) {
    return false;
  }

  const double goal_distance_m = distance3D(observation.position, observation.goal);
  const double total_speed_mps =
      std::hypot(std::hypot(observation.velocity.x, observation.velocity.y),
                 observation.velocity.z);
  return goal_distance_m <= config_.goal_radius_m &&
         total_speed_mps <= config_.stop_speed_mps &&
         distance3D(observation.route_target, observation.goal) <=
             config_.target_match_tolerance_m &&
         distance3D(observation.stationary_hold_position, observation.goal) <=
             config_.target_match_tolerance_m &&
         distance3D(observation.route_target, observation.stationary_hold_position) <=
             config_.target_match_tolerance_m;
}

void MissionWaypointCaptureGate::resetContinuity() noexcept {
  continuous_since_ns_ = 0;
  continuous_horizon_producer_instance_id_ = 0U;
  continuous_horizon_sequence_ = 0U;
  continuous_offboard_instance_id_ = 0U;
  continuous_feedback_generation_ = 0U;
}

} // namespace drone_city_nav
