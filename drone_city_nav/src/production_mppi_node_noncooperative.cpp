#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] double
diagnosticValue(const std::optional<NonCooperativeClosestApproach>& threat,
                double NonCooperativeClosestApproach::*member) noexcept {
  return threat ? (*threat).*member : -1.0;
}

} // namespace

void ProductionMppiNode::configureNonCooperativeAvoidance() {
  noncooperative_avoidance_enabled_ =
      declare_parameter<bool>("noncooperative_avoidance_enabled", false);
  noncooperative_tracks_topic_ = declare_parameter<std::string>(
      "noncooperative_tracks_topic", "/drone_city_nav/noncooperative_tracks");
  noncooperative_avoidance_config_.prediction_horizon_s =
      declare_parameter<double>("noncooperative_prediction_horizon_s", 4.0);
  noncooperative_avoidance_config_.strong_separation_m =
      declare_parameter<double>("noncooperative_strong_separation_m", 10.0);
  noncooperative_avoidance_config_.anticipation_separation_m =
      declare_parameter<double>("noncooperative_anticipation_separation_m", 20.0);
  noncooperative_avoidance_config_.release_separation_m =
      declare_parameter<double>("noncooperative_release_separation_m", 15.0);
  noncooperative_avoidance_config_.release_hysteresis_s =
      declare_parameter<double>("noncooperative_release_hysteresis_s", 1.0);
  noncooperative_avoidance_config_.maximum_track_age_s =
      declare_parameter<double>("noncooperative_maximum_track_age_s", 0.75);
  noncooperative_avoidance_config_.tracked_aircraft_radius_m =
      declare_parameter<double>("noncooperative_tracked_aircraft_radius_m", 0.82);
  noncooperative_avoidance_config_.minimum_relative_speed_mps =
      declare_parameter<double>("noncooperative_minimum_relative_speed_mps", 0.05);
  noncooperative_avoidance_config_.candidate_acceleration_fraction =
      declare_parameter<double>("noncooperative_candidate_acceleration_fraction", 0.95);
  noncooperative_avoidance_config_.candidate_duration_s =
      declare_parameter<double>("noncooperative_candidate_duration_s", 1.5);
  noncooperative_avoidance_config_.strong_cost_weight =
      declare_parameter<double>("noncooperative_strong_cost_weight", 4000.0);
  noncooperative_avoidance_config_.anticipation_cost_weight =
      declare_parameter<double>("noncooperative_anticipation_cost_weight", 40.0);
  noncooperative_avoidance_config_.time_to_collision_gain_s =
      declare_parameter<double>("noncooperative_time_to_collision_gain_s", 1.0);
  noncooperative_avoidance_config_.maximum_time_to_collision_multiplier =
      declare_parameter<double>("noncooperative_maximum_ttc_multiplier", 4.0);

  if (!noncooperative_avoidance_enabled_) {
    return;
  }
  if (cooperative_traffic_enabled_) {
    throw std::invalid_argument{
        "cooperative and non-cooperative avoidance cannot be enabled together"};
  }
  if (noncooperative_tracks_topic_.empty()) {
    throw std::invalid_argument{"non-cooperative tracks topic must not be empty"};
  }
  noncooperative_avoidance_ = std::make_unique<NonCooperativeCollisionAvoidance>(
      noncooperative_avoidance_config_);
  RCLCPP_INFO(get_logger(),
              "NONCOOPERATIVE_AVOIDANCE_CONFIG enabled=true vehicle_id='%s' "
              "tracks_topic='%s' "
              "prediction_horizon_s=%.2f strong_separation_m=%.2f "
              "anticipation_separation_m=%.2f release_separation_m=%.2f "
              "maximum_track_age_s=%.2f strong_cost_weight=%.1f",
              vehicle_id_.c_str(), noncooperative_tracks_topic_.c_str(),
              noncooperative_avoidance_config_.prediction_horizon_s,
              noncooperative_avoidance_config_.strong_separation_m,
              noncooperative_avoidance_config_.anticipation_separation_m,
              noncooperative_avoidance_config_.release_separation_m,
              noncooperative_avoidance_config_.maximum_track_age_s,
              noncooperative_avoidance_config_.strong_cost_weight);
}

void ProductionMppiNode::createNonCooperativeAvoidanceInterface(
    const rclcpp::SubscriptionOptions& subscription_options) {
  if (!noncooperative_avoidance_enabled_) {
    return;
  }
  noncooperative_tracks_sub_ = create_subscription<msg::TargetTrackArray>(
      noncooperative_tracks_topic_, rclcpp::QoS{2}.reliable(),
      [this](const msg::TargetTrackArray::SharedPtr message) {
        onNonCooperativeTracks(*message);
      },
      subscription_options);
}

void ProductionMppiNode::onNonCooperativeTracks(const msg::TargetTrackArray& message) {
  ProductionMppiNonCooperativeTracks snapshot;
  snapshot.source_scan_sequence = message.source_scan_sequence;
  snapshot.receive_stamp_ns = get_clock()->now().nanoseconds();
  if (!message.header.frame_id.empty() && message.header.frame_id != frame_id_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "NONCOOPERATIVE_AVOIDANCE rejected_tracks=true "
                         "reason=frame_mismatch expected='%s' actual='%s'",
                         frame_id_.c_str(), message.header.frame_id.c_str());
  } else {
    snapshot.tracks.reserve(message.tracks.size());
    const std::int64_t array_stamp_ns = timeNanoseconds(message.header.stamp);
    for (const msg::TargetTrack& track : message.tracks) {
      const std::int64_t track_stamp_ns = timeNanoseconds(track.header.stamp);
      snapshot.tracks.push_back(NonCooperativeAircraftTrack{
          .local_track_id = track.track_id,
          .position = Point3{track.position.x, track.position.y, track.position.z},
          .velocity = Vec3{track.velocity.x, track.velocity.y, track.velocity.z},
          .measurement_stamp_ns = track_stamp_ns > 0 ? track_stamp_ns : array_stamp_ns,
          .position_valid = track.position_valid,
          .velocity_valid = track.velocity_valid,
      });
    }
  }
  const std::scoped_lock lock{input_mutex_};
  noncooperative_tracks_ = std::move(snapshot);
}

ProductionMppiNonCooperativeUpdate ProductionMppiNode::prepareNonCooperativeTick(
    const mppi::State& ownship, const ProductionMppiNonCooperativeTracks& tracks,
    const std::int64_t now_ns) {
  ProductionMppiNonCooperativeUpdate result{
      .source_scan_sequence = tracks.source_scan_sequence,
      .transport_age_ms = tracks.receive_stamp_ns > 0
                              ? static_cast<double>(std::max<std::int64_t>(
                                    0, now_ns - tracks.receive_stamp_ns)) /
                                    1.0e6
                              : -1.0,
      .enabled = noncooperative_avoidance_enabled_,
  };
  if (!noncooperative_avoidance_enabled_ || !noncooperative_avoidance_) {
    return result;
  }
  result.avoidance = noncooperative_avoidance_->update(NonCooperativeAvoidanceInput{
      .ownship = ownship,
      .tracks = tracks.tracks,
      .now_ns = now_ns,
      .horizon_steps = mppi_config_.steps,
      .step_s = mppi_config_.dynamics.dt_s,
  });

  const NonCooperativeAvoidanceLifecycleState state = result.avoidance.lifecycle_state;
  const auto log_lifecycle = [&](const bool throttle) {
    const std::optional<NonCooperativeClosestApproach>& threat =
        result.avoidance.primary_threat;
    const char* reason =
        threat ? nonCooperativeThreatReasonName(threat->reason) : "none";
    const std::uint64_t track_id = threat ? threat->local_track_id : 0U;
    if (throttle) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NONCOOPERATIVE_AVOIDANCE state=%s generation=%" PRIu64
          " track_available=%s cost_influence_active=%s "
          "evasive_maneuver_active=%s fresh_tracks=%zu track_id=%" PRIu64
          " reason=%s radar_age_s=%.3f range_m=%.3f closing_speed_mps=%.3f "
          "tcpa_s=%.3f dcpa_m=%.3f",
          nonCooperativeAvoidanceLifecycleStateName(state),
          result.avoidance.lifecycle_generation,
          result.avoidance.influence.track_available ? "true" : "false",
          result.avoidance.influence.cost_influence_active ? "true" : "false",
          result.avoidance.influence.evasive_maneuver_active ? "true" : "false",
          result.avoidance.fresh_track_count, track_id, reason,
          result.avoidance.maximum_radar_age_s,
          diagnosticValue(threat, &NonCooperativeClosestApproach::current_range_m),
          diagnosticValue(threat, &NonCooperativeClosestApproach::closing_speed_mps),
          diagnosticValue(threat,
                          &NonCooperativeClosestApproach::time_to_closest_approach_s),
          diagnosticValue(threat,
                          &NonCooperativeClosestApproach::closest_approach_distance_m));
      return;
    }
    RCLCPP_INFO(
        get_logger(),
        "NONCOOPERATIVE_AVOIDANCE state=%s generation=%" PRIu64
        " track_available=%s cost_influence_active=%s "
        "evasive_maneuver_active=%s fresh_tracks=%zu track_id=%" PRIu64
        " reason=%s radar_age_s=%.3f range_m=%.3f closing_speed_mps=%.3f "
        "tcpa_s=%.3f dcpa_m=%.3f",
        nonCooperativeAvoidanceLifecycleStateName(state),
        result.avoidance.lifecycle_generation,
        result.avoidance.influence.track_available ? "true" : "false",
        result.avoidance.influence.cost_influence_active ? "true" : "false",
        result.avoidance.influence.evasive_maneuver_active ? "true" : "false",
        result.avoidance.fresh_track_count, track_id, reason,
        result.avoidance.maximum_radar_age_s,
        diagnosticValue(threat, &NonCooperativeClosestApproach::current_range_m),
        diagnosticValue(threat, &NonCooperativeClosestApproach::closing_speed_mps),
        diagnosticValue(threat,
                        &NonCooperativeClosestApproach::time_to_closest_approach_s),
        diagnosticValue(threat,
                        &NonCooperativeClosestApproach::closest_approach_distance_m));
  };
  if (state == NonCooperativeAvoidanceLifecycleState::kEntered ||
      state == NonCooperativeAvoidanceLifecycleState::kReleased) {
    log_lifecycle(false);
  } else if (result.avoidance.influence.cost_influence_active) {
    log_lifecycle(true);
  }
  return result;
}

} // namespace drone_city_nav
