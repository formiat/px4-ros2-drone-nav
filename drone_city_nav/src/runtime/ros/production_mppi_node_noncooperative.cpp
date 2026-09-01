#include <cinttypes>
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

void ProductionMppiNode::createNonCooperativeAvoidanceInterface(
    const rclcpp::SubscriptionOptions& subscription_options) {
  if (!config_.planning.noncooperative_avoidance_enabled) {
    return;
  }
  noncooperative_tracks_sub_ = create_subscription<msg::TargetTrackArray>(
      config_.planning.topics.noncooperative_tracks, rclcpp::QoS{2}.reliable(),
      [this](const msg::TargetTrackArray::SharedPtr message) {
        onNonCooperativeTracks(*message);
      },
      subscription_options);
}

void ProductionMppiNode::onNonCooperativeTracks(const msg::TargetTrackArray& message) {
  ProductionMppiNonCooperativeTracks snapshot;
  snapshot.source_scan_sequence = message.source_scan_sequence;
  snapshot.receive_stamp_ns = get_clock()->now().nanoseconds();
  if (!message.header.frame_id.empty() &&
      message.header.frame_id != config_.world.frame_id) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "NONCOOPERATIVE_AVOIDANCE rejected_tracks=true "
                         "reason=frame_mismatch expected='%s' actual='%s'",
                         config_.world.frame_id.c_str(),
                         message.header.frame_id.c_str());
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
  const auto lock = evidence_boundary_.input();
  noncooperative_tracks_ = std::move(snapshot);
}

void ProductionMppiNode::logNonCooperativeUpdate(
    const ProductionMppiNonCooperativeUpdate& update) {
  if (!update.enabled) {
    return;
  }
  const NonCooperativeAvoidanceLifecycleState state = update.avoidance.lifecycle_state;
  const auto log_lifecycle = [&](const bool throttle) {
    const std::optional<NonCooperativeClosestApproach>& threat =
        update.avoidance.primary_threat;
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
          update.avoidance.lifecycle_generation,
          update.avoidance.influence.track_available ? "true" : "false",
          update.avoidance.influence.cost_influence_active ? "true" : "false",
          update.avoidance.influence.evasive_maneuver_active ? "true" : "false",
          update.avoidance.fresh_track_count, track_id, reason,
          update.avoidance.maximum_radar_age_s,
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
        update.avoidance.lifecycle_generation,
        update.avoidance.influence.track_available ? "true" : "false",
        update.avoidance.influence.cost_influence_active ? "true" : "false",
        update.avoidance.influence.evasive_maneuver_active ? "true" : "false",
        update.avoidance.fresh_track_count, track_id, reason,
        update.avoidance.maximum_radar_age_s,
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
  } else if (update.avoidance.influence.cost_influence_active) {
    log_lifecycle(true);
  }
}

} // namespace drone_city_nav
