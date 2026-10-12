#pragma once

#include "drone_city_nav/mission_waypoint_acknowledgement_admission.hpp"
#include "drone_city_nav/msg/mission_waypoint_acknowledgement.hpp"
#include "drone_city_nav/msg/navigation_health.hpp"
#include "drone_city_nav/types.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <bit>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

// The mission monitor's helpers (mission_monitor_node.cpp): the stamps and
// points of its messages, the acknowledgements' fingerprint and the names of
// its verdicts.
namespace drone_city_nav::mission_monitor_node_internal {

[[nodiscard]] inline std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] inline bool finitePoint(const geometry_msgs::msg::Point& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] inline Point3 point3(const geometry_msgs::msg::Point& point) noexcept {
  return Point3{point.x, point.y, point.z};
}

inline constexpr std::uint64_t kAcknowledgementFingerprintOffset{
    14'695'981'039'346'656'037ULL};
inline constexpr std::uint64_t kAcknowledgementFingerprintPrime{1'099'511'628'211ULL};

inline void appendAcknowledgementFingerprint(std::uint64_t& fingerprint,
                                             const std::uint64_t word) noexcept {
  for (std::size_t byte_index = 0U; byte_index < sizeof(word); ++byte_index) {
    fingerprint ^= (word >> (byte_index * 8U)) & 0xFFU;
    fingerprint *= kAcknowledgementFingerprintPrime;
  }
}

[[nodiscard]] inline std::uint64_t canonicalDoubleBits(const double value) noexcept {
  if (value == 0.0) {
    return 0U;
  }
  if (std::isnan(value)) {
    return 0x7FF8'0000'0000'0000ULL;
  }
  return std::bit_cast<std::uint64_t>(value);
}

[[nodiscard]] inline std::uint64_t acknowledgementContentFingerprint(
    const msg::MissionWaypointAcknowledgement& acknowledgement) noexcept {
  std::uint64_t fingerprint{kAcknowledgementFingerprintOffset};
  appendAcknowledgementFingerprint(fingerprint, 1U); // Fingerprint schema.
  appendAcknowledgementFingerprint(
      fingerprint, static_cast<std::uint64_t>(acknowledgement.header.stamp.sec));
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.header.stamp.nanosec);
  for (const char character : acknowledgement.header.frame_id) {
    appendAcknowledgementFingerprint(fingerprint,
                                     static_cast<unsigned char>(character));
  }
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.header.frame_id.size());
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.producer_instance_id);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.acknowledgement_sequence);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.mission_epoch);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.completed_waypoint_index);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.completed_waypoint_count);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.waypoint_count);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.active_waypoint_index);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.mission_completed ? 1U : 0U);
  for (const geometry_msgs::msg::Point* const point :
       {&acknowledgement.completed_goal, &acknowledgement.route_target,
        &acknowledgement.stationary_hold_position}) {
    appendAcknowledgementFingerprint(fingerprint, canonicalDoubleBits(point->x));
    appendAcknowledgementFingerprint(fingerprint, canonicalDoubleBits(point->y));
    appendAcknowledgementFingerprint(fingerprint, canonicalDoubleBits(point->z));
  }
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.horizon_producer_instance_id);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.horizon_sequence);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.offboard_producer_instance_id);
  for (const builtin_interfaces::msg::Time* const stamp :
       {&acknowledgement.horizon_valid_from, &acknowledgement.horizon_valid_until,
        &acknowledgement.witness_stamp}) {
    appendAcknowledgementFingerprint(fingerprint,
                                     static_cast<std::uint64_t>(stamp->sec));
    appendAcknowledgementFingerprint(fingerprint, stamp->nanosec);
  }
  return fingerprint == 0U ? 1U : fingerprint;
}

[[nodiscard]] inline const char* acknowledgementRejectionReason(
    const MissionWaypointAcknowledgementAdmissionResult& admission) noexcept {
  if (admission.conflict) {
    return "identity_conflict";
  }
  if (admission.replay) {
    return "replay";
  }
  if (admission.aggregate_replay) {
    return "aggregate_replay";
  }
  if (admission.stale) {
    return "stale";
  }
  if (admission.retired_capacity_exhausted) {
    return "retired_capacity_exhausted";
  }
  if (admission.prospective_capacity_exhausted) {
    return "prospective_capacity_exhausted";
  }
  return "invalid";
}

[[nodiscard]] inline const char*
navigationFailureReason(const std::uint8_t reason) noexcept {
  switch (reason) {
    case msg::NavigationHealth::FAILURE_NONE:
      return "none";
    case msg::NavigationHealth::FAILURE_UNAVAILABLE_WORLD:
      return "navigation_unavailable_world";
    case msg::NavigationHealth::FAILURE_NO_EXECUTABLE_ROUTE:
      return "navigation_no_executable_route";
    case msg::NavigationHealth::FAILURE_NO_ACKNOWLEDGED_HORIZON:
      return "navigation_no_acknowledged_horizon";
    case msg::NavigationHealth::FAILURE_RECOVERY_BUDGET_EXHAUSTED:
      return "navigation_recovery_budget_exhausted";
    default:
      return "navigation_invalid_failure_reason";
  }
}

} // namespace drone_city_nav::mission_monitor_node_internal
