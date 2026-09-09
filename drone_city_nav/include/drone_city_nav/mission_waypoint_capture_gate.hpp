#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>

namespace drone_city_nav {

struct MissionWaypointCaptureGateConfig {
  double goal_radius_m{2.0};
  double target_match_tolerance_m{1.0e-3};
  double stop_speed_mps{0.8};
  double stop_hold_s{2.0};
  double maximum_pose_age_s{0.15};
  double maximum_vehicle_status_age_s{1.0};
  double maximum_feedback_age_s{0.2};
};

struct MissionWaypointCaptureObservation {
  std::int64_t stamp_ns{0};
  Point3 goal{};
  Point3 position{};
  Point3 velocity{};
  Point3 route_target{};
  Point3 stationary_hold_position{};
  std::int64_t pose_receive_stamp_ns{0};
  std::int64_t vehicle_status_receive_stamp_ns{0};
  std::int64_t horizon_valid_from_ns{0};
  std::int64_t horizon_valid_until_ns{0};
  std::int64_t feedback_source_stamp_ns{0};
  std::int64_t feedback_receive_stamp_ns{0};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  // Identity of the certified stationary hold the horizon leases; renewals of
  // the same hold change the horizon sequence but not this. Zero when the
  // horizon is not a stationary hold.
  std::uint64_t hold_id{0U};
  // Sequence of the previous lease of the same hold, or zero. Feedback for
  // it is still evidence of this hold while the offboard catches up with the
  // renewal.
  std::uint64_t previous_horizon_sequence_same_hold{0U};
  std::uint64_t target_offboard_instance_id{0U};
  std::uint64_t feedback_horizon_producer_instance_id{0U};
  std::uint64_t feedback_horizon_sequence{0U};
  std::uint64_t feedback_offboard_instance_id{0U};
  std::uint64_t feedback_continuity_generation{0U};
  bool goal_capture_latched{false};
  bool position_velocity_authoritative{false};
  bool vehicle_status_valid{false};
  bool vehicle_status_epoch_stable{false};
  bool armed{false};
  bool horizon_valid{false};
  bool horizon_position_hold{false};
  bool horizon_goal_capture{false};
  bool horizon_stationary_position_hold{false};
  bool feedback_valid{false};
  bool feedback_position_hold{false};
  bool feedback_control_authoritative{false};
  bool feedback_continuity_generation_valid{false};
};

struct MissionWaypointCaptureGateResult {
  bool evidence_valid{false};
  // Names the first failing evidence predicate, or "none" when valid.
  const char* ineligibility{"none"};
  bool continuity_started{false};
  bool continuity_broken{false};
  // Names why the continuity was reset in this update, or "none".
  const char* continuity_break_reason{"none"};
  bool ready{false};
  std::int64_t continuous_since_ns{0};
  std::int64_t continuous_duration_ns{0};
};

struct MissionWaypointStationaryRearmGateConfig {
  double maximum_pose_age_s{0.15};
  double maximum_vehicle_status_age_s{1.0};
  double maximum_offboard_session_age_s{0.2};
  double position_tolerance_m{0.25};
  double speed_tolerance_mps{0.25};
  double yaw_rate_tolerance_radps{0.25};
};

struct MissionWaypointStationaryRearmObservation {
  std::int64_t stamp_ns{0};
  Point3 mission_goal{};
  Point3 active_waypoint_goal{};
  Point3 position{};
  Point3 velocity{};
  std::int64_t pose_receive_stamp_ns{0};
  std::int64_t vehicle_status_receive_stamp_ns{0};
  std::int64_t offboard_session_source_stamp_ns{0};
  std::int64_t offboard_session_receive_stamp_ns{0};
  std::uint64_t offboard_instance_id{0U};
  double yaw_rate_radps{0.0};
  bool objective_eligible{false};
  bool goal_capture_latched{false};
  bool execution_input_state_authoritative{false};
  bool position_velocity_authoritative{false};
  bool yaw_rate_authoritative{false};
  bool vehicle_status_valid{false};
  bool vehicle_status_epoch_stable{false};
  bool armed{false};
  bool offboard_session_valid{false};
  bool applied_control_empty{false};
  bool horizon_owner_empty{false};
  bool execution_snapshot_revoked_empty{false};
  bool validation_policy_current{false};
  bool world_evidence_current{false};
  bool lidar_evidence_current{false};
};

// This is the only planner-side gate that may label zero previous control as a
// stationary capture rearm input. Snapshot certification and publication
// independently revalidate the same state and evidence before granting a hold.
[[nodiscard]] bool missionWaypointStationaryRearmEligible(
    const MissionWaypointStationaryRearmGateConfig& config,
    const MissionWaypointStationaryRearmObservation& observation) noexcept;

// Names the first failing rearm predicate, or nullptr when eligible.
[[nodiscard]] const char* missionWaypointStationaryRearmIneligibility(
    const MissionWaypointStationaryRearmGateConfig& config,
    const MissionWaypointStationaryRearmObservation& observation) noexcept;

// The same rearm for a vehicle at rest anywhere, not only at a captured goal:
// the state a fail-closed revocation leaves it in. Nothing owns its motion,
// the offboard holds it locally with no knowledge of obstacles, and the
// certified stationary hold this rearm grants is what takes it back -- a
// pinned position the evidence was checked against, and an owner a successor
// route can activate from without waiting for a witness the local hold never
// produces. Every check of the goal rearm applies except the ones that name
// the goal: the capture latch, the goal identity, the goal radius.
[[nodiscard]] bool stationaryRestRearmEligible(
    const MissionWaypointStationaryRearmGateConfig& config,
    const MissionWaypointStationaryRearmObservation& observation) noexcept;
[[nodiscard]] const char* stationaryRestRearmIneligibility(
    const MissionWaypointStationaryRearmGateConfig& config,
    const MissionWaypointStationaryRearmObservation& observation) noexcept;

// The gate is tick-aware: beginTick() must be paired with one update(). A
// planning tick that exits before supplying exact evidence is detected by the
// next beginTick() and breaks continuity.
class MissionWaypointCaptureGate final {
public:
  explicit MissionWaypointCaptureGate(
      const MissionWaypointCaptureGateConfig& config = {});

  void beginTick(std::int64_t stamp_ns) noexcept;
  [[nodiscard]] MissionWaypointCaptureGateResult
  update(const MissionWaypointCaptureObservation& observation) noexcept;
  void reset() noexcept;

private:
  [[nodiscard]] bool
  evidenceValid(const MissionWaypointCaptureObservation& observation) const noexcept;
  // Names the first failing evidence predicate, or nullptr when valid.
  [[nodiscard]] const char* evidenceIneligibility(
      const MissionWaypointCaptureObservation& observation) const noexcept;
  void resetContinuity() noexcept;

  MissionWaypointCaptureGateConfig config_{};
  std::int64_t required_stop_hold_ns_{0};
  std::int64_t maximum_pose_age_ns_{0};
  std::int64_t maximum_vehicle_status_age_ns_{0};
  std::int64_t maximum_feedback_age_ns_{0};
  std::int64_t open_tick_stamp_ns_{0};
  std::int64_t continuous_since_ns_{0};
  std::uint64_t continuous_horizon_producer_instance_id_{0U};
  std::uint64_t continuous_horizon_sequence_{0U};
  std::uint64_t continuous_hold_id_{0U};
  std::uint64_t continuous_offboard_instance_id_{0U};
  std::uint64_t continuous_feedback_generation_{0U};
  bool tick_open_{false};
  bool continuity_broken_on_begin_{false};
  const char* begin_break_reason_{"none"};
};

} // namespace drone_city_nav
