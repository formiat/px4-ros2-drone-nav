#pragma once

#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

inline constexpr std::size_t kRetiredMissionWaypointProducerCapacity{8U};
inline constexpr std::size_t kProspectiveMissionWaypointProducerCapacity{8U};

struct MissionWaypointAcknowledgementCandidate {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t acknowledgement_sequence{0U};
  std::uint64_t content_fingerprint{0U};
  std::uint64_t mission_epoch{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  std::uint64_t offboard_producer_instance_id{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::int64_t horizon_valid_from_ns{0};
  std::int64_t horizon_valid_until_ns{0};
  std::int64_t witness_stamp_ns{0};
  std::uint32_t completed_waypoint_index{0U};
  std::uint32_t completed_waypoint_count{0U};
  std::uint32_t waypoint_count{0U};
  std::uint32_t active_waypoint_index{0U};
  Point3 completed_goal{};
  Point3 route_target{};
  Point3 stationary_hold_position{};
  bool mission_completed{false};
  bool payload_valid{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool identityClaimable() const noexcept;
  [[nodiscard]] bool identityValid() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct MissionWaypointProspectiveIdentityClaim {
  MissionWaypointAcknowledgementCandidate candidate{};
  std::int64_t source_stamp_high_water_ns{0};
  std::int64_t receive_stamp_high_water_ns{0};
  bool tombstoned{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct MissionWaypointAcknowledgementAdmissionState {
  // `current` is the active producer's immutable wire-identity high-water. It
  // may describe a rejected payload. Aggregate progress lives separately so a
  // higher sequence can recover without accepting a mutated rejected identity.
  MissionWaypointAcknowledgementCandidate current{};
  MissionWaypointAcknowledgementCandidate last_accepted{};
  std::int64_t current_source_stamp_high_water_ns{0};
  std::int64_t current_receive_stamp_high_water_ns{0};
  bool current_identity_tombstoned{false};
  std::array<std::uint64_t, kRetiredMissionWaypointProducerCapacity>
      retired_producer_instance_ids{};
  std::size_t retired_producer_count{0U};
  std::array<MissionWaypointProspectiveIdentityClaim,
             kProspectiveMissionWaypointProducerCapacity>
      prospective_identity_claims{};
  std::size_t prospective_identity_count{0U};
  bool prospective_identity_capacity_exhausted{false};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool producerRetired(std::uint64_t producer_instance_id) const noexcept;
};

struct MissionWaypointAcknowledgementAdmissionResult {
  MissionWaypointAcknowledgementAdmissionState next_state{};
  bool accept{false};
  bool producer_transitioned{false};
  bool replay{false};
  bool aggregate_replay{false};
  bool stale{false};
  bool conflict{false};
  bool invalid{false};
  bool state_advanced{false};
  bool retired_capacity_exhausted{false};
  bool prospective_capacity_exhausted{false};
};

// Admission is exactly-once and aggregate-aware. Sequence gaps are accepted so
// a reliable transient-local latest sample can recover a late subscriber, but
// completed progress may never regress across a producer handoff. A replacement
// producer is admissible only when it reports externally preserved monotonic
// aggregate progress; resetting its local mission state to zero is rejected
// fail closed. A handoff at the same aggregate count changes lineage without
// re-emitting completion. A newer identity advances the high-water even when
// payload_valid is false, so a delayed payload cannot resurrect rejected
// geometry. Ineligible producer handoffs are claimed in a bounded prospective
// ledger without evicting the active producer. The exact tuple cannot later be
// mutated into eligibility; only a strictly higher producer-local sequence may
// recover.
[[nodiscard]] MissionWaypointAcknowledgementAdmissionResult
admitMissionWaypointAcknowledgement(
    const MissionWaypointAcknowledgementAdmissionState& state,
    const MissionWaypointAcknowledgementCandidate& candidate) noexcept;

} // namespace drone_city_nav
