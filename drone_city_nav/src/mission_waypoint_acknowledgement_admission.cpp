#include "drone_city_nav/mission_waypoint_acknowledgement_admission.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept {
  if (value == 0.0) {
    return 0U;
  }
  if (std::isnan(value)) {
    return 0x7FF8'0000'0000'0000ULL;
  }
  return std::bit_cast<std::uint64_t>(value);
}

[[nodiscard]] bool samePoint(const Point3& first, const Point3& second) noexcept {
  return canonicalDoubleBits(first.x) == canonicalDoubleBits(second.x) &&
         canonicalDoubleBits(first.y) == canonicalDoubleBits(second.y) &&
         canonicalDoubleBits(first.z) == canonicalDoubleBits(second.z);
}

[[nodiscard]] bool
sameAcknowledgement(const MissionWaypointAcknowledgementCandidate& first,
                    const MissionWaypointAcknowledgementCandidate& second) noexcept {
  // receive_stamp_ns is consumer-local metadata. Reliable/transient-local
  // redelivery of identical wire content must remain an idempotent replay and
  // must not refresh the stored receipt high-water.
  return first.producer_instance_id == second.producer_instance_id &&
         first.acknowledgement_sequence == second.acknowledgement_sequence &&
         first.content_fingerprint == second.content_fingerprint &&
         first.mission_epoch == second.mission_epoch &&
         first.horizon_producer_instance_id == second.horizon_producer_instance_id &&
         first.horizon_sequence == second.horizon_sequence &&
         first.offboard_producer_instance_id == second.offboard_producer_instance_id &&
         first.source_stamp_ns == second.source_stamp_ns &&
         first.horizon_valid_from_ns == second.horizon_valid_from_ns &&
         first.horizon_valid_until_ns == second.horizon_valid_until_ns &&
         first.witness_stamp_ns == second.witness_stamp_ns &&
         first.completed_waypoint_index == second.completed_waypoint_index &&
         first.completed_waypoint_count == second.completed_waypoint_count &&
         first.waypoint_count == second.waypoint_count &&
         first.active_waypoint_index == second.active_waypoint_index &&
         samePoint(first.completed_goal, second.completed_goal) &&
         samePoint(first.route_target, second.route_target) &&
         samePoint(first.stationary_hold_position, second.stationary_hold_position) &&
         first.mission_completed == second.mission_completed &&
         first.payload_valid == second.payload_valid;
}

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] std::optional<std::size_t>
prospectiveIdentityIndex(const MissionWaypointAcknowledgementAdmissionState& state,
                         const std::uint64_t producer_instance_id) noexcept {
  for (std::size_t index = 0U; index < state.prospective_identity_count; ++index) {
    if (state.prospective_identity_claims[index].candidate.producer_instance_id ==
        producer_instance_id) {
      return index;
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::int64_t timestampHighWater(const std::int64_t value) noexcept {
  return std::max<std::int64_t>(0, value);
}

[[nodiscard]] MissionWaypointProspectiveIdentityClaim makeProspectiveClaim(
    const MissionWaypointAcknowledgementCandidate& candidate) noexcept {
  return MissionWaypointProspectiveIdentityClaim{
      .candidate = candidate,
      .source_stamp_high_water_ns = timestampHighWater(candidate.source_stamp_ns),
      .receive_stamp_high_water_ns = timestampHighWater(candidate.receive_stamp_ns),
  };
}

void installCurrentIdentity(
    MissionWaypointAcknowledgementAdmissionState& state,
    const MissionWaypointAcknowledgementCandidate& candidate) noexcept {
  state.current = candidate;
  state.current_source_stamp_high_water_ns =
      timestampHighWater(candidate.source_stamp_ns);
  state.current_receive_stamp_high_water_ns =
      timestampHighWater(candidate.receive_stamp_ns);
  state.current_identity_tombstoned = false;
}

void eraseProspectiveIdentity(MissionWaypointAcknowledgementAdmissionState& state,
                              const std::size_t erased_index) noexcept {
  for (std::size_t index = erased_index + 1U; index < state.prospective_identity_count;
       ++index) {
    state.prospective_identity_claims[index - 1U] =
        state.prospective_identity_claims[index];
  }
  if (state.prospective_identity_count > 0U) {
    --state.prospective_identity_count;
    state.prospective_identity_claims[state.prospective_identity_count] = {};
  }
}

[[nodiscard]] bool sameProducerProgressEligible(
    const MissionWaypointAcknowledgementCandidate& last_accepted,
    const MissionWaypointAcknowledgementCandidate& candidate) noexcept {
  if (last_accepted.empty()) {
    return true;
  }
  return !last_accepted.mission_completed &&
         candidate.waypoint_count == last_accepted.waypoint_count &&
         candidate.completed_waypoint_count > last_accepted.completed_waypoint_count &&
         candidate.mission_epoch > last_accepted.mission_epoch;
}

[[nodiscard]] bool handoffProgressEligible(
    const MissionWaypointAcknowledgementCandidate& last_accepted,
    const MissionWaypointAcknowledgementCandidate& candidate) noexcept {
  if (last_accepted.empty()) {
    return true;
  }
  const bool same_aggregate =
      candidate.completed_waypoint_count == last_accepted.completed_waypoint_count &&
      candidate.mission_epoch == last_accepted.mission_epoch;
  const bool advanced_aggregate =
      !last_accepted.mission_completed &&
      candidate.completed_waypoint_count > last_accepted.completed_waypoint_count &&
      candidate.mission_epoch > last_accepted.mission_epoch;
  return candidate.waypoint_count == last_accepted.waypoint_count &&
         (same_aggregate || advanced_aggregate);
}

void acceptAggregate(MissionWaypointAcknowledgementAdmissionResult& result,
                     const MissionWaypointAcknowledgementCandidate& candidate,
                     const bool aggregate_advanced) noexcept {
  if (!candidate.valid()) {
    result.invalid = true;
    return;
  }
  if (!aggregate_advanced) {
    result.aggregate_replay = true;
    return;
  }
  result.accept = true;
  result.next_state.last_accepted = candidate;
}

} // namespace

bool MissionWaypointAcknowledgementCandidate::empty() const noexcept {
  return producer_instance_id == 0U && acknowledgement_sequence == 0U &&
         content_fingerprint == 0U && mission_epoch == 0U &&
         horizon_producer_instance_id == 0U && horizon_sequence == 0U &&
         offboard_producer_instance_id == 0U && source_stamp_ns == 0 &&
         receive_stamp_ns == 0 && horizon_valid_from_ns == 0 &&
         horizon_valid_until_ns == 0 && witness_stamp_ns == 0 &&
         completed_waypoint_index == 0U && completed_waypoint_count == 0U &&
         waypoint_count == 0U && active_waypoint_index == 0U &&
         completed_goal.x == 0.0 && completed_goal.y == 0.0 &&
         completed_goal.z == 0.0 && route_target.x == 0.0 && route_target.y == 0.0 &&
         route_target.z == 0.0 && stationary_hold_position.x == 0.0 &&
         stationary_hold_position.y == 0.0 && stationary_hold_position.z == 0.0 &&
         !mission_completed && !payload_valid;
}

bool MissionWaypointAcknowledgementCandidate::identityClaimable() const noexcept {
  return producer_instance_id != 0U && acknowledgement_sequence != 0U &&
         content_fingerprint != 0U;
}

bool MissionWaypointAcknowledgementCandidate::identityValid() const noexcept {
  const bool progress_valid =
      waypoint_count > 0U && completed_waypoint_count > 0U &&
      completed_waypoint_count <= waypoint_count &&
      completed_waypoint_index + 1U == completed_waypoint_count &&
      ((mission_completed && completed_waypoint_count == waypoint_count &&
        active_waypoint_index + 1U == waypoint_count) ||
       (!mission_completed && completed_waypoint_count < waypoint_count &&
        active_waypoint_index == completed_waypoint_count));
  return identityClaimable() && horizon_producer_instance_id == producer_instance_id &&
         horizon_sequence != 0U && offboard_producer_instance_id != 0U &&
         source_stamp_ns > 0 && receive_stamp_ns >= source_stamp_ns &&
         horizon_valid_from_ns > 0 && horizon_valid_until_ns > horizon_valid_from_ns &&
         source_stamp_ns >= horizon_valid_from_ns &&
         source_stamp_ns < horizon_valid_until_ns &&
         witness_stamp_ns >= horizon_valid_from_ns &&
         witness_stamp_ns < horizon_valid_until_ns &&
         source_stamp_ns >= witness_stamp_ns && progress_valid;
}

bool MissionWaypointAcknowledgementCandidate::valid() const noexcept {
  return identityValid() && payload_valid && finitePoint(completed_goal) &&
         finitePoint(route_target) && finitePoint(stationary_hold_position);
}

bool MissionWaypointProspectiveIdentityClaim::empty() const noexcept {
  return candidate.empty() && source_stamp_high_water_ns == 0 &&
         receive_stamp_high_water_ns == 0 && !tombstoned;
}

bool MissionWaypointProspectiveIdentityClaim::valid() const noexcept {
  return candidate.identityClaimable() && source_stamp_high_water_ns >= 0 &&
         receive_stamp_high_water_ns >= 0 &&
         source_stamp_high_water_ns >= timestampHighWater(candidate.source_stamp_ns) &&
         receive_stamp_high_water_ns >= timestampHighWater(candidate.receive_stamp_ns);
}

bool MissionWaypointAcknowledgementAdmissionState::valid() const noexcept {
  if (retired_producer_count > retired_producer_instance_ids.size() ||
      prospective_identity_count > prospective_identity_claims.size()) {
    return false;
  }
  const bool has_current = current.identityClaimable();
  if (!has_current &&
      (!current.empty() || !last_accepted.empty() ||
       current_source_stamp_high_water_ns != 0 ||
       current_receive_stamp_high_water_ns != 0 || current_identity_tombstoned ||
       retired_producer_count != 0U || prospective_identity_count != 0U ||
       prospective_identity_capacity_exhausted)) {
    return false;
  }
  if (has_current && !last_accepted.empty() && !last_accepted.valid()) {
    return false;
  }
  if (has_current && (current_source_stamp_high_water_ns <
                          timestampHighWater(current.source_stamp_ns) ||
                      current_receive_stamp_high_water_ns <
                          timestampHighWater(current.receive_stamp_ns))) {
    return false;
  }
  for (std::size_t index = 0U; index < retired_producer_count; ++index) {
    const std::uint64_t retired = retired_producer_instance_ids[index];
    if (retired == 0U || retired == current.producer_instance_id) {
      return false;
    }
    for (std::size_t prior = 0U; prior < index; ++prior) {
      if (retired_producer_instance_ids[prior] == retired) {
        return false;
      }
    }
  }
  for (std::size_t index = retired_producer_count;
       index < retired_producer_instance_ids.size(); ++index) {
    if (retired_producer_instance_ids[index] != 0U) {
      return false;
    }
  }
  if (!last_accepted.empty() &&
      last_accepted.producer_instance_id != current.producer_instance_id &&
      !producerRetired(last_accepted.producer_instance_id)) {
    return false;
  }
  for (std::size_t index = 0U; index < prospective_identity_count; ++index) {
    const MissionWaypointProspectiveIdentityClaim& claim =
        prospective_identity_claims[index];
    if (!claim.valid() ||
        claim.candidate.producer_instance_id == current.producer_instance_id ||
        producerRetired(claim.candidate.producer_instance_id)) {
      return false;
    }
    for (std::size_t prior = 0U; prior < index; ++prior) {
      if (prospective_identity_claims[prior].candidate.producer_instance_id ==
          claim.candidate.producer_instance_id) {
        return false;
      }
    }
  }
  for (std::size_t index = prospective_identity_count;
       index < prospective_identity_claims.size(); ++index) {
    if (!prospective_identity_claims[index].empty()) {
      return false;
    }
  }
  return has_current || current.empty();
}

bool MissionWaypointAcknowledgementAdmissionState::producerRetired(
    const std::uint64_t producer_instance_id) const noexcept {
  if (retired_producer_count > retired_producer_instance_ids.size()) {
    return false;
  }
  const auto retired_end = retired_producer_instance_ids.begin() +
                           static_cast<std::ptrdiff_t>(retired_producer_count);
  return producer_instance_id != 0U &&
         std::find(retired_producer_instance_ids.begin(), retired_end,
                   producer_instance_id) != retired_end;
}

MissionWaypointAcknowledgementAdmissionResult admitMissionWaypointAcknowledgement(
    const MissionWaypointAcknowledgementAdmissionState& state,
    const MissionWaypointAcknowledgementCandidate& candidate) noexcept {
  MissionWaypointAcknowledgementAdmissionResult result{.next_state = state};
  if (!state.valid() || !candidate.identityClaimable()) {
    result.invalid = true;
    return result;
  }

  if (!state.current.identityClaimable()) {
    installCurrentIdentity(result.next_state, candidate);
    result.state_advanced = true;
    result.producer_transitioned = true;
    if (!candidate.identityValid()) {
      result.invalid = true;
      return result;
    }
    acceptAggregate(result, candidate, true);
    return result;
  }

  if (state.producerRetired(candidate.producer_instance_id)) {
    result.stale = true;
    return result;
  }

  if (candidate.producer_instance_id == state.current.producer_instance_id) {
    if (candidate.acknowledgement_sequence < state.current.acknowledgement_sequence) {
      result.stale = true;
      return result;
    }
    if (candidate.acknowledgement_sequence == state.current.acknowledgement_sequence) {
      if (state.current_identity_tombstoned ||
          !sameAcknowledgement(state.current, candidate)) {
        result.conflict = true;
        if (!state.current_identity_tombstoned) {
          result.next_state.current_identity_tombstoned = true;
          result.state_advanced = true;
        }
      } else {
        result.replay = true;
      }
      return result;
    }

    const std::int64_t previous_source_high_water =
        state.current_source_stamp_high_water_ns;
    const std::int64_t previous_receive_high_water =
        state.current_receive_stamp_high_water_ns;
    result.next_state.current = candidate;
    result.next_state.current_source_stamp_high_water_ns = std::max(
        previous_source_high_water, timestampHighWater(candidate.source_stamp_ns));
    result.next_state.current_receive_stamp_high_water_ns = std::max(
        previous_receive_high_water, timestampHighWater(candidate.receive_stamp_ns));
    result.next_state.current_identity_tombstoned = false;
    result.state_advanced = true;
    if (!candidate.identityValid()) {
      result.invalid = true;
      return result;
    }
    const bool temporal_progress =
        candidate.source_stamp_ns > previous_source_high_water &&
        candidate.receive_stamp_ns >= previous_receive_high_water;
    if (!temporal_progress ||
        !sameProducerProgressEligible(state.last_accepted, candidate)) {
      result.stale = true;
      return result;
    }
    acceptAggregate(result, candidate, true);
    return result;
  }

  std::optional<std::size_t> prospective_index =
      prospectiveIdentityIndex(state, candidate.producer_instance_id);
  bool prospective_temporal_progress{true};
  if (prospective_index.has_value()) {
    MissionWaypointProspectiveIdentityClaim& claim =
        result.next_state.prospective_identity_claims[*prospective_index];
    if (candidate.acknowledgement_sequence < claim.candidate.acknowledgement_sequence) {
      result.stale = true;
      return result;
    }
    if (candidate.acknowledgement_sequence ==
        claim.candidate.acknowledgement_sequence) {
      if (claim.tombstoned || !sameAcknowledgement(claim.candidate, candidate)) {
        result.conflict = true;
        if (!claim.tombstoned) {
          claim.tombstoned = true;
          result.state_advanced = true;
        }
      } else {
        result.replay = true;
      }
      return result;
    }
    prospective_temporal_progress =
        candidate.source_stamp_ns > claim.source_stamp_high_water_ns &&
        candidate.receive_stamp_ns >= claim.receive_stamp_high_water_ns;
    const std::int64_t prior_source_high_water = claim.source_stamp_high_water_ns;
    const std::int64_t prior_receive_high_water = claim.receive_stamp_high_water_ns;
    claim = makeProspectiveClaim(candidate);
    claim.source_stamp_high_water_ns =
        std::max(prior_source_high_water, claim.source_stamp_high_water_ns);
    claim.receive_stamp_high_water_ns =
        std::max(prior_receive_high_water, claim.receive_stamp_high_water_ns);
    result.state_advanced = true;
  } else {
    if (state.prospective_identity_capacity_exhausted ||
        state.prospective_identity_count == state.prospective_identity_claims.size()) {
      result.prospective_capacity_exhausted = true;
      result.invalid = true;
      if (!state.prospective_identity_capacity_exhausted) {
        result.next_state.prospective_identity_capacity_exhausted = true;
        result.state_advanced = true;
      }
      return result;
    }
    prospective_index = state.prospective_identity_count;
    result.next_state.prospective_identity_claims[*prospective_index] =
        makeProspectiveClaim(candidate);
    ++result.next_state.prospective_identity_count;
    result.state_advanced = true;
  }

  if (!candidate.identityValid()) {
    result.invalid = true;
    return result;
  }
  // A prospective producer claims its immutable wire identity before payload
  // validation, but malformed mission geometry is not authority to retire the
  // active producer. Keep the rejected claim resident so the exact tuple
  // remains non-installable and require a higher sequence to recover.
  if (!candidate.valid()) {
    result.invalid = true;
    return result;
  }
  const bool temporal_handoff =
      candidate.source_stamp_ns > state.current_source_stamp_high_water_ns &&
      candidate.receive_stamp_ns > state.current_receive_stamp_high_water_ns;
  if (!prospective_temporal_progress || !temporal_handoff ||
      !handoffProgressEligible(state.last_accepted, candidate)) {
    result.stale = true;
    return result;
  }
  if (state.retired_producer_count == state.retired_producer_instance_ids.size()) {
    result.retired_capacity_exhausted = true;
    return result;
  }

  result.next_state
      .retired_producer_instance_ids[result.next_state.retired_producer_count] =
      state.current.producer_instance_id;
  ++result.next_state.retired_producer_count;
  installCurrentIdentity(result.next_state, candidate);
  eraseProspectiveIdentity(result.next_state, *prospective_index);
  result.producer_transitioned = true;
  const bool aggregate_advanced =
      state.last_accepted.empty() ||
      candidate.completed_waypoint_count > state.last_accepted.completed_waypoint_count;
  acceptAggregate(result, candidate, aggregate_advanced);
  return result;
}

} // namespace drone_city_nav
