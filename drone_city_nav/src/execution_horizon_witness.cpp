#include "drone_city_nav/execution_horizon_witness.hpp"

#include <algorithm>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validMode(const ExecutionHorizonWitnessMode mode) noexcept {
  switch (mode) {
    case ExecutionHorizonWitnessMode::kPlanned:
    case ExecutionHorizonWitnessMode::kPositionHold:
      return true;
  }
  return false;
}

[[nodiscard]] bool authorityMatchesMode(const ExecutionHorizonWitnessMode mode,
                                        const bool control_authoritative) noexcept {
  switch (mode) {
    case ExecutionHorizonWitnessMode::kPlanned:
      return control_authoritative;
    case ExecutionHorizonWitnessMode::kPositionHold:
      return !control_authoritative;
  }
  return false;
}

[[nodiscard]] bool
feedbackAuthorityCoherent(const ExecutionHorizonWitnessMode mode,
                          const bool control_authoritative) noexcept {
  return !control_authoritative || mode == ExecutionHorizonWitnessMode::kPlanned;
}

[[nodiscard]] bool timestampPairValid(const std::int64_t source_stamp_ns,
                                      const std::int64_t receive_stamp_ns) noexcept {
  return source_stamp_ns > 0 && receive_stamp_ns >= source_stamp_ns;
}

[[nodiscard]] bool timestampPairFreshAt(const std::int64_t source_stamp_ns,
                                        const std::int64_t receive_stamp_ns,
                                        const std::int64_t now_ns,
                                        const std::int64_t maximum_age_ns) noexcept {
  return timestampPairValid(source_stamp_ns, receive_stamp_ns) && now_ns > 0 &&
         maximum_age_ns > 0 && source_stamp_ns <= now_ns &&
         receive_stamp_ns <= now_ns && now_ns - source_stamp_ns <= maximum_age_ns &&
         now_ns - receive_stamp_ns <= maximum_age_ns;
}

[[nodiscard]] bool
sameSourceIdentity(const ExecutionHorizonFeedbackCandidate& first,
                   const ExecutionHorizonFeedbackCandidate& second) noexcept {
  // receive_stamp_ns is consumer-local metadata. It is intentionally excluded
  // so reliable redelivery cannot rejuvenate or mutate a claimed wire source.
  return first.offboard_producer_instance_id == second.offboard_producer_instance_id &&
         first.horizon_producer_instance_id == second.horizon_producer_instance_id &&
         first.horizon_sequence == second.horizon_sequence &&
         first.source_stamp_ns == second.source_stamp_ns &&
         first.content_fingerprint == second.content_fingerprint &&
         first.wire_fingerprint == second.wire_fingerprint &&
         first.execution_mode == second.execution_mode &&
         first.control_authoritative == second.control_authoritative;
}

[[nodiscard]] bool
witnessMatchesCandidate(const ExecutionHorizonFeedbackWitness& witness,
                        const ExecutionHorizonFeedbackCandidate& candidate) noexcept {
  return witness.offboard_producer_instance_id ==
             candidate.offboard_producer_instance_id &&
         witness.horizon_producer_instance_id ==
             candidate.horizon_producer_instance_id &&
         witness.horizon_sequence == candidate.horizon_sequence &&
         witness.source_stamp_ns == candidate.source_stamp_ns &&
         witness.receive_stamp_ns == candidate.receive_stamp_ns &&
         witness.content_fingerprint == candidate.content_fingerprint &&
         witness.wire_fingerprint == candidate.wire_fingerprint &&
         witness.execution_mode == candidate.execution_mode &&
         witness.control_authoritative == candidate.control_authoritative;
}

[[nodiscard]] ExecutionHorizonFeedbackWitness
makeWitness(const ExecutionHorizonFeedbackCandidate& candidate) noexcept {
  return {
      .offboard_producer_instance_id = candidate.offboard_producer_instance_id,
      .horizon_producer_instance_id = candidate.horizon_producer_instance_id,
      .horizon_sequence = candidate.horizon_sequence,
      .source_stamp_ns = candidate.source_stamp_ns,
      .receive_stamp_ns = candidate.receive_stamp_ns,
      .content_fingerprint = candidate.content_fingerprint,
      .wire_fingerprint = candidate.wire_fingerprint,
      .execution_mode = candidate.execution_mode,
      .control_authoritative = candidate.control_authoritative,
  };
}

void claimCurrentSource(ExecutionHorizonWitnessAdmissionResult& result,
                        const ExecutionHorizonWitnessState& state,
                        const ExecutionHorizonFeedbackCandidate& candidate) noexcept {
  const bool had_witness = !state.latest_horizon_feedback.empty();
  result.next_state.feedback_source_identity = {
      .candidate = candidate,
      .tombstoned = true,
  };
  result.next_state.latest_horizon_feedback = {};
  result.next_state.horizon_feedback_source_stamp_high_water_ns =
      candidate.source_stamp_ns;
  result.next_state.horizon_feedback_receive_stamp_high_water_ns = std::max(
      state.horizon_feedback_receive_stamp_high_water_ns, candidate.receive_stamp_ns);
  result.witness_revoked = had_witness;
  result.state_advanced = true;
}

void acceptCurrentSource(ExecutionHorizonWitnessAdmissionResult& result,
                         const ExecutionHorizonFeedbackCandidate& candidate) noexcept {
  result.next_state.feedback_source_identity.tombstoned = false;
  result.next_state.offboard_session.latest_source_stamp_ns = candidate.source_stamp_ns;
  result.next_state.latest_session_receive_stamp_ns = candidate.receive_stamp_ns;
  result.next_state.horizon_feedback_receive_stamp_high_water_ns =
      candidate.receive_stamp_ns;
  result.accept = true;
}

[[nodiscard]] ExecutionHorizonFeedbackProspectiveSourceIdentity
makeProspectiveSource(const ExecutionHorizonFeedbackCandidate& candidate) noexcept {
  return {
      .candidate = candidate,
      .receive_stamp_high_water_ns = candidate.receive_stamp_ns,
      .tombstoned = true,
  };
}

[[nodiscard]] std::optional<std::size_t>
prospectiveSourceIndex(const ExecutionHorizonWitnessState& state,
                       const std::uint64_t offboard_producer_instance_id) noexcept {
  for (std::size_t index = 0U; index < state.prospective_feedback_source_count;
       ++index) {
    if (state.prospective_feedback_source_identities[index]
            .candidate.offboard_producer_instance_id == offboard_producer_instance_id) {
      return index;
    }
  }
  return std::nullopt;
}

void eraseProspectiveSource(ExecutionHorizonWitnessState& state,
                            const std::size_t erased_index) noexcept {
  for (std::size_t index = erased_index + 1U;
       index < state.prospective_feedback_source_count; ++index) {
    state.prospective_feedback_source_identities[index - 1U] =
        state.prospective_feedback_source_identities[index];
  }
  if (state.prospective_feedback_source_count > 0U) {
    --state.prospective_feedback_source_count;
    state.prospective_feedback_source_identities
        [state.prospective_feedback_source_count] = {};
  }
}

[[nodiscard]] bool
claimProspectiveSource(ExecutionHorizonWitnessAdmissionResult& result,
                       const ExecutionHorizonWitnessState& state,
                       const ExecutionHorizonFeedbackCandidate& candidate,
                       std::optional<std::size_t>& prospective_index,
                       bool& receipt_high_water_advances) noexcept {
  prospective_index =
      prospectiveSourceIndex(state, candidate.offboard_producer_instance_id);
  receipt_high_water_advances = true;
  if (prospective_index.has_value()) {
    ExecutionHorizonFeedbackProspectiveSourceIdentity& claim =
        result.next_state.prospective_feedback_source_identities[*prospective_index];
    if (candidate.source_stamp_ns < claim.candidate.source_stamp_ns) {
      result.stale = true;
      return false;
    }
    if (candidate.source_stamp_ns == claim.candidate.source_stamp_ns) {
      if (sameSourceIdentity(claim.candidate, candidate)) {
        result.replay = true;
      } else {
        result.conflict = true;
      }
      return false;
    }

    const std::int64_t previous_receive_high_water = claim.receive_stamp_high_water_ns;
    receipt_high_water_advances =
        candidate.receive_stamp_ns >= previous_receive_high_water;
    claim = makeProspectiveSource(candidate);
    claim.receive_stamp_high_water_ns =
        std::max(previous_receive_high_water, candidate.receive_stamp_ns);
    result.state_advanced = true;
    return true;
  }

  if (state.prospective_feedback_source_capacity_exhausted ||
      state.prospective_feedback_source_count ==
          state.prospective_feedback_source_identities.size()) {
    result.invalid = true;
    result.prospective_capacity_exhausted = true;
    if (!state.prospective_feedback_source_capacity_exhausted) {
      result.next_state.prospective_feedback_source_capacity_exhausted = true;
      result.state_advanced = true;
    }
    return false;
  }

  prospective_index = state.prospective_feedback_source_count;
  result.next_state.prospective_feedback_source_identities[*prospective_index] =
      makeProspectiveSource(candidate);
  ++result.next_state.prospective_feedback_source_count;
  result.state_advanced = true;
  return true;
}

[[nodiscard]] ExecutionHorizonWitnessAdmissionResult
admitFeedback(const ExecutionHorizonWitnessState& state,
              const ExecutionHorizonFeedbackCandidate& candidate,
              const bool semantic_payload_valid) noexcept {
  ExecutionHorizonWitnessAdmissionResult result{.next_state = state,
                                                .heartbeat = candidate.heartbeat()};
  if (!state.valid() || !candidate.sourceIdentityClaimable()) {
    result.invalid = true;
    return result;
  }

  const std::uint64_t current_offboard =
      state.offboard_session.current_producer_instance_id;
  if (candidate.offboard_producer_instance_id == current_offboard &&
      current_offboard != 0U) {
    const ExecutionHorizonFeedbackCandidate& claimed =
        state.feedback_source_identity.candidate;
    if (candidate.source_stamp_ns < claimed.source_stamp_ns) {
      result.stale = true;
      return result;
    }
    if (candidate.source_stamp_ns == claimed.source_stamp_ns) {
      result.invalid = !semantic_payload_valid || !candidate.valid();
      if (sameSourceIdentity(claimed, candidate)) {
        if (!semantic_payload_valid && !state.feedback_source_identity.tombstoned) {
          result.next_state.feedback_source_identity.tombstoned = true;
          result.next_state.latest_horizon_feedback = {};
          result.witness_revoked = !state.latest_horizon_feedback.empty();
          result.state_advanced = true;
        } else {
          result.replay = true;
          result.witness_revoked = state.feedback_source_identity.tombstoned;
        }
      } else {
        result.conflict = true;
        result.witness_revoked = true;
        if (!state.feedback_source_identity.tombstoned) {
          result.next_state.feedback_source_identity.tombstoned = true;
          result.next_state.latest_horizon_feedback = {};
          result.state_advanced = true;
        }
      }
      return result;
    }

    const std::int64_t previous_receive_high_water =
        state.horizon_feedback_receive_stamp_high_water_ns;
    const ExecutionHorizonFeedbackWitness previous_lineage =
        state.last_accepted_horizon_feedback;
    claimCurrentSource(result, state, candidate);
    if (!semantic_payload_valid || !candidate.valid()) {
      result.invalid = true;
      return result;
    }
    if (candidate.receive_stamp_ns < state.latest_session_receive_stamp_ns ||
        candidate.receive_stamp_ns < previous_receive_high_water) {
      result.stale = true;
      return result;
    }

    const OffboardSessionAdmissionResult session = admitOffboardSession(
        state.offboard_session,
        OffboardSessionCandidate{
            .producer_instance_id = candidate.offboard_producer_instance_id,
            .source_stamp_ns = candidate.source_stamp_ns,
        });
    if (!session.accept) {
      result.stale = session.stale;
      result.invalid = session.invalid;
      return result;
    }
    if (candidate.horizonFeedback() && !previous_lineage.empty() &&
        candidate.horizon_producer_instance_id ==
            previous_lineage.horizon_producer_instance_id &&
        candidate.horizon_sequence < previous_lineage.horizon_sequence) {
      result.stale = true;
      return result;
    }

    result.next_state.offboard_session = session.next_state;
    acceptCurrentSource(result, candidate);
    if (candidate.heartbeat()) {
      result.next_state.last_accepted_horizon_feedback = {};
      result.session_transitioned = session.transitioned;
      return result;
    }

    result.next_state.latest_horizon_feedback = makeWitness(candidate);
    result.next_state.last_accepted_horizon_feedback =
        result.next_state.latest_horizon_feedback;
    result.witness_updated = true;
    return result;
  }

  if (current_offboard != 0U &&
      state.offboard_session.producerRetired(candidate.offboard_producer_instance_id)) {
    result.stale = true;
    result.non_current_session = true;
    return result;
  }

  std::optional<std::size_t> prospective_index;
  bool prospective_receipt_advances{true};
  if (!claimProspectiveSource(result, state, candidate, prospective_index,
                              prospective_receipt_advances)) {
    result.non_current_session = current_offboard != 0U;
    return result;
  }
  result.non_current_session = current_offboard != 0U;
  if (!semantic_payload_valid || !candidate.valid()) {
    result.invalid = true;
    return result;
  }
  if (!candidate.heartbeat()) {
    return result;
  }
  if (!prospective_receipt_advances ||
      (current_offboard != 0U &&
       (candidate.source_stamp_ns <= state.offboard_session.latest_source_stamp_ns ||
        candidate.receive_stamp_ns <= state.latest_session_receive_stamp_ns))) {
    result.stale = true;
    return result;
  }

  const OffboardSessionAdmissionResult session = admitOffboardSession(
      state.offboard_session,
      OffboardSessionCandidate{
          .producer_instance_id = candidate.offboard_producer_instance_id,
          .source_stamp_ns = candidate.source_stamp_ns,
      });
  if (!session.accept) {
    result.stale = session.stale;
    result.invalid = session.invalid;
    return result;
  }

  const bool had_witness = !state.latest_horizon_feedback.empty();
  result.next_state.offboard_session = session.next_state;
  result.next_state.latest_session_receive_stamp_ns = candidate.receive_stamp_ns;
  result.next_state.feedback_source_identity = {
      .candidate = candidate,
      .tombstoned = false,
  };
  result.next_state.latest_horizon_feedback = {};
  result.next_state.last_accepted_horizon_feedback = {};
  result.next_state.horizon_feedback_source_stamp_high_water_ns =
      candidate.source_stamp_ns;
  result.next_state.horizon_feedback_receive_stamp_high_water_ns =
      candidate.receive_stamp_ns;
  eraseProspectiveSource(result.next_state, *prospective_index);
  result.accept = true;
  result.heartbeat = true;
  result.witness_revoked = had_witness;
  result.session_transitioned = session.transitioned;
  result.non_current_session = false;
  result.state_advanced = true;
  return result;
}

} // namespace

bool ExecutionHorizonFeedbackCandidate::sourceIdentityClaimable() const noexcept {
  return offboard_producer_instance_id != 0U && source_stamp_ns > 0 &&
         receive_stamp_ns > 0 && wire_fingerprint != 0U;
}

bool ExecutionHorizonFeedbackCandidate::heartbeat() const noexcept {
  return offboard_producer_instance_id != 0U && horizon_producer_instance_id == 0U &&
         horizon_sequence == 0U && content_fingerprint == 0U &&
         execution_mode == ExecutionHorizonWitnessMode::kPositionHold &&
         !control_authoritative;
}

bool ExecutionHorizonFeedbackCandidate::horizonFeedback() const noexcept {
  return offboard_producer_instance_id != 0U && horizon_producer_instance_id != 0U &&
         horizon_sequence != 0U && content_fingerprint != 0U &&
         validMode(execution_mode) &&
         feedbackAuthorityCoherent(execution_mode, control_authoritative);
}

bool ExecutionHorizonFeedbackCandidate::valid() const noexcept {
  return sourceIdentityClaimable() &&
         timestampPairValid(source_stamp_ns, receive_stamp_ns) &&
         (heartbeat() || horizonFeedback());
}

bool ExecutionHorizonFeedbackWitness::empty() const noexcept {
  return offboard_producer_instance_id == 0U && horizon_producer_instance_id == 0U &&
         horizon_sequence == 0U && source_stamp_ns == 0 && receive_stamp_ns == 0 &&
         content_fingerprint == 0U && wire_fingerprint == 0U &&
         execution_mode == ExecutionHorizonWitnessMode::kPlanned &&
         !control_authoritative;
}

bool ExecutionHorizonFeedbackWitness::valid() const noexcept {
  const ExecutionHorizonFeedbackCandidate candidate{
      .offboard_producer_instance_id = offboard_producer_instance_id,
      .horizon_producer_instance_id = horizon_producer_instance_id,
      .horizon_sequence = horizon_sequence,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = content_fingerprint,
      .wire_fingerprint = wire_fingerprint,
      .execution_mode = execution_mode,
      .control_authoritative = control_authoritative,
  };
  return candidate.valid() && candidate.horizonFeedback();
}

bool ExecutionHorizonFeedbackSourceIdentity::empty() const noexcept {
  return candidate.offboard_producer_instance_id == 0U &&
         candidate.horizon_producer_instance_id == 0U &&
         candidate.horizon_sequence == 0U && candidate.source_stamp_ns == 0 &&
         candidate.receive_stamp_ns == 0 && candidate.content_fingerprint == 0U &&
         candidate.wire_fingerprint == 0U &&
         candidate.execution_mode == ExecutionHorizonWitnessMode::kPlanned &&
         !candidate.control_authoritative && !tombstoned;
}

bool ExecutionHorizonFeedbackSourceIdentity::valid() const noexcept {
  return candidate.sourceIdentityClaimable();
}

bool ExecutionHorizonFeedbackProspectiveSourceIdentity::empty() const noexcept {
  return candidate.offboard_producer_instance_id == 0U &&
         candidate.horizon_producer_instance_id == 0U &&
         candidate.horizon_sequence == 0U && candidate.source_stamp_ns == 0 &&
         candidate.receive_stamp_ns == 0 && candidate.content_fingerprint == 0U &&
         candidate.wire_fingerprint == 0U &&
         candidate.execution_mode == ExecutionHorizonWitnessMode::kPlanned &&
         !candidate.control_authoritative && receive_stamp_high_water_ns == 0 &&
         !tombstoned;
}

bool ExecutionHorizonFeedbackProspectiveSourceIdentity::valid() const noexcept {
  return candidate.sourceIdentityClaimable() && tombstoned &&
         receive_stamp_high_water_ns >= candidate.receive_stamp_ns;
}

bool ExecutionHorizonWitnessState::valid() const noexcept {
  if (!offboard_session.valid() || prospective_feedback_source_count >
                                       prospective_feedback_source_identities.size()) {
    return false;
  }

  const std::uint64_t current_offboard = offboard_session.current_producer_instance_id;
  if (current_offboard == 0U) {
    if (latest_session_receive_stamp_ns != 0 || !feedback_source_identity.empty() ||
        !latest_horizon_feedback.empty() || !last_accepted_horizon_feedback.empty() ||
        horizon_feedback_source_stamp_high_water_ns != 0 ||
        horizon_feedback_receive_stamp_high_water_ns != 0) {
      return false;
    }
  } else {
    if (latest_session_receive_stamp_ns < offboard_session.latest_source_stamp_ns ||
        !feedback_source_identity.valid()) {
      return false;
    }

    const ExecutionHorizonFeedbackCandidate& source_identity =
        feedback_source_identity.candidate;
    if (source_identity.offboard_producer_instance_id != current_offboard ||
        horizon_feedback_source_stamp_high_water_ns !=
            source_identity.source_stamp_ns ||
        horizon_feedback_receive_stamp_high_water_ns <
            source_identity.receive_stamp_ns) {
      return false;
    }
    if (!last_accepted_horizon_feedback.empty() &&
        (!last_accepted_horizon_feedback.valid() ||
         last_accepted_horizon_feedback.offboard_producer_instance_id !=
             current_offboard ||
         last_accepted_horizon_feedback.source_stamp_ns >
             offboard_session.latest_source_stamp_ns ||
         last_accepted_horizon_feedback.receive_stamp_ns >
             latest_session_receive_stamp_ns)) {
      return false;
    }
    if (feedback_source_identity.tombstoned) {
      if (!latest_horizon_feedback.empty()) {
        return false;
      }
    } else {
      if (!source_identity.valid() ||
          source_identity.source_stamp_ns != offboard_session.latest_source_stamp_ns ||
          source_identity.receive_stamp_ns != latest_session_receive_stamp_ns ||
          source_identity.receive_stamp_ns !=
              horizon_feedback_receive_stamp_high_water_ns) {
        return false;
      }
      if (source_identity.heartbeat()) {
        if (!latest_horizon_feedback.empty() ||
            !last_accepted_horizon_feedback.empty()) {
          return false;
        }
      } else if (!latest_horizon_feedback.valid() ||
                 !witnessMatchesCandidate(latest_horizon_feedback, source_identity) ||
                 !witnessMatchesCandidate(last_accepted_horizon_feedback,
                                          source_identity)) {
        return false;
      }
    }
  }

  for (std::size_t index = 0U; index < prospective_feedback_source_count; ++index) {
    const ExecutionHorizonFeedbackProspectiveSourceIdentity& claim =
        prospective_feedback_source_identities[index];
    if (!claim.valid() ||
        claim.candidate.offboard_producer_instance_id == current_offboard ||
        offboard_session.producerRetired(
            claim.candidate.offboard_producer_instance_id)) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (prospective_feedback_source_identities[previous]
              .candidate.offboard_producer_instance_id ==
          claim.candidate.offboard_producer_instance_id) {
        return false;
      }
    }
  }
  for (std::size_t index = prospective_feedback_source_count;
       index < prospective_feedback_source_identities.size(); ++index) {
    if (!prospective_feedback_source_identities[index].empty()) {
      return false;
    }
  }
  return true;
}

ExecutionHorizonWitnessAdmissionResult admitExecutionHorizonFeedback(
    const ExecutionHorizonWitnessState& state,
    const ExecutionHorizonFeedbackCandidate& candidate) noexcept {
  return admitFeedback(state, candidate, true);
}

ExecutionHorizonWitnessAdmissionResult
admitExecutionHorizonFeedbackPayload(const ExecutionHorizonWitnessState& state,
                                     const ExecutionHorizonFeedbackCandidate& candidate,
                                     const bool payload_valid) noexcept {
  return admitFeedback(state, candidate, payload_valid);
}

ExecutionHorizonWitnessAdmissionResult revokeMalformedExecutionHorizonFeedback(
    const ExecutionHorizonWitnessState& state,
    const ExecutionHorizonFeedbackCandidate& wire_identity,
    const std::int64_t receive_stamp_ns) noexcept {
  ExecutionHorizonWitnessAdmissionResult result{.next_state = state, .invalid = true};
  if (!state.valid() || receive_stamp_ns <= 0 ||
      wire_identity.offboard_producer_instance_id == 0U) {
    return result;
  }

  ExecutionHorizonFeedbackCandidate claim = wire_identity;
  claim.receive_stamp_ns = receive_stamp_ns;
  if (claim.sourceIdentityClaimable()) {
    result = admitFeedback(state, claim, false);
    result.invalid = true;
    return result;
  }

  if (claim.offboard_producer_instance_id !=
      state.offboard_session.current_producer_instance_id) {
    result.non_current_session =
        state.offboard_session.current_producer_instance_id != 0U;
    result.stale =
        state.offboard_session.producerRetired(claim.offboard_producer_instance_id);
    return result;
  }

  const ExecutionHorizonFeedbackCandidate& current =
      state.feedback_source_identity.candidate;
  const bool heartbeat_identity =
      claim.horizon_producer_instance_id == 0U && claim.horizon_sequence == 0U;
  const bool horizon_identity =
      claim.horizon_producer_instance_id != 0U && claim.horizon_sequence != 0U &&
      claim.horizon_producer_instance_id == current.horizon_producer_instance_id &&
      claim.horizon_sequence == current.horizon_sequence;
  if ((!heartbeat_identity && !horizon_identity) ||
      state.feedback_source_identity.tombstoned) {
    return result;
  }

  result.next_state.feedback_source_identity.tombstoned = true;
  result.next_state.latest_horizon_feedback = {};
  result.witness_revoked = !state.latest_horizon_feedback.empty();
  result.conflict = true;
  result.state_advanced = true;
  return result;
}

bool ExecutionHorizonWitnessRequirement::valid() const noexcept {
  return target_offboard_instance_id != 0U && horizon_producer_instance_id != 0U &&
         horizon_sequence != 0U && valid_from_ns > 0 &&
         valid_until_ns > valid_from_ns && validMode(execution_mode);
}

bool offboardSessionFreshAt(const ExecutionHorizonWitnessState& state,
                            const std::int64_t now_ns,
                            const std::int64_t maximum_age_ns) noexcept {
  return state.valid() && state.offboard_session.current_producer_instance_id != 0U &&
         timestampPairFreshAt(state.offboard_session.latest_source_stamp_ns,
                              state.latest_session_receive_stamp_ns, now_ns,
                              maximum_age_ns);
}

bool executionHorizonReceiptFreshAt(
    const ExecutionHorizonWitnessState& state,
    const ExecutionHorizonWitnessRequirement& requirement, const std::int64_t now_ns,
    const std::int64_t maximum_age_ns) noexcept {
  if (!requirement.valid() || !offboardSessionFreshAt(state, now_ns, maximum_age_ns) ||
      now_ns < requirement.valid_from_ns || now_ns >= requirement.valid_until_ns) {
    return false;
  }

  const ExecutionHorizonFeedbackWitness& witness = state.latest_horizon_feedback;
  return witness.valid() && !state.feedback_source_identity.tombstoned &&
         state.offboard_session.current_producer_instance_id ==
             requirement.target_offboard_instance_id &&
         witness.offboard_producer_instance_id ==
             requirement.target_offboard_instance_id &&
         witness.horizon_producer_instance_id ==
             requirement.horizon_producer_instance_id &&
         witness.horizon_sequence == requirement.horizon_sequence &&
         witness.execution_mode == requirement.execution_mode &&
         witness.source_stamp_ns >= requirement.valid_from_ns &&
         witness.source_stamp_ns < requirement.valid_until_ns &&
         timestampPairFreshAt(witness.source_stamp_ns, witness.receive_stamp_ns, now_ns,
                              maximum_age_ns);
}

bool executionHorizonWitnessFreshAt(
    const ExecutionHorizonWitnessState& state,
    const ExecutionHorizonWitnessRequirement& requirement, const std::int64_t now_ns,
    const std::int64_t maximum_age_ns) noexcept {
  if (!executionHorizonReceiptFreshAt(state, requirement, now_ns, maximum_age_ns)) {
    return false;
  }
  const ExecutionHorizonFeedbackWitness& witness = state.latest_horizon_feedback;
  return authorityMatchesMode(witness.execution_mode, witness.control_authoritative);
}

} // namespace drone_city_nav
