#include "drone_city_nav/execution_horizon_admission.hpp"

#include <algorithm>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool
appendRetiredProducer(ExecutionHorizonAdmissionState& state,
                      const std::uint64_t producer_instance_id) noexcept {
  if (state.retired_producer_count >= state.retired_producer_instance_ids.size()) {
    return false;
  }
  state.retired_producer_instance_ids[state.retired_producer_count] =
      producer_instance_id;
  ++state.retired_producer_count;
  return true;
}

[[nodiscard]] std::int64_t timestampHighWater(const std::int64_t value) noexcept {
  return std::max<std::int64_t>(0, value);
}

[[nodiscard]] bool
sameWireIdentity(const ExecutionHorizonAdmissionCandidate& first,
                 const ExecutionHorizonAdmissionCandidate& second) noexcept {
  return first.producer_instance_id == second.producer_instance_id &&
         first.sequence == second.sequence &&
         first.source_stamp_ns == second.source_stamp_ns &&
         first.valid_from_ns == second.valid_from_ns &&
         first.content_fingerprint == second.content_fingerprint;
}

[[nodiscard]] ExecutionHorizonAdmissionCandidate
currentCandidate(const ExecutionHorizonAdmissionState& state) noexcept {
  return {
      .producer_instance_id = state.current_producer_instance_id,
      .sequence = state.current_sequence,
      .source_stamp_ns = state.current_source_stamp_ns,
      .valid_from_ns = state.current_valid_from_ns,
      .content_fingerprint = state.current_content_fingerprint,
  };
}

void claimCurrentIdentity(
    ExecutionHorizonAdmissionState& state,
    const ExecutionHorizonAdmissionCandidate& candidate) noexcept {
  state.current_producer_instance_id = candidate.producer_instance_id;
  state.current_sequence = candidate.sequence;
  state.current_source_stamp_ns = candidate.source_stamp_ns;
  state.current_valid_from_ns = candidate.valid_from_ns;
  state.latest_source_stamp_ns = std::max(
      state.latest_source_stamp_ns, timestampHighWater(candidate.source_stamp_ns));
  state.latest_valid_from_ns =
      std::max(state.latest_valid_from_ns, timestampHighWater(candidate.valid_from_ns));
  state.current_content_fingerprint = candidate.content_fingerprint;
  state.current_identity_tombstoned = true;
}

[[nodiscard]] ExecutionHorizonProspectiveIdentityClaim
makeProspectiveClaim(const ExecutionHorizonAdmissionCandidate& candidate) noexcept {
  return {
      .candidate = candidate,
      .source_stamp_high_water_ns = timestampHighWater(candidate.source_stamp_ns),
      .valid_from_high_water_ns = timestampHighWater(candidate.valid_from_ns),
      .tombstoned = true,
  };
}

[[nodiscard]] std::optional<std::size_t>
prospectiveIdentityIndex(const ExecutionHorizonAdmissionState& state,
                         const std::uint64_t producer_instance_id) noexcept {
  for (std::size_t index = 0U; index < state.prospective_identity_count; ++index) {
    if (state.prospective_identity_claims[index].candidate.producer_instance_id ==
        producer_instance_id) {
      return index;
    }
  }
  return std::nullopt;
}

void eraseProspectiveIdentity(ExecutionHorizonAdmissionState& state,
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

} // namespace

bool ExecutionHorizonAdmissionCandidate::identityClaimable() const noexcept {
  return producer_instance_id != 0U && sequence != 0U;
}

bool ExecutionHorizonAdmissionCandidate::valid() const noexcept {
  return identityClaimable() && source_stamp_ns > 0 && valid_from_ns > 0 &&
         content_fingerprint != 0U;
}

bool ExecutionHorizonProspectiveIdentityClaim::empty() const noexcept {
  return !candidate.identityClaimable() && candidate.producer_instance_id == 0U &&
         candidate.sequence == 0U && candidate.source_stamp_ns == 0 &&
         candidate.valid_from_ns == 0 && candidate.content_fingerprint == 0U &&
         source_stamp_high_water_ns == 0 && valid_from_high_water_ns == 0 &&
         !tombstoned;
}

bool ExecutionHorizonProspectiveIdentityClaim::valid() const noexcept {
  return candidate.identityClaimable() && tombstoned &&
         source_stamp_high_water_ns >= timestampHighWater(candidate.source_stamp_ns) &&
         valid_from_high_water_ns >= timestampHighWater(candidate.valid_from_ns);
}

bool ExecutionHorizonAdmissionState::valid() const noexcept {
  if (retired_producer_count > retired_producer_instance_ids.size() ||
      prospective_identity_count > prospective_identity_claims.size()) {
    return false;
  }
  const bool has_current = current_producer_instance_id != 0U;
  if (!has_current) {
    if (current_sequence != 0U || current_source_stamp_ns != 0 ||
        current_valid_from_ns != 0 || latest_source_stamp_ns != 0 ||
        latest_valid_from_ns != 0 || current_content_fingerprint != 0U ||
        current_identity_tombstoned || retired_producer_count != 0U ||
        prospective_identity_count != 0U || prospective_identity_capacity_exhausted) {
      return false;
    }
  } else {
    const ExecutionHorizonAdmissionCandidate current = currentCandidate(*this);
    if (!current.identityClaimable() ||
        latest_source_stamp_ns < timestampHighWater(current_source_stamp_ns) ||
        latest_valid_from_ns < timestampHighWater(current_valid_from_ns) ||
        (!current_identity_tombstoned && !current.valid())) {
      return false;
    }
  }
  for (std::size_t index = 0U; index < retired_producer_count; ++index) {
    const std::uint64_t retired = retired_producer_instance_ids[index];
    if (retired == 0U || retired == current_producer_instance_id) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (retired_producer_instance_ids[previous] == retired) {
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
  for (std::size_t index = 0U; index < prospective_identity_count; ++index) {
    const ExecutionHorizonProspectiveIdentityClaim& claim =
        prospective_identity_claims[index];
    if (!claim.valid() ||
        claim.candidate.producer_instance_id == current_producer_instance_id ||
        producerRetired(claim.candidate.producer_instance_id)) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (prospective_identity_claims[previous].candidate.producer_instance_id ==
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
  return true;
}

bool ExecutionHorizonAdmissionState::producerRetired(
    const std::uint64_t producer_instance_id) const noexcept {
  if (producer_instance_id == 0U ||
      retired_producer_count > retired_producer_instance_ids.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < retired_producer_count; ++index) {
    if (retired_producer_instance_ids[index] == producer_instance_id) {
      return true;
    }
  }
  return false;
}

ExecutionHorizonAdmissionResult
admitExecutionHorizonIdentity(const ExecutionHorizonAdmissionState& state,
                              const ExecutionHorizonAdmissionCandidate& candidate,
                              const bool payload_admissible) noexcept {
  ExecutionHorizonAdmissionResult result{.next_state = state};
  if (!state.valid() || !candidate.identityClaimable()) {
    result.invalid = true;
    return result;
  }

  if (state.current_producer_instance_id == 0U) {
    claimCurrentIdentity(result.next_state, candidate);
    result.accept_identity = true;
    result.state_advanced = true;
    if (!candidate.valid() || !payload_admissible) {
      result.invalid = true;
      return result;
    }
    result.next_state.current_identity_tombstoned = false;
    result.payload_installable = true;
    return result;
  }

  if (candidate.producer_instance_id == state.current_producer_instance_id) {
    if (candidate.sequence < state.current_sequence) {
      result.stale = true;
      return result;
    }
    if (candidate.sequence == state.current_sequence) {
      if (sameWireIdentity(currentCandidate(state), candidate)) {
        result.replay = true;
        result.revoke = state.current_identity_tombstoned;
      } else {
        result.revoke = true;
        result.conflict = true;
        if (!state.current_identity_tombstoned) {
          result.next_state.current_identity_tombstoned = true;
          result.state_advanced = true;
        }
      }
      return result;
    }
    const bool timestamps_advance =
        candidate.valid() && candidate.source_stamp_ns > state.latest_source_stamp_ns &&
        candidate.valid_from_ns > state.latest_valid_from_ns;
    claimCurrentIdentity(result.next_state, candidate);
    result.accept_identity = true;
    result.revoke = true;
    result.state_advanced = true;
    if (!candidate.valid() || !payload_admissible) {
      result.invalid = true;
      return result;
    }
    if (!timestamps_advance) {
      result.stale = true;
      return result;
    }
    result.next_state.current_identity_tombstoned = false;
    result.payload_installable = true;
    return result;
  }

  if (state.producerRetired(candidate.producer_instance_id)) {
    result.stale = true;
    return result;
  }
  std::optional<std::size_t> prospective_index =
      prospectiveIdentityIndex(state, candidate.producer_instance_id);
  bool prospective_timestamps_advance{true};
  if (prospective_index.has_value()) {
    ExecutionHorizonProspectiveIdentityClaim& claim =
        result.next_state.prospective_identity_claims[*prospective_index];
    if (candidate.sequence < claim.candidate.sequence) {
      result.stale = true;
      return result;
    }
    if (candidate.sequence == claim.candidate.sequence) {
      if (sameWireIdentity(claim.candidate, candidate)) {
        result.replay = true;
      } else {
        result.conflict = true;
      }
      return result;
    }
    prospective_timestamps_advance =
        candidate.source_stamp_ns > claim.source_stamp_high_water_ns &&
        candidate.valid_from_ns > claim.valid_from_high_water_ns;
    const std::int64_t previous_source_high_water = claim.source_stamp_high_water_ns;
    const std::int64_t previous_valid_from_high_water = claim.valid_from_high_water_ns;
    claim = makeProspectiveClaim(candidate);
    claim.source_stamp_high_water_ns =
        std::max(previous_source_high_water, claim.source_stamp_high_water_ns);
    claim.valid_from_high_water_ns =
        std::max(previous_valid_from_high_water, claim.valid_from_high_water_ns);
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

  if (!candidate.valid()) {
    result.invalid = true;
    return result;
  }
  const bool handoff_timestamps_advance =
      candidate.source_stamp_ns > state.latest_source_stamp_ns &&
      candidate.valid_from_ns > state.latest_valid_from_ns;
  if (!prospective_timestamps_advance || !handoff_timestamps_advance) {
    result.stale = true;
    return result;
  }
  // The prospective wire identity is already resident and immutable at this
  // point. A malformed or otherwise unusable payload is not authority to
  // retire the active producer; only a strictly higher identity may recover.
  if (!payload_admissible) {
    result.invalid = true;
    return result;
  }
  if (!appendRetiredProducer(result.next_state, state.current_producer_instance_id)) {
    result.retired_capacity_exhausted = true;
    return result;
  }

  claimCurrentIdentity(result.next_state, candidate);
  result.next_state.current_identity_tombstoned = false;
  eraseProspectiveIdentity(result.next_state, *prospective_index);
  result.accept_identity = true;
  result.payload_installable = true;
  result.revoke = true;
  return result;
}

bool tombstoneExecutionHorizonIdentity(
    ExecutionHorizonAdmissionState& state,
    const ExecutionHorizonAdmissionCandidate& candidate) noexcept {
  if (!state.valid() || !candidate.identityClaimable() ||
      state.current_producer_instance_id != candidate.producer_instance_id ||
      state.current_sequence != candidate.sequence ||
      state.current_source_stamp_ns != candidate.source_stamp_ns ||
      state.current_valid_from_ns != candidate.valid_from_ns ||
      state.current_content_fingerprint != candidate.content_fingerprint) {
    return false;
  }
  state.current_identity_tombstoned = true;
  return state.valid();
}

} // namespace drone_city_nav
