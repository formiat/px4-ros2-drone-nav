#pragma once

#include "drone_city_nav/execution_evidence_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] inline bool
producerRetired(const LatestLidarEvidenceAdmissionState3D& state,
                const std::uint64_t producer_instance_id) noexcept {
  for (std::size_t index = 0U; index < state.retired_producer_count; ++index) {
    if (state.retired_producer_instance_ids[index] == producer_instance_id) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] inline bool
validIdentityClaim(const LatestLidarEvidenceIdentityClaim3D& claim) noexcept {
  return claim.producer_instance_id != 0U && claim.sequence != 0U &&
         claim.raw_wire_fingerprint != 0U && claim.first_receive_stamp_ns > 0;
}

[[nodiscard]] inline bool
sameIdentityClaim(const LatestLidarEvidenceIdentityClaim3D& left,
                  const LatestLidarEvidenceIdentityClaim3D& right) noexcept {
  return left.producer_instance_id == right.producer_instance_id &&
         left.sequence == right.sequence &&
         left.raw_wire_fingerprint == right.raw_wire_fingerprint &&
         left.first_receive_stamp_ns == right.first_receive_stamp_ns;
}

[[nodiscard]] inline std::size_t
prospectiveClaimIndex(const LatestLidarEvidenceAdmissionState3D& state,
                      const std::uint64_t producer_instance_id) noexcept {
  for (std::size_t index = 0U; index < state.prospective_claim_count; ++index) {
    if (state.prospective_claims[index].identity.producer_instance_id ==
        producer_instance_id) {
      return index;
    }
  }
  return state.prospective_claim_count;
}

inline void removeProspectiveClaim(LatestLidarEvidenceAdmissionState3D& state,
                                   const std::uint64_t producer_instance_id) noexcept {
  const std::size_t index = prospectiveClaimIndex(state, producer_instance_id);
  if (index == state.prospective_claim_count) {
    return;
  }
  --state.prospective_claim_count;
  state.prospective_claims[index] =
      state.prospective_claims[state.prospective_claim_count];
  state.prospective_claims[state.prospective_claim_count] = {};
}

inline void
clearPendingProducerHandoff(LatestLidarEvidenceAdmissionState3D& state) noexcept {
  state.pending_producer_instance_id = 0U;
  state.pending_sequence = 0U;
  state.pending_pose_generation = 0U;
  state.pending_source_content_fingerprint = 0U;
  state.pending_receive_stamp_ns = 0;
  state.pending_confirmation_count = 0U;
  state.pending_identity_conflicted = false;
}

inline void
beginPendingProducerHandoff(LatestLidarEvidenceAdmissionState3D& state,
                            const VersionedLatestLidarEvidence3D& candidate) noexcept {
  state.pending_producer_instance_id = candidate.producerInstanceId();
  state.pending_sequence = candidate.sequence();
  state.pending_pose_generation = candidate.poseGeneration();
  state.pending_source_content_fingerprint = candidate.sourceContentFingerprint();
  state.pending_receive_stamp_ns = candidate.receiveStampNs();
  state.pending_confirmation_count = 1U;
  state.pending_identity_conflicted = false;
}

inline void
promoteCurrentClaim(LatestLidarEvidenceAdmissionState3D& state,
                    const LatestLidarEvidenceIdentityClaim3D& claim) noexcept {
  removeProspectiveClaim(state, claim.producer_instance_id);
  state.current_producer_instance_id = claim.producer_instance_id;
  state.current_sequence = claim.sequence;
  state.current_raw_wire_fingerprint = claim.raw_wire_fingerprint;
  state.current_first_receive_stamp_ns = claim.first_receive_stamp_ns;
  state.current_identity_conflicted = false;
  clearPendingProducerHandoff(state);
}

[[nodiscard]] inline bool
validAdmissionState(const LatestLidarEvidenceAdmissionState3D& state,
                    const VersionedLatestLidarEvidence3D* const current) noexcept {
  if (state.retired_producer_count > state.retired_producer_instance_ids.size() ||
      state.prospective_claim_count > state.prospective_claims.size()) {
    return false;
  }
  std::uint64_t current_producer{0U};
  if (current == nullptr) {
    if (state.current_producer_instance_id != 0U || state.current_sequence != 0U ||
        state.current_raw_wire_fingerprint != 0U ||
        state.current_first_receive_stamp_ns != 0 ||
        state.current_identity_conflicted || state.pending_confirmation_count != 0U) {
      return false;
    }
  } else {
    if (!current->valid() ||
        state.current_producer_instance_id != current->producerInstanceId() ||
        state.current_sequence != current->sequence() ||
        state.current_raw_wire_fingerprint == 0U ||
        state.current_first_receive_stamp_ns != current->receiveStampNs()) {
      return false;
    }
    current_producer = current->producerInstanceId();
  }
  for (std::size_t index = 0U; index < state.retired_producer_count; ++index) {
    const std::uint64_t retired = state.retired_producer_instance_ids[index];
    if (retired == 0U || retired == current_producer) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (state.retired_producer_instance_ids[previous] == retired) {
        return false;
      }
    }
  }
  for (std::size_t index = 0U; index < state.prospective_claim_count; ++index) {
    const LatestLidarEvidenceIdentityClaim3D& claim =
        state.prospective_claims[index].identity;
    if (!validIdentityClaim(claim) || (claim.producer_instance_id == current_producer &&
                                       claim.sequence <= state.current_sequence)) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (state.prospective_claims[previous].identity.producer_instance_id ==
          claim.producer_instance_id) {
        return false;
      }
    }
  }
  if (state.pending_confirmation_count == 0U) {
    return state.pending_producer_instance_id == 0U && state.pending_sequence == 0U &&
           state.pending_pose_generation == 0U &&
           state.pending_source_content_fingerprint == 0U &&
           state.pending_receive_stamp_ns == 0 && !state.pending_identity_conflicted;
  }
  if (state.pending_confirmation_count != 1U ||
      state.pending_producer_instance_id == 0U ||
      state.pending_producer_instance_id == current_producer ||
      producerRetired(state, state.pending_producer_instance_id) ||
      state.pending_sequence == 0U || state.pending_source_content_fingerprint == 0U ||
      state.pending_receive_stamp_ns <= 0) {
    return false;
  }
  const std::size_t pending_index =
      prospectiveClaimIndex(state, state.pending_producer_instance_id);
  if (pending_index == state.prospective_claim_count ||
      state.prospective_claims[pending_index].identity.sequence <
          state.pending_sequence) {
    return false;
  }
  const LatestLidarEvidenceProspectiveClaim3D& pending_claim =
      state.prospective_claims[pending_index];
  return pending_claim.identity.sequence != state.pending_sequence ||
         (pending_claim.identity.first_receive_stamp_ns ==
              state.pending_receive_stamp_ns &&
          pending_claim.conflicted == state.pending_identity_conflicted);
}

} // namespace
} // namespace drone_city_nav
