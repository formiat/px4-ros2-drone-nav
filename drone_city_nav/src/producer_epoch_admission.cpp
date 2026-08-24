#include "drone_city_nav/producer_epoch_admission.hpp"

#include <algorithm>
#include <limits>
#include <optional>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validConfig(const ProducerEpochAdmissionConfig& config) noexcept {
  return config.maximum_observation_age_ns > 0 &&
         config.maximum_confirmation_interval_ns > 0;
}

[[nodiscard]] bool
claimableIdentity(const ProducerEpochObservation& observation) noexcept {
  return observation.producer_instance_id != 0U && observation.sequence != 0U &&
         observation.content_fingerprint != 0U;
}

[[nodiscard]] bool
validObservationTimes(const ProducerEpochObservation& observation) noexcept {
  return observation.source_stamp_ns > 0 && observation.receive_stamp_ns > 0;
}

[[nodiscard]] bool observationFresh(const ProducerEpochAdmissionConfig& config,
                                    const ProducerEpochObservation& observation,
                                    const std::int64_t now_ns) noexcept {
  return now_ns > 0 && observation.source_stamp_ns <= now_ns &&
         observation.receive_stamp_ns <= now_ns &&
         now_ns - observation.source_stamp_ns <= config.maximum_observation_age_ns &&
         now_ns - observation.receive_stamp_ns <= config.maximum_observation_age_ns;
}

[[nodiscard]] bool
terminalEpochStatus(const ProducerEpochAdmissionStatus status) noexcept {
  return status == ProducerEpochAdmissionStatus::kRejectedInvalid ||
         status == ProducerEpochAdmissionStatus::kRejectedStaleCandidate ||
         status == ProducerEpochAdmissionStatus::kRejectedRegression ||
         status == ProducerEpochAdmissionStatus::kRejectedCurrentProducerFresh ||
         status == ProducerEpochAdmissionStatus::kRejectedPendingProducerMismatch ||
         status == ProducerEpochAdmissionStatus::kRejectedClockDiscontinuity;
}

[[nodiscard]] bool
terminalEvidenceStatus(const ProducerEvidenceAdmissionStatus status) noexcept {
  return status == ProducerEvidenceAdmissionStatus::kRejectedInvalid ||
         status == ProducerEvidenceAdmissionStatus::kRejectedStaleCandidate ||
         status == ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired ||
         status == ProducerEvidenceAdmissionStatus::kRejectedRegression ||
         status == ProducerEvidenceAdmissionStatus::kRejectedClockDiscontinuity;
}

[[nodiscard]] bool validRejectedEpochIdentities(
    const std::array<ProducerEpochRejectedIdentity, kProducerRejectedIdentityCapacity>&
        identities,
    const std::size_t count) noexcept {
  if (count > identities.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < count; ++index) {
    const ProducerEpochRejectedIdentity& identity = identities[index];
    if (identity.producer_instance_id == 0U || identity.sequence == 0U ||
        identity.content_fingerprint == 0U ||
        !terminalEpochStatus(identity.terminal_status)) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (identities[previous].producer_instance_id == identity.producer_instance_id &&
          identities[previous].sequence == identity.sequence) {
        return false;
      }
    }
  }
  for (std::size_t index = count; index < identities.size(); ++index) {
    const ProducerEpochRejectedIdentity& identity = identities[index];
    if (identity.producer_instance_id != 0U || identity.sequence != 0U ||
        identity.source_stamp_ns != 0 || identity.first_receive_stamp_ns != 0 ||
        identity.content_fingerprint != 0U) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::optional<std::size_t> findRejectedIdentity(
    const std::array<ProducerEpochRejectedIdentity, kProducerRejectedIdentityCapacity>&
        identities,
    const std::size_t count, const ProducerEpochObservation& observation) noexcept {
  for (std::size_t index = 0U; index < count; ++index) {
    if (identities[index].producer_instance_id == observation.producer_instance_id &&
        identities[index].sequence == observation.sequence) {
      return index;
    }
  }
  return std::nullopt;
}

[[nodiscard]] bool
sameRejectedIdentity(const ProducerEpochRejectedIdentity& identity,
                     const ProducerEpochObservation& observation) noexcept {
  return identity.source_stamp_ns == observation.source_stamp_ns &&
         identity.content_fingerprint == observation.content_fingerprint;
}

[[nodiscard]] bool rememberRejectedIdentity(
    std::array<ProducerEpochRejectedIdentity, kProducerRejectedIdentityCapacity>&
        identities,
    std::size_t& count, const ProducerEpochObservation& observation,
    const ProducerEpochAdmissionStatus terminal_status) noexcept {
  if (findRejectedIdentity(identities, count, observation).has_value()) {
    return true;
  }
  if (count == identities.size()) {
    return false;
  }
  identities[count] = ProducerEpochRejectedIdentity{
      .producer_instance_id = observation.producer_instance_id,
      .sequence = observation.sequence,
      .source_stamp_ns = observation.source_stamp_ns,
      .first_receive_stamp_ns = observation.receive_stamp_ns,
      .content_fingerprint = observation.content_fingerprint,
      .terminal_status = terminal_status,
  };
  ++count;
  return true;
}

void eraseRejectedIdentitiesThrough(
    std::array<ProducerEpochRejectedIdentity, kProducerRejectedIdentityCapacity>&
        identities,
    std::size_t& count, const std::uint64_t producer_instance_id,
    const std::uint64_t sequence) noexcept {
  std::size_t write_index{0U};
  for (std::size_t read_index = 0U; read_index < count; ++read_index) {
    if (identities[read_index].producer_instance_id == producer_instance_id &&
        identities[read_index].sequence <= sequence) {
      continue;
    }
    if (write_index != read_index) {
      identities[write_index] = identities[read_index];
    }
    ++write_index;
  }
  for (std::size_t index = write_index; index < count; ++index) {
    identities[index] = ProducerEpochRejectedIdentity{};
  }
  count = write_index;
}

[[nodiscard]] bool validRejectedEvidenceIdentities(
    const std::array<ProducerEvidenceRejectedIdentity,
                     kProducerRejectedIdentityCapacity>& identities,
    const std::size_t count) noexcept {
  if (count > identities.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < count; ++index) {
    const ProducerEvidenceRejectedIdentity& identity = identities[index];
    if (identity.authority_generation == 0U || identity.producer_instance_id == 0U ||
        identity.sequence == 0U || identity.content_fingerprint == 0U ||
        !terminalEvidenceStatus(identity.terminal_status)) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (identities[previous].authority_generation == identity.authority_generation &&
          identities[previous].producer_instance_id == identity.producer_instance_id &&
          identities[previous].sequence == identity.sequence) {
        return false;
      }
    }
  }
  for (std::size_t index = count; index < identities.size(); ++index) {
    const ProducerEvidenceRejectedIdentity& identity = identities[index];
    if (identity.authority_generation != 0U || identity.producer_instance_id != 0U ||
        identity.sequence != 0U || identity.source_stamp_ns != 0 ||
        identity.first_receive_stamp_ns != 0 || identity.content_fingerprint != 0U ||
        identity.full_snapshot) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::optional<std::size_t> findRejectedEvidenceIdentity(
    const std::array<ProducerEvidenceRejectedIdentity,
                     kProducerRejectedIdentityCapacity>& identities,
    const std::size_t count, const std::uint64_t authority_generation,
    const ProducerEpochObservation& observation) noexcept {
  for (std::size_t index = 0U; index < count; ++index) {
    if (identities[index].authority_generation == authority_generation &&
        identities[index].producer_instance_id == observation.producer_instance_id &&
        identities[index].sequence == observation.sequence) {
      return index;
    }
  }
  return std::nullopt;
}

[[nodiscard]] bool
sameRejectedEvidenceIdentity(const ProducerEvidenceRejectedIdentity& identity,
                             const ProducerEpochObservation& observation,
                             const bool full_snapshot) noexcept {
  return identity.source_stamp_ns == observation.source_stamp_ns &&
         identity.content_fingerprint == observation.content_fingerprint &&
         identity.full_snapshot == full_snapshot;
}

[[nodiscard]] bool rememberRejectedEvidenceIdentity(
    std::array<ProducerEvidenceRejectedIdentity, kProducerRejectedIdentityCapacity>&
        identities,
    std::size_t& count, const std::uint64_t authority_generation,
    const ProducerEpochObservation& observation, const bool full_snapshot,
    const ProducerEvidenceAdmissionStatus terminal_status) noexcept {
  if (findRejectedEvidenceIdentity(identities, count, authority_generation, observation)
          .has_value()) {
    return true;
  }
  if (count == identities.size()) {
    return false;
  }
  identities[count] = ProducerEvidenceRejectedIdentity{
      .authority_generation = authority_generation,
      .producer_instance_id = observation.producer_instance_id,
      .sequence = observation.sequence,
      .source_stamp_ns = observation.source_stamp_ns,
      .first_receive_stamp_ns = observation.receive_stamp_ns,
      .content_fingerprint = observation.content_fingerprint,
      .full_snapshot = full_snapshot,
      .terminal_status = terminal_status,
  };
  ++count;
  return true;
}

void eraseRejectedEvidenceIdentitiesThrough(
    std::array<ProducerEvidenceRejectedIdentity, kProducerRejectedIdentityCapacity>&
        identities,
    std::size_t& count, const std::uint64_t authority_generation,
    const std::uint64_t producer_instance_id, const std::uint64_t sequence) noexcept {
  std::size_t write_index{0U};
  for (std::size_t read_index = 0U; read_index < count; ++read_index) {
    const ProducerEvidenceRejectedIdentity& identity = identities[read_index];
    if (identity.authority_generation < authority_generation ||
        (identity.authority_generation == authority_generation &&
         identity.producer_instance_id == producer_instance_id &&
         identity.sequence <= sequence)) {
      continue;
    }
    if (write_index != read_index) {
      identities[write_index] = identity;
    }
    ++write_index;
  }
  for (std::size_t index = write_index; index < count; ++index) {
    identities[index] = ProducerEvidenceRejectedIdentity{};
  }
  count = write_index;
}

void clearPending(ProducerEpochAdmissionState& state) noexcept {
  state.pending_producer_instance_id = 0U;
  state.pending_sequence = 0U;
  state.pending_source_stamp_ns = 0;
  state.pending_receive_stamp_ns = 0;
  state.pending_content_fingerprint = 0U;
  state.pending_identity_conflicted = false;
}

void beginPending(ProducerEpochAdmissionState& state,
                  const ProducerEpochObservation& observation) noexcept {
  state.pending_producer_instance_id = observation.producer_instance_id;
  state.pending_sequence = observation.sequence;
  state.pending_source_stamp_ns = observation.source_stamp_ns;
  state.pending_receive_stamp_ns = observation.receive_stamp_ns;
  state.pending_content_fingerprint = observation.content_fingerprint;
  state.pending_identity_conflicted = false;
}

[[nodiscard]] bool
rememberPendingAsRejected(ProducerEpochAdmissionState& state,
                          const ProducerEpochAdmissionStatus terminal_status) noexcept {
  if (state.pending_producer_instance_id == 0U) {
    return true;
  }
  const ProducerEpochObservation pending{
      .producer_instance_id = state.pending_producer_instance_id,
      .sequence = state.pending_sequence,
      .source_stamp_ns = state.pending_source_stamp_ns,
      .receive_stamp_ns = state.pending_receive_stamp_ns,
      .content_fingerprint = state.pending_content_fingerprint,
  };
  if (rememberRejectedIdentity(state.rejected_identities, state.rejected_identity_count,
                               pending, terminal_status)) {
    return true;
  }
  state.rejected_identity_capacity_exhausted = true;
  return false;
}

void installCurrent(ProducerEpochAdmissionState& state,
                    const ProducerEpochObservation& observation) noexcept {
  state.current_producer_instance_id = observation.producer_instance_id;
  state.current_sequence = observation.sequence;
  state.current_source_stamp_ns = observation.source_stamp_ns;
  state.current_receive_stamp_ns = observation.receive_stamp_ns;
  state.current_content_fingerprint = observation.content_fingerprint;
  state.current_identity_conflicted = false;
  eraseRejectedIdentitiesThrough(
      state.rejected_identities, state.rejected_identity_count,
      observation.producer_instance_id, observation.sequence);
  clearPending(state);
}

[[nodiscard]] bool
validAdmissionState(const ProducerEpochAdmissionState& state) noexcept {
  if (state.retired_producer_count > state.retired_producer_instance_ids.size()) {
    return false;
  }
  if (!validRejectedEpochIdentities(state.rejected_identities,
                                    state.rejected_identity_count)) {
    return false;
  }
  if (state.rejected_identity_capacity_exhausted &&
      state.rejected_identity_count != state.rejected_identities.size()) {
    return false;
  }
  const bool has_current = state.current_producer_instance_id != 0U;
  if (has_current != (state.authority_generation != 0U) ||
      has_current != (state.current_sequence != 0U) ||
      has_current != (state.current_source_stamp_ns > 0) ||
      has_current != (state.current_receive_stamp_ns > 0) ||
      has_current != (state.current_content_fingerprint != 0U) ||
      (!has_current && state.current_identity_conflicted)) {
    return false;
  }
  for (std::size_t index = 0U; index < state.retired_producer_count; ++index) {
    const std::uint64_t retired = state.retired_producer_instance_ids[index];
    if (retired == 0U || retired == state.current_producer_instance_id) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (retired == state.retired_producer_instance_ids[previous]) {
        return false;
      }
    }
  }
  for (std::size_t index = state.retired_producer_count;
       index < state.retired_producer_instance_ids.size(); ++index) {
    if (state.retired_producer_instance_ids[index] != 0U) {
      return false;
    }
  }
  const bool has_pending = state.pending_producer_instance_id != 0U;
  return has_pending == (state.pending_sequence != 0U) &&
         has_pending == (state.pending_source_stamp_ns > 0) &&
         has_pending == (state.pending_receive_stamp_ns > 0) &&
         has_pending == (state.pending_content_fingerprint != 0U) &&
         (has_pending || !state.pending_identity_conflicted) &&
         (!has_pending ||
          (has_current &&
           state.pending_producer_instance_id != state.current_producer_instance_id &&
           !producerEpochRetired(state, state.pending_producer_instance_id)));
}

[[nodiscard]] bool
validEvidenceState(const ProducerEvidenceAdmissionState& state) noexcept {
  const bool initialized = state.authority_generation != 0U;
  return validRejectedEvidenceIdentities(state.rejected_identities,
                                         state.rejected_identity_count) &&
         (!state.rejected_identity_capacity_exhausted ||
          state.rejected_identity_count == state.rejected_identities.size()) &&
         initialized == (state.producer_instance_id != 0U) &&
         initialized == (state.sequence != 0U) &&
         initialized == (state.source_stamp_ns > 0) &&
         initialized == (state.receive_stamp_ns > 0) &&
         initialized == (state.content_fingerprint != 0U) &&
         (initialized || (!state.full_snapshot && !state.current_identity_conflicted));
}

void installEvidence(ProducerEvidenceAdmissionState& state,
                     const ProducerEpochAuthority& authority,
                     const ProducerEpochObservation& observation,
                     const bool full_snapshot) noexcept {
  state.authority_generation = authority.generation;
  state.producer_instance_id = observation.producer_instance_id;
  state.sequence = observation.sequence;
  state.source_stamp_ns = observation.source_stamp_ns;
  state.receive_stamp_ns = observation.receive_stamp_ns;
  state.content_fingerprint = observation.content_fingerprint;
  state.full_snapshot = full_snapshot;
  state.current_identity_conflicted = false;
  eraseRejectedEvidenceIdentitiesThrough(
      state.rejected_identities, state.rejected_identity_count, authority.generation,
      observation.producer_instance_id, observation.sequence);
}

} // namespace

bool ProducerEpochAuthority::valid() const noexcept {
  return producer_instance_id != 0U && generation != 0U;
}

bool producerEpochRetired(const ProducerEpochAdmissionState& state,
                          const std::uint64_t producer_instance_id) noexcept {
  if (producer_instance_id == 0U ||
      state.retired_producer_count > state.retired_producer_instance_ids.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < state.retired_producer_count; ++index) {
    if (state.retired_producer_instance_ids[index] == producer_instance_id) {
      return true;
    }
  }
  return false;
}

ProducerEpochAdmissionResult
admitProducerEpoch(const ProducerEpochAdmissionConfig& config,
                   const ProducerEpochAdmissionState& state,
                   const ProducerEpochObservation& observation,
                   const std::int64_t now_ns,
                   const bool observation_contract_valid) noexcept {
  ProducerEpochAdmissionResult result{.next_state = state};
  if (!validConfig(config) || !validAdmissionState(state) ||
      !claimableIdentity(observation)) {
    return result;
  }
  if (producerEpochRetired(state, observation.producer_instance_id)) {
    result.status = ProducerEpochAdmissionStatus::kRejectedRetiredProducer;
    return result;
  }
  if (state.current_producer_instance_id != 0U &&
      observation.producer_instance_id == state.current_producer_instance_id &&
      observation.sequence == state.current_sequence) {
    const bool exact =
        observation.source_stamp_ns == state.current_source_stamp_ns &&
        observation.content_fingerprint == state.current_content_fingerprint;
    if (!exact || state.current_identity_conflicted) {
      result.status = ProducerEpochAdmissionStatus::kRejectedIdentityConflict;
      result.current_identity_conflict = !state.current_identity_conflicted;
      result.next_state.current_identity_conflicted = true;
      return result;
    }
    result.status = ProducerEpochAdmissionStatus::kIdempotentReplay;
    return result;
  }
  if (state.pending_producer_instance_id != 0U &&
      observation.producer_instance_id == state.pending_producer_instance_id &&
      observation.sequence == state.pending_sequence) {
    const bool exact =
        observation.source_stamp_ns == state.pending_source_stamp_ns &&
        observation.content_fingerprint == state.pending_content_fingerprint;
    if (!exact || state.pending_identity_conflicted) {
      result.status = ProducerEpochAdmissionStatus::kRejectedIdentityConflict;
      result.next_state.pending_identity_conflicted = true;
      return result;
    }
    result.status = ProducerEpochAdmissionStatus::kPendingProducerHandoff;
    return result;
  }
  if (const std::optional<std::size_t> rejected = findRejectedIdentity(
          state.rejected_identities, state.rejected_identity_count, observation);
      rejected.has_value()) {
    const ProducerEpochRejectedIdentity& claimed = state.rejected_identities[*rejected];
    if (sameRejectedIdentity(claimed, observation)) {
      result.status = claimed.terminal_status;
      return result;
    }
    result.status = ProducerEpochAdmissionStatus::kRejectedIdentityConflict;
    if (observation.producer_instance_id == state.current_producer_instance_id) {
      result.current_identity_conflict = !state.current_identity_conflicted;
      result.next_state.current_identity_conflicted = true;
    }
    if (observation.producer_instance_id == state.pending_producer_instance_id) {
      result.next_state.pending_identity_conflicted = true;
    }
    return result;
  }
  if (state.rejected_identity_capacity_exhausted) {
    result.status = ProducerEpochAdmissionStatus::kRejectedIdentityCapacity;
    return result;
  }
  const auto reject_prospective =
      [&result, &observation](const ProducerEpochAdmissionStatus status) noexcept {
        if (result.next_state.rejected_identity_capacity_exhausted) {
          result.status = ProducerEpochAdmissionStatus::kRejectedIdentityCapacity;
          return result;
        }
        if (!rememberRejectedIdentity(result.next_state.rejected_identities,
                                      result.next_state.rejected_identity_count,
                                      observation, status)) {
          result.next_state.rejected_identity_capacity_exhausted = true;
          result.status = ProducerEpochAdmissionStatus::kRejectedIdentityCapacity;
        } else {
          result.status = status;
        }
        return result;
      };
  std::uint64_t claimed_sequence_high_water{0U};
  if (observation.producer_instance_id == state.current_producer_instance_id) {
    claimed_sequence_high_water = state.current_sequence;
  }
  if (observation.producer_instance_id == state.pending_producer_instance_id) {
    claimed_sequence_high_water =
        std::max(claimed_sequence_high_water, state.pending_sequence);
  }
  for (std::size_t index = 0U; index < state.rejected_identity_count; ++index) {
    const ProducerEpochRejectedIdentity& rejected = state.rejected_identities[index];
    if (rejected.producer_instance_id == observation.producer_instance_id) {
      claimed_sequence_high_water =
          std::max(claimed_sequence_high_water, rejected.sequence);
    }
  }
  if (observation.sequence < claimed_sequence_high_water) {
    return reject_prospective(ProducerEpochAdmissionStatus::kRejectedRegression);
  }
  if (!observation_contract_valid) {
    return reject_prospective(ProducerEpochAdmissionStatus::kRejectedInvalid);
  }
  if (!validObservationTimes(observation)) {
    return reject_prospective(ProducerEpochAdmissionStatus::kRejectedInvalid);
  }
  if (!observationFresh(config, observation, now_ns)) {
    return reject_prospective(ProducerEpochAdmissionStatus::kRejectedStaleCandidate);
  }
  if (state.current_producer_instance_id == 0U) {
    installCurrent(result.next_state, observation);
    result.next_state.authority_generation = 1U;
    result.status = ProducerEpochAdmissionStatus::kAcceptedInitial;
    result.install_observation = true;
    return result;
  }

  if (observation.producer_instance_id == state.current_producer_instance_id) {
    if (observation.source_stamp_ns <= state.current_source_stamp_ns ||
        observation.receive_stamp_ns <= state.current_receive_stamp_ns) {
      result.status = now_ns < state.current_source_stamp_ns ||
                              now_ns < state.current_receive_stamp_ns
                          ? ProducerEpochAdmissionStatus::kRejectedClockDiscontinuity
                          : ProducerEpochAdmissionStatus::kRejectedRegression;
      return reject_prospective(result.status);
    }
    installCurrent(result.next_state, observation);
    result.status = ProducerEpochAdmissionStatus::kAcceptedNewer;
    result.install_observation = true;
    return result;
  }

  if (state.retired_producer_count == state.retired_producer_instance_ids.size()) {
    result.status = ProducerEpochAdmissionStatus::kRejectedHandoffCapacity;
    return result;
  }
  if (state.authority_generation == std::numeric_limits<std::uint64_t>::max()) {
    result.status = ProducerEpochAdmissionStatus::kRejectedGenerationExhausted;
    return result;
  }
  if (now_ns < state.current_source_stamp_ns ||
      now_ns < state.current_receive_stamp_ns) {
    return reject_prospective(
        ProducerEpochAdmissionStatus::kRejectedClockDiscontinuity);
  }
  const bool current_stale =
      now_ns - state.current_source_stamp_ns > config.maximum_observation_age_ns ||
      now_ns - state.current_receive_stamp_ns > config.maximum_observation_age_ns;
  if (!current_stale) {
    return reject_prospective(
        ProducerEpochAdmissionStatus::kRejectedCurrentProducerFresh);
  }
  if (state.pending_producer_instance_id == 0U) {
    beginPending(result.next_state, observation);
    result.status = ProducerEpochAdmissionStatus::kPendingProducerHandoff;
    return result;
  }
  if (state.pending_producer_instance_id != observation.producer_instance_id) {
    if (now_ns < state.pending_receive_stamp_ns) {
      return reject_prospective(
          ProducerEpochAdmissionStatus::kRejectedClockDiscontinuity);
    }
    if (now_ns - state.pending_receive_stamp_ns >
        config.maximum_confirmation_interval_ns) {
      if (!rememberPendingAsRejected(
              result.next_state, ProducerEpochAdmissionStatus::kRejectedRegression)) {
        result.status = ProducerEpochAdmissionStatus::kRejectedIdentityCapacity;
        return result;
      }
      beginPending(result.next_state, observation);
      result.status = ProducerEpochAdmissionStatus::kPendingProducerHandoff;
      return result;
    }
    return reject_prospective(
        ProducerEpochAdmissionStatus::kRejectedPendingProducerMismatch);
  }
  if (observation.source_stamp_ns <= state.pending_source_stamp_ns ||
      observation.receive_stamp_ns <= state.pending_receive_stamp_ns) {
    return reject_prospective(ProducerEpochAdmissionStatus::kRejectedRegression);
  }
  if (state.pending_identity_conflicted ||
      observation.receive_stamp_ns - state.pending_receive_stamp_ns >
          config.maximum_confirmation_interval_ns) {
    if (!rememberPendingAsRejected(result.next_state,
                                   ProducerEpochAdmissionStatus::kRejectedRegression)) {
      result.status = ProducerEpochAdmissionStatus::kRejectedIdentityCapacity;
      return result;
    }
    beginPending(result.next_state, observation);
    result.status = ProducerEpochAdmissionStatus::kPendingProducerHandoff;
    return result;
  }

  result.next_state
      .retired_producer_instance_ids[result.next_state.retired_producer_count] =
      state.current_producer_instance_id;
  ++result.next_state.retired_producer_count;
  eraseRejectedIdentitiesThrough(
      result.next_state.rejected_identities, result.next_state.rejected_identity_count,
      state.current_producer_instance_id, std::numeric_limits<std::uint64_t>::max());
  installCurrent(result.next_state, observation);
  result.next_state.authority_generation = state.authority_generation + 1U;
  result.status = ProducerEpochAdmissionStatus::kAcceptedProducerHandoff;
  result.install_observation = true;
  result.producer_handoff = true;
  return result;
}

bool producerEpochObservationFresh(const ProducerEpochAdmissionConfig& config,
                                   const ProducerEpochObservation& observation,
                                   const std::int64_t now_ns) noexcept {
  return validConfig(config) && claimableIdentity(observation) &&
         validObservationTimes(observation) &&
         observationFresh(config, observation, now_ns);
}

ProducerEpochAuthority
producerEpochAuthority(const ProducerEpochAdmissionState& state) noexcept {
  if (!validAdmissionState(state) || state.current_identity_conflicted) {
    return {};
  }
  return ProducerEpochAuthority{
      .producer_instance_id = state.current_producer_instance_id,
      .generation = state.authority_generation,
  };
}

std::string_view
producerEpochAdmissionStatusName(const ProducerEpochAdmissionStatus status) noexcept {
  switch (status) {
    case ProducerEpochAdmissionStatus::kAcceptedInitial:
      return "accepted_initial";
    case ProducerEpochAdmissionStatus::kAcceptedNewer:
      return "accepted_newer";
    case ProducerEpochAdmissionStatus::kAcceptedProducerHandoff:
      return "accepted_producer_handoff";
    case ProducerEpochAdmissionStatus::kIdempotentReplay:
      return "idempotent_replay";
    case ProducerEpochAdmissionStatus::kPendingProducerHandoff:
      return "pending_producer_handoff";
    case ProducerEpochAdmissionStatus::kRejectedInvalid:
      return "invalid";
    case ProducerEpochAdmissionStatus::kRejectedStaleCandidate:
      return "stale_candidate";
    case ProducerEpochAdmissionStatus::kRejectedRegression:
      return "regression";
    case ProducerEpochAdmissionStatus::kRejectedIdentityConflict:
      return "identity_conflict";
    case ProducerEpochAdmissionStatus::kRejectedIdentityCapacity:
      return "identity_capacity";
    case ProducerEpochAdmissionStatus::kRejectedCurrentProducerFresh:
      return "current_producer_fresh";
    case ProducerEpochAdmissionStatus::kRejectedPendingProducerMismatch:
      return "pending_producer_mismatch";
    case ProducerEpochAdmissionStatus::kRejectedRetiredProducer:
      return "retired_producer";
    case ProducerEpochAdmissionStatus::kRejectedHandoffCapacity:
      return "handoff_capacity";
    case ProducerEpochAdmissionStatus::kRejectedClockDiscontinuity:
      return "clock_discontinuity";
    case ProducerEpochAdmissionStatus::kRejectedGenerationExhausted:
      return "generation_exhausted";
  }
  return "unknown";
}

ProducerEvidenceAdmissionResult
admitProducerEvidence(const ProducerEpochAdmissionConfig& config,
                      const ProducerEvidenceAdmissionState& state,
                      const ProducerEpochAuthority& authority,
                      const ProducerEpochObservation& observation,
                      const bool full_snapshot, const std::int64_t now_ns,
                      const bool evidence_contract_valid) noexcept {
  ProducerEvidenceAdmissionResult result{.next_state = state};
  if (!validConfig(config) || !validEvidenceState(state) || !authority.valid() ||
      !claimableIdentity(observation)) {
    return result;
  }
  if (observation.producer_instance_id != authority.producer_instance_id ||
      (state.authority_generation != 0U &&
       (authority.generation < state.authority_generation ||
        (authority.generation == state.authority_generation &&
         state.producer_instance_id != authority.producer_instance_id)))) {
    result.status = ProducerEvidenceAdmissionStatus::kRejectedAuthorityMismatch;
    return result;
  }
  const bool authority_changed = state.authority_generation == 0U ||
                                 authority.generation > state.authority_generation;
  if (!authority_changed && observation.sequence == state.sequence) {
    const bool exact = observation.source_stamp_ns == state.source_stamp_ns &&
                       observation.content_fingerprint == state.content_fingerprint &&
                       full_snapshot == state.full_snapshot;
    if (!exact || state.current_identity_conflicted) {
      result.status = ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict;
      result.current_identity_conflict = !state.current_identity_conflicted;
      result.next_state.current_identity_conflicted = true;
      return result;
    }
    result.status = ProducerEvidenceAdmissionStatus::kIdempotentReplay;
    return result;
  }
  if (const std::optional<std::size_t> rejected = findRejectedEvidenceIdentity(
          result.next_state.rejected_identities,
          result.next_state.rejected_identity_count, authority.generation, observation);
      rejected.has_value()) {
    const ProducerEvidenceRejectedIdentity& claimed =
        result.next_state.rejected_identities[*rejected];
    if (sameRejectedEvidenceIdentity(claimed, observation, full_snapshot)) {
      result.status = claimed.terminal_status;
      return result;
    }
    result.status = ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict;
    if (!authority_changed) {
      result.current_identity_conflict = !state.current_identity_conflicted;
      result.next_state.current_identity_conflicted = true;
    }
    return result;
  }
  if (state.rejected_identity_capacity_exhausted) {
    result.status = ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity;
    return result;
  }
  const auto reject_prospective =
      [&result, &observation, &authority,
       full_snapshot](const ProducerEvidenceAdmissionStatus status) noexcept {
        if (result.next_state.rejected_identity_capacity_exhausted) {
          result.status = ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity;
          return result;
        }
        if (!rememberRejectedEvidenceIdentity(result.next_state.rejected_identities,
                                              result.next_state.rejected_identity_count,
                                              authority.generation, observation,
                                              full_snapshot, status)) {
          result.next_state.rejected_identity_capacity_exhausted = true;
          result.status = ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity;
        } else {
          result.status = status;
        }
        return result;
      };
  std::uint64_t claimed_sequence_high_water{0U};
  if (state.authority_generation == authority.generation &&
      state.producer_instance_id == observation.producer_instance_id) {
    claimed_sequence_high_water = state.sequence;
  }
  for (std::size_t index = 0U; index < state.rejected_identity_count; ++index) {
    const ProducerEvidenceRejectedIdentity& rejected = state.rejected_identities[index];
    if (rejected.authority_generation == authority.generation &&
        rejected.producer_instance_id == observation.producer_instance_id) {
      claimed_sequence_high_water =
          std::max(claimed_sequence_high_water, rejected.sequence);
    }
  }
  if (observation.sequence < claimed_sequence_high_water) {
    return reject_prospective(ProducerEvidenceAdmissionStatus::kRejectedRegression);
  }
  if (!evidence_contract_valid) {
    return reject_prospective(ProducerEvidenceAdmissionStatus::kRejectedInvalid);
  }
  if (!validObservationTimes(observation)) {
    return reject_prospective(ProducerEvidenceAdmissionStatus::kRejectedInvalid);
  }
  if (!observationFresh(config, observation, now_ns)) {
    return reject_prospective(ProducerEvidenceAdmissionStatus::kRejectedStaleCandidate);
  }
  if (authority_changed) {
    if (!full_snapshot) {
      return reject_prospective(
          ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired);
    }
    installEvidence(result.next_state, authority, observation, full_snapshot);
    result.status = state.authority_generation == 0U
                        ? ProducerEvidenceAdmissionStatus::kAcceptedInitial
                        : ProducerEvidenceAdmissionStatus::kAcceptedAuthorityHandoff;
    result.install_evidence = true;
    result.authority_handoff = state.authority_generation != 0U;
    return result;
  }
  if (observation.source_stamp_ns <= state.source_stamp_ns ||
      observation.receive_stamp_ns <= state.receive_stamp_ns) {
    result.status = now_ns < state.source_stamp_ns || now_ns < state.receive_stamp_ns
                        ? ProducerEvidenceAdmissionStatus::kRejectedClockDiscontinuity
                        : ProducerEvidenceAdmissionStatus::kRejectedRegression;
    return reject_prospective(result.status);
  }
  installEvidence(result.next_state, authority, observation, full_snapshot);
  result.status = ProducerEvidenceAdmissionStatus::kAcceptedNewer;
  result.install_evidence = true;
  return result;
}

std::string_view producerEvidenceAdmissionStatusName(
    const ProducerEvidenceAdmissionStatus status) noexcept {
  switch (status) {
    case ProducerEvidenceAdmissionStatus::kAcceptedInitial:
      return "accepted_initial";
    case ProducerEvidenceAdmissionStatus::kAcceptedNewer:
      return "accepted_newer";
    case ProducerEvidenceAdmissionStatus::kAcceptedAuthorityHandoff:
      return "accepted_authority_handoff";
    case ProducerEvidenceAdmissionStatus::kIdempotentReplay:
      return "idempotent_replay";
    case ProducerEvidenceAdmissionStatus::kRejectedInvalid:
      return "invalid";
    case ProducerEvidenceAdmissionStatus::kRejectedStaleCandidate:
      return "stale_candidate";
    case ProducerEvidenceAdmissionStatus::kRejectedAuthorityMismatch:
      return "authority_mismatch";
    case ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired:
      return "full_snapshot_required";
    case ProducerEvidenceAdmissionStatus::kRejectedRegression:
      return "regression";
    case ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict:
      return "identity_conflict";
    case ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity:
      return "identity_capacity";
    case ProducerEvidenceAdmissionStatus::kRejectedClockDiscontinuity:
      return "clock_discontinuity";
  }
  return "unknown";
}

} // namespace drone_city_nav
