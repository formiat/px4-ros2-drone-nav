#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace drone_city_nav {

inline constexpr std::size_t kProducerEpochRetiredCapacity{16U};
inline constexpr std::size_t kProducerRejectedIdentityCapacity{8U};

struct ProducerEpochAdmissionConfig {
  std::int64_t maximum_observation_age_ns{5'000'000'000LL};
  std::int64_t maximum_confirmation_interval_ns{5'000'000'000LL};
};

struct ProducerEpochObservation {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t content_fingerprint{0U};
};

enum class ProducerEpochAdmissionStatus : std::uint8_t {
  kAcceptedInitial,
  kAcceptedNewer,
  kAcceptedProducerHandoff,
  kIdempotentReplay,
  kPendingProducerHandoff,
  kRejectedInvalid,
  kRejectedStaleCandidate,
  kRejectedRegression,
  kRejectedIdentityConflict,
  kRejectedIdentityCapacity,
  kRejectedCurrentProducerFresh,
  kRejectedPendingProducerMismatch,
  kRejectedRetiredProducer,
  kRejectedHandoffCapacity,
  kRejectedClockDiscontinuity,
  kRejectedGenerationExhausted,
};

struct ProducerEpochRejectedIdentity {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t first_receive_stamp_ns{0};
  std::uint64_t content_fingerprint{0U};
  ProducerEpochAdmissionStatus terminal_status{
      ProducerEpochAdmissionStatus::kRejectedInvalid};
};

struct ProducerEpochAdmissionState {
  std::uint64_t current_producer_instance_id{0U};
  std::uint64_t current_sequence{0U};
  std::int64_t current_source_stamp_ns{0};
  std::int64_t current_receive_stamp_ns{0};
  std::uint64_t current_content_fingerprint{0U};
  std::uint64_t authority_generation{0U};
  bool current_identity_conflicted{false};
  std::array<std::uint64_t, kProducerEpochRetiredCapacity>
      retired_producer_instance_ids{};
  std::size_t retired_producer_count{0U};
  std::uint64_t pending_producer_instance_id{0U};
  std::uint64_t pending_sequence{0U};
  std::int64_t pending_source_stamp_ns{0};
  std::int64_t pending_receive_stamp_ns{0};
  std::uint64_t pending_content_fingerprint{0U};
  bool pending_identity_conflicted{false};
  std::array<ProducerEpochRejectedIdentity, kProducerRejectedIdentityCapacity>
      rejected_identities{};
  std::size_t rejected_identity_count{0U};
  bool rejected_identity_capacity_exhausted{false};
};

struct ProducerEpochAuthority {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t generation{0U};

  [[nodiscard]] bool valid() const noexcept;
};

struct ProducerEpochAdmissionResult {
  ProducerEpochAdmissionState next_state{};
  ProducerEpochAdmissionStatus status{ProducerEpochAdmissionStatus::kRejectedInvalid};
  bool install_observation{false};
  bool producer_handoff{false};
  bool current_identity_conflict{false};
};

[[nodiscard]] ProducerEpochAdmissionResult
admitProducerEpoch(const ProducerEpochAdmissionConfig& config,
                   const ProducerEpochAdmissionState& state,
                   const ProducerEpochObservation& observation, std::int64_t now_ns,
                   bool observation_contract_valid = true) noexcept;

[[nodiscard]] bool
producerEpochObservationFresh(const ProducerEpochAdmissionConfig& config,
                              const ProducerEpochObservation& observation,
                              std::int64_t now_ns) noexcept;

[[nodiscard]] ProducerEpochAuthority
producerEpochAuthority(const ProducerEpochAdmissionState& state) noexcept;

[[nodiscard]] bool producerEpochRetired(const ProducerEpochAdmissionState& state,
                                        std::uint64_t producer_instance_id) noexcept;

[[nodiscard]] std::string_view
producerEpochAdmissionStatusName(ProducerEpochAdmissionStatus status) noexcept;

enum class ProducerEvidenceAdmissionStatus : std::uint8_t {
  kAcceptedInitial,
  kAcceptedNewer,
  kAcceptedAuthorityHandoff,
  kIdempotentReplay,
  kRejectedInvalid,
  kRejectedStaleCandidate,
  kRejectedAuthorityMismatch,
  kRejectedFullSnapshotRequired,
  kRejectedRegression,
  kRejectedIdentityConflict,
  kRejectedIdentityCapacity,
  kRejectedClockDiscontinuity,
};

struct ProducerEvidenceRejectedIdentity {
  std::uint64_t authority_generation{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t first_receive_stamp_ns{0};
  std::uint64_t content_fingerprint{0U};
  bool full_snapshot{false};
  ProducerEvidenceAdmissionStatus terminal_status{
      ProducerEvidenceAdmissionStatus::kRejectedInvalid};
};

struct ProducerEvidenceAdmissionState {
  std::uint64_t authority_generation{0U};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t content_fingerprint{0U};
  bool full_snapshot{false};
  bool current_identity_conflicted{false};
  std::array<ProducerEvidenceRejectedIdentity, kProducerRejectedIdentityCapacity>
      rejected_identities{};
  std::size_t rejected_identity_count{0U};
  bool rejected_identity_capacity_exhausted{false};
};

struct ProducerEvidenceAdmissionResult {
  ProducerEvidenceAdmissionState next_state{};
  ProducerEvidenceAdmissionStatus status{
      ProducerEvidenceAdmissionStatus::kRejectedInvalid};
  bool install_evidence{false};
  bool authority_handoff{false};
  bool current_identity_conflict{false};
};

[[nodiscard]] ProducerEvidenceAdmissionResult
admitProducerEvidence(const ProducerEpochAdmissionConfig& config,
                      const ProducerEvidenceAdmissionState& state,
                      const ProducerEpochAuthority& authority,
                      const ProducerEpochObservation& observation, bool full_snapshot,
                      std::int64_t now_ns,
                      bool evidence_contract_valid = true) noexcept;

[[nodiscard]] std::string_view
producerEvidenceAdmissionStatusName(ProducerEvidenceAdmissionStatus status) noexcept;

} // namespace drone_city_nav
