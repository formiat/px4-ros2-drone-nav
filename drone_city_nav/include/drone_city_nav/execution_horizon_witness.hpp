#pragma once

#include "drone_city_nav/offboard_session_admission.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

inline constexpr std::size_t kExecutionHorizonFeedbackProspectiveCapacity{8U};

enum class ExecutionHorizonWitnessMode : std::uint8_t {
  kPlanned = 0U,
  kPositionHold = 1U,
};

struct ExecutionHorizonFeedbackCandidate {
  std::uint64_t offboard_producer_instance_id{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t content_fingerprint{0U};
  // Binds the exact serialized source stamp, frame, mode, authority, identity,
  // and control values. It remains nonzero for heartbeat and malformed input.
  std::uint64_t wire_fingerprint{0U};
  ExecutionHorizonWitnessMode execution_mode{ExecutionHorizonWitnessMode::kPlanned};
  bool control_authoritative{false};

  [[nodiscard]] bool sourceIdentityClaimable() const noexcept;
  [[nodiscard]] bool heartbeat() const noexcept;
  [[nodiscard]] bool horizonFeedback() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct ExecutionHorizonFeedbackSourceIdentity {
  // candidate.receive_stamp_ns is the first consumer-local receipt for this
  // immutable wire source. Redelivery receipts never replace it.
  ExecutionHorizonFeedbackCandidate candidate{};
  bool tombstoned{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct ExecutionHorizonFeedbackProspectiveSourceIdentity {
  ExecutionHorizonFeedbackCandidate candidate{};
  // A later source may be received earlier than an older rejected source. The
  // first receipt stays in candidate while this high-water never regresses.
  std::int64_t receive_stamp_high_water_ns{0};
  bool tombstoned{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct ExecutionHorizonFeedbackWitness {
  std::uint64_t offboard_producer_instance_id{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t content_fingerprint{0U};
  std::uint64_t wire_fingerprint{0U};
  ExecutionHorizonWitnessMode execution_mode{ExecutionHorizonWitnessMode::kPlanned};
  bool control_authoritative{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct ExecutionHorizonWitnessState {
  OffboardSessionAdmissionState offboard_session{};
  std::int64_t latest_session_receive_stamp_ns{0};
  ExecutionHorizonFeedbackSourceIdentity feedback_source_identity{};
  ExecutionHorizonFeedbackWitness latest_horizon_feedback{};
  // Retains the last semantically accepted horizon lineage while a later
  // rejected source tombstones the live witness. A valid heartbeat clears it.
  ExecutionHorizonFeedbackWitness last_accepted_horizon_feedback{};
  std::int64_t horizon_feedback_source_stamp_high_water_ns{0};
  std::int64_t horizon_feedback_receive_stamp_high_water_ns{0};
  // Rejected identities from future offboard processes cannot replace the
  // active session, but must still be remembered so the same source cannot be
  // rewritten into an eligible heartbeat. Overflow latches fail closed.
  std::array<ExecutionHorizonFeedbackProspectiveSourceIdentity,
             kExecutionHorizonFeedbackProspectiveCapacity>
      prospective_feedback_source_identities{};
  std::size_t prospective_feedback_source_count{0U};
  bool prospective_feedback_source_capacity_exhausted{false};

  [[nodiscard]] bool valid() const noexcept;
};

struct ExecutionHorizonWitnessAdmissionResult {
  ExecutionHorizonWitnessState next_state{};
  bool accept{false};
  bool heartbeat{false};
  bool witness_updated{false};
  bool witness_revoked{false};
  bool session_transitioned{false};
  bool stale{false};
  bool replay{false};
  bool conflict{false};
  bool invalid{false};
  bool non_current_session{false};
  // True whenever the source high-water or its ambiguity tombstone changed.
  // Consumers must persist next_state independently of accept/revoke actions.
  bool state_advanced{false};
  bool prospective_capacity_exhausted{false};
};

[[nodiscard]] ExecutionHorizonWitnessAdmissionResult admitExecutionHorizonFeedback(
    const ExecutionHorizonWitnessState& state,
    const ExecutionHorizonFeedbackCandidate& candidate) noexcept;

// Applies identity/currentness even when a structurally coherent payload is
// unusable. An invalid payload leaves a high-water tombstone and revokes the
// installed witness without refreshing offboard-session liveness.
[[nodiscard]] ExecutionHorizonWitnessAdmissionResult
admitExecutionHorizonFeedbackPayload(const ExecutionHorizonWitnessState& state,
                                     const ExecutionHorizonFeedbackCandidate& candidate,
                                     bool payload_valid) noexcept;

// A malformed wire payload may lack coherent timing, mode, or content while
// still exposing an offboard producer, positive source stamp, and raw-wire
// fingerprint. That minimal identity is claimed before semantic gates; an
// exact redelivery cannot become valid by changing its receipt or payload.
[[nodiscard]] ExecutionHorizonWitnessAdmissionResult
revokeMalformedExecutionHorizonFeedback(
    const ExecutionHorizonWitnessState& state,
    const ExecutionHorizonFeedbackCandidate& wire_identity,
    std::int64_t receive_stamp_ns) noexcept;

struct ExecutionHorizonWitnessRequirement {
  std::uint64_t target_offboard_instance_id{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  ExecutionHorizonWitnessMode execution_mode{ExecutionHorizonWitnessMode::kPlanned};

  [[nodiscard]] bool valid() const noexcept;
};

[[nodiscard]] bool offboardSessionFreshAt(const ExecutionHorizonWitnessState& state,
                                          std::int64_t now_ns,
                                          std::int64_t maximum_age_ns) noexcept;

// A receipt proves that the current offboard process reported the exact horizon.
// It does not imply that the reported control was authoritative.
[[nodiscard]] bool
executionHorizonReceiptFreshAt(const ExecutionHorizonWitnessState& state,
                               const ExecutionHorizonWitnessRequirement& requirement,
                               std::int64_t now_ns,
                               std::int64_t maximum_age_ns) noexcept;

// An execution witness is an exact fresh receipt whose authority matches the
// horizon mode: planned control is authoritative and position hold is not.
[[nodiscard]] bool
executionHorizonWitnessFreshAt(const ExecutionHorizonWitnessState& state,
                               const ExecutionHorizonWitnessRequirement& requirement,
                               std::int64_t now_ns,
                               std::int64_t maximum_age_ns) noexcept;

} // namespace drone_city_nav
