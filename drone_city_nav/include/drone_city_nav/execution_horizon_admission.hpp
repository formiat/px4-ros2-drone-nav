#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

// Retired producer identities are never evicted. Once this bound is reached,
// another producer handoff fails closed instead of allowing an old identity to
// become admissible again.
inline constexpr std::size_t kExecutionHorizonRetiredProducerCapacity{8U};
inline constexpr std::size_t kExecutionHorizonProspectiveProducerCapacity{8U};

struct ExecutionHorizonAdmissionCandidate {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t valid_from_ns{0};
  std::uint64_t content_fingerprint{0U};

  [[nodiscard]] bool identityClaimable() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct ExecutionHorizonProspectiveIdentityClaim {
  ExecutionHorizonAdmissionCandidate candidate{};
  std::int64_t source_stamp_high_water_ns{0};
  std::int64_t valid_from_high_water_ns{0};
  bool tombstoned{false};

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] bool valid() const noexcept;
};

struct ExecutionHorizonAdmissionState {
  std::uint64_t current_producer_instance_id{0U};
  std::uint64_t current_sequence{0U};
  // These fields preserve the first wire image of the current identity even
  // when it was rejected. The separate high-waters never regress.
  std::int64_t current_source_stamp_ns{0};
  std::int64_t current_valid_from_ns{0};
  std::int64_t latest_source_stamp_ns{0};
  std::int64_t latest_valid_from_ns{0};
  std::uint64_t current_content_fingerprint{0U};
  std::array<std::uint64_t, kExecutionHorizonRetiredProducerCapacity>
      retired_producer_instance_ids{};
  std::size_t retired_producer_count{0U};
  // Ineligible handoffs remain claimed here without replacing the active
  // producer. The ledger is bounded and a first overflow latches fail closed.
  std::array<ExecutionHorizonProspectiveIdentityClaim,
             kExecutionHorizonProspectiveProducerCapacity>
      prospective_identity_claims{};
  std::size_t prospective_identity_count{0U};
  bool current_identity_tombstoned{false};
  bool prospective_identity_capacity_exhausted{false};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool producerRetired(std::uint64_t producer_instance_id) const noexcept;
};

struct ExecutionHorizonAdmissionResult {
  ExecutionHorizonAdmissionState next_state{};
  // The reducer claims the immutable wire identity before considering the
  // caller's complete payload-admissibility result. A caller must clear its
  // accepted horizon when revoke is true. A prospective producer can make
  // payload_installable true only when the complete payload is admissible.
  bool accept_identity{false};
  bool payload_installable{false};
  bool revoke{false};
  bool stale{false};
  bool replay{false};
  bool conflict{false};
  bool invalid{false};
  // Persists identity high-water/tombstone changes independently of whether a
  // prospective producer became eligible to replace the active producer.
  bool state_advanced{false};
  bool retired_capacity_exhausted{false};
  bool prospective_capacity_exhausted{false};
};

[[nodiscard]] ExecutionHorizonAdmissionResult
admitExecutionHorizonIdentity(const ExecutionHorizonAdmissionState& state,
                              const ExecutionHorizonAdmissionCandidate& candidate,
                              bool payload_admissible) noexcept;

[[nodiscard]] bool tombstoneExecutionHorizonIdentity(
    ExecutionHorizonAdmissionState& state,
    const ExecutionHorizonAdmissionCandidate& candidate) noexcept;

} // namespace drone_city_nav
