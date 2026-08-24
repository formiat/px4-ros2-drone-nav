#pragma once

#include "drone_city_nav/execution_horizon_witness.hpp"

#include <cstdint>

namespace drone_city_nav {

struct AppliedControlExecutionOwner {
  std::uint64_t offboard_producer_instance_id{0U};
  std::uint64_t horizon_producer_instance_id{0U};
  std::uint64_t horizon_sequence{0U};
  ExecutionHorizonWitnessMode execution_mode{ExecutionHorizonWitnessMode::kPlanned};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool
  matches(const ExecutionHorizonFeedbackCandidate& candidate) const noexcept;
};

struct AppliedControlAdmissionResult {
  ExecutionHorizonWitnessAdmissionResult feedback{};
  bool install_control{false};
  bool revoke_control{false};
};

// Reduces heartbeat and applied-control evidence before the node mutates its
// control latch. The embedded witness state owns the per-session timestamp
// high-water and ambiguity tombstone. Every accepted non-owning successor
// and every claimed invalid control payload revokes the old latch instead of
// merely refreshing session liveness.
[[nodiscard]] AppliedControlAdmissionResult
admitAppliedControlEvidence(const ExecutionHorizonWitnessState& state,
                            const ExecutionHorizonFeedbackCandidate& candidate,
                            const AppliedControlExecutionOwner& owner,
                            bool control_payload_valid) noexcept;

} // namespace drone_city_nav
