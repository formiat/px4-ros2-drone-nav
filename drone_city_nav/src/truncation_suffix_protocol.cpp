#include "drone_city_nav/truncation_suffix_protocol.hpp"

namespace drone_city_nav {

std::optional<TruncationSuffixAckDecision>
truncationSuffixAckDecisionFromValue(const std::uint8_t value) noexcept {
  switch (value) {
    case static_cast<std::uint8_t>(TruncationSuffixAckDecision::kPending):
      return TruncationSuffixAckDecision::kPending;
    case static_cast<std::uint8_t>(TruncationSuffixAckDecision::kAccepted):
      return TruncationSuffixAckDecision::kAccepted;
    case static_cast<std::uint8_t>(TruncationSuffixAckDecision::kRejected):
      return TruncationSuffixAckDecision::kRejected;
    default:
      return std::nullopt;
  }
}

std::optional<TruncationSuffixActivationMode>
truncationSuffixActivationModeFromValue(const std::uint8_t value) noexcept {
  switch (value) {
    case static_cast<std::uint8_t>(TruncationSuffixActivationMode::kMovingJoin):
      return TruncationSuffixActivationMode::kMovingJoin;
    case static_cast<std::uint8_t>(TruncationSuffixActivationMode::kAfterHold):
      return TruncationSuffixActivationMode::kAfterHold;
    default:
      return std::nullopt;
  }
}

const char*
truncationSuffixActivationModeName(const TruncationSuffixActivationMode mode) noexcept {
  switch (mode) {
    case TruncationSuffixActivationMode::kMovingJoin:
      return "moving_join";
    case TruncationSuffixActivationMode::kAfterHold:
      return "after_hold";
  }
  return "unknown";
}

TruncationSuffixActivationMode
resolveTruncationSuffixActivationMode(const TruncationSuffixActivationMode planned_mode,
                                      const bool temporary_hold_active) noexcept {
  return temporary_hold_active ? TruncationSuffixActivationMode::kAfterHold
                               : planned_mode;
}

const char*
truncationSuffixAckDecisionName(const TruncationSuffixAckDecision decision) noexcept {
  switch (decision) {
    case TruncationSuffixAckDecision::kPending:
      return "pending";
    case TruncationSuffixAckDecision::kAccepted:
      return "accepted";
    case TruncationSuffixAckDecision::kRejected:
      return "rejected";
  }
  return "unknown";
}

TruncationSuffixAckEvaluation
evaluateTruncationSuffixAck(const TruncationSuffixIdentity& expected,
                            const TruncationSuffixIdentity& received,
                            const TruncationSuffixAckDecision decision) noexcept {
  if (expected.path_id == 0U || expected.generation == 0U ||
      expected.prefix_fingerprint == 0U || received.path_id == 0U ||
      received.generation == 0U || received.prefix_fingerprint == 0U) {
    return {TruncationSuffixAckAction::kIgnore, "invalid_identity"};
  }
  if (received.path_id != expected.path_id) {
    return {TruncationSuffixAckAction::kIgnore, "path_id_mismatch"};
  }
  if (received.generation != expected.generation) {
    return {TruncationSuffixAckAction::kIgnore, "generation_mismatch"};
  }
  if (received.prefix_fingerprint != expected.prefix_fingerprint) {
    return {TruncationSuffixAckAction::kIgnore, "prefix_fingerprint_mismatch"};
  }

  switch (decision) {
    case TruncationSuffixAckDecision::kPending:
      return {TruncationSuffixAckAction::kKeepWaiting, "matching_pending"};
    case TruncationSuffixAckDecision::kAccepted:
      return {TruncationSuffixAckAction::kAdopt, "matching_accepted"};
    case TruncationSuffixAckDecision::kRejected:
      return {TruncationSuffixAckAction::kRetry, "matching_rejected"};
  }
  return {TruncationSuffixAckAction::kIgnore, "invalid_decision"};
}

bool trajectoryActivationAckRequired(
    const TrajectoryActivationAckContract& contract) noexcept {
  return contract.explicitly_required || contract.truncation_suffix ||
         contract.activate_after_terminal_hold ||
         contract.endpoint_semantics == TrajectoryEndpointSemantics::kLocalHorizon;
}

TruncationSuffixAckEvaluation
evaluateOrdinaryTrajectoryAck(const std::uint64_t expected_path_id,
                              const std::uint64_t received_path_id,
                              const TruncationSuffixAckDecision decision) noexcept {
  if (expected_path_id == 0U || received_path_id == 0U) {
    return {TruncationSuffixAckAction::kIgnore, "invalid_identity"};
  }
  if (received_path_id != expected_path_id) {
    return {TruncationSuffixAckAction::kIgnore, "path_id_mismatch"};
  }
  switch (decision) {
    case TruncationSuffixAckDecision::kPending:
      return {TruncationSuffixAckAction::kKeepWaiting, "matching_pending"};
    case TruncationSuffixAckDecision::kAccepted:
      return {TruncationSuffixAckAction::kAdopt, "matching_accepted"};
    case TruncationSuffixAckDecision::kRejected:
      return {TruncationSuffixAckAction::kRetry, "matching_rejected"};
  }
  return {TruncationSuffixAckAction::kIgnore, "invalid_decision"};
}

bool trajectoryAckClearsPending(const TruncationSuffixAckAction action) noexcept {
  return action == TruncationSuffixAckAction::kAdopt ||
         action == TruncationSuffixAckAction::kRetry;
}

bool terminalHoldAllowsDeferredActivation(const bool temporary_replan_hold_active,
                                          const bool final_goal_hold_active) noexcept {
  return temporary_replan_hold_active || final_goal_hold_active;
}

TruncationSuffixPublicationEvaluation evaluateTruncationSuffixPublication(
    const TruncationSuffixPublicationContext& context,
    const TruncationSuffixIdentity& candidate) noexcept {
  if (!context.confirmed) {
    return {false, "not_confirmed"};
  }
  if (context.generation == 0U || context.prefix_fingerprint == 0U ||
      candidate.path_id == 0U || candidate.generation == 0U ||
      candidate.prefix_fingerprint == 0U) {
    return {false, "invalid_identity"};
  }
  if (candidate.generation != context.generation) {
    return {false, "generation_mismatch"};
  }
  if (candidate.prefix_fingerprint != context.prefix_fingerprint) {
    return {false, "prefix_fingerprint_mismatch"};
  }
  if (context.awaiting_ack) {
    return {false, "already_awaiting_ack"};
  }
  return {true, "allowed"};
}

} // namespace drone_city_nav
