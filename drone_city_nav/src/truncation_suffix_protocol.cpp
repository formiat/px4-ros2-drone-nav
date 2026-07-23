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
                                      const bool temporary_hold_reached) noexcept {
  return temporary_hold_reached ? TruncationSuffixActivationMode::kAfterHold
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
