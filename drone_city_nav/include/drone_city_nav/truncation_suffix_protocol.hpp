#pragma once

#include <cstdint>
#include <optional>

namespace drone_city_nav {

enum class TruncationSuffixActivationMode : std::uint8_t {
  kMovingJoin = 0U,
  kAfterHold = 1U,
};

enum class TruncationSuffixAckDecision : std::uint8_t {
  kPending = 0U,
  kAccepted = 1U,
  kRejected = 2U,
};

enum class TruncationSuffixAckAction {
  kIgnore,
  kKeepWaiting,
  kAdopt,
  kRetry,
};

struct TruncationSuffixIdentity {
  std::uint64_t path_id{0U};
  std::uint64_t generation{0U};
  std::uint64_t prefix_fingerprint{0U};
};

struct TruncationSuffixAckEvaluation {
  TruncationSuffixAckAction action{TruncationSuffixAckAction::kIgnore};
  const char* reason{"not_evaluated"};
};

struct TruncationSuffixPublicationContext {
  std::uint64_t generation{0U};
  std::uint64_t prefix_fingerprint{0U};
  bool confirmed{false};
  bool awaiting_ack{false};
};

struct TruncationSuffixPublicationEvaluation {
  bool allowed{false};
  const char* reason{"not_evaluated"};
};

[[nodiscard]] std::optional<TruncationSuffixAckDecision>
truncationSuffixAckDecisionFromValue(std::uint8_t value) noexcept;

[[nodiscard]] std::optional<TruncationSuffixActivationMode>
truncationSuffixActivationModeFromValue(std::uint8_t value) noexcept;

[[nodiscard]] const char*
truncationSuffixActivationModeName(TruncationSuffixActivationMode mode) noexcept;

[[nodiscard]] TruncationSuffixActivationMode
resolveTruncationSuffixActivationMode(TruncationSuffixActivationMode planned_mode,
                                      bool temporary_hold_reached) noexcept;

[[nodiscard]] const char*
truncationSuffixAckDecisionName(TruncationSuffixAckDecision decision) noexcept;

[[nodiscard]] TruncationSuffixAckEvaluation
evaluateTruncationSuffixAck(const TruncationSuffixIdentity& expected,
                            const TruncationSuffixIdentity& received,
                            TruncationSuffixAckDecision decision) noexcept;

[[nodiscard]] TruncationSuffixPublicationEvaluation
evaluateTruncationSuffixPublication(const TruncationSuffixPublicationContext& context,
                                    const TruncationSuffixIdentity& candidate) noexcept;

} // namespace drone_city_nav
