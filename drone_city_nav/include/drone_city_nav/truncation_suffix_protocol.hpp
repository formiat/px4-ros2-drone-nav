#pragma once

#include <cstdint>
#include <optional>

namespace drone_city_nav {

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

[[nodiscard]] std::optional<TruncationSuffixAckDecision>
truncationSuffixAckDecisionFromValue(std::uint8_t value) noexcept;

[[nodiscard]] const char*
truncationSuffixAckDecisionName(TruncationSuffixAckDecision decision) noexcept;

[[nodiscard]] TruncationSuffixAckEvaluation
evaluateTruncationSuffixAck(const TruncationSuffixIdentity& expected,
                            const TruncationSuffixIdentity& received,
                            TruncationSuffixAckDecision decision) noexcept;

} // namespace drone_city_nav
