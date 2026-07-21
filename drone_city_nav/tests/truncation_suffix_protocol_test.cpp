#include "drone_city_nav/truncation_suffix_protocol.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

constexpr TruncationSuffixIdentity kExpected{
    .path_id = 5U, .generation = 4U, .prefix_fingerprint = 123U};

TEST(TruncationSuffixProtocol, KeepsWaitingForMatchingPendingAck) {
  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, kExpected, TruncationSuffixAckDecision::kPending);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kKeepWaiting);
  EXPECT_STREQ(result.reason, "matching_pending");
}

TEST(TruncationSuffixProtocol, AdoptsMatchingAcceptedAck) {
  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, kExpected, TruncationSuffixAckDecision::kAccepted);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kAdopt);
}

TEST(TruncationSuffixProtocol, RetriesMatchingRejectedAck) {
  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, kExpected, TruncationSuffixAckDecision::kRejected);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kRetry);
}

TEST(TruncationSuffixProtocol, IgnoresAckForOldPath) {
  TruncationSuffixIdentity received = kExpected;
  received.path_id = 3U;

  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, received, TruncationSuffixAckDecision::kAccepted);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kIgnore);
  EXPECT_STREQ(result.reason, "path_id_mismatch");
}

TEST(TruncationSuffixProtocol, IgnoresAckForOldGeneration) {
  TruncationSuffixIdentity received = kExpected;
  received.generation = 3U;

  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, received, TruncationSuffixAckDecision::kRejected);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kIgnore);
  EXPECT_STREQ(result.reason, "generation_mismatch");
}

TEST(TruncationSuffixProtocol, IgnoresAckForDifferentPrefix) {
  TruncationSuffixIdentity received = kExpected;
  received.prefix_fingerprint = 456U;

  const TruncationSuffixAckEvaluation result = evaluateTruncationSuffixAck(
      kExpected, received, TruncationSuffixAckDecision::kAccepted);

  EXPECT_EQ(result.action, TruncationSuffixAckAction::kIgnore);
  EXPECT_STREQ(result.reason, "prefix_fingerprint_mismatch");
}

TEST(TruncationSuffixProtocol, RejectsUnknownDecisionValue) {
  EXPECT_FALSE(truncationSuffixAckDecisionFromValue(99U).has_value());
}

} // namespace
} // namespace drone_city_nav
