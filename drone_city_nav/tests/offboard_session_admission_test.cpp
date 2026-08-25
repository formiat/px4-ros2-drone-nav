#include "drone_city_nav/offboard_session_admission.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {
namespace {

[[nodiscard]] OffboardSessionCandidate
candidate(const std::uint64_t producer_instance_id,
          const std::int64_t source_stamp_ns) {
  return {
      .producer_instance_id = producer_instance_id,
      .source_stamp_ns = source_stamp_ns,
  };
}

[[nodiscard]] OffboardSessionAdmissionState initialState() {
  const OffboardSessionAdmissionResult result =
      admitOffboardSession({}, candidate(7U, 100));
  EXPECT_TRUE(result.accept);
  EXPECT_TRUE(result.transitioned);
  EXPECT_FALSE(result.stale);
  EXPECT_FALSE(result.invalid);
  return result.next_state;
}

TEST(OffboardSessionAdmissionTest, AdmitsInitialAndCurrentHeartbeats) {
  const OffboardSessionAdmissionState initial = initialState();

  const OffboardSessionAdmissionResult duplicate =
      admitOffboardSession(initial, candidate(7U, 100));
  EXPECT_TRUE(duplicate.accept);
  EXPECT_FALSE(duplicate.transitioned);
  EXPECT_TRUE(duplicate.replay);
  EXPECT_FALSE(duplicate.stale);
  EXPECT_FALSE(duplicate.invalid);

  const OffboardSessionAdmissionResult newer =
      admitOffboardSession(duplicate.next_state, candidate(7U, 101));
  EXPECT_TRUE(newer.accept);
  EXPECT_FALSE(newer.transitioned);
  EXPECT_FALSE(newer.replay);
  EXPECT_EQ(newer.next_state.latest_source_stamp_ns, 101);
}

TEST(OffboardSessionAdmissionTest, RejectsInvalidAndNonmonotonicHeartbeats) {
  const OffboardSessionAdmissionState state = initialState();

  for (const OffboardSessionCandidate invalid_candidate :
       {candidate(0U, 101), candidate(7U, 0)}) {
    const OffboardSessionAdmissionResult result =
        admitOffboardSession(state, invalid_candidate);
    EXPECT_FALSE(result.accept);
    EXPECT_FALSE(result.transitioned);
    EXPECT_FALSE(result.stale);
    EXPECT_TRUE(result.invalid);
  }

  const OffboardSessionAdmissionResult stale =
      admitOffboardSession(state, candidate(7U, 99));
  EXPECT_FALSE(stale.accept);
  EXPECT_FALSE(stale.transitioned);
  EXPECT_TRUE(stale.stale);
  EXPECT_FALSE(stale.invalid);
}

TEST(OffboardSessionAdmissionTest,
     NewProducerRequiresNewerStampAndRetiresPreviousProducer) {
  const OffboardSessionAdmissionState initial = initialState();
  const OffboardSessionAdmissionResult equal_stamp =
      admitOffboardSession(initial, candidate(8U, 100));
  EXPECT_FALSE(equal_stamp.accept);
  EXPECT_TRUE(equal_stamp.stale);

  const OffboardSessionAdmissionResult handoff =
      admitOffboardSession(initial, candidate(8U, 101));
  ASSERT_TRUE(handoff.accept);
  EXPECT_TRUE(handoff.transitioned);
  ASSERT_EQ(handoff.next_state.retired_producer_count, 1U);
  EXPECT_EQ(handoff.next_state.retired_producer_instance_ids.front(), 7U);

  const OffboardSessionAdmissionResult replay =
      admitOffboardSession(handoff.next_state, candidate(7U, 1'000));
  EXPECT_FALSE(replay.accept);
  EXPECT_FALSE(replay.transitioned);
  EXPECT_TRUE(replay.stale);
  EXPECT_FALSE(replay.invalid);
}

TEST(OffboardSessionAdmissionTest, RetirementCapacityFailsClosedWithoutEviction) {
  OffboardSessionAdmissionState state = initialState();
  for (std::size_t transition = 0U; transition < kRetiredOffboardSessionCapacity;
       ++transition) {
    const OffboardSessionAdmissionResult handoff = admitOffboardSession(
        state, candidate(static_cast<std::uint64_t>(transition) + 8U,
                         101 + static_cast<std::int64_t>(transition)));
    ASSERT_TRUE(handoff.accept);
    ASSERT_TRUE(handoff.transitioned);
    state = handoff.next_state;
  }
  ASSERT_EQ(state.retired_producer_count, kRetiredOffboardSessionCapacity);

  const OffboardSessionAdmissionResult overflow =
      admitOffboardSession(state, candidate(99U, 10'000));
  EXPECT_FALSE(overflow.accept);
  EXPECT_FALSE(overflow.transitioned);
  EXPECT_FALSE(overflow.stale);
  EXPECT_TRUE(overflow.invalid);
  EXPECT_EQ(overflow.next_state.current_producer_instance_id,
            state.current_producer_instance_id);
  EXPECT_EQ(overflow.next_state.retired_producer_instance_ids,
            state.retired_producer_instance_ids);

  const OffboardSessionAdmissionResult oldest_replay =
      admitOffboardSession(state, candidate(7U, 20'000));
  EXPECT_FALSE(oldest_replay.accept);
  EXPECT_TRUE(oldest_replay.stale);
}

TEST(OffboardSessionAdmissionTest,
     PublicationAcceptsNewerHeartbeatFromTheCapturedProducer) {
  const OffboardSessionAdmissionState captured = initialState();
  const OffboardSessionAdmissionResult advanced =
      admitOffboardSession(captured, candidate(7U, 110));
  ASSERT_TRUE(advanced.accept);

  EXPECT_EQ(assessOffboardSessionPublicationCurrentness(
                advanced.next_state, 1'010, captured, 1'000, 7U, 2'000, 1.0),
            OffboardSessionPublicationCurrentnessStatus::kCurrent);
  EXPECT_STREQ(offboardSessionPublicationCurrentnessStatusName(
                   OffboardSessionPublicationCurrentnessStatus::kCurrent),
               "current");
}

TEST(OffboardSessionAdmissionTest,
     PublicationRejectsHeartbeatRegressionAndProducerHandoff) {
  const OffboardSessionAdmissionState captured = initialState();
  OffboardSessionAdmissionState source_regressed = captured;
  source_regressed.latest_source_stamp_ns = 99;
  EXPECT_EQ(assessOffboardSessionPublicationCurrentness(
                source_regressed, 1'000, captured, 1'000, 7U, 2'000, 1.0),
            OffboardSessionPublicationCurrentnessStatus::kSourceRegressed);
  EXPECT_EQ(assessOffboardSessionPublicationCurrentness(captured, 999, captured, 1'000,
                                                        7U, 2'000, 1.0),
            OffboardSessionPublicationCurrentnessStatus::kReceiveRegressed);

  const OffboardSessionAdmissionResult handoff =
      admitOffboardSession(captured, candidate(8U, 110));
  ASSERT_TRUE(handoff.accept);
  ASSERT_TRUE(handoff.transitioned);
  EXPECT_EQ(assessOffboardSessionPublicationCurrentness(
                handoff.next_state, 1'010, captured, 1'000, 7U, 2'000, 1.0),
            OffboardSessionPublicationCurrentnessStatus::kProducerChanged);
}

TEST(OffboardSessionAdmissionTest, PublicationRejectsFutureAndStaleHeartbeatEvidence) {
  const OffboardSessionAdmissionState captured = initialState();
  EXPECT_EQ(assessOffboardSessionPublicationCurrentness(captured, 1'100, captured,
                                                        1'000, 7U, 1'050, 1.0),
            OffboardSessionPublicationCurrentnessStatus::kFromFuture);
  EXPECT_EQ(assessOffboardSessionPublicationCurrentness(captured, 1'000, captured,
                                                        1'000, 7U, 2'000'001, 1.0),
            OffboardSessionPublicationCurrentnessStatus::kStale);
}

} // namespace
} // namespace drone_city_nav
