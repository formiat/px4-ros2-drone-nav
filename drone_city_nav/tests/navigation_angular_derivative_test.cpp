#include "drone_city_nav/navigation_angular_derivative.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] NavigationAngularObservation
sample(const std::uint64_t sample_timestamp_us,
       const std::uint64_t publication_timestamp_us, const double yaw_rad,
       const std::uint8_t heading_reset_counter = 0U,
       const bool angular_state_authoritative = true,
       const std::int64_t receive_timestamp_ns = 0,
       const std::uint64_t source_payload_fingerprint = 0U) {
  const std::uint64_t receive_source_us =
      publication_timestamp_us != 0U ? publication_timestamp_us : sample_timestamp_us;
  return NavigationAngularObservation{
      .sample_timestamp_us = sample_timestamp_us,
      .publication_timestamp_us = publication_timestamp_us,
      .receive_timestamp_ns =
          receive_timestamp_ns != 0
              ? receive_timestamp_ns
              : static_cast<std::int64_t>(receive_source_us * 1000U),
      .yaw_rad = yaw_rad,
      .source_payload_fingerprint = source_payload_fingerprint,
      .heading_reset_counter = heading_reset_counter,
      .angular_state_authoritative = angular_state_authoritative,
  };
}

[[nodiscard]] NavigationAngularDerivativeConfig permissiveConfig() {
  return NavigationAngularDerivativeConfig{
      .minimum_interval_s = 0.001,
      .maximum_interval_s = 0.1,
      .maximum_yaw_rate_radps = 10.0,
      .maximum_yaw_acceleration_radps2 = 1000.0,
  };
}

TEST(Px4TimestampEpochAdmissionTest,
     OrdinaryReorderIsRejectedWithoutMovingTheHighWater) {
  const Px4TimestampEpochAdmissionConfig config;
  Px4TimestampEpochAdmissionState state;
  const Px4TimestampEpochAdmissionResult initial = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 10'000'000U,
                                   .receive_timestamp_ns = 20'000'000'000});
  ASSERT_TRUE(px4TimestampEpochAdmissionAccepted(initial.status));
  state = initial.next_state;

  const Px4TimestampEpochAdmissionResult reordered = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 9'900'000U,
                                   .receive_timestamp_ns = 20'010'000'000});
  EXPECT_EQ(reordered.status,
            Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicPrimaryTimestamp);
  EXPECT_EQ(reordered.next_state.primary_timestamp_high_water_us, 10'000'000U);
  EXPECT_EQ(reordered.next_state.pending_confirmation_count, 0U);
  state = reordered.next_state;

  const Px4TimestampEpochAdmissionResult recovered = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 10'010'000U,
                                   .receive_timestamp_ns = 20'020'000'000});
  EXPECT_EQ(recovered.status, Px4TimestampEpochAdmissionStatus::kAcceptedNewer);
  EXPECT_EQ(recovered.next_state.primary_timestamp_high_water_us, 10'010'000U);
}

TEST(Px4TimestampEpochAdmissionTest,
     FastEpochResetNeedsThreeSamplesAndRejectsOldEpochReplay) {
  const Px4TimestampEpochAdmissionConfig config;
  Px4TimestampEpochAdmissionState state;
  state = admitPx4TimestampEpoch(
              config, state,
              Px4TimestampEpochObservation{.primary_timestamp_us = 10'000'000U,
                                           .receive_timestamp_ns = 20'000'000'000})
              .next_state;

  const Px4TimestampEpochAdmissionResult first_low = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 100'000U,
                                   .receive_timestamp_ns = 20'010'000'000});
  ASSERT_EQ(first_low.status, Px4TimestampEpochAdmissionStatus::kPendingEpochReset);
  ASSERT_EQ(first_low.next_state.primary_timestamp_high_water_us, 10'000'000U);
  state = first_low.next_state;

  const Px4TimestampEpochAdmissionResult second_low = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 110'000U,
                                   .receive_timestamp_ns = 20'020'000'000});
  ASSERT_EQ(second_low.status, Px4TimestampEpochAdmissionStatus::kPendingEpochReset);
  ASSERT_EQ(second_low.next_state.primary_timestamp_high_water_us, 10'000'000U);
  state = second_low.next_state;

  const Px4TimestampEpochAdmissionResult confirmed = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 120'000U,
                                   .receive_timestamp_ns = 20'030'000'000});
  ASSERT_EQ(confirmed.status, Px4TimestampEpochAdmissionStatus::kAcceptedEpochReset);
  ASSERT_TRUE(confirmed.epoch_reset);
  ASSERT_EQ(confirmed.next_state.primary_timestamp_high_water_us, 120'000U);
  state = confirmed.next_state;

  const Px4TimestampEpochAdmissionResult retired_epoch_replay = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 9'000'000U,
                                   .receive_timestamp_ns = 20'040'000'000});
  EXPECT_EQ(retired_epoch_replay.status,
            Px4TimestampEpochAdmissionStatus::kRejectedImplausibleTimestampProgress);
  EXPECT_EQ(retired_epoch_replay.next_state.primary_timestamp_high_water_us, 120'000U);

  const Px4TimestampEpochAdmissionResult new_epoch_progress = admitPx4TimestampEpoch(
      config, retired_epoch_replay.next_state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 130'000U,
                                   .receive_timestamp_ns = 20'050'000'000});
  EXPECT_EQ(new_epoch_progress.status,
            Px4TimestampEpochAdmissionStatus::kAcceptedNewer);
}

TEST(Px4TimestampEpochAdmissionTest,
     SingleDeepReorderIsCancelledByCurrentEpochProgress) {
  const Px4TimestampEpochAdmissionConfig config;
  Px4TimestampEpochAdmissionState state;
  state = admitPx4TimestampEpoch(
              config, state,
              Px4TimestampEpochObservation{.primary_timestamp_us = 10'000'000U,
                                           .receive_timestamp_ns = 20'000'000'000})
              .next_state;
  const Px4TimestampEpochAdmissionResult deep_reorder = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 8'000'000U,
                                   .receive_timestamp_ns = 20'010'000'000});
  ASSERT_EQ(deep_reorder.status, Px4TimestampEpochAdmissionStatus::kPendingEpochReset);
  ASSERT_EQ(deep_reorder.next_state.pending_confirmation_count, 1U);

  const Px4TimestampEpochAdmissionResult current_epoch = admitPx4TimestampEpoch(
      config, deep_reorder.next_state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 10'010'000U,
                                   .receive_timestamp_ns = 20'020'000'000});
  EXPECT_EQ(current_epoch.status, Px4TimestampEpochAdmissionStatus::kAcceptedNewer);
  EXPECT_FALSE(current_epoch.epoch_reset);
  EXPECT_EQ(current_epoch.next_state.pending_confirmation_count, 0U);
}

TEST(Px4TimestampEpochAdmissionTest,
     ForwardClockCorrectionRebasesAfterBoundedProbation) {
  const Px4TimestampEpochAdmissionConfig config;
  Px4TimestampEpochAdmissionState state;
  state = admitPx4TimestampEpoch(
              config, state,
              Px4TimestampEpochObservation{.primary_timestamp_us = 1'000'000U,
                                           .receive_timestamp_ns = 10'000'000'000})
              .next_state;

  const Px4TimestampEpochAdmissionResult first = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 1'316'500U,
                                   .receive_timestamp_ns = 10'010'000'000});
  EXPECT_EQ(first.status,
            Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition);
  EXPECT_EQ(first.next_state.primary_timestamp_high_water_us, 1'000'000U);
  state = first.next_state;

  const Px4TimestampEpochAdmissionResult second = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 1'326'500U,
                                   .receive_timestamp_ns = 10'020'000'000});
  EXPECT_EQ(second.status,
            Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition);
  state = second.next_state;

  const Px4TimestampEpochAdmissionResult confirmed = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 1'336'500U,
                                   .receive_timestamp_ns = 10'030'000'000});
  EXPECT_EQ(confirmed.status,
            Px4TimestampEpochAdmissionStatus::kAcceptedForwardReacquisition);
  EXPECT_TRUE(confirmed.forward_reacquisition);
  EXPECT_EQ(confirmed.next_state.primary_timestamp_high_water_us, 1'336'500U);
}

TEST(Px4TimestampEpochAdmissionTest,
     LongGapOldEpochReplayCannotSilentlyReacquireAfterReset) {
  Px4TimestampEpochAdmissionConfig config;
  config.maximum_post_reset_unprobated_receive_gap_s = 0.5;
  Px4TimestampEpochAdmissionState state;
  state = admitPx4TimestampEpoch(
              config, state,
              Px4TimestampEpochObservation{.primary_timestamp_us = 10'000'000U,
                                           .receive_timestamp_ns = 20'000'000'000})
              .next_state;
  for (std::size_t index = 0U; index < 3U; ++index) {
    const Px4TimestampEpochAdmissionResult reset = admitPx4TimestampEpoch(
        config, state,
        Px4TimestampEpochObservation{
            .primary_timestamp_us = 100'000U + index * 10'000U,
            .receive_timestamp_ns =
                20'010'000'000 + static_cast<std::int64_t>(index) * 10'000'000});
    EXPECT_EQ(reset.status,
              index < 2U ? Px4TimestampEpochAdmissionStatus::kPendingEpochReset
                         : Px4TimestampEpochAdmissionStatus::kAcceptedEpochReset);
    state = reset.next_state;
  }
  ASSERT_EQ(state.primary_timestamp_high_water_us, 120'000U);
  ASSERT_TRUE(state.post_reset_replay_guard_active);

  const Px4TimestampEpochAdmissionResult delayed_old_epoch = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 9'000'000U,
                                   .receive_timestamp_ns = 30'030'000'000});
  EXPECT_EQ(delayed_old_epoch.status,
            Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition);
  EXPECT_FALSE(px4TimestampEpochAdmissionAccepted(delayed_old_epoch.status));
  EXPECT_EQ(delayed_old_epoch.next_state.primary_timestamp_high_water_us, 120'000U);
  state = delayed_old_epoch.next_state;

  const std::array<std::uint64_t, 3U> new_epoch_timestamps{130'000U, 140'000U,
                                                           150'000U};
  for (std::size_t index = 0U; index < new_epoch_timestamps.size(); ++index) {
    const Px4TimestampEpochAdmissionResult reacquisition = admitPx4TimestampEpoch(
        config, state,
        Px4TimestampEpochObservation{
            .primary_timestamp_us = new_epoch_timestamps[index],
            .receive_timestamp_ns =
                30'040'000'000 + static_cast<std::int64_t>(index) * 10'000'000});
    if (index + 1U < new_epoch_timestamps.size()) {
      EXPECT_EQ(reacquisition.status,
                Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition);
      EXPECT_FALSE(reacquisition.forward_reacquisition);
    } else {
      EXPECT_EQ(reacquisition.status,
                Px4TimestampEpochAdmissionStatus::kAcceptedForwardReacquisition);
      EXPECT_TRUE(reacquisition.forward_reacquisition);
      EXPECT_EQ(reacquisition.next_state.primary_timestamp_high_water_us, 150'000U);
    }
    state = reacquisition.next_state;
  }

  const Px4TimestampEpochAdmissionResult second_old_epoch = admitPx4TimestampEpoch(
      config, state,
      Px4TimestampEpochObservation{.primary_timestamp_us = 9'010'000U,
                                   .receive_timestamp_ns = 30'070'000'000});
  EXPECT_EQ(second_old_epoch.status,
            Px4TimestampEpochAdmissionStatus::kRejectedImplausibleTimestampProgress);
  EXPECT_EQ(second_old_epoch.next_state.primary_timestamp_high_water_us, 150'000U);
}

TEST(NavigationAngularDerivativeTest, ValidatesPhysicalAndIntervalConfiguration) {
  EXPECT_TRUE(navigationAngularDerivativeConfigIsValid(permissiveConfig()));

  NavigationAngularDerivativeConfig config = permissiveConfig();
  config.minimum_interval_s = 0.0;
  EXPECT_FALSE(navigationAngularDerivativeConfigIsValid(config));
  config = permissiveConfig();
  config.maximum_interval_s = 0.0005;
  EXPECT_FALSE(navigationAngularDerivativeConfigIsValid(config));
  config = permissiveConfig();
  config.maximum_yaw_rate_radps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(navigationAngularDerivativeConfigIsValid(config));
  config = permissiveConfig();
  config.maximum_yaw_acceleration_radps2 = 0.0;
  EXPECT_FALSE(navigationAngularDerivativeConfigIsValid(config));
  config = permissiveConfig();
  config.timestamp_epoch_admission.epoch_reset_confirmation_samples = 1U;
  EXPECT_FALSE(navigationAngularDerivativeConfigIsValid(config));
  config = permissiveConfig();
  config.timestamp_epoch_admission.maximum_post_reset_unprobated_receive_gap_s = 0.0;
  EXPECT_FALSE(navigationAngularDerivativeConfigIsValid(config));
}

TEST(NavigationLocalStateResetAssessmentTest,
     InitialCountersAreBaselineAndOriginOrEpochResetNeedsCompensation) {
  const NavigationLocalStateResetCounters baseline{
      .xy = 1U, .z = 2U, .vxy = 3U, .vz = 4U, .heading = 5U};
  const NavigationLocalStateResetCounters initial_nonzero{
      .xy = 9U, .z = 8U, .vxy = 7U, .vz = 6U, .heading = 5U};
  const NavigationLocalStateResetAssessment initial =
      assessNavigationLocalStateReset(baseline, initial_nonzero, false, false);
  EXPECT_FALSE(initial.state_lineage_reset);
  EXPECT_FALSE(initial.frame_compensation_required);

  NavigationLocalStateResetCounters reset = baseline;
  ++reset.xy;
  NavigationLocalStateResetAssessment assessment =
      assessNavigationLocalStateReset(baseline, reset, true, false);
  EXPECT_TRUE(assessment.state_lineage_reset);
  EXPECT_TRUE(assessment.frame_compensation_required);

  reset = baseline;
  ++reset.z;
  assessment = assessNavigationLocalStateReset(baseline, reset, true, false);
  EXPECT_TRUE(assessment.state_lineage_reset);
  EXPECT_TRUE(assessment.frame_compensation_required);

  assessment = assessNavigationLocalStateReset(baseline, baseline, true, true);
  EXPECT_FALSE(assessment.state_lineage_reset);
  EXPECT_TRUE(assessment.frame_compensation_required);
}

TEST(NavigationLocalStateResetAssessmentTest,
     VelocityAndHeadingResetsOnlyBreakDynamicLineage) {
  const NavigationLocalStateResetCounters baseline{
      .xy = 1U, .z = 2U, .vxy = 3U, .vz = 4U, .heading = 5U};
  using ResetCounterMember = std::uint8_t NavigationLocalStateResetCounters::*;
  constexpr std::array<ResetCounterMember, 3U> kDynamicResetCounters{
      &NavigationLocalStateResetCounters::vxy,
      &NavigationLocalStateResetCounters::vz,
      &NavigationLocalStateResetCounters::heading,
  };
  for (std::size_t index = 0U; index < kDynamicResetCounters.size(); ++index) {
    SCOPED_TRACE(index);
    NavigationLocalStateResetCounters reset = baseline;
    ++(reset.*kDynamicResetCounters[index]);
    const NavigationLocalStateResetAssessment assessment =
        assessNavigationLocalStateReset(baseline, reset, true, false);
    EXPECT_TRUE(assessment.state_lineage_reset);
    EXPECT_FALSE(assessment.frame_compensation_required);
  }
}

TEST(NavigationAngularDerivativeTest, UsesShortestAngleAcrossPiWrap) {
  NavigationAngularDerivativeConfig config = permissiveConfig();
  config.maximum_yaw_rate_radps = 3.0;
  NavigationAngularDerivativeEstimator estimator{config};

  const NavigationAngularDerivativeEstimate first =
      estimator.observe(sample(1'000'000U, 2'000'000U, 3.13));
  const NavigationAngularDerivativeEstimate second =
      estimator.observe(sample(1'010'000U, 2'010'000U, -3.13));

  EXPECT_TRUE(navigationAngularUpdateAccepted(first.status));
  EXPECT_FALSE(first.yaw_rate_authoritative);
  ASSERT_TRUE(second.yaw_rate_authoritative);
  const double expected_rate = std::remainder(-6.26, 2.0 * std::numbers::pi) / 0.01;
  EXPECT_NEAR(second.yaw_rate_radps, expected_rate, 1.0e-5);
  EXPECT_GT(second.yaw_rate_radps, 0.0F);
}

TEST(NavigationAngularDerivativeTest,
     PublicationFallbackAcceptsStateButResetsSampleAuthority) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};

  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));
  const NavigationAngularDerivativeEstimate authoritative_rate =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.01));
  ASSERT_TRUE(authoritative_rate.yaw_rate_authoritative);

  const NavigationAngularDerivativeEstimate fallback =
      estimator.observe(NavigationAngularObservation{
          .publication_timestamp_us = 2'020'000U,
          .receive_timestamp_ns = 2'020'000'000,
          .yaw_rad = 0.02,
          .angular_state_authoritative = true,
      });
  EXPECT_EQ(fallback.status,
            NavigationAngularUpdateStatus::kAcceptedPublicationFallback);
  EXPECT_EQ(fallback.provenance, NavigationTimestampProvenance::kPublication);
  EXPECT_EQ(fallback.source_timestamp_us, 2'020'000U);
  EXPECT_FALSE(fallback.yaw_rate_authoritative);
  EXPECT_FALSE(fallback.yaw_acceleration_authoritative);

  const NavigationAngularDerivativeEstimate resumed_seed =
      estimator.observe(sample(1'020'000U, 2'030'000U, 0.02));
  EXPECT_FALSE(resumed_seed.yaw_rate_authoritative);
  const NavigationAngularDerivativeEstimate resumed_rate =
      estimator.observe(sample(1'030'000U, 2'040'000U, 0.03));
  EXPECT_TRUE(resumed_rate.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest, RejectsIntervalsOutsideConfiguredBounds) {
  NavigationAngularDerivativeEstimator too_short{permissiveConfig()};
  (void)too_short.observe(sample(1'000'000U, 2'000'000U, 0.0));
  const NavigationAngularDerivativeEstimate short_interval =
      too_short.observe(sample(1'000'500U, 2'001'000U, 0.0005));
  EXPECT_FALSE(short_interval.yaw_rate_authoritative);
  const NavigationAngularDerivativeEstimate recovered_short =
      too_short.observe(sample(1'010'500U, 2'011'000U, 0.0105));
  EXPECT_TRUE(recovered_short.yaw_rate_authoritative);

  NavigationAngularDerivativeEstimator too_long{permissiveConfig()};
  (void)too_long.observe(sample(1'000'000U, 2'000'000U, 0.0));
  const NavigationAngularDerivativeEstimate long_interval =
      too_long.observe(sample(1'200'000U, 2'200'000U, 0.2));
  EXPECT_FALSE(long_interval.yaw_rate_authoritative);
  const NavigationAngularDerivativeEstimate recovered_long =
      too_long.observe(sample(1'210'000U, 2'210'000U, 0.21));
  EXPECT_TRUE(recovered_long.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest, HeadingResetCounterChangeReseedsChain) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0, 4U));

  const NavigationAngularDerivativeEstimate reset_sample =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.01, 5U));
  EXPECT_FALSE(reset_sample.yaw_rate_authoritative);
  const NavigationAngularDerivativeEstimate recovered =
      estimator.observe(sample(1'020'000U, 2'020'000U, 0.02, 5U));
  EXPECT_TRUE(recovered.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest,
     EveryLocalStateResetCounterChangeReseedsDerivativeChain) {
  using ResetCounterMember = std::uint8_t NavigationAngularObservation::*;
  constexpr std::array<ResetCounterMember, 5U> kResetCounters{
      &NavigationAngularObservation::xy_reset_counter,
      &NavigationAngularObservation::z_reset_counter,
      &NavigationAngularObservation::vxy_reset_counter,
      &NavigationAngularObservation::vz_reset_counter,
      &NavigationAngularObservation::heading_reset_counter,
  };

  for (std::size_t index = 0U; index < kResetCounters.size(); ++index) {
    SCOPED_TRACE(index);
    NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
    (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));
    const NavigationAngularDerivativeEstimate established_rate =
        estimator.observe(sample(1'010'000U, 2'010'000U, 0.01));
    ASSERT_TRUE(established_rate.yaw_rate_authoritative);

    NavigationAngularObservation reset = sample(1'020'000U, 2'020'000U, 0.02);
    reset.*kResetCounters[index] = 1U;
    const NavigationAngularDerivativeEstimate reset_seed = estimator.observe(reset);
    EXPECT_EQ(reset_seed.status, NavigationAngularUpdateStatus::kAcceptedSample);
    EXPECT_FALSE(reset_seed.yaw_rate_authoritative);
    EXPECT_FALSE(reset_seed.yaw_acceleration_authoritative);

    NavigationAngularObservation continued = sample(1'030'000U, 2'030'000U, 0.03);
    continued.*kResetCounters[index] = 1U;
    const NavigationAngularDerivativeEstimate recovered = estimator.observe(continued);
    EXPECT_TRUE(recovered.yaw_rate_authoritative);
  }
}

TEST(NavigationAngularDerivativeTest, ImplausibleYawRateReseedsFailClosed) {
  NavigationAngularDerivativeConfig config = permissiveConfig();
  config.maximum_yaw_rate_radps = 1.0;
  NavigationAngularDerivativeEstimator estimator{config};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));

  const NavigationAngularDerivativeEstimate implausible =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.1));
  EXPECT_FALSE(implausible.yaw_rate_authoritative);
  EXPECT_FALSE(implausible.yaw_acceleration_authoritative);
  const NavigationAngularDerivativeEstimate recovered =
      estimator.observe(sample(1'020'000U, 2'020'000U, 0.101));
  ASSERT_TRUE(recovered.yaw_rate_authoritative);
  EXPECT_NEAR(recovered.yaw_rate_radps, 0.1F, 1.0e-5F);
}

TEST(NavigationAngularDerivativeTest,
     YawAccelerationUsesTimeBetweenRateIntervalMidpoints) {
  NavigationAngularDerivativeConfig config = permissiveConfig();
  config.maximum_yaw_acceleration_radps2 = 100.0;
  NavigationAngularDerivativeEstimator estimator{config};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));

  const NavigationAngularDerivativeEstimate first_rate =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.01));
  ASSERT_TRUE(first_rate.yaw_rate_authoritative);
  EXPECT_FALSE(first_rate.yaw_acceleration_authoritative);
  const NavigationAngularDerivativeEstimate second_rate =
      estimator.observe(sample(1'030'000U, 2'030'000U, 0.05));

  ASSERT_TRUE(second_rate.yaw_acceleration_authoritative);
  EXPECT_NEAR(second_rate.yaw_rate_radps, 2.0F, 1.0e-5F);
  EXPECT_NEAR(second_rate.yaw_acceleration_radps2, 1.0 / 0.015, 1.0e-3);
  EXPECT_GT(std::abs(second_rate.yaw_acceleration_radps2 - 50.0F), 10.0F);
}

TEST(NavigationAngularDerivativeTest,
     ImplausibleYawAccelerationDoesNotLeakAndCanRecover) {
  NavigationAngularDerivativeConfig config = permissiveConfig();
  config.maximum_yaw_rate_radps = 6.0;
  config.maximum_yaw_acceleration_radps2 = 2.0;
  NavigationAngularDerivativeEstimator estimator{config};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));
  (void)estimator.observe(sample(1'010'000U, 2'010'000U, 0.01));

  const NavigationAngularDerivativeEstimate implausible_acceleration =
      estimator.observe(sample(1'020'000U, 2'020'000U, 0.06));
  EXPECT_TRUE(implausible_acceleration.yaw_rate_authoritative);
  EXPECT_FALSE(implausible_acceleration.yaw_acceleration_authoritative);
  EXPECT_FLOAT_EQ(implausible_acceleration.yaw_acceleration_radps2, 0.0F);

  const NavigationAngularDerivativeEstimate recovered =
      estimator.observe(sample(1'030'000U, 2'030'000U, 0.11));
  ASSERT_TRUE(recovered.yaw_acceleration_authoritative);
  EXPECT_NEAR(recovered.yaw_acceleration_radps2, 0.0F, 1.0e-3F);
}

TEST(NavigationAngularDerivativeTest,
     NonmonotonicSampleTimestampIsRejectedWithoutBreakingChain) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));
  const NavigationAngularDerivativeEstimate regression =
      estimator.observe(sample(1'000'000U, 2'010'000U, 0.01));
  EXPECT_EQ(regression.status,
            NavigationAngularUpdateStatus::kRejectedNonmonotonicSampleTimestamp);

  const NavigationAngularDerivativeEstimate continued =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.01, 0U, true, 2'015'000'000));
  EXPECT_TRUE(continued.yaw_rate_authoritative);
  const NavigationAngularDerivativeEstimate recovered =
      estimator.observe(sample(1'020'000U, 2'020'000U, 0.02));
  EXPECT_TRUE(recovered.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest,
     NonmonotonicPublicationTimestampIsRejectedIndependently) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));
  const NavigationAngularDerivativeEstimate regression =
      estimator.observe(sample(1'010'000U, 2'000'000U, 0.01, 0U, true, 2'010'000'000));
  EXPECT_EQ(regression.status,
            NavigationAngularUpdateStatus::kRejectedNonmonotonicPublicationTimestamp);

  const NavigationAngularDerivativeEstimate continued =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.01, 0U, true, 2'020'000'000));
  EXPECT_TRUE(continued.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest,
     SameTimestampPayloadConflictIsQuarantinedUntilStrictlyNewerSample) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  const NavigationAngularDerivativeEstimate initial = estimator.observe(
      sample(1'000'000U, 2'000'000U, 0.0, 0U, true, 2'000'000'000, 0x11U));
  ASSERT_EQ(initial.status, NavigationAngularUpdateStatus::kAcceptedSample);

  const NavigationAngularDerivativeEstimate exact_replay = estimator.observe(
      sample(1'000'000U, 2'000'000U, 0.0, 0U, true, 2'010'000'000, 0x11U));
  EXPECT_EQ(exact_replay.status,
            NavigationAngularUpdateStatus::kIdempotentSourceIdentityDuplicate);
  EXPECT_FALSE(exact_replay.source_identity_conflicted);

  const NavigationAngularDerivativeEstimate conflict = estimator.observe(
      sample(1'000'000U, 2'000'000U, 0.1, 0U, true, 2'020'000'000, 0x22U));
  EXPECT_EQ(conflict.status,
            NavigationAngularUpdateStatus::kRejectedSourceIdentityConflict);
  EXPECT_TRUE(conflict.source_identity_conflict);
  EXPECT_TRUE(conflict.source_identity_conflicted);

  const NavigationAngularDerivativeEstimate original_replay = estimator.observe(
      sample(1'000'000U, 2'000'000U, 0.0, 0U, true, 2'030'000'000, 0x11U));
  EXPECT_EQ(original_replay.status,
            NavigationAngularUpdateStatus::kRejectedSourceIdentityQuarantined);
  EXPECT_FALSE(original_replay.source_identity_conflict);
  EXPECT_TRUE(original_replay.source_identity_conflicted);

  const NavigationAngularDerivativeEstimate newer_seed = estimator.observe(
      sample(1'010'000U, 2'010'000U, 0.01, 0U, true, 2'040'000'000, 0x33U));
  EXPECT_EQ(newer_seed.status, NavigationAngularUpdateStatus::kAcceptedSample);
  EXPECT_FALSE(newer_seed.source_identity_conflicted);
  EXPECT_FALSE(newer_seed.yaw_rate_authoritative);

  const NavigationAngularDerivativeEstimate recovered = estimator.observe(
      sample(1'020'000U, 2'020'000U, 0.02, 0U, true, 2'050'000'000, 0x44U));
  EXPECT_EQ(recovered.status, NavigationAngularUpdateStatus::kAcceptedSample);
  EXPECT_TRUE(recovered.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest,
     ExactSourceIdentityReplayDoesNotBreakDerivativeContinuity) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(
      sample(1'000'000U, 2'000'000U, 0.0, 0U, true, 2'000'000'000, 0x11U));
  const NavigationAngularDerivativeEstimate established = estimator.observe(
      sample(1'010'000U, 2'010'000U, 0.01, 0U, true, 2'010'000'000, 0x22U));
  ASSERT_TRUE(established.yaw_rate_authoritative);

  const NavigationAngularDerivativeEstimate duplicate = estimator.observe(
      sample(1'010'000U, 2'010'000U, 0.01, 0U, true, 2'020'000'000, 0x22U));
  EXPECT_EQ(duplicate.status,
            NavigationAngularUpdateStatus::kIdempotentSourceIdentityDuplicate);

  const NavigationAngularDerivativeEstimate continued = estimator.observe(
      sample(1'020'000U, 2'020'000U, 0.02, 0U, true, 2'030'000'000, 0x33U));
  EXPECT_EQ(continued.status, NavigationAngularUpdateStatus::kAcceptedSample);
  EXPECT_TRUE(continued.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest,
     FastPx4EpochResetRecoversWithConfirmedNonDerivativeSeed) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(
      sample(10'000'000U, 11'000'000U, 0.0, 0U, true, 20'000'000'000));
  const NavigationAngularDerivativeEstimate old_epoch_rate = estimator.observe(
      sample(10'010'000U, 11'010'000U, 0.01, 0U, true, 20'010'000'000));
  ASSERT_TRUE(old_epoch_rate.yaw_rate_authoritative);

  const NavigationAngularDerivativeEstimate first_low =
      estimator.observe(sample(100'000U, 200'000U, 0.1, 0U, true, 20'020'000'000));
  EXPECT_EQ(first_low.status,
            NavigationAngularUpdateStatus::kRejectedTimestampEpochResetPending);
  const NavigationAngularDerivativeEstimate second_low =
      estimator.observe(sample(110'000U, 210'000U, 0.11, 0U, true, 20'030'000'000));
  EXPECT_EQ(second_low.status,
            NavigationAngularUpdateStatus::kRejectedTimestampEpochResetPending);

  const NavigationAngularDerivativeEstimate reset_seed =
      estimator.observe(sample(120'000U, 220'000U, 0.12, 0U, true, 20'040'000'000));
  ASSERT_EQ(reset_seed.status,
            NavigationAngularUpdateStatus::kAcceptedTimestampEpochReset);
  EXPECT_TRUE(reset_seed.timestamp_epoch_reset);
  EXPECT_EQ(reset_seed.provenance, NavigationTimestampProvenance::kSample);
  EXPECT_EQ(reset_seed.source_timestamp_us, 120'000U);
  EXPECT_FALSE(reset_seed.yaw_rate_authoritative);
  EXPECT_FALSE(reset_seed.yaw_acceleration_authoritative);

  const NavigationAngularDerivativeEstimate recovered =
      estimator.observe(sample(130'000U, 230'000U, 0.13, 0U, true, 20'050'000'000));
  ASSERT_EQ(recovered.status, NavigationAngularUpdateStatus::kAcceptedSample);
  ASSERT_TRUE(recovered.yaw_rate_authoritative);
  EXPECT_NEAR(recovered.yaw_rate_radps, 1.0F, 1.0e-5F);
}

TEST(NavigationAngularDerivativeTest,
     DeepOutOfOrderSampleDoesNotResetWhenCurrentEpochResumes) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(
      sample(10'000'000U, 11'000'000U, 0.0, 0U, true, 20'000'000'000));
  const NavigationAngularDerivativeEstimate established_rate = estimator.observe(
      sample(10'010'000U, 11'010'000U, 0.01, 0U, true, 20'010'000'000));
  ASSERT_TRUE(established_rate.yaw_rate_authoritative);

  const NavigationAngularDerivativeEstimate deep_reorder =
      estimator.observe(sample(8'000'000U, 9'000'000U, 0.1, 0U, true, 20'020'000'000));
  EXPECT_EQ(deep_reorder.status,
            NavigationAngularUpdateStatus::kRejectedTimestampEpochResetPending);
  EXPECT_FALSE(deep_reorder.timestamp_epoch_reset);
  EXPECT_FALSE(navigationAngularUpdateAccepted(deep_reorder.status));
  const NavigationAngularDerivativeEstimate current_epoch = estimator.observe(
      sample(10'020'000U, 11'020'000U, 0.02, 0U, true, 20'030'000'000));
  EXPECT_EQ(current_epoch.status, NavigationAngularUpdateStatus::kAcceptedSample);
  EXPECT_FALSE(current_epoch.timestamp_epoch_reset);
  EXPECT_TRUE(current_epoch.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest,
     LongGapOldEpochPoseNeedsCorroboratedReacquisition) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(
      sample(10'000'000U, 11'000'000U, 0.0, 0U, true, 20'000'000'000));
  (void)estimator.observe(
      sample(10'010'000U, 11'010'000U, 0.01, 0U, true, 20'010'000'000));
  (void)estimator.observe(sample(100'000U, 200'000U, 0.1, 0U, true, 20'020'000'000));
  (void)estimator.observe(sample(110'000U, 210'000U, 0.11, 0U, true, 20'030'000'000));
  const NavigationAngularDerivativeEstimate reset_seed =
      estimator.observe(sample(120'000U, 220'000U, 0.12, 0U, true, 20'040'000'000));
  ASSERT_TRUE(reset_seed.timestamp_epoch_reset);

  const NavigationAngularDerivativeEstimate delayed_old_epoch =
      estimator.observe(sample(9'000'000U, 10'000'000U, 0.5, 0U, true, 30'040'000'000));
  EXPECT_EQ(delayed_old_epoch.status,
            NavigationAngularUpdateStatus::kRejectedTimestampReacquisitionPending);
  EXPECT_FALSE(delayed_old_epoch.timestamp_epoch_reset);
  EXPECT_FALSE(delayed_old_epoch.yaw_rate_authoritative);

  const NavigationAngularDerivativeEstimate first_new =
      estimator.observe(sample(130'000U, 230'000U, 0.13, 0U, true, 30'050'000'000));
  const NavigationAngularDerivativeEstimate second_new =
      estimator.observe(sample(140'000U, 240'000U, 0.14, 0U, true, 30'060'000'000));
  const NavigationAngularDerivativeEstimate recovered =
      estimator.observe(sample(150'000U, 250'000U, 0.15, 0U, true, 30'070'000'000));
  EXPECT_EQ(first_new.status,
            NavigationAngularUpdateStatus::kRejectedTimestampReacquisitionPending);
  EXPECT_EQ(second_new.status,
            NavigationAngularUpdateStatus::kRejectedTimestampReacquisitionPending);
  EXPECT_EQ(recovered.status, NavigationAngularUpdateStatus::kAcceptedSample);
  EXPECT_TRUE(recovered.yaw_rate_authoritative);
  EXPECT_NEAR(recovered.yaw_rate_radps, 1.0F, 1.0e-5F);
}

TEST(NavigationAngularDerivativeTest, InvalidAngularInputBreaksDerivativeChain) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));
  const NavigationAngularDerivativeEstimate invalid =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.01, 0U, false));
  EXPECT_TRUE(navigationAngularUpdateAccepted(invalid.status));
  EXPECT_FALSE(invalid.yaw_rate_authoritative);

  const NavigationAngularDerivativeEstimate new_seed =
      estimator.observe(sample(1'020'000U, 2'020'000U, 0.02));
  EXPECT_FALSE(new_seed.yaw_rate_authoritative);
}

TEST(NavigationAngularDerivativeTest,
     MissingTimestampsAreRejectedWithoutMutatingChain) {
  NavigationAngularDerivativeEstimator estimator{permissiveConfig()};
  (void)estimator.observe(sample(1'000'000U, 2'000'000U, 0.0));
  const NavigationAngularDerivativeEstimate missing =
      estimator.observe(NavigationAngularObservation{
          .yaw_rad = 0.01, .angular_state_authoritative = true});
  EXPECT_EQ(missing.status, NavigationAngularUpdateStatus::kRejectedMissingTimestamp);

  const NavigationAngularDerivativeEstimate continued =
      estimator.observe(sample(1'010'000U, 2'010'000U, 0.01));
  EXPECT_TRUE(continued.yaw_rate_authoritative);
}

} // namespace
} // namespace drone_city_nav
