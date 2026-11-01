#include "drone_city_nav/gnss_trust.hpp"

#include <gtest/gtest.h>

#include "visual_inertial_gnss_input.hpp"

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecond{1'000'000'000};

[[nodiscard]] GnssFixReading agreeingFix(const std::int64_t stamp_ns) {
  return {.stamp_ns = stamp_ns,
          .valid = true,
          .position_residual_normalised = 0.5,
          .velocity_residual_normalised = 0.5,
          .self_disagreement_mps = 0.1,
          .position_step_m = 0.1,
          .clock_step_s = 0.033};
}

[[nodiscard]] GnssTrustConfig gates() {
  return {.window_s = 5.0,
          .position_gate = 3.0,
          .velocity_gate = 3.0,
          .self_gate_mps = 1.0,
          .jump_gate_m = 5.0,
          .clock_step_gate_s = 1.0};
}

TEST(GnssTrustTest, TheSignalIsTrustedOnceItHasAgreedForTheWindowAndNotBefore) {
  GnssTrust trust{gates()};
  for (int i = 0; i < 149; ++i) {
    const GnssTrustVerdict verdict = trust.read(agreeingFix(i * kSecond / 30));
    EXPECT_TRUE(verdict.agreeing);
    EXPECT_FALSE(verdict.trusted) << i;
  }
  const GnssTrustVerdict verdict = trust.read(agreeingFix(150 * kSecond / 30));
  EXPECT_TRUE(verdict.trusted);
  EXPECT_NEAR(verdict.agreeing_s, 5.0, 1.0e-9);
}

TEST(GnssTrustTest, ADisagreeingFixEndsTheTrustAtOnceAndNamesItsGate) {
  GnssTrust trust{gates()};
  for (int i = 0; i <= 150; ++i) {
    static_cast<void>(trust.read(agreeingFix(i * kSecond / 30)));
  }
  ASSERT_TRUE(trust.trusted());
  GnssFixReading walked = agreeingFix(151 * kSecond / 30);
  walked.position_residual_normalised = 3.5;
  const GnssTrustVerdict verdict = trust.read(walked);
  EXPECT_FALSE(verdict.agreeing);
  EXPECT_FALSE(verdict.trusted);
  EXPECT_EQ(verdict.reason, "position");
  GnssFixReading jumped = agreeingFix(152 * kSecond / 30);
  jumped.position_step_m = 80.0;
  EXPECT_EQ(trust.read(jumped).reason, "jump");
  GnssFixReading lost = agreeingFix(153 * kSecond / 30);
  lost.valid = false;
  EXPECT_EQ(trust.read(lost).reason, "no_fix");
  GnssFixReading late = agreeingFix(154 * kSecond / 30);
  late.clock_step_s = -0.5;
  EXPECT_EQ(trust.read(late).reason, "clock");
  GnssFixReading frozen = agreeingFix(154 * kSecond / 30 + 1);
  frozen.clock_step_s = 0.0;
  EXPECT_EQ(trust.read(frozen).reason, "clock");
  GnssFixReading inconsistent = agreeingFix(155 * kSecond / 30);
  inconsistent.self_disagreement_mps = 4.0;
  EXPECT_EQ(trust.read(inconsistent).reason, "self");
}

TEST(GnssTrustTest, TheTrustReturnsAfterTheWindowOfAgreementAndRemembersNoLie) {
  GnssTrust trust{gates()};
  for (int i = 0; i <= 150; ++i) {
    static_cast<void>(trust.read(agreeingFix(i * kSecond / 30)));
  }
  GnssFixReading walked = agreeingFix(151 * kSecond / 30);
  walked.position_residual_normalised = 10.0;
  static_cast<void>(trust.read(walked));
  // Back in agreement: the window counts from the first agreeing fix after the lie.
  const std::int64_t back_ns = 152 * kSecond / 30;
  for (int i = 0; i < 149; ++i) {
    EXPECT_FALSE(trust.read(agreeingFix(back_ns + i * kSecond / 30)).trusted) << i;
  }
  EXPECT_TRUE(trust.read(agreeingFix(back_ns + 150 * kSecond / 30)).trusted);
}

TEST(GnssTrustTest, NoFixWithinTheClockGateEndsTheTrustOncePerOutage) {
  // The law is read per fix and an outage brings none (r2637 flew 90 s of
  // "lost" with the trust still declared): the input reads the trust with no
  // fix once the clock gate has passed since the last one.
  GnssInputConfig config;
  config.trust = gates();
  config.reference_fixes = 1U;
  GnssInput input{config};
  GnssFixSample sample{.stamp_ns = 0,
                       .valid = true,
                       .latitude_deg = 37.4,
                       .longitude_deg = -121.9,
                       .altitude_m = 40.0,
                       .horizontal_sigma_m = 0.9,
                       .vertical_sigma_m = 1.5,
                       .speed_sigma_mps = 0.3};
  for (int i = 0; i <= 160; ++i) {
    sample.stamp_ns = i * kSecond / 30;
    sample.arrival_ns = sample.stamp_ns + kSecond / 10; // the relay's latency
    static_cast<void>(input.fix(sample, false, nullptr));
  }
  ASSERT_TRUE(input.trusted());
  const std::int64_t last_ns = sample.arrival_ns;
  EXPECT_FALSE(input.expire(last_ns + kSecond / 4).has_value());
  const std::optional<GnssTrustVerdict> gone = input.expire(last_ns + 2 * kSecond);
  ASSERT_TRUE(gone.has_value());
  EXPECT_FALSE(gone->trusted);
  EXPECT_EQ(gone->reason, "no_fix");
  EXPECT_FALSE(input.trusted());
  EXPECT_FALSE(input.expire(last_ns + 3 * kSecond).has_value());
  // The fixes back: the first steps the clock, the agreement restarts, the
  // trust returns after the window, and a new outage is read again.
  for (int i = 0; i <= 151; ++i) {
    sample.stamp_ns = last_ns + 90 * kSecond + i * kSecond / 30;
    sample.arrival_ns = sample.stamp_ns + kSecond / 10;
    const GnssInputDecision decision = input.fix(sample, false, nullptr);
    if (i == 0) {
      EXPECT_EQ(decision.verdict.reason, "clock");
    }
    EXPECT_EQ(decision.verdict.trusted, i >= 151) << i;
  }
  EXPECT_TRUE(input.expire(sample.arrival_ns + 2 * kSecond).has_value());
}

TEST(GnssTrustTest, TheEstimatorsDriftWhileUnaidedWidensTheResidualsGate) {
  // After a walked fix ended (r2645) the honest fix stood 2.7 m from the
  // estimate that had drifted while unaided, 1.0 normalised against the
  // filter's 0.06 m, and the trust never returned: the path flown since the
  // last fused fix lends the normalisation the measured drift, and a fix back
  // at an offset beyond it (return_offset) stays rejected.
  GnssInputConfig config;
  config.trust = gates();
  config.trust.position_gate = 0.70;
  config.reference_fixes = 1U;
  GnssInput input{config};
  VisualInertialEstimate predicted;
  predicted.position_variance_m2 = Eigen::Vector3d::Constant(0.06 * 0.06);
  GnssFixSample sample{.stamp_ns = 0,
                       .valid = true,
                       .latitude_deg = 37.4,
                       .longitude_deg = -121.9,
                       .altitude_m = 40.0,
                       .horizontal_sigma_m = 0.9,
                       .vertical_sigma_m = 1.5,
                       .speed_sigma_mps = 0.3};
  static_cast<void>(input.fix(sample, false, nullptr)); // the reference
  for (int i = 1; i <= 160; ++i) {
    sample.stamp_ns = i * kSecond / 30;
    sample.arrival_ns = sample.stamp_ns;
    predicted.stamp_ns = sample.stamp_ns;
    static_cast<void>(input.fix(sample, true, &predicted));
  }
  ASSERT_TRUE(input.trusted());
  // An outage of 90 s over 135 m of flight: the estimate drifted 2 m against
  // the honest fix that returns (the fix at the origin's reference, the
  // estimate 2 m north of it).
  ASSERT_TRUE(input.expire(sample.arrival_ns + 2 * kSecond).has_value());
  predicted.position_ned_m = Eigen::Vector3d{137.0, 0.0, 0.0};
  sample.latitude_deg = 37.4 + 135.0 / 111'000.0; // 135 m north
  for (int i = 0; i <= 151; ++i) {
    sample.stamp_ns = 160 * kSecond / 30 + 90 * kSecond + i * kSecond / 30;
    sample.arrival_ns = sample.stamp_ns;
    predicted.stamp_ns = sample.stamp_ns;
    const GnssInputDecision decision = input.fix(sample, true, &predicted);
    if (i == 1) {
      EXPECT_NEAR(decision.unaided_drift_m, 0.04 * 137.0, 1.0e-6);
      EXPECT_LT(decision.reading.position_residual_normalised, 0.70);
      EXPECT_TRUE(decision.verdict.agreeing);
    }
    EXPECT_EQ(decision.verdict.trusted, i >= 151) << i;
  }
  // Fused again: the allowance is spent, and a fix 50 m off is a disagreement.
  sample.stamp_ns += kSecond / 30;
  sample.arrival_ns = sample.stamp_ns;
  sample.latitude_deg += 50.0 / 111'000.0;
  const GnssInputDecision offset = input.fix(sample, true, &predicted);
  EXPECT_FALSE(offset.verdict.agreeing);
  EXPECT_LT(offset.unaided_drift_m, 0.01);
}

TEST(GnssTrustTest, AGateAtInfinityNeverFails) {
  GnssTrust trust{GnssTrustConfig{.window_s = 1.0}};
  GnssFixReading wild = agreeingFix(0);
  wild.position_residual_normalised = 1.0e6;
  wild.position_step_m = 1.0e6;
  EXPECT_TRUE(trust.read(wild).agreeing);
  wild.stamp_ns = kSecond;
  EXPECT_TRUE(trust.read(wild).trusted);
}

} // namespace
} // namespace drone_city_nav
