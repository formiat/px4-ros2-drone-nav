#include "drone_city_nav/simulation_truth_alignment.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace drone_city_nav {
namespace {

[[nodiscard]] TimedVehicleState state(const double x, const std::int64_t stamp_ns) {
  return TimedVehicleState{
      .position = Point3{x, 20.0, 5.0},
      .velocity = Vec3{1.0, 0.0, 0.0},
      .stamp_ns = stamp_ns,
      .position_valid = true,
      .velocity_valid = true,
  };
}

[[nodiscard]] SimulationTruthAlignmentConfig testConfig() {
  return SimulationTruthAlignmentConfig{
      .maximum_position_error_m = 2.0,
      .maximum_state_age_s = 0.5,
      .maximum_time_alignment_s = 0.15,
      .failure_confirmation_s = 1.0,
      .readiness_confirmation_samples = 3U,
  };
}

TEST(SimulationTruthAlignmentTest, RequiresConsecutiveAlignedSamples) {
  SimulationTruthAlignmentMonitor monitor{testConfig()};
  std::array<SimulationTruthAlignmentSample, 1> samples;
  for (std::int64_t tick = 1; tick <= 3; ++tick) {
    const std::int64_t stamp_ns = tick * 100'000'000LL;
    samples[0] = SimulationTruthAlignmentSample{
        .navigation = state(10.0, stamp_ns),
        .physical_truth = state(10.2, stamp_ns),
    };
    const SimulationTruthAlignmentUpdate update = monitor.update(stamp_ns, samples);
    EXPECT_EQ(update.ready, tick == 3);
    EXPECT_EQ(update.reason, tick == 3 ? SimulationTruthAlignmentReason::kAligned
                                       : SimulationTruthAlignmentReason::kConfirming);
  }
}

TEST(SimulationTruthAlignmentTest, ConfirmsPersistentCoordinateMismatchAtStartup) {
  SimulationTruthAlignmentMonitor monitor{testConfig()};
  std::array<SimulationTruthAlignmentSample, 1> samples;
  samples[0] = SimulationTruthAlignmentSample{
      .navigation = state(10.0, 100'000'000LL),
      .physical_truth = state(64.0, 100'000'000LL),
  };
  const SimulationTruthAlignmentUpdate initial = monitor.update(100'000'000LL, samples);
  EXPECT_FALSE(initial.failure_confirmed);
  EXPECT_EQ(initial.reason, SimulationTruthAlignmentReason::kPositionMismatch);

  samples[0].navigation = state(11.0, 1'200'000'000LL);
  samples[0].physical_truth = state(65.0, 1'200'000'000LL);
  const SimulationTruthAlignmentUpdate confirmed =
      monitor.update(1'200'000'000LL, samples);
  EXPECT_TRUE(confirmed.failure_confirmed);
  EXPECT_TRUE(confirmed.newly_failed);
  EXPECT_FALSE(confirmed.ready);

  samples[0].navigation = state(11.1, 1'300'000'000LL);
  samples[0].physical_truth = state(65.1, 1'300'000'000LL);
  const SimulationTruthAlignmentUpdate repeated =
      monitor.update(1'300'000'000LL, samples);
  EXPECT_TRUE(repeated.failure_confirmed);
  EXPECT_FALSE(repeated.newly_failed);
}

TEST(SimulationTruthAlignmentTest, DoesNotConfirmMissingStartupData) {
  SimulationTruthAlignmentMonitor monitor{testConfig()};
  std::array<SimulationTruthAlignmentSample, 1> samples;
  const SimulationTruthAlignmentUpdate update =
      monitor.update(5'000'000'000LL, samples);
  EXPECT_FALSE(update.ready);
  EXPECT_FALSE(update.failure_confirmed);
  EXPECT_EQ(update.reason, SimulationTruthAlignmentReason::kMissingNavigation);
}

TEST(SimulationTruthAlignmentTest, ToleratesTransientMismatchAfterReadiness) {
  SimulationTruthAlignmentMonitor monitor{testConfig()};
  std::array<SimulationTruthAlignmentSample, 1> samples;
  for (std::int64_t tick = 1; tick <= 3; ++tick) {
    const std::int64_t stamp_ns = tick * 100'000'000LL;
    samples[0] = SimulationTruthAlignmentSample{
        .navigation = state(10.0, stamp_ns),
        .physical_truth = state(10.0, stamp_ns),
    };
    (void)monitor.update(stamp_ns, samples);
  }

  samples[0] = SimulationTruthAlignmentSample{
      .navigation = state(10.0, 400'000'000LL),
      .physical_truth = state(20.0, 400'000'000LL),
  };
  const SimulationTruthAlignmentUpdate degraded =
      monitor.update(400'000'000LL, samples);
  EXPECT_TRUE(degraded.ready);
  EXPECT_FALSE(degraded.failure_confirmed);

  samples[0] = SimulationTruthAlignmentSample{
      .navigation = state(10.0, 500'000'000LL),
      .physical_truth = state(10.0, 500'000'000LL),
  };
  const SimulationTruthAlignmentUpdate recovered =
      monitor.update(500'000'000LL, samples);
  EXPECT_TRUE(recovered.ready);
  EXPECT_FALSE(recovered.failure_confirmed);
}

} // namespace
} // namespace drone_city_nav
