#include "drone_city_nav/mppi_liveness.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

MppiLivenessObservation observation(const std::int64_t stamp_ns,
                                    const mppi::State& state = {}) {
  return MppiLivenessObservation{
      .stamp_ns = stamp_ns,
      .actual_state = state,
      .controller_active = true,
      .emergency_braking = false,
      .predicted_head_progress_m = 0.1,
      .predicted_terminal_progress_m = 10.0,
  };
}

TEST(MppiLivenessTest, RequestsReseedAfterStationaryPredictionWindow) {
  MppiLivenessSupervisor supervisor;

  EXPECT_EQ(supervisor.evaluate(observation(1'000'000'000LL)).state,
            MppiLivenessState::kMonitoring);
  const MppiLivenessResult result = supervisor.evaluate(observation(2'100'000'000LL));

  EXPECT_TRUE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kReseedRequested);
  EXPECT_EQ(result.reseed_generation, 1U);
  EXPECT_DOUBLE_EQ(result.actual_displacement_m, 0.0);
}

TEST(MppiLivenessTest, DisplacementInAnyDirectionCountsAsMovement) {
  MppiLivenessSupervisor supervisor;
  (void)supervisor.evaluate(observation(1'000'000'000LL));
  mppi::State moved;
  moved.x = -0.6F;

  const MppiLivenessResult result =
      supervisor.evaluate(observation(2'100'000'000LL, moved));

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
}

TEST(MppiLivenessTest, VerticalAlignmentCountsAsMovement) {
  MppiLivenessSupervisor supervisor;
  (void)supervisor.evaluate(observation(1'000'000'000LL));
  mppi::State moved;
  moved.z = 0.6F;

  const MppiLivenessResult result =
      supervisor.evaluate(observation(2'100'000'000LL, moved));

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
}

TEST(MppiLivenessTest, VelocityWithoutNetDisplacementRequestsReseed) {
  MppiLivenessSupervisor supervisor;
  (void)supervisor.evaluate(observation(1'000'000'000LL));
  mppi::State oscillating;
  oscillating.vy = 2.0F;

  const MppiLivenessResult result =
      supervisor.evaluate(observation(2'100'000'000LL, oscillating));

  EXPECT_TRUE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kReseedRequested);
}

TEST(MppiLivenessTest, EmergencyBrakingCannotTriggerReseed) {
  MppiLivenessSupervisor supervisor;
  (void)supervisor.evaluate(observation(1'000'000'000LL));
  MppiLivenessObservation emergency = observation(2'100'000'000LL);
  emergency.emergency_braking = true;

  const MppiLivenessResult result = supervisor.evaluate(emergency);

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kEmergencyBraking);
  EXPECT_NEAR(result.emergency_braking_duration_s, 0.0, 1.0e-9);
}

TEST(MppiLivenessTest, EmergencyBrakingKeepsObservationAnchor) {
  MppiLivenessSupervisor supervisor;
  (void)supervisor.evaluate(observation(1'000'000'000LL));
  MppiLivenessObservation emergency = observation(1'500'000'000LL);
  emergency.emergency_braking = true;
  (void)supervisor.evaluate(emergency);
  emergency.stamp_ns = 2'600'000'000LL;
  const MppiLivenessResult emergency_result = supervisor.evaluate(emergency);
  EXPECT_NEAR(emergency_result.emergency_braking_duration_s, 1.1, 1.0e-9);

  const MppiLivenessResult resumed = supervisor.evaluate(observation(2'700'000'000LL));

  EXPECT_TRUE(resumed.reseed_requested);
}

} // namespace
} // namespace drone_city_nav
