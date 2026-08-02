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

TEST(MppiLivenessTest, LowPredictedProgressDoesNotResetStationaryTimer) {
  MppiLivenessSupervisor supervisor;
  MppiLivenessObservation low_prediction = observation(1'000'000'000LL);
  low_prediction.predicted_terminal_progress_m = 0.0;
  EXPECT_EQ(supervisor.evaluate(low_prediction).state, MppiLivenessState::kMonitoring);

  low_prediction.stamp_ns = 2'100'000'000LL;
  const MppiLivenessResult result = supervisor.evaluate(low_prediction);

  EXPECT_TRUE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kReseedRequested);
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

TEST(MppiLivenessTest, LateralAndVerticalMotionDoNotMaskMissingRouteProgress) {
  MppiLivenessSupervisor supervisor;
  MppiLivenessObservation first = observation(1'000'000'000LL);
  first.route_generation = 4U;
  first.route_station_m = 10.0;
  first.route_station_valid = true;
  (void)supervisor.evaluate(first);

  mppi::State moved;
  moved.y = 2.0F;
  moved.z = 1.0F;
  MppiLivenessObservation second = observation(2'100'000'000LL, moved);
  second.route_generation = 4U;
  second.route_station_m = 10.1;
  second.route_station_valid = true;
  const MppiLivenessResult result = supervisor.evaluate(second);

  EXPECT_TRUE(result.reseed_requested);
  EXPECT_TRUE(result.used_route_progress);
  EXPECT_NEAR(result.actual_route_progress_m, 0.1, 1.0e-9);
  EXPECT_GT(result.actual_displacement_m, 2.0);
}

TEST(MppiLivenessTest, AlongRouteProgressCountsAsUsefulMovement) {
  MppiLivenessSupervisor supervisor;
  MppiLivenessObservation first = observation(1'000'000'000LL);
  first.route_generation = 4U;
  first.route_station_m = 10.0;
  first.route_station_valid = true;
  (void)supervisor.evaluate(first);

  MppiLivenessObservation second = observation(2'100'000'000LL);
  second.route_generation = 4U;
  second.route_station_m = 10.6;
  second.route_station_valid = true;
  const MppiLivenessResult result = supervisor.evaluate(second);

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
  EXPECT_NEAR(result.actual_route_progress_m, 0.6, 1.0e-9);
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
