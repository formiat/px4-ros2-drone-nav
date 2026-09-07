#include "drone_city_nav/mppi_liveness.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kFirstWindowNs{1'000'000'000LL};
constexpr std::int64_t kSecondWindowNs{2'100'000'000LL};
constexpr std::int64_t kThirdWindowNs{3'200'000'000LL};

MppiLivenessObservation observation(const std::int64_t stamp_ns,
                                    const mppi::State& state = {}) {
  return MppiLivenessObservation{
      .stamp_ns = stamp_ns,
      .actual_state = state,
      .controller_active = true,
      .predicted_head_progress_m = 0.1,
  };
}

MppiLivenessSupervisor optInSupervisor() {
  return MppiLivenessSupervisor{MppiLivenessConfig{.enabled = true}};
}

TEST(MppiLivenessTest, IsInactiveByDefault) {
  MppiLivenessSupervisor supervisor;

  const MppiLivenessResult result = supervisor.evaluate(observation(kFirstWindowNs));

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kInactive);
}

TEST(MppiLivenessTest, RequestsReseedOnlyAfterTwoStationaryWindows) {
  MppiLivenessSupervisor supervisor = optInSupervisor();

  EXPECT_EQ(supervisor.evaluate(observation(kFirstWindowNs)).state,
            MppiLivenessState::kMonitoring);
  const MppiLivenessResult first_window =
      supervisor.evaluate(observation(kSecondWindowNs));
  EXPECT_FALSE(first_window.reseed_requested);
  EXPECT_EQ(first_window.state, MppiLivenessState::kMonitoring);
  EXPECT_EQ(first_window.stalled_windows, 1U);

  const MppiLivenessResult result = supervisor.evaluate(observation(kThirdWindowNs));

  EXPECT_TRUE(result.reseed_requested);
  EXPECT_TRUE(result.recovery_active);
  EXPECT_EQ(result.state, MppiLivenessState::kReseedRequested);
  EXPECT_EQ(result.reseed_generation, 1U);
  EXPECT_DOUBLE_EQ(result.actual_displacement_m, 0.0);
}

TEST(MppiLivenessTest, RecoveryRemainsActiveUntilMeasuredProgress) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  (void)supervisor.evaluate(observation(kFirstWindowNs));
  (void)supervisor.evaluate(observation(kSecondWindowNs));
  const MppiLivenessResult stalled = supervisor.evaluate(observation(kThirdWindowNs));
  ASSERT_TRUE(stalled.recovery_active);

  const MppiLivenessResult monitoring =
      supervisor.evaluate(observation(3'300'000'000LL));
  EXPECT_EQ(monitoring.state, MppiLivenessState::kMonitoring);
  EXPECT_TRUE(monitoring.recovery_active);

  mppi::State moved;
  moved.x = 0.6F;
  const MppiLivenessResult recovered =
      supervisor.evaluate(observation(4'400'000'000LL, moved));
  EXPECT_EQ(recovered.state, MppiLivenessState::kMoving);
  EXPECT_FALSE(recovered.recovery_active);
}

TEST(MppiLivenessTest, RecoverySurvivesTemporaryControllerInactivity) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  (void)supervisor.evaluate(observation(kFirstWindowNs));
  (void)supervisor.evaluate(observation(kSecondWindowNs));
  ASSERT_TRUE(supervisor.evaluate(observation(kThirdWindowNs)).recovery_active);

  MppiLivenessObservation inactive = observation(3'300'000'000LL);
  inactive.controller_active = false;
  const MppiLivenessResult result = supervisor.evaluate(inactive);

  EXPECT_EQ(result.state, MppiLivenessState::kInactive);
  EXPECT_TRUE(result.recovery_active);
}

TEST(MppiLivenessTest, ProgressInOneWindowClearsTheStallCount) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  (void)supervisor.evaluate(observation(kFirstWindowNs));
  ASSERT_EQ(supervisor.evaluate(observation(kSecondWindowNs)).stalled_windows, 1U);

  mppi::State moved;
  moved.x = 0.6F;
  EXPECT_EQ(supervisor.evaluate(observation(kThirdWindowNs, moved)).state,
            MppiLivenessState::kMoving);

  const MppiLivenessResult after_progress =
      supervisor.evaluate(observation(4'400'000'000LL, moved));
  EXPECT_FALSE(after_progress.reseed_requested);
  EXPECT_EQ(after_progress.stalled_windows, 1U);
}

TEST(MppiLivenessTest, DisplacementInAnyDirectionCountsAsMovement) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  (void)supervisor.evaluate(observation(kFirstWindowNs));
  mppi::State moved;
  moved.x = -0.6F;

  const MppiLivenessResult result =
      supervisor.evaluate(observation(kSecondWindowNs, moved));

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
}

TEST(MppiLivenessTest, VerticalAlignmentCountsAsMovement) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  (void)supervisor.evaluate(observation(kFirstWindowNs));
  mppi::State moved;
  moved.z = 0.6F;

  const MppiLivenessResult result =
      supervisor.evaluate(observation(kSecondWindowNs, moved));

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
}

TEST(MppiLivenessTest, LateralWobbleDoesNotMaskMissingRouteProgress) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  MppiLivenessObservation first = observation(kFirstWindowNs);
  first.route_generation = 4U;
  first.route_station_m = 10.0;
  first.route_station_valid = true;
  (void)supervisor.evaluate(first);

  mppi::State moved;
  moved.y = 0.4F;
  moved.z = 0.2F;
  const auto stalled_window = [&](const std::int64_t stamp_ns) {
    MppiLivenessObservation next = observation(stamp_ns, moved);
    next.route_generation = 4U;
    next.route_station_m = 10.1;
    next.route_station_valid = true;
    return supervisor.evaluate(next);
  };
  (void)stalled_window(kSecondWindowNs);
  const MppiLivenessResult result = stalled_window(kThirdWindowNs);

  EXPECT_TRUE(result.reseed_requested);
  EXPECT_TRUE(result.used_route_progress);
  EXPECT_NEAR(result.actual_route_progress_m, 0.0, 1.0e-9);
  EXPECT_LT(result.actual_displacement_m, 0.5);
}

TEST(MppiLivenessTest, ManoeuvreOffTheRouteCountsAsMovement) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  MppiLivenessObservation first = observation(kFirstWindowNs);
  first.route_generation = 4U;
  first.route_station_m = 10.0;
  first.route_station_valid = true;
  (void)supervisor.evaluate(first);

  // Ground covered around an obstacle without gaining route station: the
  // vehicle is flying a manoeuvre, so its optimised sequence is left alone.
  mppi::State moved;
  moved.y = 3.0F;
  MppiLivenessObservation second = observation(kSecondWindowNs, moved);
  second.route_generation = 4U;
  second.route_station_m = 10.1;
  second.route_station_valid = true;
  const MppiLivenessResult result = supervisor.evaluate(second);

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
}

TEST(MppiLivenessTest, DisplacementAlongTheRouteTangentCountsAsProgress) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  MppiLivenessObservation first = observation(kFirstWindowNs);
  first.route_generation = 4U;
  first.route_station_m = 10.0;
  first.route_station_valid = true;
  first.route_tangent = Vec3{1.0, 0.0, 0.0};
  first.route_tangent_valid = true;
  (void)supervisor.evaluate(first);

  // The route was replaced under the vehicle, so its station restarted from a
  // new geometry and stopped growing. The ground covered along the tangent it
  // was following is progress all the same.
  mppi::State moved;
  moved.x = 1.2F;
  MppiLivenessObservation second = observation(kSecondWindowNs, moved);
  second.route_generation = 4U;
  second.route_station_m = 10.0;
  second.route_station_valid = true;
  second.route_tangent = Vec3{1.0, 0.0, 0.0};
  second.route_tangent_valid = true;
  const MppiLivenessResult result = supervisor.evaluate(second);

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
  EXPECT_NEAR(result.actual_route_progress_m, 0.0, 1.0e-9);
  EXPECT_NEAR(result.tangential_progress_m, 1.2, 1.0e-5);
  EXPECT_NEAR(result.useful_progress_m, 1.2, 1.0e-5);
}

TEST(MppiLivenessTest, AlongRouteProgressCountsAsUsefulMovement) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  MppiLivenessObservation first = observation(kFirstWindowNs);
  first.route_generation = 4U;
  first.route_station_m = 10.0;
  first.route_station_valid = true;
  (void)supervisor.evaluate(first);

  MppiLivenessObservation second = observation(kSecondWindowNs);
  second.route_generation = 4U;
  second.route_station_m = 10.6;
  second.route_station_valid = true;
  const MppiLivenessResult result = supervisor.evaluate(second);

  EXPECT_FALSE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kMoving);
  EXPECT_NEAR(result.actual_route_progress_m, 0.6, 1.0e-9);
}

TEST(MppiLivenessTest, VelocityWithoutNetDisplacementRequestsReseed) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  (void)supervisor.evaluate(observation(kFirstWindowNs));
  mppi::State oscillating;
  oscillating.vy = 2.0F;

  (void)supervisor.evaluate(observation(kSecondWindowNs, oscillating));
  const MppiLivenessResult result =
      supervisor.evaluate(observation(kThirdWindowNs, oscillating));

  EXPECT_TRUE(result.reseed_requested);
  EXPECT_EQ(result.state, MppiLivenessState::kReseedRequested);
}

TEST(MppiLivenessTest, RouteGenerationChangeRestartsTheWindow) {
  MppiLivenessSupervisor supervisor = optInSupervisor();
  MppiLivenessObservation first = observation(kFirstWindowNs);
  first.route_generation = 4U;
  first.route_station_m = 10.0;
  first.route_station_valid = true;
  (void)supervisor.evaluate(first);
  MppiLivenessObservation stalled = observation(kSecondWindowNs);
  stalled.route_generation = 4U;
  stalled.route_station_m = 10.0;
  stalled.route_station_valid = true;
  ASSERT_EQ(supervisor.evaluate(stalled).stalled_windows, 1U);

  // A route replaced under the vehicle carries no comparable station, so the
  // stall count starts over rather than counting a window that spans both.
  MppiLivenessObservation replaced = observation(kThirdWindowNs);
  replaced.route_generation = 5U;
  replaced.route_station_m = 0.0;
  replaced.route_station_valid = true;
  const MppiLivenessResult result = supervisor.evaluate(replaced);

  EXPECT_EQ(result.state, MppiLivenessState::kMonitoring);
  EXPECT_EQ(result.stalled_windows, 0U);
}

} // namespace
} // namespace drone_city_nav
