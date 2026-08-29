#include "drone_city_nav/navigation_health_supervisor.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] NavigationHealthObservation healthyObservation() {
  return NavigationHealthObservation{
      .mission_epoch = 7U,
      .recovery_sequence = 0U,
      .now_ns = 1'000'000'000,
      .mission_active = true,
      .process_alive = true,
      .world_ready = true,
      .bootstrap_ready = true,
      .certified_route_ready = true,
      .horizon_acknowledged = true,
  };
}

TEST(NavigationHealthSupervisorTest, ExposesEveryReadinessBoundary) {
  NavigationHealthSupervisor supervisor;
  NavigationHealthObservation observation = healthyObservation();
  observation.world_ready = false;
  observation.bootstrap_ready = false;
  observation.certified_route_ready = false;
  observation.horizon_acknowledged = false;
  EXPECT_EQ(supervisor.update(observation).stage,
            NavigationReadinessStage::kWorldReady);

  observation.now_ns += 1'000'000;
  observation.world_ready = true;
  EXPECT_EQ(supervisor.update(observation).stage,
            NavigationReadinessStage::kBootstrapReady);
  observation.now_ns += 1'000'000;
  observation.bootstrap_ready = true;
  EXPECT_EQ(supervisor.update(observation).stage,
            NavigationReadinessStage::kCertifiedRouteReady);
  observation.now_ns += 1'000'000;
  observation.certified_route_ready = true;
  EXPECT_EQ(supervisor.update(observation).stage,
            NavigationReadinessStage::kFirstHorizonAcknowledged);
  observation.now_ns += 1'000'000;
  observation.horizon_acknowledged = true;
  const NavigationHealthAssessment ready = supervisor.update(observation);
  EXPECT_EQ(ready.stage, NavigationReadinessStage::kMissionReady);
  EXPECT_TRUE(ready.mission_ready);
}

TEST(NavigationHealthSupervisorTest, AdvisoryBudgetsDoNotTerminateMissionsByDefault) {
  NavigationHealthSupervisor supervisor{NavigationHealthConfig{
      .maximum_unavailable_world_age_ms = 1.0,
      .maximum_no_executable_route_age_ms = 1.0,
      .maximum_unacknowledged_horizon_age_ms = 1.0,
      .maximum_recovery_attempts = 1U,
  }};
  NavigationHealthObservation observation = healthyObservation();
  observation.certified_route_ready = false;
  observation.horizon_acknowledged = false;
  static_cast<void>(supervisor.update(observation));
  observation.now_ns += 10'000'000;
  observation.recovery_sequence = 10U;

  const NavigationHealthAssessment advisory = supervisor.update(observation);
  EXPECT_FALSE(advisory.terminal);
  EXPECT_EQ(advisory.failure, NavigationTerminalFailure::kNone);
  EXPECT_EQ(advisory.stage, NavigationReadinessStage::kCertifiedRouteReady);
  EXPECT_EQ(advisory.recovery_attempts, 10U);
}

TEST(NavigationRecoveryEpisodeTrackerTest,
     CountsContinuousRecoveryOnceAndIgnoresStaleMissions) {
  NavigationRecoveryEpisodeTracker tracker;
  EXPECT_TRUE(tracker.observe(7U, true));
  EXPECT_FALSE(tracker.observe(7U, true));
  EXPECT_FALSE(tracker.observe(7U, true));
  EXPECT_EQ(tracker.sequence(), 1U);

  EXPECT_FALSE(tracker.observe(7U, false));
  EXPECT_TRUE(tracker.observe(7U, true));
  EXPECT_EQ(tracker.sequence(), 2U);

  EXPECT_TRUE(tracker.observe(8U, true));
  EXPECT_FALSE(tracker.observe(7U, false));
  EXPECT_FALSE(tracker.observe(7U, true));
  EXPECT_FALSE(tracker.observe(8U, true));
  EXPECT_EQ(tracker.sequence(), 3U);

  EXPECT_FALSE(tracker.observe(8U, false));
  EXPECT_TRUE(tracker.observe(8U, true));
  EXPECT_EQ(tracker.sequence(), 4U);
}

TEST(NavigationHealthSupervisorTest, TerminatesBoundedNoRouteRecovery) {
  NavigationHealthSupervisor supervisor{NavigationHealthConfig{
      .terminal_failure_enabled = true,
      .maximum_unavailable_world_age_ms = 100.0,
      .maximum_no_executable_route_age_ms = 100.0,
      .maximum_unacknowledged_horizon_age_ms = 100.0,
      .maximum_recovery_attempts = 3U,
  }};
  NavigationHealthObservation observation = healthyObservation();
  observation.certified_route_ready = false;
  observation.horizon_acknowledged = false;
  EXPECT_FALSE(supervisor.update(observation).terminal);

  observation.now_ns += 1'000'000;
  observation.recovery_sequence = 3U;
  const NavigationHealthAssessment terminal = supervisor.update(observation);
  EXPECT_TRUE(terminal.terminal);
  EXPECT_EQ(terminal.failure, NavigationTerminalFailure::kRecoveryBudgetExhausted);

  observation.certified_route_ready = true;
  observation.horizon_acknowledged = true;
  EXPECT_TRUE(supervisor.update(observation).terminal);
}

TEST(NavigationHealthSupervisorTest, TerminatesNoRouteAtDeadline) {
  NavigationHealthSupervisor supervisor{NavigationHealthConfig{
      .terminal_failure_enabled = true,
      .maximum_unavailable_world_age_ms = 100.0,
      .maximum_no_executable_route_age_ms = 100.0,
      .maximum_unacknowledged_horizon_age_ms = 100.0,
      .maximum_recovery_attempts = 10U,
  }};
  NavigationHealthObservation observation = healthyObservation();
  observation.certified_route_ready = false;
  observation.horizon_acknowledged = false;
  static_cast<void>(supervisor.update(observation));
  observation.now_ns += 100'000'000;

  const NavigationHealthAssessment terminal = supervisor.update(observation);
  EXPECT_TRUE(terminal.terminal);
  EXPECT_EQ(terminal.failure, NavigationTerminalFailure::kNoExecutableRoute);
}

TEST(NavigationHealthSupervisorTest, TerminatesUnavailableWorldAfterBootstrap) {
  NavigationHealthSupervisor supervisor{NavigationHealthConfig{
      .terminal_failure_enabled = true,
      .maximum_unavailable_world_age_ms = 100.0,
      .maximum_no_executable_route_age_ms = 200.0,
      .maximum_unacknowledged_horizon_age_ms = 300.0,
      .maximum_recovery_attempts = 10U,
  }};
  NavigationHealthObservation observation = healthyObservation();
  observation.world_ready = false;
  observation.certified_route_ready = false;
  observation.horizon_acknowledged = false;
  static_cast<void>(supervisor.update(observation));
  observation.now_ns += 100'000'000;

  const NavigationHealthAssessment terminal = supervisor.update(observation);
  EXPECT_TRUE(terminal.terminal);
  EXPECT_EQ(terminal.failure, NavigationTerminalFailure::kUnavailableWorld);
}

TEST(NavigationHealthSupervisorTest, TerminatesUnacknowledgedHorizonAtDeadline) {
  NavigationHealthSupervisor supervisor{NavigationHealthConfig{
      .terminal_failure_enabled = true,
      .maximum_unavailable_world_age_ms = 300.0,
      .maximum_no_executable_route_age_ms = 200.0,
      .maximum_unacknowledged_horizon_age_ms = 100.0,
      .maximum_recovery_attempts = 10U,
  }};
  NavigationHealthObservation observation = healthyObservation();
  observation.horizon_acknowledged = false;
  static_cast<void>(supervisor.update(observation));
  observation.now_ns += 100'000'000;

  const NavigationHealthAssessment terminal = supervisor.update(observation);
  EXPECT_TRUE(terminal.terminal);
  EXPECT_EQ(terminal.failure, NavigationTerminalFailure::kNoAcknowledgedHorizon);
}

TEST(NavigationHealthSupervisorTest, NewMissionResetsTerminalLatchAndAttempts) {
  NavigationHealthSupervisor supervisor{NavigationHealthConfig{
      .terminal_failure_enabled = true,
      .maximum_unavailable_world_age_ms = 100.0,
      .maximum_no_executable_route_age_ms = 100.0,
      .maximum_unacknowledged_horizon_age_ms = 100.0,
      .maximum_recovery_attempts = 2U,
  }};
  NavigationHealthObservation observation = healthyObservation();
  observation.certified_route_ready = false;
  observation.horizon_acknowledged = false;
  static_cast<void>(supervisor.update(observation));
  observation.recovery_sequence = 2U;
  EXPECT_TRUE(supervisor.update(observation).terminal);

  observation.mission_epoch += 1U;
  observation.recovery_sequence = 8U;
  observation.certified_route_ready = true;
  observation.horizon_acknowledged = true;
  observation.now_ns += 1'000'000;
  const NavigationHealthAssessment next_mission = supervisor.update(observation);
  EXPECT_FALSE(next_mission.terminal);
  EXPECT_TRUE(next_mission.mission_ready);
  EXPECT_EQ(next_mission.recovery_attempts, 0U);
}

} // namespace
} // namespace drone_city_nav
