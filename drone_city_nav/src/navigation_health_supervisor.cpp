#include "drone_city_nav/navigation_health_supervisor.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {

bool NavigationHealthConfig::valid() const noexcept {
  return std::isfinite(maximum_unavailable_world_age_ms) &&
         maximum_unavailable_world_age_ms > 0.0 &&
         std::isfinite(maximum_no_executable_route_age_ms) &&
         maximum_no_executable_route_age_ms > 0.0 &&
         std::isfinite(maximum_unacknowledged_horizon_age_ms) &&
         maximum_unacknowledged_horizon_age_ms > 0.0 && maximum_recovery_attempts > 0U;
}

NavigationHealthSupervisor::NavigationHealthSupervisor(
    const NavigationHealthConfig& config)
    : config_{config} {
  if (!config_.valid()) {
    throw std::invalid_argument{"invalid navigation health configuration"};
  }
}

NavigationHealthAssessment NavigationHealthSupervisor::update(
    const NavigationHealthObservation& observation) noexcept {
  if (!observation.mission_active || observation.mission_epoch == 0U ||
      observation.now_ns <= 0) {
    reset();
    return {};
  }
  if (observation.mission_epoch != mission_epoch_) {
    reset();
    mission_epoch_ = observation.mission_epoch;
    recovery_sequence_ = observation.recovery_sequence;
    stage_since_ns_ = observation.now_ns;
  }
  if (terminal_) {
    return NavigationHealthAssessment{
        .stage = NavigationReadinessStage::kTerminalFailure,
        .failure = terminal_failure_,
        .mission_epoch = mission_epoch_,
        .recovery_attempts = recovery_attempts_,
        .stage_age_ms = stageAgeMs(observation.now_ns),
        .mission_ready = false,
        .terminal = true,
    };
  }
  if (observation.recovery_sequence > recovery_sequence_) {
    const std::uint64_t delta = observation.recovery_sequence - recovery_sequence_;
    const std::uint64_t bounded = std::min<std::uint64_t>(
        delta, std::numeric_limits<std::uint32_t>::max() - recovery_attempts_);
    recovery_attempts_ += static_cast<std::uint32_t>(bounded);
    recovery_sequence_ = observation.recovery_sequence;
  }

  NavigationReadinessStage desired{NavigationReadinessStage::kProcessAlive};
  if (observation.process_alive) {
    desired = NavigationReadinessStage::kWorldReady;
  }
  if (observation.process_alive && observation.world_ready) {
    desired = NavigationReadinessStage::kBootstrapReady;
  }
  if (observation.process_alive && observation.world_ready &&
      observation.bootstrap_ready) {
    desired = NavigationReadinessStage::kCertifiedRouteReady;
  }
  if (observation.process_alive && observation.world_ready &&
      observation.bootstrap_ready && observation.certified_route_ready) {
    desired = NavigationReadinessStage::kFirstHorizonAcknowledged;
  }
  if (observation.process_alive && observation.world_ready &&
      observation.bootstrap_ready && observation.certified_route_ready &&
      observation.horizon_acknowledged) {
    desired = NavigationReadinessStage::kMissionReady;
  }
  if (desired != stage_) {
    enterStage(desired, observation.now_ns);
  }
  const double age_ms = stageAgeMs(observation.now_ns);
  NavigationTerminalFailure failure{NavigationTerminalFailure::kNone};
  if (observation.bootstrap_ready && !observation.world_ready &&
      age_ms >= config_.maximum_unavailable_world_age_ms) {
    failure = NavigationTerminalFailure::kUnavailableWorld;
  } else if (observation.bootstrap_ready && observation.world_ready &&
             !observation.certified_route_ready &&
             age_ms >= config_.maximum_no_executable_route_age_ms) {
    failure = NavigationTerminalFailure::kNoExecutableRoute;
  } else if (observation.certified_route_ready && !observation.horizon_acknowledged &&
             age_ms >= config_.maximum_unacknowledged_horizon_age_ms) {
    failure = NavigationTerminalFailure::kNoAcknowledgedHorizon;
  } else if (!observation.certified_route_ready &&
             recovery_attempts_ >= config_.maximum_recovery_attempts) {
    failure = NavigationTerminalFailure::kRecoveryBudgetExhausted;
  }
  if (failure != NavigationTerminalFailure::kNone) {
    terminal_ = true;
    terminal_failure_ = failure;
    enterStage(NavigationReadinessStage::kTerminalFailure, observation.now_ns);
  }
  return NavigationHealthAssessment{
      .stage = stage_,
      .failure = terminal_failure_,
      .mission_epoch = mission_epoch_,
      .recovery_attempts = recovery_attempts_,
      .stage_age_ms = stageAgeMs(observation.now_ns),
      .mission_ready = stage_ == NavigationReadinessStage::kMissionReady,
      .terminal = terminal_,
  };
}

void NavigationHealthSupervisor::reset() noexcept {
  stage_ = NavigationReadinessStage::kProcessAlive;
  terminal_failure_ = NavigationTerminalFailure::kNone;
  mission_epoch_ = 0U;
  recovery_sequence_ = 0U;
  stage_since_ns_ = 0;
  recovery_attempts_ = 0U;
  terminal_ = false;
}

void NavigationHealthSupervisor::enterStage(const NavigationReadinessStage stage,
                                            const std::int64_t now_ns) noexcept {
  stage_ = stage;
  stage_since_ns_ = now_ns;
}

double
NavigationHealthSupervisor::stageAgeMs(const std::int64_t now_ns) const noexcept {
  if (stage_since_ns_ <= 0 || now_ns < stage_since_ns_) {
    return 0.0;
  }
  return static_cast<double>(now_ns - stage_since_ns_) * 1.0e-6;
}

const char*
navigationReadinessStageName(const NavigationReadinessStage stage) noexcept {
  switch (stage) {
    case NavigationReadinessStage::kProcessAlive:
      return "process_alive";
    case NavigationReadinessStage::kWorldReady:
      return "world_ready";
    case NavigationReadinessStage::kBootstrapReady:
      return "bootstrap_ready";
    case NavigationReadinessStage::kCertifiedRouteReady:
      return "certified_route_ready";
    case NavigationReadinessStage::kFirstHorizonAcknowledged:
      return "first_horizon_acknowledged";
    case NavigationReadinessStage::kMissionReady:
      return "mission_ready";
    case NavigationReadinessStage::kTerminalFailure:
      return "terminal_failure";
  }
  return "unknown";
}

const char*
navigationTerminalFailureName(const NavigationTerminalFailure failure) noexcept {
  switch (failure) {
    case NavigationTerminalFailure::kNone:
      return "none";
    case NavigationTerminalFailure::kUnavailableWorld:
      return "unavailable_world";
    case NavigationTerminalFailure::kNoExecutableRoute:
      return "no_executable_route";
    case NavigationTerminalFailure::kNoAcknowledgedHorizon:
      return "no_acknowledged_horizon";
    case NavigationTerminalFailure::kRecoveryBudgetExhausted:
      return "recovery_budget_exhausted";
  }
  return "unknown";
}

} // namespace drone_city_nav
