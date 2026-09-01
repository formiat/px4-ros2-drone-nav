#pragma once

#include "drone_city_nav/navigation_recovery_episode_tracker.hpp"

#include <cstdint>

namespace drone_city_nav {

enum class NavigationReadinessStage : std::uint8_t {
  kProcessAlive,
  kWorldReady,
  kBootstrapReady,
  kCertifiedRouteReady,
  kFirstHorizonAcknowledged,
  kMissionReady,
  kTerminalFailure,
};

enum class NavigationTerminalFailure : std::uint8_t {
  kNone,
  kUnavailableWorld,
  kNoExecutableRoute,
  kNoAcknowledgedHorizon,
  kRecoveryBudgetExhausted,
};

struct NavigationHealthConfig {
  bool terminal_failure_enabled{false};
  double maximum_unavailable_world_age_ms{30'000.0};
  double maximum_no_executable_route_age_ms{30'000.0};
  double maximum_unacknowledged_horizon_age_ms{10'000.0};
  std::uint32_t maximum_recovery_attempts{64U};

  [[nodiscard]] bool valid() const noexcept;
};

struct NavigationHealthObservation {
  std::uint64_t mission_epoch{0U};
  std::uint64_t recovery_sequence{0U};
  std::int64_t now_ns{0};
  bool mission_active{false};
  bool process_alive{false};
  bool world_ready{false};
  bool bootstrap_ready{false};
  bool certified_route_ready{false};
  bool horizon_acknowledged{false};
};

struct NavigationHealthAssessment {
  NavigationReadinessStage stage{NavigationReadinessStage::kProcessAlive};
  NavigationTerminalFailure failure{NavigationTerminalFailure::kNone};
  std::uint64_t mission_epoch{0U};
  std::uint32_t recovery_attempts{0U};
  double stage_age_ms{0.0};
  bool mission_ready{false};
  bool terminal{false};
};

class NavigationHealthSupervisor final {
public:
  explicit NavigationHealthSupervisor(const NavigationHealthConfig& config = {});

  [[nodiscard]] NavigationHealthAssessment
  update(const NavigationHealthObservation& observation) noexcept;
  void reset() noexcept;

private:
  void enterStage(NavigationReadinessStage stage, std::int64_t now_ns) noexcept;
  [[nodiscard]] double stageAgeMs(std::int64_t now_ns) const noexcept;

  NavigationHealthConfig config_{};
  NavigationReadinessStage stage_{NavigationReadinessStage::kProcessAlive};
  NavigationTerminalFailure terminal_failure_{NavigationTerminalFailure::kNone};
  std::uint64_t mission_epoch_{0U};
  std::uint64_t recovery_sequence_{0U};
  std::int64_t stage_since_ns_{0};
  std::uint32_t recovery_attempts_{0U};
  bool terminal_{false};
};

[[nodiscard]] const char*
navigationReadinessStageName(NavigationReadinessStage stage) noexcept;
[[nodiscard]] const char*
navigationTerminalFailureName(NavigationTerminalFailure failure) noexcept;

} // namespace drone_city_nav
