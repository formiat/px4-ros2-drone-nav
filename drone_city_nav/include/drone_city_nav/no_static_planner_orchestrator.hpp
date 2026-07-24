#pragma once

#include "drone_city_nav/receding_horizon_trajectory_planner.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

enum class NoStaticPlannerMode {
  kDirectGoalRollout,
  kAstarRecoveryRunning,
  kAstarGuidedRollout,
  kTemporaryHold,
};

enum class NoStaticPlannerAction {
  kPublish,
  kKeep,
  kRequestRecovery,
  kHold,
  kRejectStale,
};

struct NoStaticPlannerOrchestratorConfig {
  std::size_t failed_rollout_cycles_before_recovery{3U};
  double progress_timeout_s{2.0};
  double minimum_score_improvement{5.0};
};

struct NoStaticPlannerDecisionInput {
  std::uint64_t generation{0U};
  std::uint64_t latest_generation{0U};
  std::uint64_t grid_revision{0U};
  std::uint64_t latest_grid_revision{0U};
  bool candidate_valid{false};
  bool active_suffix_blocked{false};
  bool active_suffix_exhausting{false};
  bool temporary_hold_active{false};
  double candidate_score{0.0};
  std::optional<double> active_score;
  double seconds_since_progress{0.0};
};

struct NoStaticPlannerDecision {
  NoStaticPlannerAction action{NoStaticPlannerAction::kKeep};
  NoStaticPlannerMode mode{NoStaticPlannerMode::kDirectGoalRollout};
};

class NoStaticPlannerOrchestrator {
public:
  explicit NoStaticPlannerOrchestrator(
      const NoStaticPlannerOrchestratorConfig& config = {});

  [[nodiscard]] NoStaticPlannerDecision
  decide(const NoStaticPlannerDecisionInput& input);

  void setRecoveryGuideAvailable(bool available) noexcept;
  [[nodiscard]] NoStaticPlannerMode mode() const noexcept;
  [[nodiscard]] std::size_t consecutiveFailures() const noexcept;

private:
  NoStaticPlannerOrchestratorConfig config_{};
  NoStaticPlannerMode mode_{NoStaticPlannerMode::kDirectGoalRollout};
  std::size_t consecutive_failures_{0U};
  bool recovery_guide_available_{false};
};

[[nodiscard]] const char* noStaticPlannerModeName(NoStaticPlannerMode mode) noexcept;
[[nodiscard]] const char*
noStaticPlannerActionName(NoStaticPlannerAction action) noexcept;

} // namespace drone_city_nav
