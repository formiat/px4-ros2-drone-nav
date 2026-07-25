#pragma once

#include "drone_city_nav/receding_horizon_trajectory_planner.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

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
  std::size_t direction_switches_before_recovery{3U};
};

struct NoStaticPlannerDecisionInput {
  std::uint64_t generation{0U};
  std::uint64_t latest_generation{0U};
  std::uint64_t grid_revision{0U};
  std::uint64_t latest_grid_revision{0U};
  bool candidate_valid{false};
  bool active_prefix_available{true};
  bool active_suffix_blocked{false};
  bool active_suffix_exhausting{false};
  bool temporary_hold_active{false};
  double candidate_score{0.0};
  std::optional<double> active_score;
  double seconds_since_progress{0.0};
  double candidate_heading_offset_rad{0.0};
};

struct NoStaticPlannerDecision {
  NoStaticPlannerAction action{NoStaticPlannerAction::kKeep};
  NoStaticPlannerMode mode{NoStaticPlannerMode::kDirectGoalRollout};
};

struct StablePrefixStitchResult {
  bool valid{false};
  std::vector<TrajectoryPointSample> samples;
  double join_s_m{0.0};
};

[[nodiscard]] StablePrefixStitchResult
stitchStableExecutablePrefix(std::span<const TrajectoryPointSample> active_samples,
                             double current_s_m, double prefix_distance_m,
                             std::span<const TrajectoryPointSample> successor_samples,
                             double endpoint_tolerance_m = 1.0);

class NoStaticPlannerOrchestrator {
public:
  explicit NoStaticPlannerOrchestrator(
      const NoStaticPlannerOrchestratorConfig& config = {});

  [[nodiscard]] NoStaticPlannerDecision
  decide(const NoStaticPlannerDecisionInput& input);

  [[nodiscard]] NoStaticPlannerDecision
  decideRecoveryFailure(bool active_prefix_available);

  void setRecoveryGuide(std::vector<Point2> guide, std::uint64_t revision);
  void clearRecoveryGuide() noexcept;
  [[nodiscard]] Point2 recoveryPreferredTarget(Point2 current_position,
                                               double lookahead_m,
                                               Point2 fallback) const;
  [[nodiscard]] bool hasRecoveryGuide() const noexcept;
  [[nodiscard]] std::uint64_t recoveryGuideRevision() const noexcept;
  [[nodiscard]] NoStaticPlannerMode mode() const noexcept;
  [[nodiscard]] std::size_t consecutiveFailures() const noexcept;

private:
  NoStaticPlannerOrchestratorConfig config_{};
  NoStaticPlannerMode mode_{NoStaticPlannerMode::kDirectGoalRollout};
  std::size_t consecutive_failures_{0U};
  bool recovery_guide_available_{false};
  std::vector<Point2> recovery_guide_;
  std::uint64_t recovery_guide_revision_{0U};
  int last_direction_sign_{0};
  std::size_t direction_switches_{0U};
};

[[nodiscard]] Point2 recoveryGuideLookahead(std::span<const Point2> guide,
                                            Point2 current_position, double lookahead_m,
                                            Point2 fallback);

[[nodiscard]] const char* noStaticPlannerModeName(NoStaticPlannerMode mode) noexcept;
[[nodiscard]] const char*
noStaticPlannerActionName(NoStaticPlannerAction action) noexcept;

} // namespace drone_city_nav
