#include "drone_city_nav/no_static_planner_orchestrator.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

void appendDistinct(std::vector<TrajectoryPointSample>& samples,
                    const TrajectoryPointSample& sample) {
  if (!samples.empty() && distance(samples.back().point, sample.point) <= 1.0e-6) {
    samples.back() = sample;
    return;
  }
  samples.push_back(sample);
}

} // namespace

StablePrefixStitchResult stitchStableExecutablePrefix(
    const std::span<const TrajectoryPointSample> active_samples,
    const double current_s_m, const double prefix_distance_m,
    const std::span<const TrajectoryPointSample> successor_samples,
    const double endpoint_tolerance_m) {
  StablePrefixStitchResult result;
  if (!trajectorySamplesAreUsable(active_samples) ||
      !trajectorySamplesAreUsable(successor_samples) || !std::isfinite(current_s_m) ||
      !std::isfinite(prefix_distance_m)) {
    return result;
  }
  result.join_s_m = std::clamp(current_s_m + std::max(0.0, prefix_distance_m),
                               active_samples.front().s_m, active_samples.back().s_m);
  const TrajectoryPointSample start = trajectorySampleAtS(active_samples, current_s_m);
  const TrajectoryPointSample join =
      trajectorySampleAtS(active_samples, result.join_s_m);
  if (distance(join.point, successor_samples.front().point) >
      std::max(0.0, endpoint_tolerance_m)) {
    return result;
  }
  appendDistinct(result.samples, start);
  for (const TrajectoryPointSample& sample : active_samples) {
    if (sample.s_m > current_s_m && sample.s_m < result.join_s_m) {
      appendDistinct(result.samples, sample);
    }
  }
  appendDistinct(result.samples, join);
  for (std::size_t index = 1U; index < successor_samples.size(); ++index) {
    appendDistinct(result.samples, successor_samples[index]);
  }
  populateTrajectorySampleGeometry(result.samples);
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

NoStaticPlannerOrchestrator::NoStaticPlannerOrchestrator(
    const NoStaticPlannerOrchestratorConfig& config)
    : config_{config} {
  config_.failed_rollout_cycles_before_recovery =
      std::max<std::size_t>(1U, config_.failed_rollout_cycles_before_recovery);
  config_.progress_timeout_s = std::max(0.0, config_.progress_timeout_s);
  config_.minimum_score_improvement = std::max(0.0, config_.minimum_score_improvement);
}

NoStaticPlannerDecision
NoStaticPlannerOrchestrator::decide(const NoStaticPlannerDecisionInput& input) {
  if (input.generation != input.latest_generation ||
      input.grid_revision != input.latest_grid_revision) {
    return {NoStaticPlannerAction::kRejectStale, mode_};
  }
  if (input.temporary_hold_active) {
    mode_ = NoStaticPlannerMode::kTemporaryHold;
  }
  if (!input.candidate_valid) {
    ++consecutive_failures_;
    if (consecutive_failures_ >= config_.failed_rollout_cycles_before_recovery ||
        input.seconds_since_progress >= config_.progress_timeout_s) {
      mode_ = NoStaticPlannerMode::kAstarRecoveryRunning;
      return {NoStaticPlannerAction::kRequestRecovery, mode_};
    }
    return {NoStaticPlannerAction::kKeep, mode_};
  }

  consecutive_failures_ = 0U;
  mode_ = recovery_guide_available_ ? NoStaticPlannerMode::kAstarGuidedRollout
                                    : NoStaticPlannerMode::kDirectGoalRollout;
  const bool replacement_required =
      input.active_suffix_blocked || input.active_suffix_exhausting ||
      input.temporary_hold_active || !input.active_score.has_value();
  const bool materially_better =
      input.active_score.has_value() &&
      input.candidate_score + config_.minimum_score_improvement < *input.active_score;
  return {replacement_required || materially_better ? NoStaticPlannerAction::kPublish
                                                    : NoStaticPlannerAction::kKeep,
          mode_};
}

void NoStaticPlannerOrchestrator::setRecoveryGuideAvailable(
    const bool available) noexcept {
  recovery_guide_available_ = available;
  if (!available && mode_ == NoStaticPlannerMode::kAstarGuidedRollout) {
    mode_ = NoStaticPlannerMode::kDirectGoalRollout;
  }
}

NoStaticPlannerMode NoStaticPlannerOrchestrator::mode() const noexcept {
  return mode_;
}

std::size_t NoStaticPlannerOrchestrator::consecutiveFailures() const noexcept {
  return consecutive_failures_;
}

const char* noStaticPlannerModeName(const NoStaticPlannerMode mode) noexcept {
  switch (mode) {
    case NoStaticPlannerMode::kDirectGoalRollout:
      return "direct_goal_rollout";
    case NoStaticPlannerMode::kAstarRecoveryRunning:
      return "astar_recovery_running";
    case NoStaticPlannerMode::kAstarGuidedRollout:
      return "astar_guided_rollout";
    case NoStaticPlannerMode::kTemporaryHold:
      return "temporary_hold";
  }
  return "unknown";
}

const char* noStaticPlannerActionName(const NoStaticPlannerAction action) noexcept {
  switch (action) {
    case NoStaticPlannerAction::kPublish:
      return "publish";
    case NoStaticPlannerAction::kKeep:
      return "keep";
    case NoStaticPlannerAction::kRequestRecovery:
      return "request_recovery";
    case NoStaticPlannerAction::kHold:
      return "hold";
    case NoStaticPlannerAction::kRejectStale:
      return "reject_stale";
  }
  return "unknown";
}

} // namespace drone_city_nav
