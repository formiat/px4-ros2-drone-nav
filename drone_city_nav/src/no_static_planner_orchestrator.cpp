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

Point2 recoveryGuideLookahead(const std::span<const Point2> guide,
                              const Point2 current_position, const double lookahead_m,
                              const Point2 fallback) {
  if (guide.empty()) {
    return fallback;
  }
  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < guide.size(); ++index) {
    const double candidate_distance = distance(current_position, guide[index]);
    if (candidate_distance < nearest_distance) {
      nearest_distance = candidate_distance;
      nearest_index = index;
    }
  }
  double accumulated_m = 0.0;
  for (std::size_t index = nearest_index; index + 1U < guide.size(); ++index) {
    accumulated_m += distance(guide[index], guide[index + 1U]);
    if (accumulated_m >= std::max(0.0, lookahead_m)) {
      return guide[index + 1U];
    }
  }
  return guide.back();
}

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
  config_.direction_switches_before_recovery =
      std::max<std::size_t>(1U, config_.direction_switches_before_recovery);
}

NoStaticPlannerDecision
NoStaticPlannerOrchestrator::decide(const NoStaticPlannerDecisionInput& input) {
  if (input.generation != input.latest_generation ||
      input.grid_revision != input.latest_grid_revision) {
    return {NoStaticPlannerAction::kRejectStale, mode_};
  }
  if (input.temporary_hold_active && !recovery_guide_available_) {
    mode_ = NoStaticPlannerMode::kAstarRecoveryRunning;
    return {NoStaticPlannerAction::kRequestRecovery, mode_};
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
  int direction_sign = 0;
  if (input.candidate_heading_offset_rad > 1.0e-3) {
    direction_sign = 1;
  } else if (input.candidate_heading_offset_rad < -1.0e-3) {
    direction_sign = -1;
  }
  if (direction_sign != 0 && last_direction_sign_ != 0 &&
      direction_sign != last_direction_sign_) {
    ++direction_switches_;
  }
  if (direction_sign != 0) {
    last_direction_sign_ = direction_sign;
  }
  if (direction_switches_ >= config_.direction_switches_before_recovery) {
    direction_switches_ = 0U;
    mode_ = NoStaticPlannerMode::kAstarRecoveryRunning;
    return {NoStaticPlannerAction::kRequestRecovery, mode_};
  }
  mode_ = recovery_guide_available_ ? NoStaticPlannerMode::kAstarGuidedRollout
                                    : NoStaticPlannerMode::kDirectGoalRollout;
  const bool replacement_required =
      !input.active_prefix_available || input.active_suffix_blocked ||
      input.active_suffix_exhausting || input.temporary_hold_active ||
      !input.active_score.has_value();
  const bool materially_better =
      input.active_score.has_value() &&
      input.candidate_score + config_.minimum_score_improvement < *input.active_score;
  return {replacement_required || materially_better ? NoStaticPlannerAction::kPublish
                                                    : NoStaticPlannerAction::kKeep,
          mode_};
}

NoStaticPlannerDecision
NoStaticPlannerOrchestrator::decideRecoveryFailure(const bool active_prefix_available) {
  mode_ = active_prefix_available ? NoStaticPlannerMode::kAstarRecoveryRunning
                                  : NoStaticPlannerMode::kTemporaryHold;
  return {active_prefix_available ? NoStaticPlannerAction::kKeep
                                  : NoStaticPlannerAction::kHold,
          mode_};
}

void NoStaticPlannerOrchestrator::setRecoveryGuide(std::vector<Point2> guide,
                                                   const std::uint64_t revision) {
  recovery_guide_ = std::move(guide);
  recovery_guide_revision_ = revision;
  recovery_guide_available_ = !recovery_guide_.empty();
}

void NoStaticPlannerOrchestrator::clearRecoveryGuide() noexcept {
  recovery_guide_.clear();
  recovery_guide_available_ = false;
  if (mode_ == NoStaticPlannerMode::kAstarGuidedRollout) {
    mode_ = NoStaticPlannerMode::kDirectGoalRollout;
  }
}

Point2
NoStaticPlannerOrchestrator::recoveryPreferredTarget(const Point2 current_position,
                                                     const double lookahead_m,
                                                     const Point2 fallback) const {
  return recoveryGuideLookahead(recovery_guide_, current_position, lookahead_m,
                                fallback);
}

bool NoStaticPlannerOrchestrator::hasRecoveryGuide() const noexcept {
  return recovery_guide_available_;
}

std::uint64_t NoStaticPlannerOrchestrator::recoveryGuideRevision() const noexcept {
  return recovery_guide_revision_;
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
