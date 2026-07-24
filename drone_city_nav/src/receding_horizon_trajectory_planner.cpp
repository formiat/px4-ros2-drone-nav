#include "drone_city_nav/receding_horizon_trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] double clampAngle(double angle) noexcept {
  while (angle > std::numbers::pi) {
    angle -= 2.0 * std::numbers::pi;
  }
  while (angle < -std::numbers::pi) {
    angle += 2.0 * std::numbers::pi;
  }
  return angle;
}

[[nodiscard]] bool finitePoint(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] bool rawClear(const OccupancyGrid2D& grid,
                            std::span<const TrajectoryPointSample> samples,
                            RolloutDiagnostics& diagnostics) {
  for (const TrajectoryPointSample& sample : samples) {
    const std::optional<GridIndex> cell = grid.worldToCell(sample.point);
    if (!cell.has_value()) {
      ++diagnostics.outside_grid_rejections;
      return false;
    }
    if (grid.isOccupied(*cell)) {
      ++diagnostics.raw_occupied_rejections;
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool traversable(const OccupancyGrid2D& grid,
                               std::span<const TrajectoryPointSample> samples) {
  return std::ranges::all_of(samples, [&grid](const TrajectoryPointSample& sample) {
    const std::optional<GridIndex> cell = grid.worldToCell(sample.point);
    return cell.has_value() && !grid.isProhibited(*cell);
  });
}

[[nodiscard]] RolloutTraversabilityTier
classifyTier(const RolloutInput& input,
             std::span<const TrajectoryPointSample> samples) {
  if (traversable(*input.planning_grid, samples)) {
    return RolloutTraversabilityTier::kPlanningClearance;
  }
  if (traversable(*input.prohibited_grid, samples)) {
    return RolloutTraversabilityTier::kRuntimeProhibited;
  }
  return RolloutTraversabilityTier::kRawClear;
}

[[nodiscard]] double tierPenalty(const RolloutTraversabilityTier tier,
                                 const RolloutPlannerConfig& config) noexcept {
  switch (tier) {
    case RolloutTraversabilityTier::kPlanningClearance:
      return 0.0;
    case RolloutTraversabilityTier::kRuntimeProhibited:
      return config.degraded_tier_penalty;
    case RolloutTraversabilityTier::kRawClear:
      return 2.0 * config.degraded_tier_penalty;
  }
  return 2.0 * config.degraded_tier_penalty;
}

} // namespace

std::span<const RolloutCandidate>
RolloutResult::rankedShortlist(const std::size_t maximum) const noexcept {
  return std::span<const RolloutCandidate>{ranked_candidates}.first(
      std::min(maximum, ranked_candidates.size()));
}

RecedingHorizonTrajectoryPlanner::RecedingHorizonTrajectoryPlanner(
    const RolloutPlannerConfig& config)
    : config_{config} {
  config_.horizon_m = std::max(1.0, config_.horizon_m);
  config_.sample_step_m = std::clamp(config_.sample_step_m, 0.1, config_.horizon_m);
  config_.heading_samples = std::clamp<std::size_t>(config_.heading_samples, 1U, 63U);
  if ((config_.heading_samples % 2U) == 0U) {
    ++config_.heading_samples;
  }
  config_.max_heading_offset_rad =
      std::clamp(config_.max_heading_offset_rad, 0.0, std::numbers::pi);
  config_.max_finalists = std::clamp<std::size_t>(config_.max_finalists, 1U, 16U);
}

RolloutResult RecedingHorizonTrajectoryPlanner::plan(const RolloutInput& input) const {
  RolloutResult result;
  result.generation = input.generation;
  result.grid_revision = input.grid_revision;
  if (!finitePoint(input.position) || !finitePoint(input.velocity) ||
      !finitePoint(input.preferred_target) || input.raw_grid == nullptr ||
      input.prohibited_grid == nullptr || input.planning_grid == nullptr) {
    result.reject_reason = RolloutRejectReason::kInvalidInput;
    return result;
  }

  const Point2 goal_delta{input.preferred_target.x - input.position.x,
                          input.preferred_target.y - input.position.y};
  const double goal_distance = std::hypot(goal_delta.x, goal_delta.y);
  if (!(goal_distance > 1.0e-6)) {
    result.reject_reason = RolloutRejectReason::kInvalidInput;
    return result;
  }
  const double target_heading = std::atan2(goal_delta.y, goal_delta.x);
  const double speed = std::hypot(input.velocity.x, input.velocity.y);
  const double initial_heading =
      speed > 0.25 ? std::atan2(input.velocity.y, input.velocity.x) : target_heading;
  const double base_heading_error = clampAngle(target_heading - initial_heading);
  const double rollout_length = std::min(config_.horizon_m, goal_distance);
  const std::size_t sample_count = std::max<std::size_t>(
      2U,
      static_cast<std::size_t>(std::ceil(rollout_length / config_.sample_step_m)) + 1U);

  for (std::size_t candidate_index = 0U; candidate_index < config_.heading_samples;
       ++candidate_index) {
    const double normalized =
        config_.heading_samples == 1U
            ? 0.0
            : (2.0 * static_cast<double>(candidate_index) /
                   static_cast<double>(config_.heading_samples - 1U) -
               1.0);
    const double offset = normalized * config_.max_heading_offset_rad;
    const double terminal_heading = target_heading + offset;
    RolloutCandidate candidate;
    candidate.heading_offset_rad = offset;
    candidate.deterministic_index = candidate_index;
    candidate.samples.reserve(sample_count);
    for (std::size_t index = 0U; index < sample_count; ++index) {
      const double t =
          static_cast<double>(index) / static_cast<double>(sample_count - 1U);
      const double smooth_t = t * t * (3.0 - 2.0 * t);
      const double heading =
          initial_heading + clampAngle(terminal_heading - initial_heading) * smooth_t;
      const double s = rollout_length * t;
      TrajectoryPointSample sample;
      sample.s_m = s;
      sample.point = Point2{input.position.x + s * std::cos(heading),
                            input.position.y + s * std::sin(heading)};
      sample.tangent = Point2{std::cos(heading), std::sin(heading)};
      candidate.samples.push_back(std::move(sample));
    }
    populateTrajectorySampleGeometry(candidate.samples);
    ++result.diagnostics.generated;
    if (!rawClear(*input.raw_grid, candidate.samples, result.diagnostics)) {
      continue;
    }
    candidate.tier = classifyTier(input, candidate.samples);
    const double before = goal_distance;
    const double after =
        distance(candidate.samples.back().point, input.preferred_target);
    candidate.progress_m = before - after;
    double curvature_cost = 0.0;
    for (const TrajectoryPointSample& sample : candidate.samples) {
      curvature_cost += std::abs(sample.curvature_1pm);
    }
    candidate.score =
        tierPenalty(candidate.tier, config_) -
        config_.progress_weight * candidate.progress_m +
        config_.lateral_deviation_weight * std::abs(offset) * rollout_length +
        config_.heading_change_weight * std::abs(base_heading_error + offset) +
        config_.curvature_weight * curvature_cost;
    result.ranked_candidates.push_back(std::move(candidate));
  }

  std::ranges::sort(result.ranked_candidates,
                    [](const RolloutCandidate& lhs, const RolloutCandidate& rhs) {
                      if (lhs.score != rhs.score) {
                        return lhs.score < rhs.score;
                      }
                      return lhs.deterministic_index < rhs.deterministic_index;
                    });
  if (result.ranked_candidates.empty()) {
    result.reject_reason = RolloutRejectReason::kNoCandidate;
  }
  return result;
}

const RolloutPlannerConfig& RecedingHorizonTrajectoryPlanner::config() const noexcept {
  return config_;
}

const char*
rolloutTraversabilityTierName(const RolloutTraversabilityTier tier) noexcept {
  switch (tier) {
    case RolloutTraversabilityTier::kPlanningClearance:
      return "planning_clearance";
    case RolloutTraversabilityTier::kRuntimeProhibited:
      return "runtime_prohibited";
    case RolloutTraversabilityTier::kRawClear:
      return "raw_clear";
  }
  return "unknown";
}

const char* rolloutRejectReasonName(const RolloutRejectReason reason) noexcept {
  switch (reason) {
    case RolloutRejectReason::kNone:
      return "none";
    case RolloutRejectReason::kInvalidInput:
      return "invalid_input";
    case RolloutRejectReason::kOutsideGrid:
      return "outside_grid";
    case RolloutRejectReason::kRawOccupied:
      return "raw_occupied";
    case RolloutRejectReason::kNoCandidate:
      return "no_candidate";
  }
  return "unknown";
}

} // namespace drone_city_nav
