#include "drone_city_nav/repair_race.hpp"

#include "drone_city_nav/path_smoothing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stop_token>
#include <thread>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kEndpointToleranceM = 1.0;

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

class CompletionMailbox {
public:
  void push(RepairResult result) {
    {
      const std::scoped_lock lock{mutex_};
      results_.push_back(std::move(result));
    }
    condition_.notify_one();
  }

  [[nodiscard]] RepairResult next() {
    std::unique_lock lock{mutex_};
    condition_.wait(lock, [this]() { return !results_.empty(); });
    RepairResult result = std::move(results_.front());
    results_.pop_front();
    return result;
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<RepairResult> results_;
};

[[nodiscard]] std::vector<TrajectoryGridCandidate>
gridCandidates(const RepairSnapshot& snapshot) {
  std::vector<TrajectoryGridCandidate> candidates;
  candidates.reserve(snapshot.grids.size());
  for (const RepairGridSnapshot& grid : snapshot.grids) {
    candidates.push_back(TrajectoryGridCandidate{
        .name = grid.name,
        .grid = &grid.grid,
        .clearance_field = &grid.clearance,
        .clearance_field_cache_hit = true,
    });
  }
  return candidates;
}

[[nodiscard]] AStarConfig astarConfig(const RepairRaceConfig& config,
                                      const TrajectoryPointSample& anchor,
                                      const bool after_hold) {
  AStarConfig astar = config.moving_astar;
  if (after_hold) {
    astar.initial_heading_bias_enabled = false;
    astar.initial_heading_bias_velocity_x_mps = 0.0;
    astar.initial_heading_bias_velocity_y_mps = 0.0;
    return astar;
  }
  const double tangent_norm = std::hypot(anchor.tangent.x, anchor.tangent.y);
  if (tangent_norm > 1.0e-6) {
    const double speed_mps = std::max(1.0, astar.initial_heading_bias_min_speed_mps);
    astar.initial_heading_bias_velocity_x_mps =
        anchor.tangent.x * speed_mps / tangent_norm;
    astar.initial_heading_bias_velocity_y_mps =
        anchor.tangent.y * speed_mps / tangent_norm;
  }
  return astar;
}

struct RouteBuild {
  std::vector<Point2> points;
  std::size_t grid_index{0U};
};

[[nodiscard]] std::optional<RouteBuild>
computeRoute(const RepairSnapshot& snapshot, const RepairRaceConfig& config,
             const Point2 start, const Point2 goal, const bool after_hold,
             const std::stop_token stop_token) {
  for (std::size_t grid_index = 0U; grid_index < snapshot.grids.size(); ++grid_index) {
    if (stop_token.stop_requested()) {
      return std::nullopt;
    }
    const RepairGridSnapshot& grid = snapshot.grids[grid_index];
    PlannerCore core{config.planner_core};
    const auto path = core.computePath(PathComputationInput{
        .grid = &grid.grid,
        .current_position = start,
        .goal = goal,
        .astar = astarConfig(config, snapshot.anchor, after_hold),
        .prohibited_clearance_field = &grid.clearance,
        .prohibited_clearance_field_cache_hit = true,
        .stop_token = stop_token,
    });
    if (!path.has_value()) {
      continue;
    }
    const std::vector<GridIndex>& cells =
        path->smoothed_cells.empty() ? path->astar.path : path->smoothed_cells;
    std::vector<Point2> points = cellsToPoints(grid.grid, cells);
    if (points.empty()) {
      continue;
    }
    if (distance(start, points.front()) > 1.0e-6) {
      points.insert(points.begin(), start);
    } else {
      points.front() = start;
    }
    if (distance(goal, points.back()) > 1.0e-6) {
      points.push_back(goal);
    } else {
      points.back() = goal;
    }
    if (!pathIsTraversable(grid.grid, points)) {
      continue;
    }
    return RouteBuild{.points = std::move(points), .grid_index = grid_index};
  }
  return std::nullopt;
}

[[nodiscard]] TrajectoryPlannerConfig
singleThreadConfig(const RepairRaceConfig& config) {
  TrajectoryPlannerConfig trajectory = config.trajectory;
  trajectory.corridor.parallel_workers = 1U;
  trajectory.trajectory_optimizer.parallel_workers = 1U;
  return trajectory;
}

[[nodiscard]] TrajectoryPlannerResult planGeometry(const RepairSnapshot& snapshot,
                                                   const RepairRaceConfig& config,
                                                   const RouteBuild& route,
                                                   const bool after_hold,
                                                   const std::stop_token stop_token) {
  TrajectoryPlannerConfig geometry_config = singleThreadConfig(config);
  geometry_config.vertical_profile.enabled = false;
  geometry_config.known_passage_validation.enabled = false;
  geometry_config.passage_insertion.enabled = false;
  const std::vector<TrajectoryGridCandidate> grids = gridCandidates(snapshot);
  return planOptimizedTrajectory(
      TrajectoryPlannerInput{
          .route_points = route.points,
          .prohibited_grid = &snapshot.grids[route.grid_index].grid,
          .prohibited_clearance_field = &snapshot.grids[route.grid_index].clearance,
          .prohibited_clearance_field_cache_hit = true,
          .precomputed_corridor_samples = {},
          .known_passage_map = nullptr,
          .grid_candidates = grids,
          .passage_insertion_start_mode =
              after_hold ? PassageInsertionStartMode::kTerminalHoldRestart
                         : PassageInsertionStartMode::kMovingJoin,
          .stop_token = stop_token,
      },
      geometry_config);
}

[[nodiscard]] TrajectoryPlannerResult
finalize(const RepairSnapshot& snapshot, const RepairRaceConfig& config,
         const std::span<const TrajectoryPointSample> samples, const bool after_hold) {
  const std::vector<TrajectoryGridCandidate> grids = gridCandidates(snapshot);
  TrajectoryPlannerConfig trajectory_config = singleThreadConfig(config);
  trajectory_config.initial_altitude_m = snapshot.anchor.z_m;
  return finalizeStitchedTrajectory(
      StitchedTrajectoryFinalizationInput{
          .geometry_samples = samples,
          .known_passage_map =
              snapshot.passages.has_value() ? &*snapshot.passages : nullptr,
          .grid_candidates = grids,
          .start_mode = after_hold ? PassageInsertionStartMode::kTerminalHoldRestart
                                   : PassageInsertionStartMode::kMovingJoin,
      },
      trajectory_config);
}

[[nodiscard]] RepairResult partialJob(const RepairSnapshot& snapshot,
                                      const RepairRaceConfig& config,
                                      const ReconnectCandidate& reconnect,
                                      const std::stop_token stop_token) {
  const auto started_at = std::chrono::steady_clock::now();
  RepairResult result{};
  result.kind = RepairJobKind::kPartial;
  result.generation = snapshot.generation;
  result.blocked_path_id = snapshot.blocked_path_id;
  result.temporary_prefix_fingerprint = snapshot.temporary_prefix_fingerprint;
  result.source_grid_version = snapshot.grid_version;
  result.reconnect_margin_m = reconnect.margin_m;
  result.reconnect_s_m = reconnect.reconnect_s_m;
  for (const bool after_hold : {false, true}) {
    if (stop_token.stop_requested()) {
      result.canceled = true;
      result.reason = "canceled";
      break;
    }
    const auto route =
        computeRoute(snapshot, config, snapshot.anchor.point,
                     reconnect.reconnect_sample.point, after_hold, stop_token);
    if (!route.has_value()) {
      result.reason = "astar_failed";
      continue;
    }
    result.source_grid_index = route->grid_index;
    TrajectoryPlannerResult geometry =
        planGeometry(snapshot, config, *route, after_hold, stop_token);
    if (!geometry.valid) {
      result.canceled = geometry.stats.status == TrajectoryPlannerStatus::kCanceled;
      result.reason = result.canceled ? "canceled" : "geometry_invalid";
      continue;
    }
    const TrajectoryRepairStitchResult stitched =
        stitchTrajectoryRepair(geometry.samples, snapshot.old_trajectory,
                               reconnect.reconnect_s_m, kEndpointToleranceM);
    if (!stitched.valid) {
      result.reason = stitched.reason;
      continue;
    }
    result.trajectory = finalize(snapshot, config, stitched.samples, after_hold);
    if (!result.trajectory.valid) {
      result.reason = "finalization_invalid";
      continue;
    }
    result.route_points.reserve(result.trajectory.samples.size());
    for (const TrajectoryPointSample& sample : result.trajectory.samples) {
      result.route_points.push_back(sample.point);
    }
    result.activation_mode = after_hold ? TruncationSuffixActivationMode::kAfterHold
                                        : TruncationSuffixActivationMode::kMovingJoin;
    result.valid = true;
    result.reason = "ok";
    break;
  }
  result.duration_ms = elapsedMilliseconds(started_at);
  return result;
}

[[nodiscard]] RepairResult fullJob(const RepairSnapshot& snapshot,
                                   const RepairRaceConfig& config,
                                   const std::stop_token stop_token) {
  const auto started_at = std::chrono::steady_clock::now();
  RepairResult result{};
  result.kind = RepairJobKind::kFull;
  result.generation = snapshot.generation;
  result.blocked_path_id = snapshot.blocked_path_id;
  result.temporary_prefix_fingerprint = snapshot.temporary_prefix_fingerprint;
  result.source_grid_version = snapshot.grid_version;
  for (const bool after_hold : {false, true}) {
    if (stop_token.stop_requested()) {
      result.canceled = true;
      result.reason = "canceled";
      break;
    }
    const auto route =
        computeRoute(snapshot, config, snapshot.anchor.point,
                     snapshot.old_trajectory.mission_goal, after_hold, stop_token);
    if (!route.has_value()) {
      result.reason = "astar_failed";
      continue;
    }
    result.source_grid_index = route->grid_index;
    const std::vector<TrajectoryGridCandidate> grids = gridCandidates(snapshot);
    TrajectoryPlannerConfig trajectory_config = singleThreadConfig(config);
    trajectory_config.initial_altitude_m = snapshot.anchor.z_m;
    result.trajectory = planOptimizedTrajectory(
        TrajectoryPlannerInput{
            .route_points = route->points,
            .prohibited_grid = &snapshot.grids[route->grid_index].grid,
            .prohibited_clearance_field = &snapshot.grids[route->grid_index].clearance,
            .prohibited_clearance_field_cache_hit = true,
            .precomputed_corridor_samples = {},
            .known_passage_map =
                snapshot.passages.has_value() ? &*snapshot.passages : nullptr,
            .grid_candidates = grids,
            .passage_insertion_start_mode =
                after_hold ? PassageInsertionStartMode::kTerminalHoldRestart
                           : PassageInsertionStartMode::kMovingJoin,
            .stop_token = stop_token,
        },
        trajectory_config);
    if (!result.trajectory.valid) {
      result.canceled =
          result.trajectory.stats.status == TrajectoryPlannerStatus::kCanceled;
      result.reason = result.canceled ? "canceled" : "trajectory_invalid";
      continue;
    }
    result.route_points = route->points;
    result.activation_mode = after_hold ? TruncationSuffixActivationMode::kAfterHold
                                        : TruncationSuffixActivationMode::kMovingJoin;
    result.valid = true;
    result.reason = "ok";
    break;
  }
  result.duration_ms = elapsedMilliseconds(started_at);
  return result;
}

} // namespace

RepairRaceArbiter::RepairRaceArbiter(const RepairSnapshot& snapshot)
    : generation_{snapshot.generation},
      blocked_path_id_{snapshot.blocked_path_id},
      temporary_prefix_fingerprint_{snapshot.temporary_prefix_fingerprint},
      grid_version_{snapshot.grid_version} {
}

bool RepairRaceArbiter::consider(const RepairResult& result) {
  const std::scoped_lock lock{mutex_};
  if (winner_selected_ || !result.valid || result.generation != generation_ ||
      result.blocked_path_id != blocked_path_id_ ||
      result.temporary_prefix_fingerprint != temporary_prefix_fingerprint_ ||
      !planningGridVersionsEqual(result.source_grid_version, grid_version_)) {
    return false;
  }
  winner_selected_ = true;
  return true;
}

bool RepairRaceArbiter::winnerSelected() const noexcept {
  const std::scoped_lock lock{mutex_};
  return winner_selected_;
}

RepairRaceOutcome runRepairRace(std::shared_ptr<const RepairSnapshot> snapshot,
                                const RepairRaceConfig& config,
                                const RepairAcceptanceValidator& acceptance_validator,
                                const RepairWinnerHandoff& winner_handoff) {
  RepairRaceOutcome outcome{};
  if (snapshot == nullptr || snapshot->generation == 0U ||
      snapshot->blocked_path_id == 0U || snapshot->grids.empty() ||
      !trajectorySamplesAreUsable(snapshot->old_trajectory.samples)) {
    return outcome;
  }

  std::vector<const OccupancyGrid2D*> endpoint_grids;
  endpoint_grids.reserve(snapshot->grids.size());
  for (const RepairGridSnapshot& grid : snapshot->grids) {
    endpoint_grids.push_back(&grid.grid);
  }
  const std::vector<ReconnectCandidate> reconnects = makeReconnectCandidates(
      snapshot->old_trajectory, snapshot->blocked_span, snapshot->truncation_s_m,
      config.reconnect_margins_m, endpoint_grids);
  std::vector<RepairJob> repair_jobs;
  repair_jobs.reserve(reconnects.size() + 1U);
  for (const ReconnectCandidate& reconnect : reconnects) {
    repair_jobs.emplace_back(
        [snapshot, config, reconnect](const std::stop_token stop_token) {
          return partialJob(*snapshot, config, reconnect, stop_token);
        });
  }
  repair_jobs.emplace_back([snapshot, config](const std::stop_token stop_token) {
    return fullJob(*snapshot, config, stop_token);
  });
  return runRepairJobs(snapshot, repair_jobs, acceptance_validator, winner_handoff);
}

RepairRaceOutcome runRepairJobs(std::shared_ptr<const RepairSnapshot> snapshot,
                                const std::span<const RepairJob> repair_jobs,
                                const RepairAcceptanceValidator& acceptance_validator,
                                const RepairWinnerHandoff& winner_handoff) {
  RepairRaceOutcome outcome{};
  if (snapshot == nullptr || snapshot->generation == 0U ||
      snapshot->blocked_path_id == 0U || repair_jobs.empty()) {
    return outcome;
  }

  const std::size_t job_count = repair_jobs.size();
  outcome.summary.jobs_started = job_count;
  CompletionMailbox mailbox;
  std::stop_source stop_source;
  std::vector<std::jthread> jobs;
  jobs.reserve(job_count);
  for (const RepairJob& repair_job : repair_jobs) {
    jobs.emplace_back([repair_job, &mailbox, token = stop_source.get_token()]() {
      mailbox.push(repair_job(token));
    });
  }

  RepairRaceArbiter arbiter{*snapshot};
  for (std::size_t completion = 0U; completion < job_count; ++completion) {
    RepairResult result = mailbox.next();
    ++outcome.summary.completions;
    if (result.valid && acceptance_validator && !acceptance_validator(result)) {
      result.valid = false;
      result.reason = "fresh_validation_failed";
    }
    outcome.completions.push_back(RepairCompletionDiagnostic{
        .kind = result.kind,
        .reconnect_margin_m = result.reconnect_margin_m,
        .reconnect_s_m = result.reconnect_s_m,
        .source_grid_index = result.source_grid_index,
        .activation_mode = result.activation_mode,
        .reason = result.reason,
        .duration_ms = result.duration_ms,
        .valid = result.valid,
        .canceled = result.canceled,
    });
    if (result.canceled) {
      ++outcome.summary.canceled_results;
    } else if (!result.valid) {
      ++outcome.summary.invalid_results;
    }
    if (!arbiter.consider(result)) {
      continue;
    }
    stop_source.request_stop();
    outcome.summary.winner_selected = true;
    if (winner_handoff) {
      winner_handoff(result, outcome.summary.completions, job_count);
    }
    outcome.winner = std::move(result);
  }
  return outcome;
}

RepairFreshValidationResult
validateRepairResultOnFreshGrid(const RepairFreshValidationInput& input) {
  if (input.candidate == nullptr || input.fresh_grid_version == nullptr ||
      input.fresh_runtime_grid == nullptr) {
    return {false, RepairFreshValidationReason::kInvalidInput};
  }
  if (input.fresh_grid_version->build_revision <
      input.candidate->source_grid_version.build_revision) {
    return {false, RepairFreshValidationReason::kStaleGridRevision};
  }
  if (!input.candidate->trajectory.valid ||
      !trajectorySamplesAreUsable(input.candidate->trajectory.samples)) {
    return {false, RepairFreshValidationReason::kInvalidTrajectory};
  }
  if (!input.candidate->trajectory.stats.known_passage_solid_validation.valid) {
    return {false, RepairFreshValidationReason::kKnownSolidIntersection};
  }
  std::vector<Point2> candidate_points;
  candidate_points.reserve(input.candidate->trajectory.samples.size());
  for (const TrajectoryPointSample& sample : input.candidate->trajectory.samples) {
    candidate_points.push_back(sample.point);
  }
  if (!pathIsTraversable(*input.fresh_runtime_grid, candidate_points)) {
    return {false, RepairFreshValidationReason::kCandidateBlocked};
  }
  if (!input.remaining_prefix.empty() &&
      !pathIsTraversable(*input.fresh_runtime_grid, input.remaining_prefix)) {
    return {false, RepairFreshValidationReason::kPrefixBlocked};
  }
  return {true, RepairFreshValidationReason::kAccepted};
}

const char*
repairFreshValidationReasonName(const RepairFreshValidationReason reason) noexcept {
  switch (reason) {
    case RepairFreshValidationReason::kAccepted:
      return "accepted";
    case RepairFreshValidationReason::kInvalidInput:
      return "invalid_input";
    case RepairFreshValidationReason::kStaleGridRevision:
      return "stale_grid_revision";
    case RepairFreshValidationReason::kInvalidTrajectory:
      return "invalid_trajectory";
    case RepairFreshValidationReason::kKnownSolidIntersection:
      return "known_solid_intersection";
    case RepairFreshValidationReason::kCandidateBlocked:
      return "candidate_blocked";
    case RepairFreshValidationReason::kPrefixBlocked:
      return "prefix_blocked";
  }
  return "unknown";
}

const char* repairJobKindName(const RepairJobKind kind) noexcept {
  switch (kind) {
    case RepairJobKind::kPartial:
      return "partial";
    case RepairJobKind::kFull:
      return "full";
  }
  return "unknown";
}

} // namespace drone_city_nav
