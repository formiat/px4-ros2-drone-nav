#include <algorithm>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "planner_node.hpp"

namespace drone_city_nav {
void PlannerNode::startAsyncTrajectoryRefinement(
    const OccupancyGrid2D& grid, const std::span<const Point2> route_points,
    const std::uint64_t generation, const std::uint64_t baseline_path_id,
    const TrajectoryPlannerResult& baseline, const char* source_label,
    const ClearanceField2D* prohibited_clearance_field,
    const bool prohibited_clearance_field_cache_hit) {
  if (route_points.size() < 2U || !baseline.valid) {
    return;
  }

  const TrajectoryRefinementJob job{generation, baseline_path_id};
  const TrajectoryRefinementScheduleDecision schedule =
      refinement_scheduler_.submit(job);
  if (schedule.action == TrajectoryRefinementScheduleAction::kDisabled) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s refined trajectory async build disabled: generation=%" PRIu64
        " baseline_path_id=%" PRIu64 " async_workers=%zu",
        source_label, generation, baseline_path_id,
        refinement_scheduler_.workerCount());
    return;
  }

  TrajectoryRefinementRequest request{
      .generation = generation,
      .baseline_path_id = baseline_path_id,
      .route_start = route_points.front(),
      .goal = route_points.back(),
      .baseline_length_m = baseline.stats.length_m,
      .route_points = std::vector<Point2>{route_points.begin(), route_points.end()},
      .source_label = source_label,
      .grid = grid,
      .prohibited_clearance_field =
          prohibited_clearance_field != nullptr
              ? std::optional<ClearanceField2D>{*prohibited_clearance_field}
              : std::nullopt,
      .prohibited_clearance_field_cache_hit = prohibited_clearance_field_cache_hit,
      .corridor_samples = baseline.corridor_samples,
      .corridor_stats = baseline.stats.corridor,
      .known_passages = known_passages_,
      .config = trajectory_planner_config_};
  if (schedule.action == TrajectoryRefinementScheduleAction::kQueuedLatest ||
      schedule.action == TrajectoryRefinementScheduleAction::kReplacedQueuedLatest) {
    queued_refinement_ = std::move(request);
    const std::uint64_t active_generation =
        schedule.active_job.has_value() ? schedule.active_job->generation : 0U;
    RCLCPP_INFO(get_logger(),
                "%s refined trajectory async build queued: action=%s "
                "active_generation=%" PRIu64 " queued_generation=%" PRIu64
                " baseline_path_id=%" PRIu64,
                source_label,
                schedule.action ==
                        TrajectoryRefinementScheduleAction::kReplacedQueuedLatest
                    ? "replaced_latest"
                    : "queued_latest",
                active_generation, generation, baseline_path_id);
    return;
  }

  launchScheduledTrajectoryRefinement(std::move(request));
}

void PlannerNode::launchScheduledTrajectoryRefinement(
    TrajectoryRefinementRequest request) {
  std::vector<Point2> route_for_build = request.route_points;
  const bool clearance_snapshot_available =
      request.prohibited_clearance_field.has_value();
  const std::size_t corridor_sample_count = request.corridor_samples.size();
  std::future<TrajectoryPlannerResult> future = std::async(
      std::launch::async,
      [grid = std::move(request.grid), route = std::move(route_for_build),
       clearance_field = std::move(request.prohibited_clearance_field),
       clearance_cache_hit = request.prohibited_clearance_field_cache_hit,
       corridor_samples = std::move(request.corridor_samples),
       corridor_stats = request.corridor_stats,
       known_passages = std::move(request.known_passages),
       config = request.config]() mutable {
        const ClearanceField2D* clearance_field_ptr =
            clearance_field.has_value() ? &*clearance_field : nullptr;
        const CorridorStats* corridor_stats_ptr =
            corridor_samples.size() >= 2U ? &corridor_stats : nullptr;
        TrajectoryPlannerResult refined = planOptimizedTrajectoryFromSnapshots(
            std::span<const Point2>{route.data(), route.size()}, grid,
            clearance_field_ptr, clearance_cache_hit,
            std::span<const CorridorSample>{corridor_samples.data(),
                                            corridor_samples.size()},
            corridor_stats_ptr, known_passages.has_value() ? &*known_passages : nullptr,
            config);
        refined.stats.quality = TrajectoryQuality::kRefined;
        refined.stats.trajectory_optimizer.async_refined = true;
        return refined;
      });

  PendingTrajectoryRefinement pending{};
  pending.generation = request.generation;
  pending.baseline_path_id = request.baseline_path_id;
  pending.route_start = request.route_start;
  pending.goal = request.goal;
  pending.baseline_length_m = request.baseline_length_m;
  pending.route_points = std::move(request.route_points);
  pending.source_label = std::move(request.source_label);
  pending.future = std::move(future);
  pending_refinement_ = std::move(pending);
  RCLCPP_INFO(get_logger(),
              "%s refined trajectory async build started: generation=%" PRIu64
              " baseline_path_id=%" PRIu64 " route_points=%zu baseline_length=%.2fm "
              "clearance_snapshot=%s corridor_samples=%zu",
              pending_refinement_->source_label.c_str(),
              pending_refinement_->generation, pending_refinement_->baseline_path_id,
              pending_refinement_->route_points.size(),
              pending_refinement_->baseline_length_m,
              clearance_snapshot_available ? "true" : "false", corridor_sample_count);
}

void PlannerNode::launchQueuedTrajectoryRefinement(
    const std::optional<TrajectoryRefinementJob> expected_job) {
  if (!expected_job.has_value()) {
    return;
  }
  if (!queued_refinement_.has_value()) {
    RCLCPP_WARN(get_logger(),
                "Refined trajectory scheduler expected queued job but no queued "
                "request snapshot is available: generation=%" PRIu64
                " baseline_path_id=%" PRIu64,
                expected_job->generation, expected_job->baseline_path_id);
    return;
  }
  if (queued_refinement_->generation != expected_job->generation ||
      queued_refinement_->baseline_path_id != expected_job->baseline_path_id) {
    RCLCPP_WARN(
        get_logger(),
        "Refined trajectory queued request mismatch: expected_generation=%" PRIu64
        " queued_generation=%" PRIu64 " expected_baseline_path_id=%" PRIu64
        " queued_baseline_path_id=%" PRIu64,
        expected_job->generation, queued_refinement_->generation,
        expected_job->baseline_path_id, queued_refinement_->baseline_path_id);
    queued_refinement_.reset();
    return;
  }

  TrajectoryRefinementRequest request = std::move(*queued_refinement_);
  queued_refinement_.reset();
  launchScheduledTrajectoryRefinement(std::move(request));
}

bool PlannerNode::pollPendingTrajectoryRefinement(
    const OccupancyGrid2D& validation_grid) {
  if (!pending_refinement_.has_value() || !pending_refinement_->future.valid()) {
    return false;
  }
  if (pending_refinement_->future.wait_for(std::chrono::seconds{0}) !=
      std::future_status::ready) {
    return false;
  }

  PendingTrajectoryRefinement pending = std::move(*pending_refinement_);
  pending_refinement_.reset();
  const std::optional<TrajectoryRefinementJob> queued_job =
      refinement_scheduler_.completeActive(
          TrajectoryRefinementJob{pending.generation, pending.baseline_path_id});
  TrajectoryPlannerResult refined{};
  try {
    refined = pending.future.get();
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(),
                "%s refined trajectory async build failed with exception: "
                "generation=%" PRIu64 " baseline_path_id=%" PRIu64 " error='%s'",
                pending.source_label.c_str(), pending.generation,
                pending.baseline_path_id, error.what());
    launchQueuedTrajectoryRefinement(queued_job);
    return false;
  }

  if (last_published_path_id_ != pending.baseline_path_id) {
    RCLCPP_WARN(
        get_logger(),
        "%s refined trajectory rejected: reason=path_id_mismatch generation=%" PRIu64
        " baseline_path_id=%" PRIu64 " last_published_path_id=%" PRIu64,
        pending.source_label.c_str(), pending.generation, pending.baseline_path_id,
        last_published_path_id_);
    launchQueuedTrajectoryRefinement(queued_job);
    return false;
  }

  const std::vector<Point2> refined_points = trajectorySamplePoints(refined.samples);
  const TrajectoryRefinementDecision decision =
      evaluateTrajectoryRefinement(TrajectoryRefinementDecisionInput{
          .current_generation = trajectory_generation_,
          .snapshot_generation = pending.generation,
          .expected_start = pending.route_start,
          .expected_goal = pending.goal,
          .endpoint_tolerance_m = stable_path_goal_tolerance_m_,
          .refined = &refined,
          .refined_points =
              std::span<const Point2>{refined_points.data(), refined_points.size()},
          .validation_grid = &validation_grid,
      });
  if (!decision.accepted) {
    RCLCPP_WARN(get_logger(),
                "%s refined trajectory rejected: reason=%.*s generation=%" PRIu64
                " current_generation=%" PRIu64 " baseline_path_id=%" PRIu64
                " route_points=%zu refined_points=%zu refined_time=%.2fs "
                "baseline_length=%.2fm refined_length=%.2fm",
                pending.source_label.c_str(),
                static_cast<int>(refinementDecisionReasonName(decision.reason).size()),
                refinementDecisionReasonName(decision.reason).data(),
                pending.generation, trajectory_generation_, pending.baseline_path_id,
                pending.route_points.size(), refined_points.size(),
                refined.stats.trajectory_optimizer.estimated_time_s,
                pending.baseline_length_m, refined.stats.length_m);
    launchQueuedTrajectoryRefinement(queued_job);
    return false;
  }

  RCLCPP_INFO(get_logger(),
              "%s refined trajectory accepted: generation=%" PRIu64
              " baseline_path_id=%" PRIu64 " refined_points=%zu refined_time=%.2fs "
              "baseline_length=%.2fm refined_length=%.2fm",
              pending.source_label.c_str(), pending.generation,
              pending.baseline_path_id, refined_points.size(),
              refined.stats.trajectory_optimizer.estimated_time_s,
              pending.baseline_length_m, refined.stats.length_m);
  const bool published = publishTrajectoryResult(
      validation_grid, refined, pending.route_points, pending.source_label.c_str(),
      refined.stats.total_duration_ms);
  launchQueuedTrajectoryRefinement(queued_job);
  return published;
}

} // namespace drone_city_nav
