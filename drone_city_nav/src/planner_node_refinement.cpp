#include <exception>
#include <string>
#include <utility>

#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::startAsyncExecutableTrajectoryBuild(
    const OccupancyGrid2D& grid, const std::span<const Point2> route_points,
    const std::uint64_t generation, const char* source_label,
    const ClearanceField2D* prohibited_clearance_field,
    const bool prohibited_clearance_field_cache_hit,
    const TrajectoryPlannerConfig& config) {
  if (async_trajectory_build_workers_ == 0U || route_points.size() < 2U ||
      pending_trajectory_build_.has_value()) {
    return;
  }

  std::vector<Point2> route_snapshot{route_points.begin(), route_points.end()};
  std::optional<ClearanceField2D> clearance_snapshot =
      prohibited_clearance_field != nullptr
          ? std::optional<ClearanceField2D>{*prohibited_clearance_field}
          : std::nullopt;
  std::optional<KnownPassageMap> known_passage_snapshot = known_passages_;
  std::future<TrajectoryPlannerResult> future = std::async(
      std::launch::async,
      [grid_snapshot = grid, route = route_snapshot,
       clearance = std::move(clearance_snapshot),
       clearance_cache_hit = prohibited_clearance_field_cache_hit,
       known_passages = std::move(known_passage_snapshot), config]() mutable {
        const ClearanceField2D* clearance_ptr =
            clearance.has_value() ? &*clearance : nullptr;
        TrajectoryPlannerResult result = planOptimizedTrajectoryFromSnapshots(
            route, grid_snapshot, clearance_ptr, clearance_cache_hit, {}, nullptr,
            known_passages.has_value() ? &*known_passages : nullptr, config);
        result.stats.quality = TrajectoryQuality::kRefined;
        result.stats.trajectory_optimizer.async_refined = true;
        return result;
      });

  pending_trajectory_build_ = PendingExecutableTrajectoryBuild{
      .generation = generation,
      .route_start = route_snapshot.front(),
      .goal = route_snapshot.back(),
      .route_points = std::move(route_snapshot),
      .source_label = source_label,
      .started_at = std::chrono::steady_clock::now(),
      .future = std::move(future),
  };
  RCLCPP_INFO(get_logger(),
              "%s executable trajectory async build started: generation=%" PRIu64
              " route_points=%zu clearance_snapshot=%s",
              source_label, generation, pending_trajectory_build_->route_points.size(),
              prohibited_clearance_field != nullptr ? "true" : "false");
}

bool PlannerNode::pollPendingExecutableTrajectoryBuild(
    const OccupancyGrid2D& validation_grid) {
  if (!pending_trajectory_build_.has_value() ||
      !pending_trajectory_build_->future.valid() ||
      pending_trajectory_build_->future.wait_for(std::chrono::seconds{0}) !=
          std::future_status::ready) {
    return false;
  }

  PendingExecutableTrajectoryBuild pending = std::move(*pending_trajectory_build_);
  pending_trajectory_build_.reset();
  TrajectoryPlannerResult result{};
  try {
    result = pending.future.get();
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(),
                "%s executable trajectory async build failed: generation=%" PRIu64
                " error='%s'",
                pending.source_label.c_str(), pending.generation, error.what());
    return false;
  }

  const double wall_duration_ms = elapsedMilliseconds(pending.started_at);
  const std::vector<Point2> result_points = trajectorySamplePoints(result.samples);
  const TrajectoryRefinementDecision decision =
      evaluateTrajectoryRefinement(TrajectoryRefinementDecisionInput{
          .current_generation = trajectory_generation_,
          .snapshot_generation = pending.generation,
          .expected_start = pending.route_start,
          .expected_goal = pending.goal,
          .endpoint_tolerance_m = stable_path_goal_tolerance_m_,
          .refined = &result,
          .refined_points = result_points,
          .validation_grid = &validation_grid,
      });
  if (!decision.accepted) {
    RCLCPP_WARN(get_logger(),
                "%s executable trajectory async build rejected: reason=%.*s "
                "generation=%" PRIu64 " current_generation=%" PRIu64
                " route_points=%zu result_points=%zu wall_duration_ms=%.1f",
                pending.source_label.c_str(),
                static_cast<int>(refinementDecisionReasonName(decision.reason).size()),
                refinementDecisionReasonName(decision.reason).data(),
                pending.generation, trajectory_generation_, pending.route_points.size(),
                result_points.size(), wall_duration_ms);
    return false;
  }

  RCLCPP_INFO(get_logger(),
              "%s executable trajectory async build accepted: generation=%" PRIu64
              " route_points=%zu result_points=%zu wall_duration_ms=%.1f "
              "planner_duration_ms=%.1f",
              pending.source_label.c_str(), pending.generation,
              pending.route_points.size(), result_points.size(), wall_duration_ms,
              result.stats.total_duration_ms);
  return publishTrajectoryResult(validation_grid, result, pending.route_points,
                                 pending.source_label.c_str(), wall_duration_ms);
}

} // namespace drone_city_nav
