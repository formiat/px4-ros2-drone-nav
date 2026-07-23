#include <chrono>
#include <memory>
#include <utility>

#include "planner_node.hpp"

namespace drone_city_nav {

bool PlannerNode::runConfirmedRepairRace(
    const PreparedPlanningGridSnapshot& prepared,
    const TruncationReplanState& truncation_replan,
    const TrajectoryDeliveryDiagnostics& delivery) {
  if (!partial_replan_config_.enabled || !truncation_replan.repair_context_valid) {
    return false;
  }

  auto snapshot = std::make_shared<RepairSnapshot>();
  snapshot->generation = truncation_replan.generation;
  snapshot->blocked_path_id = truncation_replan.blocked_path_id;
  snapshot->temporary_prefix_fingerprint =
      truncation_replan.temporary_prefix_fingerprint;
  snapshot->grid_version = prepared.version;
  snapshot->grids.push_back(RepairGridSnapshot{
      .name = "planning_clearance",
      .grid = prepared.planning_clearance_grid,
      .clearance = prepared.planning_clearance,
  });
  snapshot->grids.push_back(RepairGridSnapshot{
      .name = "runtime_prohibited",
      .grid = prepared.runtime_prohibited_grid,
      .clearance = prepared.runtime_clearance,
  });
  snapshot->old_trajectory = truncation_replan.old_trajectory;
  snapshot->anchor = trajectorySampleAtS(truncation_replan.old_trajectory.samples,
                                         truncation_replan.truncation_s_m);
  snapshot->anchor.point =
      Point2{truncation_replan.position.x, truncation_replan.position.y};
  snapshot->anchor.tangent = truncation_replan.tangent;
  snapshot->anchor.z_m = truncation_replan.altitude_m;
  snapshot->truncation_s_m = truncation_replan.truncation_s_m;
  snapshot->blocked_span = truncation_replan.blocked_span;
  snapshot->passages = known_passages_;

  RCLCPP_WARN(get_logger(),
              "REPAIR_RACE start generation=%" PRIu64 " blocked_path_id=%" PRIu64
              " prefix_fingerprint=%" PRIu64 " current_s=%.2f truncation_s=%.2f "
              "blocked_span=[%.2f,%.2f] trigger=%s A=(%.2f,%.2f,%.2f) "
              "grid_revision=%" PRIu64 " memory=%" PRIu64 "/%" PRIu64
              " lidar_update_ns=%" PRId64 " partial_jobs=%zu full_jobs=1",
              snapshot->generation, snapshot->blocked_path_id,
              snapshot->temporary_prefix_fingerprint, truncation_replan.current_s_m,
              snapshot->truncation_s_m, snapshot->blocked_span.first_blocked_s_m,
              snapshot->blocked_span.last_blocked_s_m,
              blockedSpanTriggerName(snapshot->blocked_span.trigger),
              snapshot->anchor.point.x, snapshot->anchor.point.y, snapshot->anchor.z_m,
              snapshot->grid_version.build_revision,
              snapshot->grid_version.memory_producer_instance_id,
              snapshot->grid_version.memory_sequence,
              snapshot->grid_version.lidar_update_ns,
              partial_replan_config_.reconnect_margins_m.size());

  const RepairAcceptanceValidator fresh_validator = [this, snapshot](
                                                        const RepairResult& candidate) {
    {
      const std::scoped_lock lock{truncation_replan_mutex_};
      if (!truncation_replan_state_.has_value() ||
          truncation_replan_state_->generation != candidate.generation ||
          truncation_replan_state_->blocked_path_id != candidate.blocked_path_id ||
          truncation_replan_state_->temporary_prefix_fingerprint !=
              candidate.temporary_prefix_fingerprint ||
          truncation_replan_state_->awaiting_ack ||
          executable_trajectory_artifact_.path_id != candidate.blocked_path_id) {
        return false;
      }
    }
    const NavigationStateSnapshot navigation = navigationStateSnapshot();
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    if (!navigation.pose_valid ||
        !timestampIsFresh(navigation.stamp_ns, now_ns, max_pose_staleness_ns_)) {
      return false;
    }
    applyNavigationStateSnapshot(navigation);
    applyPendingMemorySnapshot(now_ns);
    applyLatestLidarInputSnapshot();
    const std::optional<PlanningGridBuildResult> latest_build =
        buildPlanningGrid(now_ns);
    if (!latest_build.has_value()) {
      return false;
    }
    const std::optional<PreparedPlanningGridSnapshot> latest =
        preparePlanningGridSnapshot(*latest_build, navigation.pose.position);
    if (!latest.has_value()) {
      return false;
    }

    const std::optional<TrajectoryProjection> current_projection =
        projectOnTrajectorySamples(snapshot->old_trajectory.samples,
                                   navigation.pose.position,
                                   snapshot->old_trajectory.current_s_m);
    if (!current_projection.has_value() ||
        current_projection->s_m > snapshot->truncation_s_m + 1.0) {
      return false;
    }
    const double prefix_start_s_m =
        std::min(current_projection->s_m, snapshot->truncation_s_m);
    std::vector<Point2> prefix_points;
    prefix_points.push_back(
        trajectorySampleAtS(snapshot->old_trajectory.samples, prefix_start_s_m).point);
    for (const TrajectoryPointSample& sample : snapshot->old_trajectory.samples) {
      if (sample.s_m > prefix_start_s_m && sample.s_m < snapshot->truncation_s_m) {
        prefix_points.push_back(sample.point);
      }
    }
    prefix_points.push_back(snapshot->anchor.point);
    const RepairFreshValidationResult validation =
        validateRepairResultOnFreshGrid(RepairFreshValidationInput{
            .candidate = &candidate,
            .fresh_grid_version = &latest->version,
            .fresh_runtime_grid = &latest->runtime_prohibited_grid,
            .remaining_prefix = prefix_points,
        });
    if (!validation.valid) {
      RCLCPP_WARN(get_logger(),
                  "REPAIR_RACE fresh validation rejected generation=%" PRIu64
                  " kind=%s margin=%.1f reason=%s source_revision=%" PRIu64
                  " fresh_revision=%" PRIu64,
                  candidate.generation, repairJobKindName(candidate.kind),
                  candidate.reconnect_margin_m,
                  repairFreshValidationReasonName(validation.reason),
                  candidate.source_grid_version.build_revision,
                  latest->version.build_revision);
    }
    return validation.valid;
  };
  const RepairRaceOutcome outcome = runRepairRace(
      snapshot,
      RepairRaceConfig{
          .planner_core = planner_core_.config(),
          .moving_astar = astarConfigForCurrentVelocity(truncation_replan.tangent),
          .trajectory =
              trajectoryPlannerConfigForCurrentAltitude(truncation_replan.altitude_m),
          .reconnect_margins_m = partial_replan_config_.reconnect_margins_m,
      },
      fresh_validator,
      [this, snapshot, truncation_replan,
       delivery](const RepairResult& winner, const std::size_t completion_index,
                 const std::size_t jobs_started) mutable {
        handoffRepairRaceWinner(winner, snapshot, truncation_replan, delivery,
                                completion_index, jobs_started);
      });
  for (std::size_t index = 0U; index < outcome.completions.size(); ++index) {
    const RepairCompletionDiagnostic& completion = outcome.completions[index];
    RCLCPP_INFO(
        get_logger(),
        "REPAIR_RACE completion=%zu kind=%s margin=%.1f reconnect_s=%.2f "
        "grid_index=%zu activation=%s valid=%s canceled=%s reason=%s "
        "duration_ms=%.1f",
        index + 1U, repairJobKindName(completion.kind), completion.reconnect_margin_m,
        completion.reconnect_s_m, completion.source_grid_index,
        truncationSuffixActivationModeName(completion.activation_mode),
        completion.valid ? "true" : "false", completion.canceled ? "true" : "false",
        completion.reason.c_str(), completion.duration_ms);
  }
  if (!outcome.winner.has_value()) {
    RCLCPP_ERROR(get_logger(),
                 "REPAIR_RACE completed generation=%" PRIu64
                 " winner=none jobs=%zu completions=%zu invalid=%zu canceled=%zu "
                 "action=keep_temporary_hold",
                 snapshot->generation, outcome.summary.jobs_started,
                 outcome.summary.completions, outcome.summary.invalid_results,
                 outcome.summary.canceled_results);
    return true;
  }
  RCLCPP_INFO(get_logger(),
              "REPAIR_RACE cleanup completed generation=%" PRIu64
              " jobs=%zu completions=%zu invalid=%zu canceled=%zu",
              snapshot->generation, outcome.summary.jobs_started,
              outcome.summary.completions, outcome.summary.invalid_results,
              outcome.summary.canceled_results);
  return true;
}

void PlannerNode::handoffRepairRaceWinner(
    const RepairResult& winner, const std::shared_ptr<const RepairSnapshot>& snapshot,
    const TruncationReplanState& truncation_replan,
    TrajectoryDeliveryDiagnostics delivery, const std::size_t completion_index,
    const std::size_t jobs_started) {
  {
    const std::scoped_lock lock{truncation_replan_mutex_};
    if (!truncation_replan_state_.has_value() ||
        truncation_replan_state_->generation != winner.generation ||
        truncation_replan_state_->blocked_path_id != winner.blocked_path_id ||
        truncation_replan_state_->temporary_prefix_fingerprint !=
            winner.temporary_prefix_fingerprint ||
        truncation_replan_state_->awaiting_ack) {
      RCLCPP_WARN(get_logger(),
                  "REPAIR_RACE winner discarded reason=stale_generation "
                  "generation=%" PRIu64 " kind=%s margin=%.1f",
                  winner.generation, repairJobKindName(winner.kind),
                  winner.reconnect_margin_m);
      return;
    }
  }

  delivery.generation = ++trajectory_generation_;
  const std::int64_t build_stamp_ns = get_clock()->now().nanoseconds();
  delivery.trajectory_build_started_stamp_ns =
      build_stamp_ns > 0 ? static_cast<std::uint64_t>(build_stamp_ns) : 0U;
  delivery.blocked_path_id = truncation_replan.blocked_path_id;
  delivery.truncation_generation = truncation_replan.generation;
  delivery.temporary_prefix_fingerprint =
      truncation_replan.temporary_prefix_fingerprint;
  delivery.truncation_suffix = true;
  delivery.truncation_immediate_hold = truncation_replan.immediate_hold;
  delivery.truncation_suffix_activation_mode =
      static_cast<std::uint8_t>(winner.activation_mode);
  delivery.candidate_start_position = winner.trajectory.samples.front().point;
  delivery.planning_start_position = snapshot->anchor.point;
  delivery.planning_start_velocity = current_velocity_;
  delivery.planning_start_velocity_valid = current_velocity_valid_;

  RCLCPP_WARN(get_logger(),
              "REPAIR_RACE winner generation=%" PRIu64 " kind=%s margin=%.1f "
              "reconnect_s=%.2f grid_index=%zu activation=%s duration_ms=%.1f "
              "snapshot_revision=%" PRIu64 " completion=%zu/%zu",
              winner.generation, repairJobKindName(winner.kind),
              winner.reconnect_margin_m, winner.reconnect_s_m, winner.source_grid_index,
              truncationSuffixActivationModeName(winner.activation_mode),
              winner.duration_ms, winner.source_grid_version.build_revision,
              completion_index, jobs_started);

  const bool published = publishTrajectoryResult(
      winner.trajectory, winner.route_points, "repair_race", winner.duration_ms,
      delivery,
      winner.source_grid_index == 0U ? "planning_clearance" : "runtime_prohibited",
      winner.kind == RepairJobKind::kPartial ? "stitched_old_suffix" : "full_replan",
      nullptr, &winner.source_grid_version);
  RCLCPP_INFO(get_logger(),
              "REPAIR_RACE publication generation=%" PRIu64
              " published=%s snapshot_revision=%" PRIu64,
              winner.generation, published ? "true" : "false",
              winner.source_grid_version.build_revision);
}

} // namespace drone_city_nav
