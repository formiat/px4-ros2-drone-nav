#include "planner_node.hpp"

namespace drone_city_nav {

std::optional<std::uint64_t>
PlannerNode::beginTruncationReplan(const std::uint64_t blocked_path_id) {
  if (blocked_path_id == 0U) {
    return std::nullopt;
  }
  std::scoped_lock lock{truncation_replan_mutex_};
  if (truncation_replan_state_.has_value() &&
      truncation_replan_state_->blocked_path_id == blocked_path_id) {
    return std::nullopt;
  }
  const std::uint64_t generation = next_truncation_generation_++;
  truncation_replan_state_ = TruncationReplanState{
      .blocked_path_id = blocked_path_id,
      .generation = generation,
  };
  return generation;
}

std::optional<PlannerNode::TruncationReplanState>
PlannerNode::truncationReplanState() const {
  std::scoped_lock lock{truncation_replan_mutex_};
  return truncation_replan_state_;
}

void PlannerNode::onReplanTruncation(const msg::ReplanTruncation& message) {
  const Point3 position{message.truncation_position.x, message.truncation_position.y,
                        message.truncation_position.z};
  const Point2 tangent{message.truncation_tangent.x, message.truncation_tangent.y};
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(tangent.x) ||
      !std::isfinite(tangent.y) || !std::isfinite(message.truncation_altitude_m) ||
      message.blocked_path_id == 0U || message.truncation_generation == 0U ||
      message.temporary_prefix_fingerprint == 0U) {
    RCLCPP_ERROR(get_logger(),
                 "REPLAN_TRUNCATION rejected confirmation: reason=invalid_payload "
                 "blocked_path_id=%" PRIu64 " generation=%" PRIu64,
                 message.blocked_path_id, message.truncation_generation);
    return;
  }

  {
    std::scoped_lock lock{truncation_replan_mutex_};
    if (!truncation_replan_state_.has_value() ||
        truncation_replan_state_->blocked_path_id != message.blocked_path_id ||
        truncation_replan_state_->generation != message.truncation_generation) {
      RCLCPP_WARN(get_logger(),
                  "REPLAN_TRUNCATION ignored confirmation: reason=generation_mismatch "
                  "blocked_path_id=%" PRIu64 " generation=%" PRIu64
                  " expected_path_id=%" PRIu64 " expected_generation=%" PRIu64,
                  message.blocked_path_id, message.truncation_generation,
                  truncation_replan_state_.has_value()
                      ? truncation_replan_state_->blocked_path_id
                      : 0U,
                  truncation_replan_state_.has_value()
                      ? truncation_replan_state_->generation
                      : 0U);
      return;
    }
    truncation_replan_state_->position = position;
    truncation_replan_state_->tangent = tangent;
    truncation_replan_state_->altitude_m = message.truncation_altitude_m;
    truncation_replan_state_->temporary_prefix_fingerprint =
        message.temporary_prefix_fingerprint;
    truncation_replan_state_->confirmed = true;
    truncation_replan_state_->immediate_hold = message.immediate_hold;
  }

  RCLCPP_WARN(get_logger(),
              "REPLAN_TRUNCATION confirmed=true blocked_path_id=%" PRIu64
              " generation=%" PRIu64 " start=(%.2f,%.2f,%.2f) tangent=(%.3f,%.3f) "
              "prefix_fingerprint=%" PRIu64 " immediate_hold=%s",
              message.blocked_path_id, message.truncation_generation, position.x,
              position.y, message.truncation_altitude_m, tangent.x, tangent.y,
              message.temporary_prefix_fingerprint,
              message.immediate_hold ? "true" : "false");
  requestPlanningCycle();
}

void PlannerNode::completeTruncationReplan(const std::uint64_t generation) {
  std::scoped_lock lock{truncation_replan_mutex_};
  if (truncation_replan_state_.has_value() &&
      truncation_replan_state_->generation == generation) {
    truncation_replan_state_.reset();
  }
}

bool PlannerNode::adoptTrajectoryForRuntimeChecks(
    const std::span<const TrajectoryPointSample> samples,
    const std::span<const Point2> trajectory_points,
    const TrajectoryDeliveryDiagnostics& delivery, const char* source_label) {
  if (!delivery.truncation_suffix || delivery.truncation_immediate_hold) {
    last_valid_path_points_.assign(trajectory_points.begin(), trajectory_points.end());
    last_valid_trajectory_samples_.assign(samples.begin(), samples.end());
    return true;
  }
  if (!trajectorySamplesAreUsable(last_valid_trajectory_samples_)) {
    RCLCPP_ERROR(get_logger(),
                 "%s truncation suffix discarded before publication: "
                 "reason=planner_prefix_unavailable generation=%" PRIu64,
                 source_label, delivery.truncation_generation);
    requestPlanningCycle();
    return false;
  }
  const TruncatedPrefixStitchResult planner_stitch = stitchTruncatedPrefixWithSuffix(
      last_valid_trajectory_samples_, samples,
      TruncatedPrefixStitchRequest{.current_position =
                                       last_valid_trajectory_samples_.front().point,
                                   .truncation_point = delivery.planning_start_position,
                                   .max_join_distance_m = 1.0});
  if (!planner_stitch.applied) {
    RCLCPP_ERROR(get_logger(),
                 "%s truncation suffix discarded before publication: reason=%s "
                 "blocked_path_id=%" PRIu64 " generation=%" PRIu64
                 " planning_start=(%.2f,%.2f)",
                 source_label, planner_stitch.reason, delivery.blocked_path_id,
                 delivery.truncation_generation, delivery.planning_start_position.x,
                 delivery.planning_start_position.y);
    requestPlanningCycle();
    return false;
  }
  last_valid_trajectory_samples_ = planner_stitch.samples;
  last_valid_path_points_ = trajectorySamplePoints(last_valid_trajectory_samples_);
  RCLCPP_INFO(
      get_logger(),
      "REPLAN_TRUNCATION planner_prefix_suffix_stitched=true "
      "blocked_path_id=%" PRIu64 " generation=%" PRIu64 " prefix_fingerprint=%" PRIu64
      " suffix_station_offset=%.2fm combined_samples=%zu",
      delivery.blocked_path_id, delivery.truncation_generation,
      delivery.temporary_prefix_fingerprint, planner_stitch.suffix_station_offset_m,
      last_valid_trajectory_samples_.size());
  return true;
}

} // namespace drone_city_nav
