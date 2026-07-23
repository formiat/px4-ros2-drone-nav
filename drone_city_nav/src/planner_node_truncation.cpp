#include "planner_node.hpp"

namespace drone_city_nav {

std::optional<std::uint64_t>
PlannerNode::beginTruncationReplan(const std::uint64_t blocked_path_id,
                                   const BlockedSpan& blocked_span) {
  if (blocked_path_id == 0U) {
    return std::nullopt;
  }
  std::scoped_lock lock{truncation_replan_mutex_};
  if (truncation_replan_state_.has_value() &&
      truncation_replan_state_->blocked_path_id == blocked_path_id) {
    return std::nullopt;
  }
  const std::uint64_t generation = next_truncation_generation_++;
  pending_truncation_runtime_trajectory_.reset();
  const bool artifact_matches =
      executable_trajectory_artifact_.path_id == blocked_path_id &&
      trajectorySamplesAreUsable(executable_trajectory_artifact_.samples) &&
      std::isfinite(blocked_span.first_blocked_s_m) &&
      std::isfinite(blocked_span.last_blocked_s_m) &&
      blocked_span.last_blocked_s_m >= blocked_span.first_blocked_s_m;
  truncation_replan_state_ = TruncationReplanState{
      .blocked_path_id = blocked_path_id,
      .generation = generation,
      .blocked_span = blocked_span,
      .old_trajectory = artifact_matches ? executable_trajectory_artifact_
                                         : ExecutableTrajectoryArtifact{},
      .current_s_m = artifact_matches ? executable_trajectory_artifact_.current_s_m
                                      : std::numeric_limits<double>::quiet_NaN(),
      .repair_context_valid = artifact_matches,
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
    truncation_replan_state_->published_suffix_path_id = 0U;
    truncation_replan_state_->confirmed = true;
    truncation_replan_state_->immediate_hold = message.immediate_hold;
    truncation_replan_state_->awaiting_ack = false;
    if (truncation_replan_state_->repair_context_valid) {
      const std::optional<TrajectoryProjection> projection = projectOnTrajectorySamples(
          truncation_replan_state_->old_trajectory.samples,
          Point2{position.x, position.y}, truncation_replan_state_->current_s_m);
      const bool station_valid =
          projection.has_value() &&
          projection->s_m + 1.0e-6 >= truncation_replan_state_->current_s_m &&
          projection->s_m + 1.0e-6 <
              truncation_replan_state_->blocked_span.first_blocked_s_m;
      if (station_valid) {
        truncation_replan_state_->truncation_s_m = projection->s_m;
      } else {
        truncation_replan_state_->repair_context_valid = false;
      }
    }
    pending_truncation_runtime_trajectory_.reset();
  }

  RCLCPP_WARN(get_logger(),
              "REPLAN_TRUNCATION confirmed=true blocked_path_id=%" PRIu64
              " generation=%" PRIu64 " start=(%.2f,%.2f,%.2f) tangent=(%.3f,%.3f) "
              "prefix_fingerprint=%" PRIu64 " immediate_hold=%s",
              message.blocked_path_id, message.truncation_generation, position.x,
              position.y, message.truncation_altitude_m, tangent.x, tangent.y,
              message.temporary_prefix_fingerprint,
              message.immediate_hold ? "true" : "false");
  const std::optional<TruncationReplanState> confirmed_state = truncationReplanState();
  if (confirmed_state.has_value()) {
    RCLCPP_INFO(get_logger(),
                "REPAIR_CONTEXT confirmed=%s current_s=%.2f truncation_s=%.2f "
                "blocked_span=[%.2f,%.2f] trigger=%s artifact_path_id=%" PRIu64,
                confirmed_state->repair_context_valid ? "true" : "false",
                confirmed_state->current_s_m, confirmed_state->truncation_s_m,
                confirmed_state->blocked_span.first_blocked_s_m,
                confirmed_state->blocked_span.last_blocked_s_m,
                blockedSpanTriggerName(confirmed_state->blocked_span.trigger),
                confirmed_state->old_trajectory.path_id);
  }
  requestPlanningCycle();
}

void PlannerNode::onTruncationSuffixAck(const msg::TruncationSuffixAck& message) {
  const std::optional<TruncationSuffixAckDecision> decision =
      truncationSuffixAckDecisionFromValue(message.decision);
  if (!decision.has_value()) {
    RCLCPP_WARN(get_logger(),
                "REPLAN_TRUNCATION ignored suffix ACK: reason=invalid_decision "
                "path_id=%" PRIu64 " generation=%" PRIu64 " decision=%u",
                message.path_id, message.truncation_generation,
                static_cast<unsigned int>(message.decision));
    return;
  }

  const TruncationSuffixIdentity received{
      .path_id = message.path_id,
      .generation = message.truncation_generation,
      .prefix_fingerprint = message.temporary_prefix_fingerprint,
  };
  std::optional<PendingTruncationRuntimeTrajectory> accepted_trajectory;
  TruncationSuffixAckEvaluation evaluation{};
  std::size_t publication_attempts = 0U;
  {
    std::scoped_lock lock{truncation_replan_mutex_};
    if (!truncation_replan_state_.has_value() ||
        !truncation_replan_state_->awaiting_ack) {
      RCLCPP_WARN(get_logger(),
                  "REPLAN_TRUNCATION ignored suffix ACK: reason=no_awaiting_suffix "
                  "path_id=%" PRIu64 " generation=%" PRIu64 " decision=%s",
                  message.path_id, message.truncation_generation,
                  truncationSuffixAckDecisionName(*decision));
      return;
    }
    const TruncationSuffixIdentity expected{
        .path_id = truncation_replan_state_->published_suffix_path_id,
        .generation = truncation_replan_state_->generation,
        .prefix_fingerprint = truncation_replan_state_->temporary_prefix_fingerprint,
    };
    evaluation = evaluateTruncationSuffixAck(expected, received, *decision);
    publication_attempts = truncation_replan_state_->publication_attempts;
    if (evaluation.action == TruncationSuffixAckAction::kAdopt) {
      if (!pending_truncation_runtime_trajectory_.has_value() ||
          pending_truncation_runtime_trajectory_->identity.path_id !=
              expected.path_id ||
          pending_truncation_runtime_trajectory_->identity.generation !=
              expected.generation ||
          pending_truncation_runtime_trajectory_->identity.prefix_fingerprint !=
              expected.prefix_fingerprint) {
        evaluation = {TruncationSuffixAckAction::kRetry,
                      "accepted_runtime_candidate_missing"};
      } else {
        accepted_trajectory = std::move(*pending_truncation_runtime_trajectory_);
        pending_truncation_runtime_trajectory_.reset();
        truncation_replan_state_.reset();
      }
    }
    if (evaluation.action == TruncationSuffixAckAction::kRetry &&
        truncation_replan_state_.has_value()) {
      truncation_replan_state_->awaiting_ack = false;
      truncation_replan_state_->published_suffix_path_id = 0U;
      pending_truncation_runtime_trajectory_.reset();
    }
  }

  if (evaluation.action == TruncationSuffixAckAction::kIgnore) {
    RCLCPP_WARN(get_logger(),
                "REPLAN_TRUNCATION ignored suffix ACK: reason=%s path_id=%" PRIu64
                " generation=%" PRIu64 " decision=%s",
                evaluation.reason, message.path_id, message.truncation_generation,
                truncationSuffixAckDecisionName(*decision));
    return;
  }
  if (evaluation.action == TruncationSuffixAckAction::kKeepWaiting) {
    RCLCPP_INFO(get_logger(),
                "REPLAN_TRUNCATION suffix ACK pending: path_id=%" PRIu64
                " generation=%" PRIu64 " reason='%s' attempt=%zu",
                message.path_id, message.truncation_generation, message.reason.c_str(),
                publication_attempts);
    return;
  }
  if (evaluation.action == TruncationSuffixAckAction::kAdopt &&
      accepted_trajectory.has_value()) {
    last_valid_path_points_ = std::move(accepted_trajectory->path_points);
    last_valid_trajectory_samples_ = std::move(accepted_trajectory->trajectory_samples);
    executable_trajectory_artifact_ = ExecutableTrajectoryArtifact{
        .path_id = message.path_id,
        .geometry_fingerprint =
            trajectoryPrefixFingerprint(last_valid_trajectory_samples_),
        .mission_goal = goal_,
        .samples = last_valid_trajectory_samples_,
        .current_s_m = 0.0,
    };
    RCLCPP_INFO(get_logger(),
                "REPLAN_TRUNCATION suffix ACK accepted: path_id=%" PRIu64
                " generation=%" PRIu64 " reason='%s' attempt=%zu "
                "runtime_samples=%zu",
                message.path_id, message.truncation_generation, message.reason.c_str(),
                publication_attempts, last_valid_trajectory_samples_.size());
    return;
  }

  RCLCPP_WARN(get_logger(),
              "REPLAN_TRUNCATION suffix ACK rejected: path_id=%" PRIu64
              " generation=%" PRIu64 " reason='%s' protocol_reason=%s attempt=%zu "
              "action=retry",
              message.path_id, message.truncation_generation, message.reason.c_str(),
              evaluation.reason, publication_attempts);
  requestPlanningCycle();
}

bool PlannerNode::prepareTrajectoryForRuntimeChecks(
    const std::span<const TrajectoryPointSample> samples,
    const std::span<const Point2> trajectory_points,
    const TrajectoryDeliveryDiagnostics& delivery, const char* source_label,
    const std::uint64_t path_id) {
  if (!delivery.truncation_suffix) {
    last_valid_path_points_.assign(trajectory_points.begin(), trajectory_points.end());
    last_valid_trajectory_samples_.assign(samples.begin(), samples.end());
    executable_trajectory_artifact_ = ExecutableTrajectoryArtifact{
        .path_id = path_id,
        .geometry_fingerprint = trajectoryPrefixFingerprint(samples),
        .mission_goal = goal_,
        .samples = std::vector<TrajectoryPointSample>{samples.begin(), samples.end()},
        .current_s_m = 0.0,
    };
    return true;
  }

  std::vector<TrajectoryPointSample> runtime_samples;
  const bool activation_after_hold =
      delivery.truncation_suffix_activation_mode ==
      static_cast<std::uint8_t>(TruncationSuffixActivationMode::kAfterHold);
  if (delivery.truncation_immediate_hold || activation_after_hold) {
    runtime_samples.assign(samples.begin(), samples.end());
  } else if (!trajectorySamplesAreUsable(last_valid_trajectory_samples_)) {
    RCLCPP_ERROR(get_logger(),
                 "%s truncation suffix discarded before publication: "
                 "reason=planner_prefix_unavailable generation=%" PRIu64,
                 source_label, delivery.truncation_generation);
    requestPlanningCycle();
    return false;
  } else {
    const TruncatedPrefixStitchResult planner_stitch = stitchTruncatedPrefixWithSuffix(
        last_valid_trajectory_samples_, samples,
        TruncatedPrefixStitchRequest{
            .current_position = last_valid_trajectory_samples_.front().point,
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
    runtime_samples = planner_stitch.samples;
  }

  const std::vector<Point2> runtime_points = trajectorySamplePoints(runtime_samples);
  const TruncationSuffixIdentity identity{
      .path_id = path_id,
      .generation = delivery.truncation_generation,
      .prefix_fingerprint = delivery.temporary_prefix_fingerprint,
  };
  std::size_t publication_attempt = 0U;
  {
    std::scoped_lock lock{truncation_replan_mutex_};
    const TruncationSuffixPublicationEvaluation publication =
        truncation_replan_state_.has_value()
            ? evaluateTruncationSuffixPublication(
                  TruncationSuffixPublicationContext{
                      .generation = truncation_replan_state_->generation,
                      .prefix_fingerprint =
                          truncation_replan_state_->temporary_prefix_fingerprint,
                      .confirmed = truncation_replan_state_->confirmed,
                      .awaiting_ack = truncation_replan_state_->awaiting_ack,
                  },
                  identity)
            : TruncationSuffixPublicationEvaluation{false, "state_missing"};
    if (!publication.allowed) {
      RCLCPP_ERROR(get_logger(),
                   "%s truncation suffix discarded before publication: "
                   "reason=%s path_id=%" PRIu64 " generation=%" PRIu64,
                   source_label, publication.reason, path_id,
                   delivery.truncation_generation);
      return false;
    }
    truncation_replan_state_->published_suffix_path_id = path_id;
    truncation_replan_state_->awaiting_ack = true;
    publication_attempt = ++truncation_replan_state_->publication_attempts;
    pending_truncation_runtime_trajectory_ = PendingTruncationRuntimeTrajectory{
        .identity = identity,
        .path_points = runtime_points,
        .trajectory_samples = runtime_samples,
    };
  }
  RCLCPP_INFO(get_logger(),
              "REPLAN_TRUNCATION suffix_activation=%s suffix awaiting ACK=true "
              "blocked_path_id=%" PRIu64 " generation=%" PRIu64
              " prefix_fingerprint=%" PRIu64 " path_id=%" PRIu64
              " attempt=%zu runtime_samples=%zu",
              truncationSuffixActivationModeName(
                  activation_after_hold ? TruncationSuffixActivationMode::kAfterHold
                                        : TruncationSuffixActivationMode::kMovingJoin),
              delivery.blocked_path_id, delivery.truncation_generation,
              delivery.temporary_prefix_fingerprint, path_id, publication_attempt,
              runtime_samples.size());
  return true;
}

} // namespace drone_city_nav
