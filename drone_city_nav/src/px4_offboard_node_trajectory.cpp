#include "drone_city_nav/offboard_trajectory_delivery_diagnostics.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include "px4_offboard_node.hpp"
#include "scoped_offboard_callback_duration.hpp"

namespace drone_city_nav {
namespace {

constexpr double kTruncationSuffixPositionToleranceM{1.0};

[[nodiscard]] std::string
formatHorizontalHandoverDiagnostic(const HorizontalTrajectoryHandoverResult& handover) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3);
  stream << "horizontal_handover[attempted=" << (handover.attempted ? "true" : "false")
         << " applied=" << (handover.applied ? "true" : "false")
         << " reason=" << handover.reason
         << " old_projection_s=" << handover.old_projection_s_m
         << " candidate_projection_s=" << handover.candidate_projection_s_m
         << " projection_jump=" << handover.projection_jump_m
         << " tangent_jump=" << handover.tangent_jump_rad
         << " prefix_distance=" << handover.prefix_distance_m
         << " old_join_s=" << handover.old_join_s_m
         << " candidate_join_s=" << handover.candidate_join_s_m
         << " join_distance=" << handover.join_distance_m
         << " max_heading_delta=" << handover.max_sample_heading_delta_rad
         << " max_abs_curvature=" << handover.max_abs_curvature_1pm
         << " non_traversable_segment=" << handover.non_traversable_segment_index
         << ']';
  return stream.str();
}

} // namespace

[[nodiscard]] OffboardPathFollowerConfig Px4OffboardNode::pathFollowerConfig() const {
  return OffboardPathFollowerConfig{acceptance_radius_m_,
                                    diagnostic_turn_preview_distance_m_};
}

[[nodiscard]] std_msgs::msg::Header Px4OffboardNode::makeDebugHeader() const {
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = "map";
  return header;
}

[[nodiscard]] bool Px4OffboardNode::trajectoryDiagnosticsMatchesCurrentPath(
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics) const {
  return trajectoryDiagnosticsMatchesPath(diagnostics, last_received_path_stamp_ns_,
                                          false, 0U);
}

void Px4OffboardNode::mergePlannerDiagnosticsIntoCurrentTrajectoryStats(
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics) {
  if (!trajectoryDiagnosticsMatchesCurrentPath(diagnostics)) {
    return;
  }
  accepted_planner_path_id_ = diagnostics.planner_path_id;
  accepted_planner_path_id_seen_ = true;
  VerticalProfileStats vertical_profile = diagnostics.stats.vertical_profile;
  if (active_horizontal_handover_applied_) {
    shiftVerticalProfileStations(
        vertical_profile, active_horizontal_handover_candidate_station_offset_m_);
  }
  const bool vertical_metadata_applied =
      applyPlannerVerticalProfileMetadata(final_trajectory_samples_, vertical_profile);
  if (vertical_metadata_applied &&
      trajectorySamplesAreUsable(final_trajectory_samples_)) {
    populateTrajectoryVerticalSpeedConstraints(final_trajectory_samples_,
                                               velocity_follower_config_);
    trajectory_speed_profile_ = buildTrajectorySpeedProfile(final_trajectory_samples_,
                                                            velocity_follower_config_);
    last_trajectory_metrics_ = trajectoryMetrics(trajectory_);
    last_trajectory_shape_diagnostics_ =
        computeTrajectoryShapeDiagnostics(final_trajectory_samples_);
    last_trajectory_planner_stats_ = buildReceivedTrajectoryPlannerStats(
        path_points_, final_trajectory_samples_, trajectory_, last_trajectory_metrics_,
        trajectory_speed_profile_, velocity_follower_config_, trajectory_valid_);
  }
  if (configFingerprintMismatch(
          last_trajectory_planner_stats_.speed_profile_construction_config_fingerprint,
          diagnostics.stats.speed_profile_construction_config_fingerprint)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "speed_profile_construction_config_fingerprint_mismatch: runtime=%" PRIu64
        " planning=%" PRIu64 " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64,
        last_trajectory_planner_stats_.speed_profile_construction_config_fingerprint,
        diagnostics.stats.speed_profile_construction_config_fingerprint,
        diagnostics.planner_path_id, diagnostics.path_stamp_ns);
  }
  mergePlannerDiagnosticsIntoTrajectoryStats(last_trajectory_planner_stats_,
                                             diagnostics);
  last_trajectory_planner_stats_.vertical_profile = std::move(vertical_profile);
}

void Px4OffboardNode::updatePlannerStatsForReceivedTrajectory() {
  last_trajectory_planner_stats_ = buildReceivedTrajectoryPlannerStats(
      path_points_, final_trajectory_samples_, trajectory_, last_trajectory_metrics_,
      trajectory_speed_profile_, velocity_follower_config_, trajectory_valid_);
  if (latest_trajectory_diagnostics_.has_value()) {
    mergePlannerDiagnosticsIntoCurrentTrajectoryStats(*latest_trajectory_diagnostics_);
  }
}

void Px4OffboardNode::resetVelocitySmootherState(const std::string_view reason,
                                                 const bool count_path_update_reset,
                                                 const bool reset_vertical_smoother) {
  velocity_follower_state_ = VelocityFollowerState{};
  if (reset_vertical_smoother) {
    vertical_follower_state_ = VerticalFollowerState{};
  }
  last_velocity_smoother_reset_reason_ = std::string{reason};
  if (count_path_update_reset) {
    ++path_update_velocity_smoother_reset_count_;
  }
}

bool Px4OffboardNode::receivedFinalTrajectoryIsFreshEnough(
    const OffboardTrajectoryState& state, const std::uint64_t candidate_update_id,
    const std::uint64_t candidate_path_stamp_ns,
    const std::size_t candidate_path_points, const std::int64_t path_receive_stamp_ns,
    const HorizontalTrajectoryHandoverResult& horizontal_handover,
    const TrajectoryDeliveryDiagnostics* const delivery,
    const bool delivery_timing_applicable) const {
  if (!state.valid || state.samples.empty() || !localPositionFresh()) {
    return true;
  }

  const double threshold_m = trajectory_update_max_start_cross_track_m_;
  if (!std::isfinite(threshold_m) || threshold_m <= 0.0) {
    return true;
  }

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(state.samples, current_position_);
  const std::string delivery_diagnostic =
      delivery_timing_applicable
          ? formatTrajectoryDeliveryAtReceive(delivery, candidate_path_stamp_ns,
                                              path_receive_stamp_ns, current_position_)
          : "delivery[available=false event=pending_suffix_activated "
            "timing_applicable=false]";
  const std::string handover_diagnostic =
      formatHorizontalHandoverDiagnostic(horizontal_handover);
  if (!projection.has_value()) {
    RCLCPP_WARN(get_logger(),
                "stale_trajectory_rejected: reason=projection_unavailable "
                "local_path_update_id=%" PRIu64 " planner_path_id=%" PRIu64
                " path_stamp_ns=%" PRIu64 " points=%zu current=(%.2f, %.2f) "
                "threshold=%.2f current_velocity=(%.2f, %.2f) speed=%.2f "
                "pose_age_s=%.3f "
                "keeping_previous_trajectory=%s %s %s",
                candidate_update_id, latest_planner_path_id_, candidate_path_stamp_ns,
                candidate_path_points, current_position_.x, current_position_.y,
                threshold_m, current_velocity_.x, current_velocity_.y,
                current_speed_mps_, localPositionAgeSeconds(),
                trajectory_valid_ ? "true" : "false", delivery_diagnostic.c_str(),
                handover_diagnostic.c_str());
    return false;
  }

  const double cross_track_m = std::sqrt(projection->distance_sq);
  if (cross_track_m <= threshold_m) {
    return true;
  }

  const Point2 first = state.samples.front().point;
  const Point2 last = state.samples.back().point;
  const double start_distance_m = distance(current_position_, first);
  RCLCPP_WARN(get_logger(),
              "stale_trajectory_rejected: reason=start_cross_track_exceeded "
              "local_path_update_id=%" PRIu64 " planner_path_id=%" PRIu64
              " path_stamp_ns=%" PRIu64 " points=%zu cross_track=%.2f "
              "start_distance=%.2f threshold=%.2f current=(%.2f, %.2f) "
              "projection=(%.2f, %.2f) projection_s=%.2f first=(%.2f, %.2f) "
              "last=(%.2f, %.2f) current_velocity=(%.2f, %.2f) speed=%.2f "
              "pose_age_s=%.3f "
              "keeping_previous_trajectory=%s %s %s",
              candidate_update_id, latest_planner_path_id_, candidate_path_stamp_ns,
              candidate_path_points, cross_track_m, start_distance_m, threshold_m,
              current_position_.x, current_position_.y, projection->point.x,
              projection->point.y, projection->s_m, first.x, first.y, last.x, last.y,
              current_velocity_.x, current_velocity_.y, current_speed_mps_,
              localPositionAgeSeconds(), trajectory_valid_ ? "true" : "false",
              delivery_diagnostic.c_str(), handover_diagnostic.c_str());
  return false;
}

TrajectoryContinuityResult Px4OffboardNode::evaluateReceivedTrajectoryContinuity(
    const OffboardTrajectoryState& state) const {
  return evaluateOffboardTrajectoryUpdateContinuity(
      final_trajectory_samples_, trajectory_speed_profile_, state, current_position_,
      velocity_follower_state_.previous_velocity_setpoint,
      velocity_follower_state_.previous_velocity_setpoint_valid, localPositionFresh(),
      current_altitude_m_, altitude_valid_, trajectory_continuity_thresholds_);
}

void Px4OffboardNode::applyReceivedFinalTrajectoryPath(
    const char* source_label, const OffboardTrajectoryState& state,
    const TrajectoryContinuityResult& continuity) {
  const bool preserve_velocity_smoother_state =
      continuity.preserve_horizontal_smoother_state;
  active_horizontal_handover_applied_ = continuity.horizontal_handover_applied;
  active_horizontal_handover_candidate_station_offset_m_ =
      continuity.horizontal_handover_applied
          ? continuity.horizontal_handover_candidate_station_offset_m
          : 0.0;
  final_trajectory_samples_ = state.samples;
  trajectory_ = state.trajectory;
  trajectory_speed_profile_ = state.speed_profile;
  trajectory_valid_ = state.valid;
  terminal_position_capture_latched_ = false;
  clearTerminalPositionCaptureAltitude();
  terminal_capture_state_ = TerminalCaptureState{};
  last_trajectory_route_points_ = path_points_.size();
  last_trajectory_metrics_ = state.metrics;
  last_trajectory_shape_diagnostics_ = state.shape;
  last_trajectory_planner_stats_ = state.stats;
  if (latest_trajectory_diagnostics_.has_value()) {
    mergePlannerDiagnosticsIntoCurrentTrajectoryStats(*latest_trajectory_diagnostics_);
  }
  if (!trajectory_valid_) {
    resetVelocityDiagnostics();
  } else if (!preserve_velocity_smoother_state) {
    resetVelocitySmootherState("trajectory_continuity_reset", true,
                               !continuity.preserve_vertical_smoother_state);
  } else if (!continuity.preserve_vertical_smoother_state) {
    vertical_follower_state_ = VerticalFollowerState{};
  }
  publishFinalTrajectoryDebug();
  publishOffboardDebugMarkers();
  const std::string samples_csv_path = writeFinalTrajectorySamplesCsv(source_label);
  const SpeedProfileConstraintDiagnostic* top_speed_constraint =
      last_trajectory_planner_stats_.top_speed_constraints.empty()
          ? nullptr
          : &last_trajectory_planner_stats_.top_speed_constraints.front();
  const double top_speed_constraint_s = top_speed_constraint != nullptr
                                            ? top_speed_constraint->s_m
                                            : std::numeric_limits<double>::quiet_NaN();
  const double top_speed_constraint_radius =
      top_speed_constraint != nullptr ? top_speed_constraint->radius_m
                                      : std::numeric_limits<double>::quiet_NaN();
  const double top_speed_constraint_limit =
      top_speed_constraint != nullptr ? top_speed_constraint->speed_limit_mps
                                      : std::numeric_limits<double>::quiet_NaN();
  const char* top_speed_constraint_source =
      top_speed_constraint != nullptr
          ? speedConstraintTypeName(top_speed_constraint->source)
          : speedConstraintTypeName(SpeedConstraintType::kNone);
  const TrajectoryAltitudeStats altitude_stats =
      trajectoryAltitudeStats(final_trajectory_samples_);

  RCLCPP_INFO(
      get_logger(),
      "Received executable final trajectory: source=%s local_path_update_id=%" PRIu64
      " planner_path_id=%" PRIu64
      " points=%zu valid=%s line_segments=%zu total_length=%.2f samples=%zu "
      "altitude[min=%.2f mean=%.2f max=%.2f valid=%s] "
      "speed_profile[min=%.2f mean=%.2f max=%.2f curvature_limited=%zu] "
      "speed_profile_construction_config_fingerprint=%" PRIu64
      " runtime_speed_policy_config_fingerprint=%" PRIu64
      " runtime_velocity_control_config_fingerprint=%" PRIu64 " "
      "top_speed_constraint[s=%.2f radius=%.2f limit=%.2f source=%s] "
      "continuity[decision=%s reason=%s projection_jump=%.2f tangent_jump=%.3f "
      "curvature_jump=%.4f speed_limit_jump=%.2f "
      "tangent_speed_command_jump=%.2f reference_speed=%.2f "
      "preserve_horizontal_smoother=%s vertical_target_z_jump=%.2f "
      "vertical_target_vz_jump=%.2f vertical_hard_window_changed=%s "
      "vertical_hard_window_unsafe=%s preserve_vertical_smoother=%s "
      "horizontal_handover[applied=%s reason=%s old_join_s=%.2f "
      "candidate_join_s=%.2f stitched_join_s=%.2f join_distance=%.2f "
      "max_heading_delta_rad=%.3f max_abs_curvature_1pm=%.4f "
      "station_offset=%.2f] "
      "vertical_handover[applied=%s reason=%s "
      "candidate_s=%.2f join_s=%.2f]] "
      "isolated_spikes[candidates=%zu geometry_smoothed=%zu "
      "max_before=%.4f max_after=%.4f] "
      "shape[segments=%zu segment_len_min=%.2f mean=%.2f max=%.2f "
      "max_heading_delta=%.1fdeg max_curvature_jump=%.4f] samples_csv='%s'",
      source_label, received_path_update_id_, accepted_planner_path_id_,
      path_points_.size(), trajectory_valid_ ? "true" : "false",
      last_trajectory_metrics_.line_segments, last_trajectory_metrics_.length_m,
      final_trajectory_samples_.size(), altitude_stats.min_z_m, altitude_stats.mean_z_m,
      altitude_stats.max_z_m, altitude_stats.valid ? "true" : "false",
      last_trajectory_planner_stats_.speed_profile_min_mps,
      last_trajectory_planner_stats_.speed_profile_mean_mps,
      last_trajectory_planner_stats_.speed_profile_max_mps,
      last_trajectory_planner_stats_.speed_profile_curvature_limited_samples,
      last_trajectory_planner_stats_.speed_profile_construction_config_fingerprint,
      last_trajectory_planner_stats_.runtime_speed_policy_config_fingerprint,
      last_trajectory_planner_stats_.runtime_velocity_control_config_fingerprint,
      top_speed_constraint_s, top_speed_constraint_radius, top_speed_constraint_limit,
      top_speed_constraint_source,
      trajectoryContinuityDecisionName(continuity.decision), continuity.reason,
      continuity.projection_jump_m, continuity.tangent_jump_rad,
      continuity.curvature_jump_1pm, continuity.speed_limit_jump_mps,
      continuity.tangent_speed_command_jump_mps, continuity.reference_speed_mps,
      continuity.preserve_horizontal_smoother_state ? "true" : "false",
      continuity.vertical_target_z_jump_m, continuity.vertical_target_vz_jump_mps,
      continuity.vertical_hard_window_changed ? "true" : "false",
      continuity.vertical_hard_window_unsafe ? "true" : "false",
      continuity.preserve_vertical_smoother_state ? "true" : "false",
      continuity.horizontal_handover_applied ? "true" : "false",
      continuity.horizontal_handover_reason,
      continuity.horizontal_handover_old_join_s_m,
      continuity.horizontal_handover_candidate_join_s_m,
      continuity.horizontal_handover_stitched_join_s_m,
      continuity.horizontal_handover_join_distance_m,
      continuity.horizontal_handover_max_heading_delta_rad,
      continuity.horizontal_handover_max_abs_curvature_1pm,
      continuity.horizontal_handover_candidate_station_offset_m,
      continuity.vertical_handover_applied ? "true" : "false",
      continuity.vertical_handover_reason, continuity.vertical_handover_candidate_s_m,
      continuity.vertical_handover_join_s_m,
      last_trajectory_planner_stats_.isolated_curvature_spike_candidates,
      last_trajectory_planner_stats_.isolated_curvature_spikes_smoothed_geometry,
      last_trajectory_planner_stats_.isolated_curvature_spike_max_before_1pm,
      last_trajectory_planner_stats_.isolated_curvature_spike_max_after_1pm,
      last_trajectory_shape_diagnostics_.segment_count,
      last_trajectory_shape_diagnostics_.min_segment_length_m,
      last_trajectory_shape_diagnostics_.mean_segment_length_m,
      last_trajectory_shape_diagnostics_.max_segment_length_m,
      radiansToDegrees(last_trajectory_shape_diagnostics_.max_heading_delta_rad),
      last_trajectory_shape_diagnostics_.max_curvature_jump_1pm,
      samples_csv_path.c_str());
}

void Px4OffboardNode::onExecutableTrajectory(const msg::ExecutableTrajectory& command) {
  processExecutableTrajectory(command, false);
}

void Px4OffboardNode::tryActivatePendingTruncationSuffix() {
  if (!pending_truncation_suffix_.has_value()) {
    return;
  }
  if (!temporary_replan_truncation_active_) {
    pending_truncation_suffix_.reset();
    return;
  }
  if (!localPositionFresh()) {
    return;
  }
  const std::optional<TruncationSuffixActivationMode> requested_activation_mode =
      truncationSuffixActivationModeFromValue(
          pending_truncation_suffix_->truncation_suffix_activation_mode);
  if (!requested_activation_mode.has_value()) {
    msg::ExecutableTrajectory command = std::move(*pending_truncation_suffix_);
    pending_truncation_suffix_.reset();
    publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                               "invalid_activation_mode");
    return;
  }
  const TruncationSuffixActivationMode activation_mode =
      resolveTruncationSuffixActivationMode(*requested_activation_mode,
                                            temporary_replan_hold_active_);
  if (activation_mode == TruncationSuffixActivationMode::kAfterHold &&
      !temporary_replan_hold_active_) {
    return;
  }
  const double join_distance_m =
      distance(current_position_, active_truncation_terminal_sample_.point);
  if (join_distance_m > kTruncationSuffixPositionToleranceM) {
    return;
  }

  if (pending_truncation_suffix_->vertical_pre_alignment_required) {
    if (!vertical_pre_alignment_active_) {
      vertical_pre_alignment_active_ = true;
      vertical_pre_alignment_captured_ = false;
      vertical_pre_alignment_target_z_m_ =
          pending_truncation_suffix_->vertical_pre_alignment_target_z_m;
      vertical_pre_alignment_profile_ = VerticalCaptureProfileState{};
      vertical_pre_alignment_last_update_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      vertical_pre_alignment_stable_since_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
      publishTruncationSuffixAck(*pending_truncation_suffix_,
                                 TruncationSuffixAckDecision::kPending,
                                 "waiting_for_altitude_capture");
      RCLCPP_INFO(get_logger(),
                  "REPLAN_TRUNCATION vertical_pre_alignment_started=true "
                  "path_id=%" PRIu64 " generation=%" PRIu64
                  " hold=(%.2f,%.2f) current_z=%.2f target_z=%.2f",
                  pending_truncation_suffix_->path_id,
                  pending_truncation_suffix_->truncation_generation,
                  temporary_replan_hold_target_.x, temporary_replan_hold_target_.y,
                  current_altitude_m_, vertical_pre_alignment_target_z_m_);
      return;
    }
    if (!vertical_pre_alignment_captured_) {
      return;
    }
  }

  msg::ExecutableTrajectory command = std::move(*pending_truncation_suffix_);
  pending_truncation_suffix_.reset();
  RCLCPP_INFO(get_logger(),
              "REPLAN_DELIVERY event=pending_suffix_activated suffix_activation=%s "
              "path_id=%" PRIu64 " generation=%" PRIu64 " path_stamp_ns=%" PRIu64
              " activation_stamp_ns=%" PRId64 " current=(%.2f,%.2f) join=(%.2f,%.2f) "
              "distance=%.3fm",
              truncationSuffixActivationModeName(activation_mode), command.path_id,
              command.truncation_generation,
              messageStampNanoseconds(command.path.header.stamp),
              get_clock()->now().nanoseconds(), current_position_.x,
              current_position_.y, active_truncation_terminal_sample_.point.x,
              active_truncation_terminal_sample_.point.y, join_distance_m);
  processExecutableTrajectory(command, true);
}

void Px4OffboardNode::processExecutableTrajectory(
    const msg::ExecutableTrajectory& command, const bool pending_retry) {
  const nav_msgs::msg::Path& path = command.path;
  if (crashed_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring planner path after physical collision");
    return;
  }
  ScopedOffboardCallbackDuration callback_duration{get_logger(), "path",
                                                   path.poses.size()};
  const std::int64_t path_receive_stamp_ns = get_clock()->now().nanoseconds();
  const std::uint64_t candidate_update_id = received_path_update_id_ + 1U;
  latest_planner_path_id_ = command.path_id;
  latest_planner_path_id_seen_ = command.path_id != 0U;
  const std::uint64_t candidate_path_stamp_ns =
      messageStampNanoseconds(path.header.stamp);
  if (!pending_retry) {
    recent_path_receipts_.push_back(PathReceiptDiagnostic{
        .path_stamp_ns = candidate_path_stamp_ns,
        .receive_stamp_ns = path_receive_stamp_ns,
        .position = current_position_,
        .point_count = path.poses.size(),
    });
    constexpr std::size_t kMaxRecentPathReceipts{8U};
    if (recent_path_receipts_.size() > kMaxRecentPathReceipts) {
      recent_path_receipts_.erase(recent_path_receipts_.begin());
    }
  }
  callback_duration.setTrajectoryIdentity(latest_planner_path_id_,
                                          candidate_path_stamp_ns);
  std::vector<Point2> candidate_path_points =
      drone_city_nav::pathPointsFromMessage(path);
  std::vector<TrajectoryPointSample> candidate_path_samples =
      drone_city_nav::pathSamplesFromMessage(path);

  if (candidate_path_points.empty()) {
    if (temporary_replan_truncation_active_) {
      callback_duration.setOutcome("empty_path_preserved_temporary_prefix");
      publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                                 "empty_suffix");
      RCLCPP_WARN(get_logger(),
                  "SAFE_TRAJECTORY_TRUNCATION preserving temporary prefix after empty "
                  "replanning result: blocked_path_id=%" PRIu64 " temporary_hold=%s",
                  accepted_planner_path_id_,
                  temporary_replan_hold_active_ ? "true" : "false");
      return;
    }
    callback_duration.setOutcome("empty_path");
    received_path_update_id_ = candidate_update_id;
    last_received_path_stamp_ns_ = candidate_path_stamp_ns;
    path_points_ = std::move(candidate_path_points);
    path_valid_ = false;
    trajectory_goal_valid_ = false;
    clearFinalTrajectory();
    if (last_logged_path_size_ != 0U) {
      if (local_position_valid_) {
        no_path_hold_target_ = current_position_;
        no_path_hold_target_valid_ = true;
        commanded_target_ = no_path_hold_target_;
        commanded_target_valid_ = true;
        waypoint_index_ = 0U;
        RCLCPP_WARN(get_logger(),
                    "Received empty path: local_path_update_id=%" PRIu64
                    " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                    " holding fixed target at current position (%.2f, %.2f) "
                    "and resetting commanded target",
                    received_path_update_id_, latest_planner_path_id_,
                    last_received_path_stamp_ns_, no_path_hold_target_.x,
                    no_path_hold_target_.y);
      } else {
        no_path_hold_target_valid_ = false;
        commanded_target_valid_ = false;
        waypoint_index_ = 0U;
        RCLCPP_WARN(get_logger(),
                    "Received empty path: local_path_update_id=%" PRIu64
                    " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                    " before local position; holding configured fallback target",
                    received_path_update_id_, latest_planner_path_id_,
                    last_received_path_stamp_ns_);
      }
      last_logged_path_size_ = 0U;
    }
    return;
  }

  if (temporary_replan_truncation_active_) {
    const bool contract_matches =
        command.truncation_suffix &&
        command.truncation_generation == active_truncation_generation_ &&
        command.temporary_prefix_fingerprint == active_temporary_prefix_fingerprint_;
    if (!contract_matches) {
      callback_duration.setOutcome("truncation_contract_rejected");
      RCLCPP_WARN(
          get_logger(),
          "REPLAN_TRUNCATION rejected suffix: reason=contract_mismatch path_id=%" PRIu64
          " suffix=%s generation=%" PRIu64 "/%" PRIu64 " prefix_fingerprint=%" PRIu64
          "/%" PRIu64,
          command.path_id, command.truncation_suffix ? "true" : "false",
          command.truncation_generation, active_truncation_generation_,
          command.temporary_prefix_fingerprint, active_temporary_prefix_fingerprint_);
      publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                                 "contract_mismatch");
      return;
    }
  } else if (command.truncation_suffix) {
    callback_duration.setOutcome("stale_truncation_suffix_rejected");
    RCLCPP_WARN(get_logger(),
                "REPLAN_TRUNCATION rejected suffix: reason=no_active_prefix "
                "path_id=%" PRIu64 " generation=%" PRIu64,
                command.path_id, command.truncation_generation);
    publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                               "no_active_prefix");
    return;
  }

  const std::optional<TruncationSuffixActivationMode>
      requested_truncation_activation_mode =
          command.truncation_suffix ? truncationSuffixActivationModeFromValue(
                                          command.truncation_suffix_activation_mode)
                                    : std::optional<TruncationSuffixActivationMode>{};
  if (command.truncation_suffix && !requested_truncation_activation_mode.has_value()) {
    callback_duration.setOutcome("truncation_activation_mode_rejected");
    RCLCPP_WARN(get_logger(),
                "REPLAN_TRUNCATION rejected suffix: reason=invalid_activation_mode "
                "path_id=%" PRIu64 " generation=%" PRIu64 " value=%u",
                command.path_id, command.truncation_generation,
                static_cast<unsigned int>(command.truncation_suffix_activation_mode));
    publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                               "invalid_activation_mode");
    return;
  }
  const std::optional<TruncationSuffixActivationMode> truncation_activation_mode =
      requested_truncation_activation_mode.has_value()
          ? std::optional{resolveTruncationSuffixActivationMode(
                *requested_truncation_activation_mode, temporary_replan_hold_active_)}
          : std::optional<TruncationSuffixActivationMode>{};

  const TrajectoryPlannerDiagnosticsEnvelope* candidate_diagnostics = nullptr;
  const TrajectoryPlannerStats* candidate_planner_stats = nullptr;
  if (latest_trajectory_diagnostics_.has_value() &&
      trajectoryDiagnosticsMatchesPath(*latest_trajectory_diagnostics_,
                                       candidate_path_stamp_ns, false, 0U)) {
    candidate_diagnostics = &*latest_trajectory_diagnostics_;
    candidate_planner_stats = &latest_trajectory_diagnostics_->stats;
  }
  if (!pending_retry) {
    const std::string path_delivery_diagnostic = formatTrajectoryDeliveryAtReceive(
        candidate_diagnostics != nullptr ? &candidate_diagnostics->delivery : nullptr,
        candidate_path_stamp_ns, path_receive_stamp_ns, current_position_);
    RCLCPP_INFO(get_logger(),
                "REPLAN_DELIVERY event=%s local_path_update_id=%" PRIu64
                " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                " path_receive_stamp_ns=%" PRId64 " points=%zu %s",
                command.truncation_suffix ? "suffix_received" : "path_received",
                candidate_update_id, latest_planner_path_id_, candidate_path_stamp_ns,
                path_receive_stamp_ns, candidate_path_points.size(),
                path_delivery_diagnostic.c_str());
  }
  OffboardTrajectoryState candidate_state = buildOffboardTrajectoryState(
      candidate_path_samples, velocity_follower_config_, candidate_planner_stats);
  HorizontalTrajectoryHandoverResult horizontal_handover{};
  if (temporary_replan_truncation_active_) {
    if (!candidate_state.valid || candidate_state.samples.empty()) {
      callback_duration.setOutcome("truncation_suffix_invalid");
      RCLCPP_WARN(get_logger(),
                  "REPLAN_TRUNCATION rejected suffix: reason=invalid_trajectory "
                  "path_id=%" PRIu64 " generation=%" PRIu64,
                  command.path_id, command.truncation_generation);
      publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                                 "invalid_trajectory");
      return;
    }
    const bool vertical_pre_alignment = command.vertical_pre_alignment_required;
    const bool vertical_pre_alignment_contract_valid =
        !vertical_pre_alignment ||
        (*truncation_activation_mode == TruncationSuffixActivationMode::kAfterHold &&
         std::isfinite(command.vertical_pre_alignment_target_z_m) &&
         std::abs(candidate_state.samples.front().z_m -
                  command.vertical_pre_alignment_target_z_m) <= 0.3);
    if (!vertical_pre_alignment_contract_valid) {
      callback_duration.setOutcome("vertical_pre_alignment_contract_rejected");
      RCLCPP_WARN(get_logger(),
                  "REPLAN_TRUNCATION rejected suffix: "
                  "reason=vertical_pre_alignment_contract path_id=%" PRIu64
                  " generation=%" PRIu64 " required=%s target_z=%.2f first_z=%.2f",
                  command.path_id, command.truncation_generation,
                  vertical_pre_alignment ? "true" : "false",
                  command.vertical_pre_alignment_target_z_m,
                  candidate_state.samples.front().z_m);
      publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                                 "vertical_pre_alignment_contract");
      return;
    }
    const TruncationSuffixJoinValidation join_validation = validateTruncationSuffixJoin(
        active_truncation_terminal_sample_, candidate_state.samples.front(),
        TruncationSuffixJoinRequest{
            .max_position_jump_m = kTruncationSuffixPositionToleranceM,
            .max_tangent_jump_rad =
                trajectory_continuity_thresholds_.reject_tangent_jump_rad,
            .max_altitude_jump_m = trajectory_continuity_thresholds_
                                       .vertical_hard_window_altitude_tolerance_m,
            .require_tangent_match = *truncation_activation_mode !=
                                     TruncationSuffixActivationMode::kAfterHold,
            .require_altitude_match = !vertical_pre_alignment,
        });
    if (!join_validation.valid) {
      callback_duration.setOutcome("truncation_join_rejected");
      RCLCPP_WARN(
          get_logger(),
          "REPLAN_TRUNCATION suffix_activation=%s rejected suffix: reason=%s "
          "path_id=%" PRIu64 " generation=%" PRIu64
          " join[position=%.3fm/%.3fm tangent=%.3frad/%.3frad "
          "altitude=%.3fm/%.3fm]",
          truncationSuffixActivationModeName(*truncation_activation_mode),
          join_validation.reason, command.path_id, command.truncation_generation,
          join_validation.position_jump_m, kTruncationSuffixPositionToleranceM,
          join_validation.tangent_jump_rad,
          trajectory_continuity_thresholds_.reject_tangent_jump_rad,
          join_validation.altitude_jump_m,
          trajectory_continuity_thresholds_.vertical_hard_window_altitude_tolerance_m);
      publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                                 join_validation.reason);
      return;
    }

    const double current_join_distance_m =
        distance(current_position_, active_truncation_terminal_sample_.point);
    const bool wait_for_terminal_hold =
        *truncation_activation_mode == TruncationSuffixActivationMode::kAfterHold &&
        !temporary_replan_hold_active_;
    const bool wait_for_moving_join =
        *truncation_activation_mode == TruncationSuffixActivationMode::kMovingJoin &&
        !temporary_replan_immediate_hold_ &&
        current_join_distance_m > kTruncationSuffixPositionToleranceM;
    const bool wait_for_vertical_pre_alignment =
        vertical_pre_alignment && !vertical_pre_alignment_captured_;
    if (!pending_retry && (wait_for_terminal_hold || wait_for_moving_join ||
                           wait_for_vertical_pre_alignment)) {
      const char* pending_reason = "waiting_for_join_point";
      if (wait_for_terminal_hold) {
        pending_reason = "waiting_for_terminal_hold";
      } else if (wait_for_vertical_pre_alignment) {
        pending_reason = "waiting_for_altitude_capture";
      }
      const bool replaced = pending_truncation_suffix_.has_value();
      if (replaced) {
        resetVerticalPreAlignment();
      }
      pending_truncation_suffix_ = command;
      callback_duration.setOutcome("truncation_suffix_pending");
      RCLCPP_INFO(
          get_logger(),
          "REPLAN_DELIVERY event=suffix_pending path_id=%" PRIu64 " generation=%" PRIu64
          " path_stamp_ns=%" PRIu64 " pending_stamp_ns=%" PRId64 " reason=%s",
          command.path_id, command.truncation_generation, candidate_path_stamp_ns,
          get_clock()->now().nanoseconds(), pending_reason);
      RCLCPP_INFO(
          get_logger(),
          "REPLAN_TRUNCATION suffix_activation=%s suffix pending: path_id=%" PRIu64
          " generation=%" PRIu64 " current=(%.2f,%.2f) join=(%.2f,%.2f) "
          "distance=%.3fm replaced=%s join[position=%.3fm tangent=%.3frad "
          "altitude=%.3fm]",
          truncationSuffixActivationModeName(*truncation_activation_mode),
          command.path_id, command.truncation_generation, current_position_.x,
          current_position_.y, active_truncation_terminal_sample_.point.x,
          active_truncation_terminal_sample_.point.y, current_join_distance_m,
          replaced ? "true" : "false", join_validation.position_jump_m,
          join_validation.tangent_jump_rad, join_validation.altitude_jump_m);
      publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kPending,
                                 pending_reason);
      return;
    }

    if (*truncation_activation_mode == TruncationSuffixActivationMode::kAfterHold) {
      horizontal_handover.reason = "truncation_after_hold_release";
    } else if (!temporary_replan_immediate_hold_) {
      const Point2 truncation_point = active_truncation_terminal_sample_.point;
      const TruncatedPrefixStitchResult stitch = stitchTruncatedPrefixWithSuffix(
          final_trajectory_samples_, candidate_state.samples,
          TruncatedPrefixStitchRequest{.current_position = current_position_,
                                       .truncation_point = truncation_point,
                                       .max_join_distance_m = 1.0});
      if (!stitch.applied) {
        callback_duration.setOutcome("truncation_stitch_rejected");
        RCLCPP_WARN(get_logger(),
                    "REPLAN_TRUNCATION rejected suffix: reason=%s path_id=%" PRIu64
                    " generation=%" PRIu64 " current=(%.2f,%.2f) "
                    "truncation=(%.2f,%.2f) suffix_start=(%.2f,%.2f)",
                    stitch.reason, command.path_id, command.truncation_generation,
                    current_position_.x, current_position_.y, truncation_point.x,
                    truncation_point.y, candidate_state.samples.front().point.x,
                    candidate_state.samples.front().point.y);
        publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                                   stitch.reason);
        return;
      }
      candidate_state =
          buildOffboardTrajectoryState(stitch.samples, velocity_follower_config_);
      horizontal_handover.attempted = true;
      horizontal_handover.applied = true;
      horizontal_handover.reason = "truncation_prefix_stitched";
      horizontal_handover.old_projection_s_m = stitch.current_s_m;
      horizontal_handover.old_join_s_m = stitch.prefix_join_s_m;
      horizontal_handover.candidate_join_s_m = 0.0;
      horizontal_handover.stitched_join_s_m = stitch.suffix_station_offset_m;
      horizontal_handover.join_distance_m = stitch.join_distance_m;
      horizontal_handover.candidate_station_offset_m = stitch.suffix_station_offset_m;
      RCLCPP_INFO(get_logger(),
                  "REPLAN_TRUNCATION prefix_suffix_stitched=true path_id=%" PRIu64
                  " generation=%" PRIu64 " remaining_prefix=%.2fm join_distance=%.3fm "
                  "combined_samples=%zu",
                  command.path_id, command.truncation_generation,
                  stitch.suffix_station_offset_m, stitch.join_distance_m,
                  candidate_state.samples.size());
    } else {
      const double hold_join_distance_m = distance(
          candidate_state.samples.front().point, temporary_replan_hold_target_);
      if (hold_join_distance_m > 1.0) {
        callback_duration.setOutcome("immediate_hold_join_rejected");
        RCLCPP_WARN(
            get_logger(),
            "REPLAN_TRUNCATION rejected suffix: reason=immediate_hold_join_mismatch "
            "path_id=%" PRIu64 " generation=%" PRIu64
            " hold=(%.2f,%.2f) suffix_start=(%.2f,%.2f) distance=%.3fm",
            command.path_id, command.truncation_generation,
            temporary_replan_hold_target_.x, temporary_replan_hold_target_.y,
            candidate_state.samples.front().point.x,
            candidate_state.samples.front().point.y, hold_join_distance_m);
        publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                                   "immediate_hold_join_mismatch");
        return;
      }
      horizontal_handover.reason = "truncation_immediate_hold_release";
    }
  } else if (!trajectory_valid_) {
    horizontal_handover.reason = "current_trajectory_unavailable";
  } else if (!trajectorySamplesAreUsable(final_trajectory_samples_)) {
    horizontal_handover.reason = "current_trajectory_invalid";
  } else if (!candidate_state.valid) {
    horizontal_handover.reason = "candidate_trajectory_invalid";
  } else if (!localPositionFresh()) {
    horizontal_handover.reason = "local_position_stale";
  } else {
    const TrajectoryContinuityResult raw_continuity =
        evaluateReceivedTrajectoryContinuity(candidate_state);
    if (raw_continuity.preserve_horizontal_smoother_state) {
      horizontal_handover.reason = "continuity_already_compatible";
    } else {
      std::optional<OccupancyGrid2D> handover_grid;
      if (prohibitedGridFresh()) {
        handover_grid = currentProhibitedGrid();
      }
      horizontal_handover = buildHorizontalTrajectoryHandover(
          final_trajectory_samples_, candidate_state.samples,
          HorizontalTrajectoryHandoverState{
              .current_position = current_position_,
              .current_horizontal_speed_mps = current_speed_mps_,
              .current_position_valid = true,
              .current_horizontal_speed_valid = current_velocity_valid_,
          },
          trajectory_handover_config_,
          handover_grid.has_value() ? &*handover_grid : nullptr);
      if (horizontal_handover.applied) {
        candidate_state = buildOffboardTrajectoryState(horizontal_handover.samples,
                                                       velocity_follower_config_);
      }
    }
  }
  VerticalTrajectoryHandoverResult vertical_handover{};
  if (!temporary_replan_truncation_active_ && trajectory_valid_ &&
      trajectorySamplesAreUsable(final_trajectory_samples_) &&
      candidate_planner_stats != nullptr && localPositionFresh()) {
    vertical_handover = reanchorTrajectoryVerticalPrefix(
        final_trajectory_samples_, candidate_state.samples, current_position_,
        VerticalTrajectoryHandoverState{
            .current_altitude_m = current_altitude_m_,
            .current_vertical_velocity_mps = current_vertical_velocity_up_mps_,
            .current_horizontal_speed_mps = current_speed_mps_,
            .altitude_valid = altitude_valid_,
            .vertical_velocity_valid = current_vertical_velocity_valid_,
        });
    if (vertical_handover.applied) {
      candidate_state = buildOffboardTrajectoryState(
          candidate_state.samples, velocity_follower_config_,
          horizontal_handover.applied ? nullptr : candidate_planner_stats);
    }
  }
  if (!receivedFinalTrajectoryIsFreshEnough(
          candidate_state, candidate_update_id, candidate_path_stamp_ns,
          candidate_path_points.size(), path_receive_stamp_ns, horizontal_handover,
          candidate_diagnostics != nullptr ? &candidate_diagnostics->delivery : nullptr,
          !pending_retry)) {
    callback_duration.setOutcome("stale_rejected");
    publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                               "stale_trajectory");
    return;
  }
  TrajectoryContinuityResult continuity{};
  if (temporary_replan_truncation_active_) {
    continuity.decision = TrajectoryContinuityDecision::kResetSmoother;
    if (*truncation_activation_mode == TruncationSuffixActivationMode::kAfterHold) {
      continuity.reason = "truncation_after_hold_release";
    } else if (temporary_replan_immediate_hold_) {
      continuity.reason = "truncation_immediate_hold_release";
    } else {
      continuity.reason = "truncation_suffix_join_valid";
    }
  } else {
    continuity = evaluateReceivedTrajectoryContinuity(candidate_state);
  }
  continuity.vertical_handover_applied = vertical_handover.applied;
  continuity.vertical_handover_reason = vertical_handover.reason;
  continuity.vertical_handover_candidate_s_m = vertical_handover.candidate_s_m;
  continuity.vertical_handover_join_s_m = vertical_handover.join_s_m;
  continuity.horizontal_handover_applied = horizontal_handover.applied;
  continuity.horizontal_handover_reason = horizontal_handover.reason;
  continuity.horizontal_handover_old_join_s_m = horizontal_handover.old_join_s_m;
  continuity.horizontal_handover_candidate_join_s_m =
      horizontal_handover.candidate_join_s_m;
  continuity.horizontal_handover_stitched_join_s_m =
      horizontal_handover.stitched_join_s_m;
  continuity.horizontal_handover_join_distance_m = horizontal_handover.join_distance_m;
  continuity.horizontal_handover_max_heading_delta_rad =
      horizontal_handover.max_sample_heading_delta_rad;
  continuity.horizontal_handover_max_abs_curvature_1pm =
      horizontal_handover.max_abs_curvature_1pm;
  continuity.horizontal_handover_candidate_station_offset_m =
      horizontal_handover.candidate_station_offset_m;
  const bool continuity_blocks_update =
      continuity.decision == TrajectoryContinuityDecision::kRejectTrajectory ||
      continuity.decision == TrajectoryContinuityDecision::kDeferTrajectory;
  if (continuity_blocks_update) {
    callback_duration.setOutcome(continuity.decision ==
                                         TrajectoryContinuityDecision::kDeferTrajectory
                                     ? "continuity_deferred"
                                     : "continuity_rejected");
    RCLCPP_WARN(
        get_logger(),
        "trajectory_update_blocked: reason=%s decision=%s "
        "local_path_update_id=%" PRIu64 " planner_path_id=%" PRIu64
        " path_stamp_ns=%" PRIu64 " points=%zu projection_jump=%.2f "
        "tangent_jump=%.3f curvature_jump=%.4f speed_limit_jump=%.2f "
        "tangent_speed_command_jump=%.2f reference_speed=%.2f "
        "vertical_target_z_jump=%.2f "
        "vertical_target_vz_jump=%.2f vertical_hard_window_changed=%s "
        "vertical_hard_window_unsafe=%s horizontal_handover[applied=%s reason=%s "
        "old_join_s=%.2f candidate_join_s=%.2f stitched_join_s=%.2f "
        "join_distance=%.2f max_heading_delta_rad=%.3f "
        "max_abs_curvature_1pm=%.4f "
        "station_offset=%.2f] "
        "vertical_handover[applied=%s reason=%s candidate_s=%.2f join_s=%.2f] "
        "keeping_previous_trajectory=%s",
        continuity.reason, trajectoryContinuityDecisionName(continuity.decision),
        candidate_update_id, latest_planner_path_id_, candidate_path_stamp_ns,
        candidate_path_points.size(), continuity.projection_jump_m,
        continuity.tangent_jump_rad, continuity.curvature_jump_1pm,
        continuity.speed_limit_jump_mps, continuity.tangent_speed_command_jump_mps,
        continuity.reference_speed_mps, continuity.vertical_target_z_jump_m,
        continuity.vertical_target_vz_jump_mps,
        continuity.vertical_hard_window_changed ? "true" : "false",
        continuity.vertical_hard_window_unsafe ? "true" : "false",
        continuity.horizontal_handover_applied ? "true" : "false",
        continuity.horizontal_handover_reason,
        continuity.horizontal_handover_old_join_s_m,
        continuity.horizontal_handover_candidate_join_s_m,
        continuity.horizontal_handover_stitched_join_s_m,
        continuity.horizontal_handover_join_distance_m,
        continuity.horizontal_handover_max_heading_delta_rad,
        continuity.horizontal_handover_max_abs_curvature_1pm,
        continuity.horizontal_handover_candidate_station_offset_m,
        continuity.vertical_handover_applied ? "true" : "false",
        continuity.vertical_handover_reason, continuity.vertical_handover_candidate_s_m,
        continuity.vertical_handover_join_s_m, trajectory_valid_ ? "true" : "false");
    publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kRejected,
                               continuity.reason);
    return;
  }

  if (temporary_replan_truncation_active_) {
    RCLCPP_INFO(get_logger(),
                "SAFE_TRAJECTORY_TRUNCATION cleared by accepted replacement path: "
                "blocked_path_id=%" PRIu64 " generation=%" PRIu64
                " new_planner_path_id=%" PRIu64,
                accepted_planner_path_id_, active_truncation_generation_,
                latest_planner_path_id_);
  }
  temporary_replan_truncation_active_ = false;
  temporary_replan_hold_active_ = false;
  temporary_replan_immediate_hold_ = false;
  active_truncation_generation_ = 0U;
  active_temporary_prefix_fingerprint_ = 0U;
  pending_truncation_suffix_.reset();
  resetVerticalPreAlignment();
  no_path_hold_target_valid_ = false;
  std::vector<Point2> accepted_path_points;
  accepted_path_points.reserve(candidate_state.samples.size());
  for (const TrajectoryPointSample& sample : candidate_state.samples) {
    accepted_path_points.push_back(sample.point);
  }
  const std::size_t candidate_index =
      localPositionFresh() ? drone_city_nav::advanceWaypointIndex(accepted_path_points,
                                                                  current_position_, 0U,
                                                                  pathFollowerConfig())
                           : 0U;
  received_path_update_id_ = candidate_update_id;
  last_received_path_stamp_ns_ = candidate_path_stamp_ns;
  accepted_planner_path_id_ = command.path_id;
  accepted_planner_path_id_seen_ = command.path_id != 0U;
  path_points_ = std::move(accepted_path_points);
  path_valid_ = true;
  trajectory_goal_ = path_points_.back();
  trajectory_goal_valid_ = true;
  waypoint_index_ = candidate_index;
  applyReceivedFinalTrajectoryPath("path_update", candidate_state, continuity);
  publishTruncationSuffixAck(command, TruncationSuffixAckDecision::kAccepted,
                             "trajectory_activated");
  const Point2 first = path_points_.front();
  const Point2 last = path_points_.back();
  const bool path_changed = path_points_.size() != last_logged_path_size_ ||
                            squaredDistance(first, last_logged_path_first_) > 0.01 ||
                            squaredDistance(last, last_logged_path_last_) > 0.01;
  if (path_changed) {
    const PathMetrics metrics = pointPathMetrics(path_points_);
    RCLCPP_INFO(
        get_logger(),
        "Received path: local_path_update_id=%" PRIu64 " planner_path_id=%" PRIu64
        " path_stamp_ns=%" PRIu64 " waypoints=%zu segments=%zu straight_segments=%zu "
        "turns=%zu length=%.2f selected=%zu first=(%.2f, %.2f) "
        "segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu lt10=%zu] "
        "last=(%.2f, %.2f) altitude[z_min=%.2f z_max=%.2f "
        "vertical_profile_active=%s passages=%zu profiled=%zu "
        "min_cap=%.2f]",
        received_path_update_id_, accepted_planner_path_id_,
        last_received_path_stamp_ns_, path_points_.size(), metrics.segments,
        metrics.straight_segments, metrics.turns, metrics.length_m,
        waypoint_index_ + 1U, first.x, first.y, metrics.min_segment_length_m,
        metrics.mean_segment_length_m, metrics.max_segment_length_m,
        metrics.segments_shorter_than_2m, metrics.segments_shorter_than_5m,
        metrics.segments_shorter_than_10m, last.x, last.y,
        last_trajectory_planner_stats_.vertical_profile.min_z_m,
        last_trajectory_planner_stats_.vertical_profile.max_z_m,
        last_trajectory_planner_stats_.vertical_profile.active ? "true" : "false",
        last_trajectory_planner_stats_.vertical_profile.passages_matched,
        last_trajectory_planner_stats_.vertical_profile.passages_profiled,
        last_trajectory_planner_stats_.vertical_profile.min_vertical_speed_cap_mps);
    last_logged_path_size_ = path_points_.size();
    last_logged_path_first_ = first;
    last_logged_path_last_ = last;
  }
  callback_duration.setOutcome("accepted");
}

} // namespace drone_city_nav
