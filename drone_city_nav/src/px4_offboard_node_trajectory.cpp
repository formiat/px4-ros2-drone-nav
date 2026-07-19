#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <limits>
#include <utility>

#include "px4_offboard_node.hpp"

namespace drone_city_nav {
namespace {

inline constexpr auto kSlowOffboardCallbackThreshold = std::chrono::milliseconds{100};

class ScopedOffboardCallbackDuration final {
public:
  ScopedOffboardCallbackDuration(rclcpp::Logger logger,
                                 const std::string_view callback_name,
                                 const std::size_t payload_size)
      : logger_{std::move(logger)},
        callback_name_{callback_name},
        payload_size_{payload_size},
        started_at_{std::chrono::steady_clock::now()} {
  }

  ScopedOffboardCallbackDuration(const ScopedOffboardCallbackDuration&) = delete;
  ScopedOffboardCallbackDuration&
  operator=(const ScopedOffboardCallbackDuration&) = delete;
  ScopedOffboardCallbackDuration(ScopedOffboardCallbackDuration&&) = delete;
  ScopedOffboardCallbackDuration& operator=(ScopedOffboardCallbackDuration&&) = delete;

  ~ScopedOffboardCallbackDuration() {
    const double duration_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - started_at_)
                                   .count();
    if (duration_ms < static_cast<double>(kSlowOffboardCallbackThreshold.count())) {
      return;
    }
    RCLCPP_WARN(logger_,
                "Slow offboard callback: callback=%.*s duration_ms=%.1f "
                "payload_size=%zu outcome=%.*s planner_path_id=%" PRIu64
                " path_stamp_ns=%" PRIu64,
                static_cast<int>(callback_name_.size()), callback_name_.data(),
                duration_ms, payload_size_, static_cast<int>(outcome_.size()),
                outcome_.data(), planner_path_id_, path_stamp_ns_);
  }

  void setOutcome(const std::string_view outcome) noexcept {
    outcome_ = outcome;
  }

  void setTrajectoryIdentity(const std::uint64_t planner_path_id,
                             const std::uint64_t path_stamp_ns) noexcept {
    planner_path_id_ = planner_path_id;
    path_stamp_ns_ = path_stamp_ns;
  }

private:
  rclcpp::Logger logger_;
  std::string_view callback_name_;
  std::size_t payload_size_{0U};
  std::chrono::steady_clock::time_point started_at_;
  std::string_view outcome_{"completed"};
  std::uint64_t planner_path_id_{0U};
  std::uint64_t path_stamp_ns_{0U};
};

[[nodiscard]] bool
configFingerprintMismatch(const std::uint64_t runtime_fingerprint,
                          const std::uint64_t planning_fingerprint) noexcept {
  return runtime_fingerprint != 0U && planning_fingerprint != 0U &&
         runtime_fingerprint != planning_fingerprint;
}

[[nodiscard]] nav_msgs::msg::Path
pathToGazeboAlignedRvizDebugPath(const std::span<const TrajectoryPointSample> samples,
                                 const std_msgs::msg::Header& header) {
  nav_msgs::msg::Path path = pathToRos(samples, header);
  for (auto& pose : path.poses) {
    pose.pose.position.z = gazeboAlignedRvizZ(pose.pose.position.z);
  }
  return path;
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

void Px4OffboardNode::publishFinalTrajectoryDebug() {
  if (!final_trajectory_pub_) {
    return;
  }
  last_final_trajectory_debug_samples_ = final_trajectory_samples_.size();
  final_trajectory_pub_->publish(
      pathToGazeboAlignedRvizDebugPath(final_trajectory_samples_, makeDebugHeader()));
}

void Px4OffboardNode::publishOffboardDebugMarkers() {
  if (!offboard_debug_marker_pub_) {
    return;
  }
  const DroneDebugMarkerState drone_state{localPositionFresh(), current_position_,
                                          current_altitude_m_, altitude_valid_,
                                          current_heading_rad_};
  visualization_msgs::msg::MarkerArray markers =
      buildOffboardDebugMarkers(makeDebugHeader(), drone_state,
                                final_trajectory_samples_, trajectory_speed_profile_);
  offboard_debug_marker_pub_->publish(markers);
}

void Px4OffboardNode::publishRvizDroneFollowTransform() {
  if (!rviz_drone_follow_tf_enabled_ || !rviz_drone_follow_tf_broadcaster_) {
    return;
  }
  if (!localPositionFresh() || !altitude_valid_ ||
      !std::isfinite(current_altitude_m_)) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = get_clock()->now();
  transform.header.frame_id = rviz_drone_follow_parent_frame_;
  transform.child_frame_id = rviz_drone_follow_frame_;
  // This is a visualization-only frame published directly in gazebo_map. The
  // legacy RViz map transform intentionally swaps X/Y and flips Z; publishing the
  // follow target in gazebo_map keeps the camera aligned with the operator-facing
  // Gazebo view without changing the navigation/control map frame.
  transform.transform.translation.x = current_position_.y;
  transform.transform.translation.y = current_position_.x;
  transform.transform.translation.z = current_altitude_m_;
  transform.transform.rotation.w = 1.0;
  rviz_drone_follow_tf_broadcaster_->sendTransform(transform);
}

void Px4OffboardNode::clearFinalTrajectory() {
  trajectory_.clear();
  final_trajectory_samples_.clear();
  trajectory_speed_profile_ = TrajectorySpeedProfile{};
  trajectory_valid_ = false;
  trajectory_goal_valid_ = false;
  terminal_position_capture_latched_ = false;
  clearTerminalPositionCaptureAltitude();
  terminal_capture_state_ = TerminalCaptureState{};
  last_trajectory_metrics_ = TrajectoryMetrics{};
  last_trajectory_planner_stats_ = TrajectoryPlannerStats{};
  last_trajectory_shape_diagnostics_ = TrajectoryShapeDiagnostics{};
  last_final_trajectory_debug_samples_ = 0U;
  last_trajectory_route_points_ = 0U;
  accepted_planner_path_id_ = 0U;
  accepted_planner_path_id_seen_ = false;
  active_horizontal_handover_applied_ = false;
  active_horizontal_handover_candidate_station_offset_m_ = 0.0;
  publishFinalTrajectoryDebug();
  publishOffboardDebugMarkers();
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
    const std::size_t candidate_path_points) const {
  if (!state.valid || state.samples.empty() || !localPositionFresh()) {
    return true;
  }

  const double threshold_m = trajectory_update_max_start_cross_track_m_;
  if (!std::isfinite(threshold_m) || threshold_m <= 0.0) {
    return true;
  }

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(state.samples, current_position_);
  if (!projection.has_value()) {
    RCLCPP_WARN(get_logger(),
                "stale_trajectory_rejected: reason=projection_unavailable "
                "local_path_update_id=%" PRIu64 " planner_path_id=%" PRIu64
                " path_stamp_ns=%" PRIu64 " points=%zu current=(%.2f, %.2f) "
                "threshold=%.2f keeping_previous_trajectory=%s",
                candidate_update_id, latest_planner_path_id_, candidate_path_stamp_ns,
                candidate_path_points, current_position_.x, current_position_.y,
                threshold_m, trajectory_valid_ ? "true" : "false");
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
              "last=(%.2f, %.2f) keeping_previous_trajectory=%s",
              candidate_update_id, latest_planner_path_id_, candidate_path_stamp_ns,
              candidate_path_points, cross_track_m, start_distance_m, threshold_m,
              current_position_.x, current_position_.y, projection->point.x,
              projection->point.y, projection->s_m, first.x, first.y, last.x, last.y,
              trajectory_valid_ ? "true" : "false");
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

void Px4OffboardNode::onPath(const nav_msgs::msg::Path& path) {
  ScopedOffboardCallbackDuration callback_duration{get_logger(), "path",
                                                   path.poses.size()};
  const std::uint64_t candidate_update_id = received_path_update_id_ + 1U;
  const std::uint64_t candidate_path_stamp_ns =
      messageStampNanoseconds(path.header.stamp);
  callback_duration.setTrajectoryIdentity(latest_planner_path_id_,
                                          candidate_path_stamp_ns);
  std::vector<Point2> candidate_path_points =
      drone_city_nav::pathPointsFromMessage(path);
  std::vector<TrajectoryPointSample> candidate_path_samples =
      drone_city_nav::pathSamplesFromMessage(path);

  if (candidate_path_points.empty()) {
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

  const TrajectoryPlannerStats* candidate_planner_stats = nullptr;
  if (latest_trajectory_diagnostics_.has_value() &&
      trajectoryDiagnosticsMatchesPath(*latest_trajectory_diagnostics_,
                                       candidate_path_stamp_ns, false, 0U)) {
    candidate_planner_stats = &latest_trajectory_diagnostics_->stats;
  }
  OffboardTrajectoryState candidate_state = buildOffboardTrajectoryState(
      candidate_path_samples, velocity_follower_config_, candidate_planner_stats);
  HorizontalTrajectoryHandoverResult horizontal_handover{};
  if (trajectory_valid_ && trajectorySamplesAreUsable(final_trajectory_samples_) &&
      candidate_state.valid && localPositionFresh()) {
    const TrajectoryContinuityResult raw_continuity =
        evaluateReceivedTrajectoryContinuity(candidate_state);
    if (!raw_continuity.preserve_horizontal_smoother_state &&
        !raw_continuity.vertical_hard_window_unsafe) {
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
  if (trajectory_valid_ && trajectorySamplesAreUsable(final_trajectory_samples_) &&
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
  if (!receivedFinalTrajectoryIsFreshEnough(candidate_state, candidate_update_id,
                                            candidate_path_stamp_ns,
                                            candidate_path_points.size())) {
    callback_duration.setOutcome("stale_rejected");
    return;
  }
  TrajectoryContinuityResult continuity =
      evaluateReceivedTrajectoryContinuity(candidate_state);
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
    return;
  }

  no_path_hold_target_valid_ = false;
  const std::size_t candidate_index =
      localPositionFresh() ? drone_city_nav::advanceWaypointIndex(candidate_path_points,
                                                                  current_position_, 0U,
                                                                  pathFollowerConfig())
                           : 0U;
  received_path_update_id_ = candidate_update_id;
  last_received_path_stamp_ns_ = candidate_path_stamp_ns;
  accepted_planner_path_id_ = 0U;
  accepted_planner_path_id_seen_ = false;
  path_points_ = std::move(candidate_path_points);
  path_valid_ = true;
  trajectory_goal_ = path_points_.back();
  trajectory_goal_valid_ = true;
  waypoint_index_ = candidate_index;
  applyReceivedFinalTrajectoryPath("path_update", candidate_state, continuity);
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

void Px4OffboardNode::onPathId(const std_msgs::msg::UInt64& msg) {
  latest_planner_path_id_ = msg.data;
  latest_planner_path_id_seen_ = true;
}

void Px4OffboardNode::onTrajectoryDiagnostics(const std_msgs::msg::String& msg) {
  ScopedOffboardCallbackDuration callback_duration{
      get_logger(), "trajectory_diagnostics", msg.data.size()};
  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> diagnostics =
      parseTrajectoryPlannerDiagnosticsJson(msg.data);
  if (!diagnostics.has_value()) {
    callback_duration.setOutcome("malformed");
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring malformed trajectory diagnostics message: bytes=%zu",
                         msg.data.size());
    return;
  }

  latest_trajectory_diagnostics_ = diagnostics;
  if (!path_valid_ || !trajectoryDiagnosticsMatchesCurrentPath(*diagnostics)) {
    callback_duration.setTrajectoryIdentity(diagnostics->planner_path_id,
                                            diagnostics->path_stamp_ns);
    callback_duration.setOutcome("not_current_path");
    return;
  }

  callback_duration.setTrajectoryIdentity(diagnostics->planner_path_id,
                                          diagnostics->path_stamp_ns);
  mergePlannerDiagnosticsIntoCurrentTrajectoryStats(*diagnostics);
  const bool runtime_speed_policy_mismatch = configFingerprintMismatch(
      last_trajectory_planner_stats_.runtime_speed_policy_config_fingerprint,
      diagnostics->stats.runtime_speed_policy_config_fingerprint);
  const bool runtime_velocity_control_mismatch = configFingerprintMismatch(
      last_trajectory_planner_stats_.runtime_velocity_control_config_fingerprint,
      diagnostics->stats.runtime_velocity_control_config_fingerprint);
  RCLCPP_INFO(get_logger(),
              "Applied planner trajectory diagnostics: planner_path_id=%" PRIu64
              " path_stamp_ns=%" PRIu64 " corridor_width[min=%.2f mean=%.2f max=%.2f] "
              "optimizer[length=%.2f time=%.2f max_offset=%.2f] "
              "runtime_fingerprint_mismatch[speed_policy=%s velocity_control=%s]",
              diagnostics->planner_path_id, diagnostics->path_stamp_ns,
              diagnostics->stats.corridor.min_width_m,
              diagnostics->stats.corridor.mean_width_m,
              diagnostics->stats.corridor.max_width_m,
              diagnostics->stats.trajectory_optimizer.final_length_m,
              diagnostics->stats.trajectory_optimizer.estimated_time_s,
              diagnostics->stats.trajectory_optimizer.max_abs_offset_m,
              runtime_speed_policy_mismatch ? "true" : "false",
              runtime_velocity_control_mismatch ? "true" : "false");
  callback_duration.setOutcome("applied");
}

void Px4OffboardNode::openFlightBlackbox() {
  if (!flight_blackbox_enabled_) {
    return;
  }

  const std::filesystem::path path{flight_blackbox_path_};
  const std::filesystem::path parent = path.parent_path();
  std::error_code error;
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, error);
    if (error) {
      RCLCPP_WARN(get_logger(), "Failed to create flight blackbox directory '%s': %s",
                  parent.string().c_str(), error.message().c_str());
      flight_blackbox_enabled_ = false;
      return;
    }
  }

  flight_blackbox_stream_.open(path, std::ios::out | std::ios::trunc);
  if (!flight_blackbox_stream_.is_open()) {
    RCLCPP_WARN(get_logger(), "Failed to open flight blackbox '%s'",
                flight_blackbox_path_.c_str());
    flight_blackbox_enabled_ = false;
    return;
  }

  flight_blackbox_stream_ << std::setprecision(6);
  RCLCPP_INFO(get_logger(), "Writing flight blackbox telemetry to '%s'",
              flight_blackbox_path_.c_str());
}

[[nodiscard]] std::filesystem::path
Px4OffboardNode::diagnosticDumpDirectory(const std::string_view name) const {
  const std::filesystem::path blackbox_path{flight_blackbox_path_};
  const std::filesystem::path parent = blackbox_path.parent_path();
  if (parent.empty()) {
    return std::filesystem::path{"log"} / name;
  }
  return parent / name;
}

[[nodiscard]] std::filesystem::path
Px4OffboardNode::finalTrajectorySamplesDirectory() const {
  return diagnosticDumpDirectory("final_trajectory_samples");
}

bool Px4OffboardNode::writeFinalTrajectorySamplesCsvFile(
    const std::filesystem::path& path, const char* source_label) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }

  stream << std::setprecision(9);
  const FinalTrajectorySamplesCsvInput input{
      .source_label = source_label,
      .local_path_update_id = received_path_update_id_,
      .planner_path_id = accepted_planner_path_id_,
      .trajectory_valid = trajectory_valid_,
      .trajectory_status = last_trajectory_planner_stats_.status,
      .samples = final_trajectory_samples_,
      .speed_profile = &trajectory_speed_profile_,
  };
  return drone_city_nav::writeFinalTrajectorySamplesCsv(stream, input);
}

bool Px4OffboardNode::writeFinalTrajectorySummaryJsonFile(
    const std::filesystem::path& path) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }
  return drone_city_nav::writeFinalTrajectorySummaryJson(
      stream, last_trajectory_planner_stats_, last_trajectory_shape_diagnostics_);
}

[[nodiscard]] std::string
Px4OffboardNode::writeFinalTrajectorySamplesCsv(const char* source_label) const {
  if (final_trajectory_samples_.empty()) {
    return {};
  }

  const std::filesystem::path directory = finalTrajectorySamplesDirectory();
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    RCLCPP_WARN(get_logger(),
                "Failed to create final trajectory samples directory '%s': %s",
                directory.string().c_str(), error.message().c_str());
    return {};
  }

  const std::filesystem::path timestamped_path =
      directory / ("trajectory_" + std::to_string(get_clock()->now().nanoseconds()) +
                   "_local_" + std::to_string(received_path_update_id_) + "_planner_" +
                   std::to_string(accepted_planner_path_id_) + ".csv");
  const std::filesystem::path timestamped_summary_path =
      timestamped_path.parent_path() /
      (timestamped_path.stem().string() + "_summary.json");
  const std::filesystem::path latest_path = directory / "latest.csv";
  const std::filesystem::path latest_summary_path = directory / "latest_summary.json";

  if (!writeFinalTrajectorySamplesCsvFile(timestamped_path, source_label)) {
    RCLCPP_WARN(get_logger(), "Failed to write final trajectory samples CSV '%s'",
                timestamped_path.string().c_str());
    return {};
  }
  if (!writeFinalTrajectorySamplesCsvFile(latest_path, source_label)) {
    RCLCPP_WARN(get_logger(),
                "Failed to update latest final trajectory samples CSV '%s'",
                latest_path.string().c_str());
  }
  if (!writeFinalTrajectorySummaryJsonFile(timestamped_summary_path)) {
    RCLCPP_WARN(get_logger(), "Failed to write final trajectory summary JSON '%s'",
                timestamped_summary_path.string().c_str());
  }
  if (!writeFinalTrajectorySummaryJsonFile(latest_summary_path)) {
    RCLCPP_WARN(get_logger(),
                "Failed to update latest final trajectory summary JSON '%s'",
                latest_summary_path.string().c_str());
  }
  return timestamped_path.string();
}

} // namespace drone_city_nav
