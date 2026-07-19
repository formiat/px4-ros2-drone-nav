#include "drone_city_nav/visualization_marker_helpers.hpp"

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

[[nodiscard]] bool
configFingerprintMismatch(const std::uint64_t runtime_fingerprint,
                          const std::uint64_t planning_fingerprint) noexcept {
  return runtime_fingerprint != 0U && planning_fingerprint != 0U &&
         runtime_fingerprint != planning_fingerprint;
}

struct TimestampInterval {
  std::uint64_t earlier_stamp_ns{0U};
  std::int64_t later_stamp_ns{0};
};

[[nodiscard]] double elapsedMilliseconds(const TimestampInterval interval) noexcept {
  if (interval.later_stamp_ns <= 0 || interval.earlier_stamp_ns == 0U ||
      static_cast<std::uint64_t>(interval.later_stamp_ns) < interval.earlier_stamp_ns) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 1.0e-6 *
         static_cast<double>(static_cast<std::uint64_t>(interval.later_stamp_ns) -
                             interval.earlier_stamp_ns);
}

[[nodiscard]] std::string
formatTrajectoryDeliveryAtReceive(const TrajectoryDeliveryDiagnostics* const delivery,
                                  const std::uint64_t path_stamp_ns,
                                  const std::int64_t receive_stamp_ns,
                                  const Point2 actual_receive_position) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "delivery[available=" << (delivery != nullptr ? "true" : "false")
         << " publish_to_receive_ms="
         << elapsedMilliseconds(TimestampInterval{
                .earlier_stamp_ns = path_stamp_ns,
                .later_stamp_ns = receive_stamp_ns,
            });
  if (delivery == nullptr) {
    stream << " actual_receive=(" << actual_receive_position.x << ", "
           << actual_receive_position.y << ")]";
    return stream.str();
  }
  const double blocker_to_receive_ms =
      delivery->replan_triggered
          ? elapsedMilliseconds(TimestampInterval{
                .earlier_stamp_ns = delivery->blocker_detected_stamp_ns,
                .later_stamp_ns = receive_stamp_ns,
            })
          : std::numeric_limits<double>::quiet_NaN();
  Point2 predicted_receive_position{};
  bool predicted_receive_position_valid = false;
  if (delivery->planning_start_velocity_valid &&
      delivery->trajectory_build_started_stamp_ns > 0U && receive_stamp_ns > 0 &&
      static_cast<std::uint64_t>(receive_stamp_ns) >=
          delivery->trajectory_build_started_stamp_ns) {
    const double prediction_time_s =
        1.0e-9 * static_cast<double>(static_cast<std::uint64_t>(receive_stamp_ns) -
                                     delivery->trajectory_build_started_stamp_ns);
    predicted_receive_position = Point2{
        delivery->planning_start_position.x +
            delivery->planning_start_velocity.x * prediction_time_s,
        delivery->planning_start_position.y +
            delivery->planning_start_velocity.y * prediction_time_s,
    };
    predicted_receive_position_valid = std::isfinite(predicted_receive_position.x) &&
                                       std::isfinite(predicted_receive_position.y);
  }
  const double predicted_to_receive_error_m =
      predicted_receive_position_valid
          ? distance(predicted_receive_position, actual_receive_position)
          : std::numeric_limits<double>::quiet_NaN();
  stream << " generation=" << delivery->generation
         << " replan_triggered=" << (delivery->replan_triggered ? "true" : "false")
         << " blocker_stamp_ns=" << delivery->blocker_detected_stamp_ns
         << " build_stamp_ns=" << delivery->trajectory_build_started_stamp_ns
         << " publish_stamp_ns=" << delivery->path_published_stamp_ns
         << " blocker_to_build_ms=" << delivery->blocker_to_build_start_ms
         << " build_to_publish_ms=" << delivery->build_start_to_publish_ms
         << " blocker_to_publish_ms=" << delivery->blocker_to_publish_ms
         << " blocker_to_receive_ms=" << blocker_to_receive_ms << " candidate_start=("
         << delivery->candidate_start_position.x << ", "
         << delivery->candidate_start_position.y << ") planning_start=("
         << delivery->planning_start_position.x << ", "
         << delivery->planning_start_position.y << ")" << " planning_velocity=("
         << delivery->planning_start_velocity.x << ", "
         << delivery->planning_start_velocity.y << ")" << " velocity_valid="
         << (delivery->planning_start_velocity_valid ? "true" : "false")
         << " predicted_publication=(" << delivery->predicted_publication_position.x
         << ", " << delivery->predicted_publication_position.y << ")"
         << " predicted_valid="
         << (delivery->predicted_publication_position_valid ? "true" : "false")
         << " planner_actual_publication=(" << delivery->actual_publication_position.x
         << ", " << delivery->actual_publication_position.y << ")"
         << " planner_actual_valid="
         << (delivery->actual_publication_position_valid ? "true" : "false")
         << " publication_prediction_error=" << delivery->publication_prediction_error_m
         << " actual_receive=(" << actual_receive_position.x << ", "
         << actual_receive_position.y << ")" << " predicted_receive=("
         << predicted_receive_position.x << ", " << predicted_receive_position.y
         << ") predicted_receive_valid="
         << (predicted_receive_position_valid ? "true" : "false")
         << " predicted_to_receive_error=" << predicted_to_receive_error_m << ']';
  return stream.str();
}

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
    const std::size_t candidate_path_points, const std::int64_t path_receive_stamp_ns,
    const HorizontalTrajectoryHandoverResult& horizontal_handover,
    const TrajectoryDeliveryDiagnostics* const delivery) const {
  if (!state.valid || state.samples.empty() || !localPositionFresh()) {
    return true;
  }

  const double threshold_m = trajectory_update_max_start_cross_track_m_;
  if (!std::isfinite(threshold_m) || threshold_m <= 0.0) {
    return true;
  }

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(state.samples, current_position_);
  const std::string delivery_diagnostic = formatTrajectoryDeliveryAtReceive(
      delivery, candidate_path_stamp_ns, path_receive_stamp_ns, current_position_);
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

void Px4OffboardNode::onPath(const nav_msgs::msg::Path& path) {
  ScopedOffboardCallbackDuration callback_duration{get_logger(), "path",
                                                   path.poses.size()};
  const std::int64_t path_receive_stamp_ns = get_clock()->now().nanoseconds();
  const std::uint64_t candidate_update_id = received_path_update_id_ + 1U;
  const std::uint64_t candidate_path_stamp_ns =
      messageStampNanoseconds(path.header.stamp);
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

  const TrajectoryPlannerDiagnosticsEnvelope* candidate_diagnostics = nullptr;
  const TrajectoryPlannerStats* candidate_planner_stats = nullptr;
  if (latest_trajectory_diagnostics_.has_value() &&
      trajectoryDiagnosticsMatchesPath(*latest_trajectory_diagnostics_,
                                       candidate_path_stamp_ns, false, 0U)) {
    candidate_diagnostics = &*latest_trajectory_diagnostics_;
    candidate_planner_stats = &latest_trajectory_diagnostics_->stats;
  }
  const std::string path_delivery_diagnostic = formatTrajectoryDeliveryAtReceive(
      candidate_diagnostics != nullptr ? &candidate_diagnostics->delivery : nullptr,
      candidate_path_stamp_ns, path_receive_stamp_ns, current_position_);
  RCLCPP_INFO(get_logger(),
              "REPLAN_DELIVERY event=path_received local_path_update_id=%" PRIu64
              " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
              " path_receive_stamp_ns=%" PRId64 " points=%zu %s",
              candidate_update_id, latest_planner_path_id_, candidate_path_stamp_ns,
              path_receive_stamp_ns, candidate_path_points.size(),
              path_delivery_diagnostic.c_str());
  OffboardTrajectoryState candidate_state = buildOffboardTrajectoryState(
      candidate_path_samples, velocity_follower_config_, candidate_planner_stats);
  HorizontalTrajectoryHandoverResult horizontal_handover{};
  if (!trajectory_valid_) {
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
    } else if (raw_continuity.vertical_hard_window_unsafe) {
      horizontal_handover.reason = "vertical_hard_window_unsafe";
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
  if (!receivedFinalTrajectoryIsFreshEnough(
          candidate_state, candidate_update_id, candidate_path_stamp_ns,
          candidate_path_points.size(), path_receive_stamp_ns, horizontal_handover,
          candidate_diagnostics != nullptr ? &candidate_diagnostics->delivery
                                           : nullptr)) {
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
  const auto receipt =
      std::find_if(recent_path_receipts_.rbegin(), recent_path_receipts_.rend(),
                   [&diagnostics](const PathReceiptDiagnostic& candidate) {
                     return candidate.path_stamp_ns == diagnostics->path_stamp_ns;
                   });
  if (receipt != recent_path_receipts_.rend()) {
    const std::int64_t diagnostics_receive_stamp_ns = get_clock()->now().nanoseconds();
    const std::string correlated_delivery = formatTrajectoryDeliveryAtReceive(
        &diagnostics->delivery, diagnostics->path_stamp_ns, receipt->receive_stamp_ns,
        receipt->position);
    const double diagnostics_after_path_ms =
        receipt->receive_stamp_ns > 0
            ? 1.0e-6 * static_cast<double>(diagnostics_receive_stamp_ns -
                                           receipt->receive_stamp_ns)
            : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(
        get_logger(),
        "REPLAN_DELIVERY event=diagnostics_correlated_to_path_receipt "
        "planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
        " path_receive_stamp_ns=%" PRId64 " diagnostics_receive_stamp_ns=%" PRId64
        " diagnostics_after_path_ms=%.1f points=%zu %s",
        diagnostics->planner_path_id, diagnostics->path_stamp_ns,
        receipt->receive_stamp_ns, diagnostics_receive_stamp_ns,
        diagnostics_after_path_ms, receipt->point_count, correlated_delivery.c_str());
  }
  if (!path_valid_ || !trajectoryDiagnosticsMatchesCurrentPath(*diagnostics)) {
    callback_duration.setTrajectoryIdentity(diagnostics->planner_path_id,
                                            diagnostics->path_stamp_ns);
    callback_duration.setOutcome("not_current_path");
    const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
    const std::string delivery_diagnostic = formatTrajectoryDeliveryAtReceive(
        &diagnostics->delivery, diagnostics->path_stamp_ns, receive_stamp_ns,
        current_position_);
    RCLCPP_INFO(get_logger(),
                "REPLAN_DELIVERY event=diagnostics_received_not_current "
                "planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                " path_valid=%s current_path_stamp_ns=%" PRIu64 " %s",
                diagnostics->planner_path_id, diagnostics->path_stamp_ns,
                path_valid_ ? "true" : "false", last_received_path_stamp_ns_,
                delivery_diagnostic.c_str());
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
