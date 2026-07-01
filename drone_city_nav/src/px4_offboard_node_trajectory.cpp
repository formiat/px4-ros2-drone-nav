#include <limits>
#include <utility>

#include "px4_offboard_node.hpp"

namespace drone_city_nav {

[[nodiscard]] OffboardPathFollowerConfig Px4OffboardNode::pathFollowerConfig() const {
  return OffboardPathFollowerConfig{acceptance_radius_m_, turn_preview_distance_m_};
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
  std::vector<Point2> samples;
  samples.reserve(final_trajectory_samples_.size());
  for (const TrajectoryPointSample& sample : final_trajectory_samples_) {
    samples.push_back(sample.point);
  }
  last_final_trajectory_debug_samples_ = samples.size();
  final_trajectory_pub_->publish(pathToRos(
      std::span<const Point2>{samples.data(), samples.size()}, makeDebugHeader(), 0.0));
}

void Px4OffboardNode::publishOffboardDebugMarkers() {
  if (!offboard_debug_marker_pub_) {
    return;
  }
  const DroneDebugMarkerState drone_state{localPositionFresh(), current_position_,
                                          current_heading_rad_};
  visualization_msgs::msg::MarkerArray markers = buildOffboardDebugMarkers(
      makeDebugHeader(), drone_state, final_trajectory_samples_,
      trajectory_speed_profile_, kRvizGroundZ);
  offboard_debug_marker_pub_->publish(markers);
}

void Px4OffboardNode::clearFinalTrajectory() {
  trajectory_.clear();
  final_trajectory_samples_.clear();
  trajectory_speed_profile_ = TrajectorySpeedProfile{};
  trajectory_valid_ = false;
  last_trajectory_metrics_ = TrajectoryMetrics{};
  last_trajectory_planner_stats_ = TrajectoryPlannerStats{};
  last_trajectory_shape_diagnostics_ = TrajectoryShapeDiagnostics{};
  last_final_trajectory_debug_samples_ = 0U;
  last_trajectory_route_points_ = 0U;
  publishFinalTrajectoryDebug();
  publishOffboardDebugMarkers();
}

[[nodiscard]] bool Px4OffboardNode::trajectoryDiagnosticsMatchesCurrentPath(
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics) const {
  return trajectoryDiagnosticsMatchesPath(diagnostics, last_received_path_stamp_ns_,
                                          latest_planner_path_id_seen_,
                                          latest_planner_path_id_);
}

void Px4OffboardNode::mergePlannerDiagnosticsIntoCurrentTrajectoryStats(
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics) {
  if (!trajectoryDiagnosticsMatchesCurrentPath(diagnostics)) {
    return;
  }
  mergePlannerDiagnosticsIntoTrajectoryStats(last_trajectory_planner_stats_,
                                             diagnostics);
}

void Px4OffboardNode::updatePlannerStatsForReceivedTrajectory() {
  last_trajectory_planner_stats_ = buildReceivedTrajectoryPlannerStats(
      path_points_, final_trajectory_samples_, trajectory_, last_trajectory_metrics_,
      trajectory_speed_profile_, trajectory_valid_);
  if (latest_trajectory_diagnostics_.has_value()) {
    mergePlannerDiagnosticsIntoCurrentTrajectoryStats(*latest_trajectory_diagnostics_);
  }
}

void Px4OffboardNode::resetVelocitySmootherState(const std::string_view reason,
                                                 const bool count_path_update_reset) {
  velocity_follower_state_ = VelocityFollowerState{};
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

void Px4OffboardNode::applyReceivedFinalTrajectoryPath(
    const char* source_label, const OffboardTrajectoryState& state) {
  final_trajectory_samples_ = state.samples;
  trajectory_ = state.trajectory;
  trajectory_speed_profile_ = state.speed_profile;
  trajectory_valid_ = state.valid;
  last_trajectory_route_points_ = path_points_.size();
  last_trajectory_metrics_ = state.metrics;
  last_trajectory_shape_diagnostics_ = state.shape;
  last_trajectory_planner_stats_ = state.stats;
  if (latest_trajectory_diagnostics_.has_value()) {
    mergePlannerDiagnosticsIntoCurrentTrajectoryStats(*latest_trajectory_diagnostics_);
  }
  if (!trajectory_valid_) {
    resetVelocityDiagnostics();
  } else {
    resetVelocitySmootherState("new_trajectory", true);
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

  RCLCPP_INFO(
      get_logger(),
      "Received executable final trajectory: source=%s local_path_update_id=%" PRIu64
      " planner_path_id=%" PRIu64
      " points=%zu valid=%s line_segments=%zu total_length=%.2f samples=%zu "
      "speed_profile[min=%.2f mean=%.2f max=%.2f curvature_limited=%zu] "
      "top_speed_constraint[s=%.2f radius=%.2f limit=%.2f source=%s] "
      "isolated_spikes[candidates=%zu geometry_smoothed=%zu "
      "speed_profile_smoothed=%zu max_before=%.4f max_after=%.4f] "
      "shape[segments=%zu segment_len_min=%.2f mean=%.2f max=%.2f "
      "max_heading_delta=%.1fdeg max_curvature_jump=%.4f] samples_csv='%s'",
      source_label, received_path_update_id_, latest_planner_path_id_,
      path_points_.size(), trajectory_valid_ ? "true" : "false",
      last_trajectory_metrics_.line_segments, last_trajectory_metrics_.length_m,
      final_trajectory_samples_.size(),
      last_trajectory_planner_stats_.speed_profile_min_mps,
      last_trajectory_planner_stats_.speed_profile_mean_mps,
      last_trajectory_planner_stats_.speed_profile_max_mps,
      last_trajectory_planner_stats_.speed_profile_curvature_limited_samples,
      top_speed_constraint_s, top_speed_constraint_radius, top_speed_constraint_limit,
      top_speed_constraint_source,
      last_trajectory_planner_stats_.isolated_curvature_spike_candidates,
      last_trajectory_planner_stats_.isolated_curvature_spikes_smoothed_geometry,
      last_trajectory_planner_stats_.isolated_curvature_spikes_smoothed_speed_profile,
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
  const std::uint64_t candidate_update_id = received_path_update_id_ + 1U;
  const std::uint64_t candidate_path_stamp_ns =
      messageStampNanoseconds(path.header.stamp);
  std::vector<Point2> candidate_path_points =
      drone_city_nav::pathPointsFromMessage(path);

  if (candidate_path_points.empty()) {
    received_path_update_id_ = candidate_update_id;
    last_received_path_stamp_ns_ = candidate_path_stamp_ns;
    path_points_ = std::move(candidate_path_points);
    path_valid_ = false;
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

  const OffboardTrajectoryState candidate_state =
      buildOffboardTrajectoryState(candidate_path_points, velocity_follower_config_);
  if (!receivedFinalTrajectoryIsFreshEnough(candidate_state, candidate_update_id,
                                            candidate_path_stamp_ns,
                                            candidate_path_points.size())) {
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
  path_points_ = std::move(candidate_path_points);
  path_valid_ = true;
  waypoint_index_ = candidate_index;
  applyReceivedFinalTrajectoryPath("path_update", candidate_state);
  const Point2 first = path_points_.front();
  const Point2 last = path_points_.back();
  const bool path_changed = path_points_.size() != last_logged_path_size_ ||
                            squaredDistance(first, last_logged_path_first_) > 0.01 ||
                            squaredDistance(last, last_logged_path_last_) > 0.01;
  if (path_changed) {
    const PathMetrics metrics = pointPathMetrics(path_points_);
    RCLCPP_INFO(get_logger(),
                "Received path: local_path_update_id=%" PRIu64
                " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                " waypoints=%zu segments=%zu straight_segments=%zu "
                "turns=%zu length=%.2f selected=%zu first=(%.2f, %.2f) "
                "segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu lt10=%zu] "
                "last=(%.2f, %.2f)",
                received_path_update_id_, latest_planner_path_id_,
                last_received_path_stamp_ns_, path_points_.size(), metrics.segments,
                metrics.straight_segments, metrics.turns, metrics.length_m,
                waypoint_index_ + 1U, first.x, first.y, metrics.min_segment_length_m,
                metrics.mean_segment_length_m, metrics.max_segment_length_m,
                metrics.segments_shorter_than_2m, metrics.segments_shorter_than_5m,
                metrics.segments_shorter_than_10m, last.x, last.y);
    last_logged_path_size_ = path_points_.size();
    last_logged_path_first_ = first;
    last_logged_path_last_ = last;
  }
}

void Px4OffboardNode::onPathId(const std_msgs::msg::UInt64& msg) {
  latest_planner_path_id_ = msg.data;
  latest_planner_path_id_seen_ = true;
}

void Px4OffboardNode::onTrajectoryDiagnostics(const std_msgs::msg::String& msg) {
  const std::optional<TrajectoryPlannerDiagnosticsEnvelope> diagnostics =
      parseTrajectoryPlannerDiagnosticsJson(msg.data);
  if (!diagnostics.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Ignoring malformed trajectory diagnostics message: bytes=%zu",
                         msg.data.size());
    return;
  }

  latest_trajectory_diagnostics_ = diagnostics;
  if (!path_valid_ || !trajectoryDiagnosticsMatchesCurrentPath(*diagnostics)) {
    return;
  }

  mergePlannerDiagnosticsIntoCurrentTrajectoryStats(*diagnostics);
  RCLCPP_INFO(get_logger(),
              "Applied planner trajectory diagnostics: planner_path_id=%" PRIu64
              " path_stamp_ns=%" PRIu64 " corridor_width[min=%.2f mean=%.2f max=%.2f] "
              "racing[length=%.2f time=%.2f gain=%.2f max_offset=%.2f]",
              diagnostics->planner_path_id, diagnostics->path_stamp_ns,
              diagnostics->stats.corridor.min_width_m,
              diagnostics->stats.corridor.mean_width_m,
              diagnostics->stats.corridor.max_width_m,
              diagnostics->stats.racing_line.final_length_m,
              diagnostics->stats.racing_line.estimated_time_s,
              diagnostics->stats.racing_line.time_gain_s,
              diagnostics->stats.racing_line.max_abs_offset_m);
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
      .planner_path_id = latest_planner_path_id_,
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
                   std::to_string(latest_planner_path_id_) + ".csv");
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
