#include "px4_offboard_node.hpp"

namespace drone_city_nav {

[[nodiscard]] std::vector<Point2>
Px4OffboardNode::pathPointsFromMessage(const nav_msgs::msg::Path& path) const {
  std::vector<Point2> points;
  points.reserve(path.poses.size());
  for (const auto& pose : path.poses) {
    points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
  }
  return points;
}

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

[[nodiscard]] visualization_msgs::msg::Marker
Px4OffboardNode::makeDebugMarker(const std::string& marker_namespace,
                                 const int marker_id, const int marker_type) const {
  visualization_msgs::msg::Marker marker;
  marker.header = makeDebugHeader();
  marker.ns = marker_namespace;
  marker.id = marker_id;
  marker.type = marker_type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

void Px4OffboardNode::addDroneDebugMarkers(
    visualization_msgs::msg::MarkerArray& markers) const {
  auto position =
      makeDebugMarker("drone_position", 0, visualization_msgs::msg::Marker::SPHERE);
  position.scale.x = 2.5;
  position.scale.y = 2.5;
  position.scale.z = 0.25;
  position.color.r = 0.68F;
  position.color.g = 0.20F;
  position.color.b = 1.0F;
  position.color.a = 1.0F;

  auto heading =
      makeDebugMarker("drone_heading", 0, visualization_msgs::msg::Marker::ARROW);
  heading.scale.x = 0.25;
  heading.scale.y = 0.75;
  heading.scale.z = 1.0;
  heading.color = position.color;

  if (!localPositionFresh()) {
    position.action = visualization_msgs::msg::Marker::DELETE;
    heading.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(position);
    markers.markers.push_back(heading);
    return;
  }

  position.pose.position = markerPoint(current_position_, kRvizGroundZ);
  const Point2 heading_end{current_position_.x + std::cos(current_heading_rad_) * 4.0,
                           current_position_.y + std::sin(current_heading_rad_) * 4.0};
  heading.points.push_back(markerPoint(current_position_, kRvizGroundZ + 0.06));
  heading.points.push_back(markerPoint(heading_end, kRvizGroundZ + 0.06));
  markers.markers.push_back(position);
  markers.markers.push_back(heading);
}

void Px4OffboardNode::publishOffboardDebugMarkers() {
  if (!offboard_debug_marker_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  addDroneDebugMarkers(markers);
  visualization_msgs::msg::MarkerArray trajectory_markers =
      buildTrajectoryDebugMarkers(makeDebugHeader(), final_trajectory_samples_,
                                  trajectory_speed_profile_, kRvizGroundZ);
  markers.markers.insert(markers.markers.end(),
                         std::make_move_iterator(trajectory_markers.markers.begin()),
                         std::make_move_iterator(trajectory_markers.markers.end()));
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
  if (diagnostics.path_stamp_ns != last_received_path_stamp_ns_) {
    return false;
  }
  return !latest_planner_path_id_seen_ ||
         diagnostics.planner_path_id == latest_planner_path_id_;
}

void Px4OffboardNode::mergePlannerDiagnosticsIntoCurrentTrajectoryStats(
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics) {
  if (!trajectoryDiagnosticsMatchesCurrentPath(diagnostics)) {
    return;
  }
  last_trajectory_planner_stats_.corridor = diagnostics.stats.corridor;
  last_trajectory_planner_stats_.racing_line = diagnostics.stats.racing_line;
  last_trajectory_planner_stats_.turn_smoothing = diagnostics.stats.turn_smoothing;
  last_trajectory_planner_stats_.total_duration_ms =
      diagnostics.stats.total_duration_ms;
  last_trajectory_planner_stats_.corridor_duration_ms =
      diagnostics.stats.corridor_duration_ms;
  last_trajectory_planner_stats_.racing_line_duration_ms =
      diagnostics.stats.racing_line_duration_ms;
  last_trajectory_planner_stats_.turn_smoothing_duration_ms =
      diagnostics.stats.turn_smoothing_duration_ms;
  last_trajectory_planner_stats_.speed_profile_duration_ms =
      diagnostics.stats.speed_profile_duration_ms;
}

void Px4OffboardNode::updatePlannerStatsForReceivedTrajectory() {
  last_trajectory_planner_stats_ = TrajectoryPlannerStats{};
  last_trajectory_planner_stats_.status =
      trajectory_valid_ ? TrajectoryPlannerStatus::kOk
                        : TrajectoryPlannerStatus::kInvalidTrajectory;
  last_trajectory_planner_stats_.input_points = path_points_.size();
  last_trajectory_planner_stats_.samples = final_trajectory_samples_.size();
  last_trajectory_planner_stats_.compact_segments = trajectory_.size();
  last_trajectory_planner_stats_.line_segments = last_trajectory_metrics_.line_segments;
  last_trajectory_planner_stats_.arc_segments = last_trajectory_metrics_.arc_segments;
  last_trajectory_planner_stats_.length_m = last_trajectory_metrics_.length_m;

  double curvature_abs_sum = 0.0;
  for (std::size_t i = 0U; i < final_trajectory_samples_.size(); ++i) {
    const double curvature = final_trajectory_samples_[i].curvature_1pm;
    if (i == 0U) {
      last_trajectory_planner_stats_.curvature_min_1pm = curvature;
      last_trajectory_planner_stats_.curvature_max_1pm = curvature;
    } else {
      last_trajectory_planner_stats_.curvature_min_1pm =
          std::min(last_trajectory_planner_stats_.curvature_min_1pm, curvature);
      last_trajectory_planner_stats_.curvature_max_1pm =
          std::max(last_trajectory_planner_stats_.curvature_max_1pm, curvature);
    }
    curvature_abs_sum += std::abs(curvature);
  }
  if (!final_trajectory_samples_.empty()) {
    last_trajectory_planner_stats_.curvature_mean_abs_1pm =
        curvature_abs_sum / static_cast<double>(final_trajectory_samples_.size());
  }

  double speed_sum = 0.0;
  for (std::size_t i = 0U; i < trajectory_speed_profile_.samples.size(); ++i) {
    const TrajectorySpeedSample& sample = trajectory_speed_profile_.samples[i];
    const double speed = sample.profiled_limit_mps;
    if (i == 0U) {
      last_trajectory_planner_stats_.speed_profile_min_mps = speed;
      last_trajectory_planner_stats_.speed_profile_max_mps = speed;
    } else {
      last_trajectory_planner_stats_.speed_profile_min_mps =
          std::min(last_trajectory_planner_stats_.speed_profile_min_mps, speed);
      last_trajectory_planner_stats_.speed_profile_max_mps =
          std::max(last_trajectory_planner_stats_.speed_profile_max_mps, speed);
    }
    speed_sum += speed;
    if (sample.reason == SpeedConstraintType::kArc) {
      ++last_trajectory_planner_stats_.speed_profile_curvature_limited_samples;
    }
  }
  if (!trajectory_speed_profile_.samples.empty()) {
    last_trajectory_planner_stats_.speed_profile_mean_mps =
        speed_sum / static_cast<double>(trajectory_speed_profile_.samples.size());
  }
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

void Px4OffboardNode::applyReceivedFinalTrajectoryPath(const char* source_label) {
  final_trajectory_samples_ = trajectoryPointSamplesFromPoints(path_points_);
  trajectory_ = lineTrajectoryFromSamples(final_trajectory_samples_);
  trajectory_speed_profile_ =
      buildTrajectorySpeedProfile(final_trajectory_samples_, velocity_follower_config_);
  trajectory_valid_ = trajectorySamplesAreUsable(final_trajectory_samples_) &&
                      trajectory_speed_profile_.valid;
  last_trajectory_route_points_ = path_points_.size();
  last_trajectory_metrics_ = trajectoryMetrics(trajectory_);
  last_trajectory_shape_diagnostics_ =
      computeTrajectoryShapeDiagnostics(final_trajectory_samples_);
  updatePlannerStatsForReceivedTrajectory();
  if (!trajectory_valid_) {
    resetVelocityDiagnostics();
  } else {
    resetVelocitySmootherState("new_trajectory", true);
  }
  publishFinalTrajectoryDebug();
  publishOffboardDebugMarkers();
  const std::string samples_csv_path = writeFinalTrajectorySamplesCsv(source_label);
  RCLCPP_INFO(
      get_logger(),
      "Received executable final trajectory: source=%s local_path_update_id=%" PRIu64
      " planner_path_id=%" PRIu64
      " points=%zu valid=%s line_segments=%zu total_length=%.2f samples=%zu "
      "speed_profile[min=%.2f mean=%.2f max=%.2f curvature_limited=%zu] "
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
      last_trajectory_shape_diagnostics_.segment_count,
      last_trajectory_shape_diagnostics_.min_segment_length_m,
      last_trajectory_shape_diagnostics_.mean_segment_length_m,
      last_trajectory_shape_diagnostics_.max_segment_length_m,
      radiansToDegrees(last_trajectory_shape_diagnostics_.max_heading_delta_rad),
      last_trajectory_shape_diagnostics_.max_curvature_jump_1pm,
      samples_csv_path.c_str());
}

void Px4OffboardNode::onPath(const nav_msgs::msg::Path& path) {
  ++received_path_update_id_;
  last_received_path_stamp_ns_ = stampNanoseconds(path.header.stamp);

  path_points_ = pathPointsFromMessage(path);
  path_valid_ = !path_points_.empty();

  if (!path_valid_) {
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

  no_path_hold_target_valid_ = false;
  const std::size_t candidate_index =
      localPositionFresh()
          ? drone_city_nav::advanceWaypointIndex(path_points_, current_position_, 0U,
                                                 pathFollowerConfig())
          : 0U;
  waypoint_index_ = candidate_index;
  applyReceivedFinalTrajectoryPath("path_update");
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

[[nodiscard]] std::vector<double>
Px4OffboardNode::finalTrajectoryProfiledTimesFromStart() const {
  std::vector<double> times(final_trajectory_samples_.size(),
                            std::numeric_limits<double>::quiet_NaN());
  if (final_trajectory_samples_.empty() || !trajectory_speed_profile_.valid) {
    return times;
  }

  constexpr double kMinimumIntegrationSpeedMps = 0.1;
  times.front() = 0.0;
  for (std::size_t i = 1U; i < final_trajectory_samples_.size(); ++i) {
    double ds =
        final_trajectory_samples_[i].s_m - final_trajectory_samples_[i - 1U].s_m;
    if (!(ds > kTinyDistanceM) || !std::isfinite(ds)) {
      ds = distance(final_trajectory_samples_[i - 1U].point,
                    final_trajectory_samples_[i].point);
    }
    if (!(ds > kTinyDistanceM) || !std::isfinite(ds) || !std::isfinite(times[i - 1U])) {
      continue;
    }
    const TrajectorySpeedSample start = speedProfileSampleAtS(
        trajectory_speed_profile_, final_trajectory_samples_[i - 1U].s_m);
    const TrajectorySpeedSample end = speedProfileSampleAtS(
        trajectory_speed_profile_, final_trajectory_samples_[i].s_m);
    const double average_speed =
        std::max(kMinimumIntegrationSpeedMps,
                 0.5 * (start.profiled_limit_mps + end.profiled_limit_mps));
    if (std::isfinite(average_speed)) {
      times[i] = times[i - 1U] + ds / average_speed;
    }
  }
  return times;
}

bool Px4OffboardNode::writeFinalTrajectorySamplesCsvFile(
    const std::filesystem::path& path, const char* source_label) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << source_label
         << " local_path_update_id=" << received_path_update_id_
         << " planner_path_id=" << latest_planner_path_id_
         << " trajectory_valid=" << (trajectory_valid_ ? "true" : "false")
         << " trajectory_status="
         << trajectoryPlannerStatusName(last_trajectory_planner_stats_.status) << "\n";
  stream << finalTrajectorySamplesCsvHeader() << "\n";
  const std::vector<double> times_from_start = finalTrajectoryProfiledTimesFromStart();
  const double total_time_s = times_from_start.empty()
                                  ? std::numeric_limits<double>::quiet_NaN()
                                  : times_from_start.back();
  for (std::size_t i = 0U; i < final_trajectory_samples_.size(); ++i) {
    const TrajectoryPointSample& sample = final_trajectory_samples_[i];
    const TrajectorySpeedSample speed_sample =
        trajectory_speed_profile_.valid
            ? speedProfileSampleAtS(trajectory_speed_profile_, sample.s_m)
            : TrajectorySpeedSample{};
    const double time_from_start_s = i < times_from_start.size()
                                         ? times_from_start[i]
                                         : std::numeric_limits<double>::quiet_NaN();
    const double time_to_finish_s =
        std::isfinite(total_time_s) && std::isfinite(time_from_start_s)
            ? std::max(0.0, total_time_s - time_from_start_s)
            : std::numeric_limits<double>::quiet_NaN();
    stream << finalTrajectorySamplesCsvRow(i, sample, speed_sample, time_from_start_s,
                                           time_to_finish_s)
           << "\n";
  }
  return stream.good();
}

bool Px4OffboardNode::writeFinalTrajectorySummaryJsonFile(
    const std::filesystem::path& path) const {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }
  stream << finalTrajectoryDiagnosticsSummaryJson(last_trajectory_planner_stats_,
                                                  last_trajectory_shape_diagnostics_)
         << "\n";
  return stream.good();
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
