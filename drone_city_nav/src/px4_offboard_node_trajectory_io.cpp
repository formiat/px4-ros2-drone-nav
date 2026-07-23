#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <span>
#include <string>
#include <string_view>

#include "px4_offboard_node.hpp"

namespace drone_city_nav {
namespace {

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
  visualization_msgs::msg::MarkerArray markers = buildOffboardDebugMarkers(
      makeDebugHeader(), drone_state, final_trajectory_samples_,
      trajectory_speed_profile_, px4_local_origin_, mission_goal_);
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
