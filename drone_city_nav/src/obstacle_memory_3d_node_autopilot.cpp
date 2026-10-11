#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/cooperative_traffic_ros.hpp"
#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/latest_processing_queue.hpp"
#include "drone_city_nav/latest_sensor_obstacle_scan.hpp"
#include "drone_city_nav/latest_sensor_obstacle_scan_ros.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/lidar_scan_3d.hpp"
#include "drone_city_nav/lidar_self_filter.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/latest_sensor_obstacle_scan.hpp"
#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/spectator_diagnostics_selection.hpp"
#include "drone_city_nav/spectator_diagnostics_selection_ros.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"
#include "drone_city_nav/transport_latency_ros.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "obstacle_memory_3d_node.hpp"
#include "obstacle_memory_3d_worker.hpp"
#include "obstacle_memory_node_helpers.hpp"

namespace drone_city_nav {

// The autopilot's state: the position, the attitude and the clock, each
// recorded into the pose history and each a chance to process a waiting scan.

void ObstacleMemory3DNode::onLocalState(const AutopilotLocalState& message) {
  {
    const std::scoped_lock pose_lock{pose_history_mutex_};
    const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
    const bool heading_ready = px4HeadingReadyForMapping(
        message.yaw_rad, message.heading_variance_rad2, maximum_heading_variance_rad2_);
    const MappingYawSelection mapping_yaw =
        mapping_yaw_tracker_.update(heading_ready, message.yaw_rad);
    if (use_px4_heading_for_scan_ &&
        mapping_yaw.source == MappingYawSource::kPx4Heading &&
        last_mapping_yaw_source_ != MappingYawSource::kPx4Heading) {
      lidar_pose_history_.startNewGeneration();
    }
    if (mapping_yaw.source != last_mapping_yaw_source_) {
      RCLCPP_INFO(
          get_logger(),
          "LIDAR3D_MAPPING_YAW source=%s yaw=%.3f autopilot_yaw=%.3f "
          "heading_valid=%s mapping_ready=%s stable_samples=%zu "
          "required_samples=%zu maximum_sample_delta_rad=%.3f "
          "pose_history_generation=%" PRIu64,
          mappingYawSourceName(mapping_yaw.source), mapping_yaw.yaw_rad,
          message.yaw_rad, message.heading_valid ? "true" : "false",
          heading_ready ? "true" : "false", mapping_yaw_tracker_.stableSampleCount(),
          startup_heading_stable_sample_count_,
          startup_heading_maximum_sample_delta_rad_, lidar_pose_history_.generation());
    }
    last_mapping_yaw_source_ = mapping_yaw.source;
    const MapPositionSample sample{
        .position = Point2{message.position.x, message.position.y},
        .altitude_m = message.position.z,
        .yaw_rad = mapping_yaw.yaw_rad,
        .stamp_ns = static_cast<std::int64_t>(message.timestamp_sample_us) * 1000LL,
        .position_valid = message.position_valid,
        .altitude_valid = message.altitude_valid,
        .yaw_valid = mapping_yaw.valid};
    const Px4LocalPoseUpdateStatus status =
        updateNavigationPoseFromMapPosition(sample, current_pose_);
    if (status != Px4LocalPoseUpdateStatus::kAccepted) {
      last_pose_update_ns_ = 0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LIDAR3D_POSE_HISTORY position_rejected=true status=%s position_valid=%s "
          "altitude_valid=%s heading_valid=%s yaw=%.3f "
          "heading_variance=%.6f maximum_heading_variance=%.6f mapping_ready=%s",
          status == Px4LocalPoseUpdateStatus::kInvalidPosition ? "invalid_position"
                                                               : "invalid_yaw",
          message.position_valid ? "true" : "false",
          message.altitude_valid ? "true" : "false",
          message.heading_valid ? "true" : "false", message.yaw_rad,
          message.heading_variance_rad2, maximum_heading_variance_rad2_,
          heading_ready ? "true" : "false");
      return;
    }
    last_pose_update_ns_ = receive_stamp_ns;
    const LidarPoseSourceStampResult source_stamp =
        resolveLidarPoseSourceStamp(time_mapper_, message.timestamp_sample_us,
                                    receive_stamp_ns, pose_source_stamp_config_);
    if (source_stamp.resolved()) {
      lidar_pose_history_.addPosition(
          receive_stamp_ns,
          Point3{current_pose_.pose.position.x, current_pose_.pose.position.y,
                 current_pose_.altitude_m},
          current_pose_.pose.yaw_rad,
          current_pose_.yaw_valid && current_pose_.altitude_valid,
          source_stamp.acquisition_stamp_ns,
          lidarPoseSourceTimestampNanoseconds(message.timestamp_sample_us));
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LIDAR3D_POSE_HISTORY position_source_rejected=true status=%s "
          "timestamp_sample_us=%" PRIu64 " acquisition_stamp_ns=%" PRId64
          " mapped_ros_stamp_ns=%" PRId64 " receive_delta_ms=%.3f",
          lidarPoseSourceStampStatusName(source_stamp.status),
          message.timestamp_sample_us, source_stamp.acquisition_stamp_ns,
          source_stamp.mapped_ros_stamp_ns,
          1.0e-6 * static_cast<double>(source_stamp.receive_delta_ns));
    }
  }
  processPendingClouds();
}

void ObstacleMemory3DNode::onAttitude(const AutopilotAttitude& message) {
  {
    const std::scoped_lock pose_lock{pose_history_mutex_};
    const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
    const LidarPoseSourceStampResult source_stamp =
        resolveLidarPoseSourceStamp(time_mapper_, message.timestamp_sample_us,
                                    receive_stamp_ns, pose_source_stamp_config_);
    if (source_stamp.resolved()) {
      lidar_pose_history_.addAttitude(
          receive_stamp_ns, message.quaternion, source_stamp.acquisition_stamp_ns,
          lidarPoseSourceTimestampNanoseconds(message.timestamp_sample_us));
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LIDAR3D_POSE_HISTORY attitude_source_rejected=true status=%s "
          "timestamp_sample_us=%" PRIu64 " acquisition_stamp_ns=%" PRId64
          " mapped_ros_stamp_ns=%" PRId64 " receive_delta_ms=%.3f",
          lidarPoseSourceStampStatusName(source_stamp.status),
          message.timestamp_sample_us, source_stamp.acquisition_stamp_ns,
          source_stamp.mapped_ros_stamp_ns,
          1.0e-6 * static_cast<double>(source_stamp.receive_delta_ns));
    }
  }
  processPendingClouds();
}

void ObstacleMemory3DNode::onClockSync(const AutopilotClockSync& message) {
  {
    const std::scoped_lock pose_lock{pose_history_mutex_};
    const Px4RosTimeObservation observation = time_mapper_.observeTimesync(
        message.timestamp_us, message.estimated_offset_us, message.round_trip_time_us,
        get_clock()->now().nanoseconds());
    if (observation.rebased()) {
      lidar_pose_history_.startNewGeneration();
      const Px4RosTimeMappingDiagnostics diagnostics = time_mapper_.diagnostics();
      RCLCPP_WARN(get_logger(),
                  "LIDAR3D_TIME_MAPPING rebased=true time_generation=%" PRIu64
                  " pose_history_generation=%" PRIu64 " samples=%zu rejected=%" PRIu64
                  " discontinuities=%" PRIu64 " rebases=%" PRIu64,
                  diagnostics.generation, lidar_pose_history_.generation(),
                  diagnostics.sample_count, diagnostics.rejected_sample_count,
                  diagnostics.clock_discontinuity_count, diagnostics.rebase_count);
    }
  }
  processPendingClouds();
}

} // namespace drone_city_nav
