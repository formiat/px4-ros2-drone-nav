#pragma once

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

#include "obstacle_memory_3d_worker.hpp"
#include "obstacle_memory_node_helpers.hpp"

namespace drone_city_nav {

struct PendingPointCloud3D {
  sensor_msgs::msg::PointCloud2 cloud;
  std::int64_t receive_stamp_ns{0};
};

enum class PendingPointCloudDisposition : std::uint8_t {
  kWaitForPoseBracket,
  kConsumed,
};

class ObstacleMemory3DNode final : public rclcpp::Node {
public:
  ObstacleMemory3DNode();

  ObstacleMemory3DNode(const ObstacleMemory3DNode&) = delete;
  ObstacleMemory3DNode& operator=(const ObstacleMemory3DNode&) = delete;
  ObstacleMemory3DNode(ObstacleMemory3DNode&&) = delete;
  ObstacleMemory3DNode& operator=(ObstacleMemory3DNode&&) = delete;

  ~ObstacleMemory3DNode() override {
    memory_worker_.reset();
  }

private:
  void onLocalState(const AutopilotLocalState& message);

  void onAttitude(const AutopilotAttitude& message);

  void onClockSync(const AutopilotClockSync& message);

  void onPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    const auto submission = pending_clouds_.submit(
        PendingPointCloud3D{std::move(*cloud), get_clock()->now().nanoseconds()});
    if (submission.replaced_pending) {
      ++alignment_coalesced_clouds_;
      RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LIDAR3D_ALIGNMENT coalesced=true reason=newer_cloud_available "
          "coalesced_total=%" PRIu64,
          alignment_coalesced_clouds_);
    }
    if (submission.processor_acquired) {
      drainPendingClouds();
    }
  }

  void processPendingClouds() {
    if (pending_clouds_.tryAcquireProcessor()) {
      drainPendingClouds();
    }
  }

  void drainPendingClouds() {
    while (std::optional<PendingPointCloud3D> pending = pending_clouds_.take()) {
      const PendingPointCloudDisposition disposition = processPendingCloud(*pending);
      if (disposition == PendingPointCloudDisposition::kWaitForPoseBracket) {
        // The scan keeps its place until its pose bracket arrives or its wait
        // expires; a newer scan waits behind it rather than replacing it. Its
        // bracket lies a scan period ahead of this one's, so replacing the
        // scan starved the memory of every scan whenever the bracket wait
        // came close to the scan period.
        pending_clouds_.defer(std::move(*pending));
        return;
      }
    }
  }

  [[nodiscard]] PendingPointCloudDisposition
  processPendingCloud(const PendingPointCloud3D& pending);

  GridBounds3D bounds_{};
  ObservedOccupancyGrid3D grid_geometry_;
  OrganizedLidarScan3DConfig scan_config_{};
  bool hit_only_returns_{false};
  // The sensor's positions along the last 60 m flown, every half metre, and
  // how close to them unobservable evidence may not come: the body's 0.55 m
  // radius and half a voxel.
  static constexpr double kFlownPathSpacingM{0.5};
  static constexpr std::size_t kFlownPathPoses{120U};
  static constexpr double kFlownPathClearanceM{0.7};
  std::deque<Point3> flown_path_;
  LidarSurfaceInterpolation3DConfig surface_interpolation_config_{};
  LidarProjectionConfig projection_config_{};
  LidarSelfFilterConfig self_filter_config_{};
  std::unique_ptr<ObstacleMemory3DWorker> memory_worker_;
  std::unique_ptr<DynamicAgentLidarState> dynamic_agent_state_;
  NavigationPose2D current_pose_{};
  Px4LocalPoseConfig px4_local_pose_config_{};
  MappingYawTracker mapping_yaw_tracker_;
  MappingYawSource last_mapping_yaw_source_{MappingYawSource::kUnavailable};
  LidarPoseHistory lidar_pose_history_;
  Px4RosTimeMapper time_mapper_;
  LidarAcquisitionPoseConfig acquisition_pose_config_{};
  LidarPoseSourceStampConfig pose_source_stamp_config_{};
  SpectatorDiagnosticsSelection persistent_memory_selection_;
  LatestProcessingQueue<PendingPointCloud3D> pending_clouds_;
  std::mutex pose_history_mutex_;
  std::string frame_id_{"map"};
  bool gazebo_aligned_rviz_axes_swapped_{true};
  double initial_heading_rad_{0.0};
  double maximum_heading_variance_rad2_{0.05};
  double startup_heading_maximum_sample_delta_rad_{0.05};
  double min_mapping_altitude_m_{0.0};
  std::size_t startup_heading_stable_sample_count_{5U};
  std::int64_t alignment_maximum_wait_ns_{350'000'000};
  std::int64_t last_pose_update_ns_{0};
  const std::uint64_t latest_scan_producer_instance_id_{
      createLatestSensorObstacleProducerInstanceId()};
  std::uint64_t latest_scan_sequence_{0U};
  std::uint64_t alignment_coalesced_clouds_{0U};
  bool persistent_memory_enabled_{true};
  bool persistent_memory_diagnostics_enabled_{true};
  bool use_px4_heading_for_scan_{true};

  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr pose_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  // The hop from the Gazebo bridge: what the transport took to deliver each
  // scan, reported with the alignment diagnostic.
  TransportLatencySamples cloud_delivery_ms_;
  Px4MapFrameTransform map_transform_{};
  std::unique_ptr<AutopilotStateSource> autopilot_state_source_;
  rclcpp::Subscription<msg::CooperativeFlightIntent>::SharedPtr cooperative_intent_sub_;
  rclcpp::Subscription<msg::SpectatorTarget>::SharedPtr spectator_target_sub_;
  rclcpp::Publisher<msg::LatestSensorObstacleScan>::SharedPtr latest_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_returns_pub_;
};

} // namespace drone_city_nav
