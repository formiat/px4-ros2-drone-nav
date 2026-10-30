#include "obstacle_memory_3d_node.hpp"

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
namespace {

[[nodiscard]] GridBounds3D declareGridBounds3D(rclcpp::Node& node) {
  const double resolution = node.declare_parameter<double>("grid_resolution_m", 0.25);
  const double width_m = node.declare_parameter<double>("grid_width_m", 120.0);
  const double height_m = node.declare_parameter<double>("grid_height_m", 80.0);
  const double depth_m = node.declare_parameter<double>("grid_depth_m", 40.0);
  if (!std::isfinite(resolution) || resolution <= 0.0 || !std::isfinite(width_m) ||
      !std::isfinite(height_m) || !std::isfinite(depth_m) || width_m <= 0.0 ||
      height_m <= 0.0 || depth_m <= 0.0) {
    throw std::invalid_argument{"invalid online Occupancy3D dimensions"};
  }
  const GridBounds3D bounds{
      .origin_x = node.declare_parameter<double>("grid_origin_x", -20.0),
      .origin_y = node.declare_parameter<double>("grid_origin_y", -40.0),
      .origin_z = node.declare_parameter<double>("grid_origin_z", 0.0),
      .resolution_m = resolution,
      .width_cells = static_cast<int>(std::ceil(width_m / resolution)),
      .height_cells = static_cast<int>(std::ceil(height_m / resolution)),
      .depth_cells = static_cast<int>(std::ceil(depth_m / resolution)),
  };
  if (bounds.width_cells <= 0 || bounds.height_cells <= 0 || bounds.depth_cells <= 0) {
    throw std::invalid_argument{"invalid online Occupancy3D bounds"};
  }
  return bounds;
}

} // namespace

ObstacleMemory3DNode::ObstacleMemory3DNode()
    : Node{"obstacle_memory_3d_node"},
      bounds_{declareGridBounds3D(*this)},
      grid_geometry_{bounds_} {
  cloud_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pose_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions cloud_subscription_options;
  cloud_subscription_options.callback_group = cloud_callback_group_;
  rclcpp::SubscriptionOptions pose_subscription_options;
  pose_subscription_options.callback_group = pose_callback_group_;
  // Roadmap item 36: the step acknowledgement follows the clouds' callbacks and
  // waits for the integration thread to finish the cloud it is on.
  step_acknowledger_ = std::make_unique<StepAcknowledger>(
      *this, "memory", cloud_callback_group_,
      [this] { return !pending_clouds_.processing(); });
  persistent_memory_enabled_ =
      declare_parameter<bool>("persistent_memory_enabled", true);
  persistent_memory_diagnostics_enabled_ = declare_parameter<bool>(
      "persistent_memory_diagnostics_enabled", persistent_memory_enabled_);
  persistent_memory_selection_ = SpectatorDiagnosticsSelection{
      declare_parameter<std::string>("persistent_memory_spectator_vehicle_id", "")};
  const std::string spectator_target_topic = declare_parameter<std::string>(
      "persistent_memory_spectator_target_topic", "/drone_city_nav/spectator_target");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  gazebo_aligned_rviz_axes_swapped_ = declare_parameter<bool>(
      std::string{kGazeboAlignedRvizAxesSwappedParameter}, true);
  static_cast<void>(declare_parameter<bool>("use_static_map", false));

  scan_config_.horizontal_samples = static_cast<std::size_t>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("lidar_3d_horizontal_samples", 240), 1, 4096));
  scan_config_.vertical_samples = static_cast<std::size_t>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("lidar_3d_vertical_samples", 19), 1, 1024));
  scan_config_.horizontal_min_angle_rad =
      declare_parameter<double>("lidar_3d_horizontal_min_angle_rad", -std::numbers::pi);
  scan_config_.horizontal_max_angle_rad =
      declare_parameter<double>("lidar_3d_horizontal_max_angle_rad", std::numbers::pi);
  scan_config_.vertical_min_angle_rad =
      declare_parameter<double>("lidar_3d_vertical_min_angle_rad", -1.5707963268);
  scan_config_.vertical_max_angle_rad =
      declare_parameter<double>("lidar_3d_vertical_max_angle_rad", 1.5707963268);
  scan_config_.minimum_range_m =
      declare_parameter<double>("lidar_3d_minimum_range_m", 0.2);
  scan_config_.maximum_range_m = declare_parameter<double>("max_lidar_range_m", 35.0);
  scan_config_.hit_epsilon_m = declare_parameter<double>("range_hit_epsilon_m", 0.05);
  if (!organizedLidarScan3DConfigIsValid(scan_config_)) {
    throw std::invalid_argument{"invalid organized 3D lidar configuration"};
  }
  // A sensor that answers where it measured and says nothing elsewhere,
  // depth recovered from a stereo pair: its returns are hits along their own
  // rays, with no organized grid and no misses.
  hit_only_returns_ = declare_parameter<bool>("lidar_3d_hit_only_returns", false);
  surface_interpolation_config_.enabled =
      declare_parameter<bool>("lidar_surface_interpolation_enabled", true);
  surface_interpolation_config_.maximum_incidence_rad =
      declare_parameter<double>("lidar_surface_interpolation_maximum_incidence_deg",
                                30.0) *
      std::numbers::pi / 180.0;
  surface_interpolation_config_.maximum_row_incidence_rad =
      declare_parameter<double>("lidar_surface_interpolation_maximum_row_incidence_deg",
                                80.0) *
      std::numbers::pi / 180.0;
  // Samples denser than a voxel edge so a joined surface marks every voxel
  // it crosses.
  surface_interpolation_config_.sample_spacing_m = 0.7 * bounds_.resolution_m;
  if (!lidarSurfaceInterpolation3DConfigIsValid(surface_interpolation_config_)) {
    throw std::invalid_argument{"invalid 3D lidar surface interpolation configuration"};
  }

  ObstacleMemory3DConfig memory_config;
  // A hit is placed with the vehicle's heading estimate, and a heading a
  // fraction of a degree in error moves the hit sideways in proportion to
  // its range. The persistent memory integrates a beam only as far as one
  // heading standard deviation still lands its hit within two voxels of
  // the surface it came from; beyond that the same wall seen at different
  // headings became several rotated copies of itself, and single hits
  // landed in the corridors between them.
  const double heading_uncertainty_rad =
      declare_parameter<double>("lidar_pose_heading_uncertainty_rad", 0.0);
  const double heading_bounded_range_m =
      heading_uncertainty_rad > 0.0
          ? 2.0 * bounds_.resolution_m / std::tan(heading_uncertainty_rad)
          : scan_config_.maximum_range_m;
  memory_config.maximum_range_m =
      std::min(scan_config_.maximum_range_m, heading_bounded_range_m);
  RCLCPP_INFO(get_logger(),
              "3D obstacle memory integration range: %.2f m (sensor %.2f m, "
              "heading uncertainty %.4f rad)",
              memory_config.maximum_range_m, scan_config_.maximum_range_m,
              heading_uncertainty_rad);
  memory_config.minimum_range_m = scan_config_.minimum_range_m;
  memory_config.scan_stride = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("scan_stride", 1), 1, 100000));
  memory_config.hit_weight = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("hit_weight", 4), 1, 100000));
  memory_config.miss_weight = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("miss_weight", 1), 1, 100000));
  memory_config.minimum_score = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("min_score", -8), -100000, 0));
  memory_config.maximum_score = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("max_score", 12), 1, 100000));
  memory_config.occupied_score = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("occupied_score", 3), 1, 100000));
  memory_config.free_score = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("free_score", -1), -100000, -1));
  memory_config.decay_seconds_per_confirmation =
      std::max(0.0, declare_parameter<double>("decay_seconds_per_confirmation", 0.0));
  const LidarMappingYawConfig yaw_config = declareLidarMappingYawConfig(*this);
  use_px4_heading_for_scan_ = yaw_config.use_px4_heading;
  initial_heading_rad_ = yaw_config.initial_heading_rad;
  maximum_heading_variance_rad2_ = yaw_config.maximum_heading_variance_rad2;
  startup_heading_stable_sample_count_ = yaw_config.startup_stable_sample_count;
  startup_heading_maximum_sample_delta_rad_ =
      yaw_config.startup_maximum_sample_delta_rad;
  mapping_yaw_tracker_ = MappingYawTracker{
      use_px4_heading_for_scan_, initial_heading_rad_,
      startup_heading_stable_sample_count_, startup_heading_maximum_sample_delta_rad_};

  acquisition_pose_config_ = declareLidarAcquisitionPoseConfig(*this);
  alignment_maximum_wait_ns_ = static_cast<std::int64_t>(
      std::clamp(declare_parameter<double>("lidar_scan_alignment_maximum_wait_s", 0.35),
                 0.0, 2.0) *
      1.0e9);
  pose_source_stamp_config_.maximum_receive_delay_ns = static_cast<std::int64_t>(
      std::clamp(
          declare_parameter<double>("lidar_pose_source_maximum_receive_delay_s", 1.0),
          0.0, 10.0) *
      1.0e9);
  pose_source_stamp_config_.maximum_future_skew_ns = static_cast<std::int64_t>(
      std::clamp(
          declare_parameter<double>("lidar_pose_source_maximum_future_skew_s", 0.1),
          0.0, 1.0) *
      1.0e9);
  static_cast<void>(declare_parameter<double>("max_pose_staleness_s", 1.0));
  min_mapping_altitude_m_ = declare_parameter<double>("min_mapping_altitude_m", 0.0);
  const std::size_t memory_scan_queue_capacity =
      static_cast<std::size_t>(std::clamp<std::int64_t>(
          declare_parameter<std::int64_t>("lidar_memory_scan_queue_capacity", 8), 1,
          128));
  if (persistent_memory_enabled_) {
    memory_worker_ = std::make_unique<ObstacleMemory3DWorker>(
        *this, bounds_, memory_config, min_mapping_altitude_m_, frame_id_,
        gazebo_aligned_rviz_axes_swapped_, memory_scan_queue_capacity);
  }

  projection_config_.max_lidar_range_m = scan_config_.maximum_range_m;
  projection_config_.range_hit_epsilon_m = scan_config_.hit_epsilon_m;
  projection_config_.scan_yaw_offset_rad =
      declare_parameter<double>("scan_yaw_offset_rad", 0.0);
  projection_config_.lidar_z_offset_m =
      declare_parameter<double>("lidar_z_offset_m", 0.0);
  projection_config_.compensate_attitude =
      declare_parameter<bool>("compensate_lidar_attitude", true);
  projection_config_.lidar_mount_roll_rad =
      declare_parameter<double>("lidar_mount_roll_rad", 0.0);
  projection_config_.lidar_mount_pitch_rad =
      declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
  projection_config_.lidar_mount_yaw_rad =
      declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
  projection_config_.use_full_lidar_extrinsic =
      declare_parameter<bool>("use_full_lidar_extrinsic", true);
  const std::vector<double> translation = declare_parameter<std::vector<double>>(
      "lidar_extrinsic_translation_body_frd_m", {0.12, 0.0, -0.315});
  const std::vector<double> rotation = declare_parameter<std::vector<double>>(
      "lidar_extrinsic_quaternion_lidar_flu_to_body_frd", {0.0, 1.0, 0.0, 0.0});
  if (translation.size() != 3U || rotation.size() != 4U) {
    throw std::invalid_argument{"invalid 3D lidar extrinsic"};
  }
  projection_config_.lidar_translation_body_frd_m = {
      translation.at(0), translation.at(1), translation.at(2)};
  projection_config_.lidar_flu_to_body_frd_quaternion = {
      rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3)};
  self_filter_config_.horizontal_radius_m =
      declare_parameter<double>("lidar_self_filter_radius_m", 0.9);
  self_filter_config_.upward_extent_m =
      declare_parameter<double>("lidar_self_filter_upward_extent_m", 0.5);
  self_filter_config_.downward_extent_m =
      declare_parameter<double>("lidar_self_filter_downward_extent_m", 0.5);
  if (!lidarSelfFilterConfigIsValid(self_filter_config_)) {
    throw std::invalid_argument{"invalid 3D lidar self-filter configuration"};
  }

  const bool use_initial_pose =
      declare_parameter<bool>("use_initial_pose_until_px4", true);
  px4_local_pose_config_ =
      Px4LocalPoseConfig{use_px4_heading_for_scan_,
                         initial_heading_rad_,
                         declare_parameter<double>("px4_local_origin_x_m", 0.0),
                         declare_parameter<double>("px4_local_origin_y_m", 0.0),
                         declare_parameter<double>("px4_local_origin_z_m", 0.0),
                         declare_parameter<double>("px4_to_map_m00", 1.0),
                         declare_parameter<double>("px4_to_map_m01", 0.0),
                         declare_parameter<double>("px4_to_map_m10", 0.0),
                         declare_parameter<double>("px4_to_map_m11", 1.0)};
  projection_config_.px4_to_map_m00 = px4_local_pose_config_.px4_to_map_m00;
  projection_config_.px4_to_map_m01 = px4_local_pose_config_.px4_to_map_m01;
  projection_config_.px4_to_map_m10 = px4_local_pose_config_.px4_to_map_m10;
  projection_config_.px4_to_map_m11 = px4_local_pose_config_.px4_to_map_m11;
  map_transform_ = Px4MapFrameTransform{
      .map_origin = Point3{px4_local_pose_config_.map_origin_x_m,
                           px4_local_pose_config_.map_origin_y_m,
                           px4_local_pose_config_.map_origin_z_m},
      .m00 = px4_local_pose_config_.px4_to_map_m00,
      .m01 = px4_local_pose_config_.px4_to_map_m01,
      .m10 = px4_local_pose_config_.px4_to_map_m10,
      .m11 = px4_local_pose_config_.px4_to_map_m11,
  };
  current_pose_.pose.yaw_rad = initial_heading_rad_;
  current_pose_.yaw_valid =
      !use_px4_heading_for_scan_ && std::isfinite(initial_heading_rad_);
  if (use_initial_pose) {
    current_pose_.pose.position = Point2{declare_parameter<double>("initial_x_m", 0.0),
                                         declare_parameter<double>("initial_y_m", 0.0)};
    current_pose_.position_valid = true;
    last_pose_update_ns_ = get_clock()->now().nanoseconds();
  }

  const std::string cloud_topic =
      declare_parameter<std::string>("lidar_3d_topic", "/lidar_3d/points");
  const std::string local_position_topic = declare_parameter<std::string>(
      "px4_local_position_topic", "/fmu/out/vehicle_local_position");
  const std::string attitude_topic = declare_parameter<std::string>(
      "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
  const std::string timesync_topic = declare_parameter<std::string>(
      "px4_timesync_status_topic", "/fmu/out/timesync_status");
  // Lockstep simulation: the autopilot stamps on the simulation clock.
  time_mapper_ = Px4RosTimeMapper{Px4RosTimeMapperConfig{
      .shared_clock = declare_parameter<bool>("px4_clock_is_ros_clock", false)}};
  const std::string vehicle_status_topic = declare_parameter<std::string>(
      "px4_vehicle_status_topic", "/fmu/out/vehicle_status_v1");
  const auto sensor_qos = rclcpp::SensorDataQoS{};
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS{}.keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud,
             const rclcpp::MessageInfo& info) {
        cloud_delivery_ms_.add(transportDeliveryLatencyMs(info));
        onPointCloud(std::move(cloud));
      },
      cloud_subscription_options);
  autopilot_state_source_ = std::make_unique<AutopilotStateSource>(
      *this, map_transform_,
      AutopilotStateTopics{
          .local_state = local_position_topic,
          .attitude = attitude_topic,
          .imu = {},
          .clock_sync = timesync_topic,
          .status = vehicle_status_topic,
          .ground_contact = {},
          .links = {},
          .battery = {},
      },
      sensor_qos,
      AutopilotStateCallbacks{
          .local_state =
              [this](const AutopilotLocalState& message) { onLocalState(message); },
          .attitude = [this](const AutopilotAttitude& message) { onAttitude(message); },
          .imu = {},
          .clock_sync =
              [this](const AutopilotClockSync& message) { onClockSync(message); },
          .status =
              [this](const AutopilotStatus& status) {
                if (memory_worker_) {
                  memory_worker_->updateArmed(status.armed);
                }
              },
          .ground_contact = {},
          .links = {},
          .battery = {},
      },
      pose_subscription_options, cloud_subscription_options);

  const DynamicAgentLidarStateConfig dynamic_config =
      declareDynamicAgentLidarStateConfig(*this);
  dynamic_agent_state_ = std::make_unique<DynamicAgentLidarState>(dynamic_config);
  if (dynamic_config.cooperative_enabled) {
    cooperative_intent_sub_ = create_subscription<msg::CooperativeFlightIntent>(
        declare_parameter<std::string>("cooperative_flight_intent_topic",
                                       "/cooperative_traffic/flight_intents"),
        cooperativeFlightIntentQos(),
        [this](const msg::CooperativeFlightIntent::SharedPtr intent) {
          static_cast<void>(dynamic_agent_state_->updateCooperativeIntent(
              cooperativeFlightIntentData(*intent), get_clock()->now().nanoseconds()));
        },
        cloud_subscription_options);
  }

  latest_scan_pub_ = create_publisher<msg::LatestSensorObstacleScan>(
      declare_parameter<std::string>("latest_sensor_obstacle_scan_topic",
                                     "/drone_city_nav/latest_sensor_obstacle_scan"),
      sensor_qos);
  current_returns_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("current_lidar_3d_pointcloud_topic",
                                     "/drone_city_nav/current_lidar_returns_3d"),
      rclcpp::QoS{1}.best_effort());
  spectator_target_sub_ = subscribeSpectatorDiagnosticsSelection(
      *this, spectator_target_topic, persistent_memory_selection_,
      "OBSTACLE_MEMORY_3D_SPECTATOR");

  RCLCPP_INFO(
      get_logger(),
      "3D lidar obstacle memory ready: topic='%s' beams=%zux%zu range=[%.2f,%.2f] "
      "vertical=[%.3f,%.3f] grid=%dx%dx%d resolution=%.2f origin=(%.1f,%.1f,%.1f) "
      "persistent=%s",
      cloud_topic.c_str(), scan_config_.horizontal_samples,
      scan_config_.vertical_samples, scan_config_.minimum_range_m,
      scan_config_.maximum_range_m, scan_config_.vertical_min_angle_rad,
      scan_config_.vertical_max_angle_rad, bounds_.width_cells, bounds_.height_cells,
      bounds_.depth_cells, bounds_.resolution_m, bounds_.origin_x, bounds_.origin_y,
      bounds_.origin_z, persistent_memory_enabled_ ? "true" : "false");
}

} // namespace drone_city_nav

int main(int argc, char* argv[]) noexcept {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<drone_city_nav::ObstacleMemory3DNode>();
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2U};
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "obstacle_memory_3d_node failed: %s\n", error.what());
  } catch (...) {
    std::fputs("obstacle_memory_3d_node failed: unknown exception\n", stderr);
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 1;
}
