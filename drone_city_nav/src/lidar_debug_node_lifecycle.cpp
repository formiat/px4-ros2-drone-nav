#include "lidar_debug_node.hpp"

namespace drone_city_nav {

void LidarDebugNode::applyConfig(const LidarDebugNodeConfig& config) {
  output_dir_ = config.output_dir;
  snapshot_period_s_ = config.snapshot_period_s;
  image_size_px_ = config.image_size_px;
  view_radius_m_ = config.view_radius_m;
  max_lidar_range_m_ = config.max_lidar_range_m;
  range_hit_epsilon_m_ = config.range_hit_epsilon_m;
  initial_heading_rad_ = config.initial_heading_rad;
  current_pose_.yaw_rad = initial_heading_rad_;
  px4_local_origin_ = config.px4_local_origin;
  scan_yaw_offset_rad_ = config.scan_yaw_offset_rad;
  motion_compensate_lidar_pose_ = config.motion_compensate_lidar_pose;
  lidar_pose_latency_s_ = config.lidar_pose_latency_s;
  lidar_acquisition_pose_config_.apply_sensor_time_offset =
      config.motion_compensate_lidar_pose;
  lidar_acquisition_pose_config_.sensor_time_offset_s = config.lidar_pose_latency_s;
  lidar_acquisition_pose_config_.require_source_timestamp_alignment = true;
  lidar_acquisition_pose_config_.require_bracketed_pose = true;
  lidar_scan_alignment_maximum_wait_ns_ =
      static_cast<std::int64_t>(config.lidar_scan_alignment_maximum_wait_s * 1.0e9);
  lidar_scan_alignment_queue_capacity_ = config.lidar_scan_alignment_queue_capacity;
  lidar_scan_duration_override_s_ = config.lidar_scan_duration_override_s;
  compensate_lidar_attitude_ = config.compensate_lidar_attitude;
  lidar_z_offset_m_ = config.lidar_z_offset_m;
  min_projected_lidar_altitude_m_ = config.min_projected_lidar_altitude_m;
  max_projected_lidar_altitude_m_ = config.max_projected_lidar_altitude_m;
  use_px4_heading_for_scan_ = config.use_px4_heading_for_scan;
  maximum_heading_variance_rad2_ = config.maximum_heading_variance_rad2;
  mapping_yaw_tracker_ =
      MappingYawTracker{use_px4_heading_for_scan_, initial_heading_rad_,
                        config.startup_heading_stable_sample_count,
                        config.startup_heading_maximum_sample_delta_rad};
  lidar_mount_roll_rad_ = config.lidar_mount_roll_rad;
  lidar_mount_pitch_rad_ = config.lidar_mount_pitch_rad;
  lidar_mount_yaw_rad_ = config.lidar_mount_yaw_rad;
  use_full_lidar_extrinsic_ = config.use_full_lidar_extrinsic;
  lidar_translation_body_frd_m_ = config.lidar_translation_body_frd_m;
  lidar_flu_to_body_frd_quaternion_ = config.lidar_flu_to_body_frd_quaternion;
  beam_csv_stride_ = config.beam_csv_stride;
  max_logged_hit_points_ = config.max_logged_hit_points;
  max_snapshots_ = config.max_snapshots;
  pointcloud_topic_ = config.topics.pointcloud;
  raw_lidar_3d_pointcloud_topic_ = config.topics.raw_lidar_3d_pointcloud;
  remembered_pointcloud_topic_ = config.topics.remembered_pointcloud;
  occupied_pointcloud_topic_ = config.topics.occupied_pointcloud;
  raw_memory_pointcloud_topic_ = config.topics.raw_memory_pointcloud;
  hit_memory_resolution_m_ = config.hit_memory_resolution_m;
  min_remember_altitude_m_ = config.min_remember_altitude_m;
  max_remembered_hit_points_ = config.max_remembered_hit_points;
  current_pointcloud_z_m_ = config.current_pointcloud_z_m;
  remembered_pointcloud_z_m_ = config.remembered_pointcloud_z_m;
  occupied_pointcloud_z_m_ = config.occupied_pointcloud_z_m;
  raw_memory_pointcloud_z_m_ = config.raw_memory_pointcloud_z_m;
}

LidarDebugNode::LidarDebugNode(const rclcpp::NodeOptions& options)
    : Node{"lidar_debug_node", options} {
  const LidarDebugNodeConfig config = loadLidarDebugNodeConfig(*this);
  applyConfig(config);
  const LidarDebugNodeTopics& topics = config.topics;
  spectator_vehicle_id_ = declare_parameter<std::string>("spectator_vehicle_id", "");
  const std::string spectator_target_topic = declare_parameter<std::string>(
      "spectator_target_topic", "/drone_city_nav/spectator_target");
  selected_for_spectator_ = spectator_vehicle_id_.empty();

  std::filesystem::create_directories(output_dir_);
  summary_path_ = std::filesystem::path{output_dir_} / "snapshots.jsonl";
  summary_stream_.open(summary_path_, std::ios::out | std::ios::trunc);
  if (!summary_stream_.is_open()) {
    throw std::runtime_error{"Failed to open lidar debug summary file"};
  }

  const auto sensor_qos = rclcpp::SensorDataQoS{};
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      topics.lidar, sensor_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topics.px4_local_position, sensor_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        onLocalPosition(*msg);
      });
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      topics.px4_vehicle_attitude, sensor_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        onAttitude(*msg);
      });
  timesync_status_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
      topics.px4_timesync_status, sensor_qos,
      [this](const px4_msgs::msg::TimesyncStatus::SharedPtr msg) {
        onTimesyncStatus(*msg);
      });
  raw_obstacle_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      topics.raw_obstacle_grid, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_grid_ = *msg;
        grid_seen_ = true;
        if (diagnosticsSelected()) {
          publishOccupiedPointCloud();
        }
      });
  memory_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      topics.memory_grid, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_memory_grid_ = *msg;
        memory_grid_seen_ = true;
        if (diagnosticsSelected()) {
          publishPointCloud(collectOccupiedGridPoints(*msg), raw_memory_pointcloud_z_m_,
                            raw_memory_pointcloud_pub_);
        }
      });
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      topics.path, rclcpp::QoS{1}.reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        last_path_ = *msg;
        path_seen_ = true;
      });
  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::QoS{1}.reliable());
  raw_lidar_3d_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      raw_lidar_3d_pointcloud_topic_, rclcpp::QoS{1}.reliable());
  remembered_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      remembered_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
  occupied_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      occupied_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
  raw_memory_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      raw_memory_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());

  if (!spectator_vehicle_id_.empty()) {
    spectator_target_sub_ = create_subscription<msg::SpectatorTarget>(
        spectator_target_topic, rclcpp::QoS{1}.reliable().transient_local(),
        [this](const msg::SpectatorTarget::SharedPtr message) {
          onSpectatorTarget(*message);
        });
  }

  timer_ = create_wall_timer(std::chrono::duration<double>{snapshot_period_s_},
                             [this]() { writeSnapshot(); });

  RCLCPP_INFO(
      get_logger(),
      "Lidar debug ready: output_dir='%s' period=%.2fs image=%dpx "
      "fallback_view_radius=%.1fm topics scan='%s' raw_obstacle_grid='%s' "
      "memory_grid='%s' path='%s' "
      "pose='%s' attitude='%s' timesync='%s' current_hits='%s' "
      "raw_current_hits_3d='%s' "
      "remembered_hits='%s' "
      "occupied_points='%s' raw_memory_points='%s' "
      "hit_memory_resolution=%.2fm "
      "min_remember_altitude=%.2fm "
      "max_remembered_hits=%zu "
      "compensate_attitude=%s lidar_z_offset=%.2f "
      "projected_altitude_range=[%.2f, %.2f] "
      "lidar_mount_rpy=(%.3f, %.3f, %.3f) "
      "motion_compensation=%s pose_latency=%.3fs "
      "scan_duration_override=%.3fs "
      "pointcloud_z[current=%.2f, remembered=%.2f, occupied=%.2f, "
      "raw_memory=%.2f] "
      "yaw_source=%s initial_heading=%.3f max_heading_variance=%.6frad2 "
      "startup_stable_samples=%zu startup_maximum_delta=%.3frad "
      "spectator_vehicle_id='%s' selected=%s",
      output_dir_.c_str(), snapshot_period_s_, image_size_px_, view_radius_m_,
      topics.lidar.c_str(), topics.raw_obstacle_grid.c_str(),
      topics.memory_grid.c_str(), topics.path.c_str(),
      topics.px4_local_position.c_str(), topics.px4_vehicle_attitude.c_str(),
      topics.px4_timesync_status.c_str(), pointcloud_topic_.c_str(),
      raw_lidar_3d_pointcloud_topic_.c_str(), remembered_pointcloud_topic_.c_str(),
      occupied_pointcloud_topic_.c_str(), raw_memory_pointcloud_topic_.c_str(),
      hit_memory_resolution_m_, min_remember_altitude_m_, max_remembered_hit_points_,
      compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
      min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
      lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_,
      motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
      lidar_scan_duration_override_s_, current_pointcloud_z_m_,
      remembered_pointcloud_z_m_, occupied_pointcloud_z_m_, raw_memory_pointcloud_z_m_,
      yawSourceName(), initial_heading_rad_, maximum_heading_variance_rad2_,
      config.startup_heading_stable_sample_count,
      config.startup_heading_maximum_sample_delta_rad, spectator_vehicle_id_.c_str(),
      diagnosticsSelected() ? "true" : "false");
}

bool LidarDebugNode::diagnosticsSelected() const noexcept {
  return spectator_vehicle_id_.empty() || selected_for_spectator_;
}

void LidarDebugNode::onSpectatorTarget(const msg::SpectatorTarget& msg) {
  const bool selected = msg.vehicle_id == spectator_vehicle_id_;
  if (selected == selected_for_spectator_) {
    return;
  }

  selected_for_spectator_ = selected;
  if (!selected) {
    pending_lidar_scans_.clear();
    clearPublishedPointClouds();
  } else {
    if (grid_seen_) {
      publishOccupiedPointCloud();
    }
    if (memory_grid_seen_) {
      publishPointCloud(collectOccupiedGridPoints(last_memory_grid_),
                        raw_memory_pointcloud_z_m_, raw_memory_pointcloud_pub_);
    }
  }
  RCLCPP_INFO(get_logger(),
              "LIDAR_DEBUG_SPECTATOR selected=%s vehicle_id='%s' "
              "spectator_vehicle_id='%s'",
              selected ? "true" : "false", msg.vehicle_id.c_str(),
              spectator_vehicle_id_.c_str());
}

void LidarDebugNode::clearPublishedPointClouds() {
  publishRawLidarPointCloud({});
  publishPointCloud({}, current_pointcloud_z_m_, pointcloud_pub_);
  publishPointCloud({}, remembered_pointcloud_z_m_, remembered_pointcloud_pub_);
  publishPointCloud({}, occupied_pointcloud_z_m_, occupied_pointcloud_pub_);
  publishPointCloud({}, raw_memory_pointcloud_z_m_, raw_memory_pointcloud_pub_);
}

} // namespace drone_city_nav
