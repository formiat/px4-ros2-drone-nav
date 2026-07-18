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
  lidar_scan_duration_override_s_ = config.lidar_scan_duration_override_s;
  compensate_lidar_attitude_ = config.compensate_lidar_attitude;
  lidar_z_offset_m_ = config.lidar_z_offset_m;
  min_projected_lidar_altitude_m_ = config.min_projected_lidar_altitude_m;
  max_projected_lidar_altitude_m_ = config.max_projected_lidar_altitude_m;
  use_px4_heading_for_scan_ = config.use_px4_heading_for_scan;
  lidar_mount_roll_rad_ = config.lidar_mount_roll_rad;
  lidar_mount_pitch_rad_ = config.lidar_mount_pitch_rad;
  lidar_mount_yaw_rad_ = config.lidar_mount_yaw_rad;
  beam_csv_stride_ = config.beam_csv_stride;
  max_logged_hit_points_ = config.max_logged_hit_points;
  max_snapshots_ = config.max_snapshots;
  pointcloud_topic_ = config.topics.pointcloud;
  raw_lidar_3d_pointcloud_topic_ = config.topics.raw_lidar_3d_pointcloud;
  remembered_pointcloud_topic_ = config.topics.remembered_pointcloud;
  prohibited_pointcloud_topic_ = config.topics.prohibited_pointcloud;
  raw_memory_pointcloud_topic_ = config.topics.raw_memory_pointcloud;
  hit_memory_resolution_m_ = config.hit_memory_resolution_m;
  min_remember_altitude_m_ = config.min_remember_altitude_m;
  max_remembered_hit_points_ = config.max_remembered_hit_points;
  current_pointcloud_z_m_ = config.current_pointcloud_z_m;
  remembered_pointcloud_z_m_ = config.remembered_pointcloud_z_m;
  prohibited_pointcloud_z_m_ = config.prohibited_pointcloud_z_m;
  raw_memory_pointcloud_z_m_ = config.raw_memory_pointcloud_z_m;
}

LidarDebugNode::LidarDebugNode()
    : Node{"lidar_debug_node"} {
  const LidarDebugNodeConfig config = loadLidarDebugNodeConfig(*this);
  applyConfig(config);
  const LidarDebugNodeTopics& topics = config.topics;

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
  prohibited_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      topics.prohibited_grid, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_grid_ = *msg;
        grid_seen_ = true;
        publishProhibitedPointCloud();
      });
  memory_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      topics.memory_grid, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        publishPointCloud(collectOccupiedGridPoints(*msg), raw_memory_pointcloud_z_m_,
                          raw_memory_pointcloud_pub_);
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
  prohibited_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      prohibited_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
  raw_memory_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      raw_memory_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());

  timer_ = create_wall_timer(std::chrono::duration<double>{snapshot_period_s_},
                             [this]() { writeSnapshot(); });

  RCLCPP_INFO(
      get_logger(),
      "Lidar debug ready: output_dir='%s' period=%.2fs image=%dpx "
      "fallback_view_radius=%.1fm topics scan='%s' prohibited_grid='%s' "
      "memory_grid='%s' path='%s' "
      "pose='%s' attitude='%s' current_hits='%s' raw_current_hits_3d='%s' "
      "remembered_hits='%s' "
      "prohibited_points='%s' raw_memory_points='%s' "
      "hit_memory_resolution=%.2fm "
      "min_remember_altitude=%.2fm "
      "max_remembered_hits=%zu "
      "compensate_attitude=%s lidar_z_offset=%.2f "
      "projected_altitude_range=[%.2f, %.2f] "
      "lidar_mount_rpy=(%.3f, %.3f, %.3f) "
      "motion_compensation=%s pose_latency=%.3fs "
      "scan_duration_override=%.3fs "
      "pointcloud_z[current=%.2f, remembered=%.2f, prohibited=%.2f, "
      "raw_memory=%.2f] "
      "yaw_source=%s initial_heading=%.3f",
      output_dir_.c_str(), snapshot_period_s_, image_size_px_, view_radius_m_,
      topics.lidar.c_str(), topics.prohibited_grid.c_str(), topics.memory_grid.c_str(),
      topics.path.c_str(), topics.px4_local_position.c_str(),
      topics.px4_vehicle_attitude.c_str(), pointcloud_topic_.c_str(),
      raw_lidar_3d_pointcloud_topic_.c_str(), remembered_pointcloud_topic_.c_str(),
      prohibited_pointcloud_topic_.c_str(), raw_memory_pointcloud_topic_.c_str(),
      hit_memory_resolution_m_, min_remember_altitude_m_, max_remembered_hit_points_,
      compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
      min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
      lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_,
      motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
      lidar_scan_duration_override_s_, current_pointcloud_z_m_,
      remembered_pointcloud_z_m_, prohibited_pointcloud_z_m_,
      raw_memory_pointcloud_z_m_, yawSourceName(), initial_heading_rad_);
}

} // namespace drone_city_nav
