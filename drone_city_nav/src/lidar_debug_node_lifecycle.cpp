#include "lidar_debug_node.hpp"

namespace drone_city_nav {

LidarDebugNode::LidarDebugNode()
    : Node{"lidar_debug_node"} {
  LidarDebugNodeConfig config;
  config.output_dir = declare_parameter<std::string>("output_dir", config.output_dir);
  config.snapshot_period_s =
      declare_parameter<double>("snapshot_period_s", config.snapshot_period_s);
  image_size_px_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<std::int64_t>("image_size_px", 900), 200, 4000));
  config.view_radius_m =
      declare_parameter<double>("view_radius_m", config.view_radius_m);
  config.max_lidar_range_m =
      declare_parameter<double>("max_lidar_range_m", config.max_lidar_range_m);
  config.range_hit_epsilon_m =
      declare_parameter<double>("range_hit_epsilon_m", config.range_hit_epsilon_m);
  initial_heading_rad_ = declare_parameter<double>("initial_heading_rad", 0.0);
  current_pose_.yaw_rad = initial_heading_rad_;
  px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 0.0),
                             declare_parameter<double>("px4_local_origin_y_m", 0.0)};
  scan_yaw_offset_rad_ = declare_parameter<double>("scan_yaw_offset_rad", 0.0);
  motion_compensate_lidar_pose_ =
      declare_parameter<bool>("motion_compensate_lidar_pose", true);
  lidar_pose_latency_s_ =
      std::clamp(declare_parameter<double>("lidar_pose_latency_s", 0.05), 0.0, 1.0);
  lidar_scan_deskew_ = declare_parameter<bool>("lidar_scan_deskew", false);
  lidar_scan_duration_override_s_ = std::clamp(
      declare_parameter<double>("lidar_scan_duration_override_s", 0.0), 0.0, 1.0);
  compensate_lidar_attitude_ =
      declare_parameter<bool>("compensate_lidar_attitude", false);
  lidar_z_offset_m_ = declare_parameter<double>("lidar_z_offset_m", 0.0);
  min_projected_lidar_altitude_m_ =
      declare_parameter<double>("min_projected_lidar_altitude_m", 0.0);
  max_projected_lidar_altitude_m_ =
      declare_parameter<double>("max_projected_lidar_altitude_m", 100000.0);
  use_px4_heading_for_scan_ =
      declare_parameter<bool>("use_px4_heading_for_scan", false);
  lidar_mount_roll_rad_ = declare_parameter<double>("lidar_mount_roll_rad", 0.0);
  lidar_mount_pitch_rad_ = declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
  lidar_mount_yaw_rad_ = declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
  config.beam_csv_stride = static_cast<std::size_t>(
      std::max<std::int64_t>(declare_parameter<std::int64_t>("beam_csv_stride", 1), 0));
  config.max_logged_hit_points = static_cast<std::size_t>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("max_logged_hit_points", 256), 0));
  config.max_snapshots = static_cast<std::uint64_t>(
      std::clamp<std::int64_t>(declare_parameter<std::int64_t>("max_snapshots", 0), 0,
                               std::numeric_limits<std::int32_t>::max()));

  const std::string lidar_topic =
      declare_parameter<std::string>("lidar_topic", "/scan");
  const std::string prohibited_grid_topic = declare_parameter<std::string>(
      "prohibited_grid_topic", "/drone_city_nav/prohibited_grid");
  const std::string memory_grid_topic = declare_parameter<std::string>(
      "memory_grid_topic", "/drone_city_nav/obstacle_memory_grid");
  const std::string path_topic = declare_parameter<std::string>(
      "path_topic", "/drone_city_nav/final_trajectory_path");
  pointcloud_topic_ = declare_parameter<std::string>(
      "pointcloud_topic", "/drone_city_nav/lidar_debug_points");
  remembered_pointcloud_topic_ = declare_parameter<std::string>(
      "remembered_pointcloud_topic", "/drone_city_nav/remembered_lidar_points");
  prohibited_pointcloud_topic_ = declare_parameter<std::string>(
      "prohibited_pointcloud_topic", "/drone_city_nav/prohibited_obstacle_points");
  raw_memory_pointcloud_topic_ = declare_parameter<std::string>(
      "raw_memory_pointcloud_topic", "/drone_city_nav/raw_memory_obstacle_points");
  marker_topic_ = declare_parameter<std::string>("marker_topic",
                                                 "/drone_city_nav/lidar_radar_markers");
  publish_lidar_radar_markers_ =
      declare_parameter<bool>("publish_lidar_radar_markers", false);
  config.hit_memory_resolution_m = declare_parameter<double>(
      "hit_memory_resolution_m", config.hit_memory_resolution_m);
  min_remember_altitude_m_ =
      std::max(0.0, declare_parameter<double>("min_remember_altitude_m", 0.0));
  config.max_remembered_hit_points = static_cast<std::size_t>(std::max<std::int64_t>(
      declare_parameter<std::int64_t>("max_remembered_hit_points", 50000), 0));
  sanitizeLidarDebugNodeConfig(config);
  output_dir_ = config.output_dir;
  snapshot_period_s_ = config.snapshot_period_s;
  view_radius_m_ = config.view_radius_m;
  max_lidar_range_m_ = config.max_lidar_range_m;
  range_hit_epsilon_m_ = config.range_hit_epsilon_m;
  beam_csv_stride_ = config.beam_csv_stride;
  max_logged_hit_points_ = config.max_logged_hit_points;
  max_snapshots_ = config.max_snapshots;
  hit_memory_resolution_m_ = config.hit_memory_resolution_m;
  max_remembered_hit_points_ = config.max_remembered_hit_points;
  current_pointcloud_z_m_ =
      declare_parameter<double>("current_lidar_pointcloud_z_m", kGroundDebugZ);
  remembered_pointcloud_z_m_ =
      declare_parameter<double>("remembered_lidar_pointcloud_z_m", kGroundDebugZ);
  prohibited_pointcloud_z_m_ =
      declare_parameter<double>("prohibited_pointcloud_z_m", kGroundDebugZ);
  raw_memory_pointcloud_z_m_ =
      declare_parameter<double>("raw_memory_pointcloud_z_m", kGroundDebugZ);
  marker_z_m_ = declare_parameter<double>("lidar_radar_marker_z_m", kGroundDebugZ);
  const std::string local_position_topic = declare_parameter<std::string>(
      "px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");
  const std::string attitude_topic = declare_parameter<std::string>(
      "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");

  std::filesystem::create_directories(output_dir_);
  summary_path_ = std::filesystem::path{output_dir_} / "snapshots.jsonl";
  summary_stream_.open(summary_path_, std::ios::out | std::ios::trunc);
  if (!summary_stream_.is_open()) {
    throw std::runtime_error{"Failed to open lidar debug summary file"};
  }

  const auto sensor_qos = rclcpp::SensorDataQoS{};
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic, sensor_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      local_position_topic, sensor_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        onLocalPosition(*msg);
      });
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      attitude_topic, sensor_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        onAttitude(*msg);
      });
  prohibited_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prohibited_grid_topic, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_grid_ = *msg;
        grid_seen_ = true;
        publishProhibitedPointCloud();
      });
  memory_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      memory_grid_topic, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        publishPointCloud(collectOccupiedGridPoints(*msg), raw_memory_pointcloud_z_m_,
                          raw_memory_pointcloud_pub_);
      });
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS{1}.reliable(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        last_path_ = *msg;
        path_seen_ = true;
      });
  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::QoS{1}.reliable());
  remembered_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      remembered_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
  prohibited_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      prohibited_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
  raw_memory_pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      raw_memory_pointcloud_topic_, rclcpp::QoS{1}.reliable().transient_local());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS{1}.reliable());

  timer_ = create_wall_timer(std::chrono::duration<double>{snapshot_period_s_},
                             [this]() { writeSnapshot(); });

  RCLCPP_INFO(
      get_logger(),
      "Lidar debug ready: output_dir='%s' period=%.2fs image=%dpx "
      "fallback_view_radius=%.1fm topics scan='%s' prohibited_grid='%s' "
      "memory_grid='%s' path='%s' "
      "pose='%s' attitude='%s' current_hits='%s' remembered_hits='%s' "
      "prohibited_points='%s' raw_memory_points='%s' "
      "markers='%s' lidar_radar_markers=%s hit_memory_resolution=%.2fm "
      "min_remember_altitude=%.2fm "
      "max_remembered_hits=%zu "
      "compensate_attitude=%s lidar_z_offset=%.2f "
      "projected_altitude_range=[%.2f, %.2f] "
      "lidar_mount_rpy=(%.3f, %.3f, %.3f) "
      "motion_compensation=%s pose_latency=%.3fs scan_deskew=%s "
      "scan_duration_override=%.3fs "
      "pointcloud_z[current=%.2f, remembered=%.2f, prohibited=%.2f, "
      "raw_memory=%.2f] "
      "marker_z=%.2f "
      "yaw_source=%s initial_heading=%.3f",
      output_dir_.c_str(), snapshot_period_s_, image_size_px_, view_radius_m_,
      lidar_topic.c_str(), prohibited_grid_topic.c_str(), memory_grid_topic.c_str(),
      path_topic.c_str(), local_position_topic.c_str(), attitude_topic.c_str(),
      pointcloud_topic_.c_str(), remembered_pointcloud_topic_.c_str(),
      prohibited_pointcloud_topic_.c_str(), raw_memory_pointcloud_topic_.c_str(),
      marker_topic_.c_str(), publish_lidar_radar_markers_ ? "true" : "false",
      hit_memory_resolution_m_, min_remember_altitude_m_, max_remembered_hit_points_,
      compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
      min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
      lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_,
      motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
      lidar_scan_deskew_ ? "true" : "false", lidar_scan_duration_override_s_,
      current_pointcloud_z_m_, remembered_pointcloud_z_m_, prohibited_pointcloud_z_m_,
      raw_memory_pointcloud_z_m_, marker_z_m_, yawSourceName(), initial_heading_rad_);
}

} // namespace drone_city_nav
