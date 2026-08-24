#include <chrono>
#include <filesystem>
#include <string>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::initializeRuntimeInterfaces() {
  std::filesystem::create_directories(diagnostics_output_dir_);
  diagnostics_stream_.open(diagnostics_output_dir_ / "mppi_ticks.jsonl",
                           std::ios::trunc);
  diagnostics_error_stream_.open(diagnostics_output_dir_ / "mppi_error_context.jsonl",
                                 std::ios::trunc);
  last_diagnostics_flush_time_ = std::chrono::steady_clock::now();

  input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  world_input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  planning_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input_subscription_options;
  input_subscription_options.callback_group = input_callback_group_;
  rclcpp::SubscriptionOptions world_subscription_options;
  world_subscription_options.callback_group = world_input_callback_group_;
  const auto sensor_qos = rclcpp::SensorDataQoS{};
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      declare_parameter<std::string>("px4_local_position_topic",
                                     "/fmu/out/vehicle_local_position_v1"),
      sensor_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
        onLocalPosition(*message);
      },
      input_subscription_options);
  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      declare_parameter<std::string>("px4_vehicle_status_topic",
                                     "/fmu/out/vehicle_status_v1"),
      sensor_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
        onVehicleStatus(*message);
      },
      input_subscription_options);
  vehicle_land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      declare_parameter<std::string>("px4_vehicle_land_detected_topic",
                                     "/fmu/out/vehicle_land_detected"),
      sensor_qos,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr message) {
        onVehicleLandDetected(*message);
      },
      input_subscription_options);
  navigation_readiness_sub_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("navigation_readiness_topic",
                                     "/drone_city_nav/navigation_ready"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        onNavigationReadiness(*message);
      },
      input_subscription_options);

  const std::string raw_snapshot_topic = declare_parameter<std::string>(
      "raw_obstacle_snapshot_topic", "/drone_city_nav/raw_obstacle_snapshot");
  const std::string raw_delta_topic = declare_parameter<std::string>(
      "raw_obstacle_delta_topic", "/drone_city_nav/raw_obstacle_delta");
  const std::string raw_snapshot_3d_topic = declare_parameter<std::string>(
      "raw_obstacle_snapshot_3d_topic", "/drone_city_nav/raw_obstacle_snapshot_3d");
  const std::string raw_delta_3d_topic = declare_parameter<std::string>(
      "raw_obstacle_delta_3d_topic", "/drone_city_nav/raw_obstacle_delta_3d");
  if (!use_static_map_ &&
      no_static_world_model_ == ProductionNoStaticWorldModel::kOccupancy2D) {
    raw_snapshot_sub_ = create_subscription<msg::RawObstacleSnapshot>(
        raw_snapshot_topic, rclcpp::QoS{1}.reliable().transient_local(),
        [this](msg::RawObstacleSnapshot::ConstSharedPtr message) {
          onRawObstacleSnapshot(std::move(message));
        },
        world_subscription_options);
    raw_delta_sub_ = create_subscription<msg::RawObstacleDelta>(
        raw_delta_topic, rclcpp::QoS{1}.reliable().transient_local(),
        [this](msg::RawObstacleDelta::ConstSharedPtr message) {
          onRawObstacleDelta(std::move(message));
        },
        world_subscription_options);
  } else if (!use_static_map_) {
    raw_snapshot_3d_sub_ = create_subscription<msg::RawObstacleSnapshot3D>(
        raw_snapshot_3d_topic, rclcpp::QoS{1}.reliable().transient_local(),
        [this](msg::RawObstacleSnapshot3D::ConstSharedPtr message) {
          onRawObstacleSnapshot3D(std::move(message));
        },
        world_subscription_options);
    raw_delta_3d_sub_ = create_subscription<msg::RawObstacleDelta3D>(
        raw_delta_3d_topic, rclcpp::QoS{1}.best_effort().transient_local(),
        [this](msg::RawObstacleDelta3D::ConstSharedPtr message) {
          onRawObstacleDelta3D(std::move(message));
        },
        world_subscription_options);
  }
  latest_lidar_obstacle_scan_sub_ = create_subscription<msg::LatestLidarObstacleScan>(
      declare_parameter<std::string>("latest_lidar_obstacle_scan_topic",
                                     "/drone_city_nav/latest_lidar_obstacle_scan"),
      rclcpp::SensorDataQoS{},
      [this](const msg::LatestLidarObstacleScan::SharedPtr message) {
        onLatestLidarObstacleScan(*message);
      },
      input_subscription_options);
  memory_status_sub_ = create_subscription<msg::ObstacleMemoryStatus>(
      declare_parameter<std::string>("obstacle_memory_status_topic",
                                     "/drone_city_nav/obstacle_memory_status"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::ObstacleMemoryStatus::SharedPtr message) {
        onMemoryStatus(*message);
      },
      input_subscription_options);
  applied_control_sub_ = create_subscription<msg::MppiControlFeedback>(
      declare_parameter<std::string>("applied_control_feedback_topic",
                                     "/drone_city_nav/mppi/applied_control"),
      rclcpp::QoS{10}.reliable(),
      [this](const msg::MppiControlFeedback::SharedPtr message) {
        onAppliedControl(*message);
      },
      input_subscription_options);
  navigation_objective_sub_ = create_subscription<msg::NavigationObjective>(
      declare_parameter<std::string>("navigation_objective_topic",
                                     "/drone_city_nav/navigation_objective"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::NavigationObjective::SharedPtr message) {
        onNavigationObjective(*message);
      },
      input_subscription_options);
  createCooperativeTrafficInterfaces(input_subscription_options);
  createNonCooperativeAvoidanceInterface(input_subscription_options);

  radar_track_mode_command_pub_ = create_publisher<msg::RadarTrackModeCommand>(
      declare_parameter<std::string>("radar_track_mode_command_topic",
                                     "/drone_city_nav/radar/track_mode_command"),
      rclcpp::QoS{1}.reliable().transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic", "/drone_city_nav/mppi/path"),
      rclcpp::QoS{1}.reliable());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      declare_parameter<std::string>("markers_topic", "/drone_city_nav/mppi/markers"),
      rclcpp::QoS{1}.reliable());
  status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/drone_city_nav/mppi/status"),
      rclcpp::QoS{10}.best_effort());
  world_readiness_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("world_readiness_topic",
                                     "/drone_city_nav/mppi/world_ready"),
      rclcpp::QoS{1}.reliable().transient_local());
  execution_horizon_pub_ = create_publisher<msg::MppiTrajectoryHorizon>(
      declare_parameter<std::string>("execution_horizon_topic",
                                     "/drone_city_nav/mppi/execution_horizon"),
      rclcpp::QoS{2}.reliable());
  mission_waypoint_acknowledgement_pub_ =
      create_publisher<msg::MissionWaypointAcknowledgement>(
          declare_parameter<std::string>(
              "mission_waypoint_acknowledgement_topic",
              "/drone_city_nav/mission_waypoint_acknowledgement"),
          rclcpp::QoS{32}.reliable().transient_local());
  publishWorldReadiness(false);

  diagnostics_worker_ =
      std::jthread([this](const std::stop_token token) { diagnosticsWorker(token); });
  esdf_worker_ =
      std::jthread([this](const std::stop_token token) { esdfWorker(token); });
  topology_worker_ =
      std::jthread([this](const std::stop_token token) { topologyWorker(token); });
  guide_worker_ =
      std::jthread([this](const std::stop_token token) { guideWorker(token); });
  if (planning_tick_phase_offset_s_ > 0.0) {
    planning_start_timer_ = create_wall_timer(
        std::chrono::duration<double>{planning_tick_phase_offset_s_},
        [this]() {
          planning_start_timer_->cancel();
          startPlanningTimer();
        },
        planning_callback_group_);
  } else {
    startPlanningTimer();
  }
}

} // namespace drone_city_nav
