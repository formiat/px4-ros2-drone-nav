#include "production_mppi_node.hpp"

#include "drone_city_nav/known_passage_solid_volumes.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] mppi::KnownSolid toMppiSolid(const KnownPassageSolidVolume& solid) {
  constexpr float kVehicleFootprintMarginM{0.75F};
  return mppi::KnownSolid{
      .center_x_m = static_cast<float>(solid.center.x),
      .center_y_m = static_cast<float>(solid.center.y),
      .normal_x = static_cast<float>(solid.normal_xy.x),
      .normal_y = static_cast<float>(solid.normal_xy.y),
      .lateral_x = static_cast<float>(solid.lateral_xy.x),
      .lateral_y = static_cast<float>(solid.lateral_xy.y),
      .half_depth_m =
          static_cast<float>(0.5 * solid.depth_m) + kVehicleFootprintMarginM,
      .half_width_m =
          static_cast<float>(0.5 * solid.width_m) + kVehicleFootprintMarginM,
      .min_z_m = static_cast<float>(solid.min_z_m) - kVehicleFootprintMarginM,
      .max_z_m = static_cast<float>(solid.max_z_m) + kVehicleFootprintMarginM,
  };
}

} // namespace

ProductionMppiNode::ProductionMppiNode()
    : Node{"production_mppi_node"} {
  tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 50.0);
  rviz_rate_hz_ = declare_parameter<double>("rviz_rate_hz", 10.0);
  deadline_ms_ = declare_parameter<double>("deadline_ms", 20.0);
  maximum_pose_age_ms_ = declare_parameter<double>("maximum_pose_age_ms", 150.0);
  maximum_esdf_age_ms_ = declare_parameter<double>("maximum_esdf_age_ms", 1000.0);
  guide_lookahead_m_ = declare_parameter<double>("guide_lookahead_m", 30.0);
  passage_activation_distance_m_ =
      declare_parameter<double>("passage_activation_distance_m", 45.0);
  target_mode_ = declare_parameter<std::string>("target_mode", "active_route_guide");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  diagnostics_output_dir_ =
      declare_parameter<std::string>("diagnostics_output_dir", "log/mppi");
  px4_local_origin_.x = declare_parameter<double>("px4_local_origin_x_m", 54.0);
  px4_local_origin_.y = declare_parameter<double>("px4_local_origin_y_m", 54.0);
  mission_goal_.x = declare_parameter<double>("goal_x_m", 216.0);
  mission_goal_.y = declare_parameter<double>("goal_y_m", 378.0);
  mission_goal_.z = declare_parameter<double>("goal_z_m", 18.0);
  mppi_config_.rollouts =
      static_cast<std::size_t>(declare_parameter<int>("rollouts", 8192));
  mppi_config_.steps = static_cast<std::size_t>(declare_parameter<int>("steps", 80));
  mppi_config_.dynamics.dt_s =
      static_cast<float>(declare_parameter<double>("dt_s", 0.05));
  mppi_config_.dynamics.maximum_vertical_speed_mps =
      static_cast<float>(declare_parameter<double>("maximum_vertical_speed_mps", 5.0));
  mppi_config_.risk.collision_radius_m =
      static_cast<float>(declare_parameter<double>("raw_collision_radius_m", 0.5));
  mppi_config_.risk.critical_distance_m =
      static_cast<float>(declare_parameter<double>("critical_distance_m", 1.0));
  mppi_config_.risk.preferred_distance_m =
      static_cast<float>(declare_parameter<double>("preferred_distance_m", 6.0));
  mppi_config_.seed = static_cast<std::uint64_t>(declare_parameter<int>("seed", 42));
  lattice_config_.heading_bins = static_cast<int>(
      declare_parameter<std::int64_t>("global_lattice_heading_bins", 16));
  lattice_config_.primitive_length_m =
      declare_parameter<double>("global_lattice_primitive_length_m", 4.0);
  lattice_config_.receding_goal_distance_m =
      declare_parameter<double>("global_lattice_receding_goal_distance_m", 60.0);
  lattice_config_.maximum_expansions = static_cast<std::size_t>(
      declare_parameter<int>("global_lattice_maximum_expansions", 60000));
  lattice_config_.collision_radius_m = mppi_config_.risk.collision_radius_m;
  lattice_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  lattice_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  mppi_config_.early_exit_on_collision = true;
  safety_config_.collision_radius_m = mppi_config_.risk.collision_radius_m;
  safety_config_.reaction_latency_s =
      declare_parameter<double>("safety_reaction_latency_s", 0.10);
  safety_config_.maximum_braking_acceleration_mps2 =
      declare_parameter<double>("safety_maximum_braking_acceleration_mps2", 8.0);
  safety_config_.minimum_time_to_collision_s =
      declare_parameter<double>("safety_minimum_time_to_collision_s", 0.50);
  safety_config_.fallback_duration_s =
      declare_parameter<double>("safety_fallback_duration_s", 2.0);
  safety_config_.dt_s = mppi_config_.dynamics.dt_s;
  rviz_period_ns_ = static_cast<std::int64_t>(1.0e9 / std::max(0.1, rviz_rate_hz_));

  if (!(tick_rate_hz_ > 0.0) || !(deadline_ms_ > 0.0)) {
    throw std::invalid_argument{"invalid production MPPI timing configuration"};
  }

  engine_ = std::make_unique<mppi::MppiCudaEngine>(mppi_config_);
  const bool passages_enabled = declare_parameter<bool>("known_passages_enabled", true);
  const std::string passages_path = declare_parameter<std::string>(
      "known_passages_path", "worlds/known_passages.passages3d");
  const auto package_share = std::filesystem::path{
      ament_index_cpp::get_package_share_directory("drone_city_nav")};
  const KnownPassageSourceResult passage_source =
      loadKnownPassageMapSource(KnownPassageSourceConfig{
          passages_enabled, passages_path, package_share, frame_id_});
  if (passage_source.map.has_value()) {
    known_passages_ = passage_source.map;
    for (const KnownPassageSolidVolume& solid :
         knownPassageSolidVolumes(*known_passages_)) {
      known_solids_.push_back(toMppiSolid(solid));
    }
    engine_->updateKnownSolids(known_solids_);
  }

  std::filesystem::create_directories(diagnostics_output_dir_);
  diagnostics_stream_.open(diagnostics_output_dir_ / "mppi_ticks.jsonl", std::ios::app);
  const auto sensor_qos = rclcpp::SensorDataQoS{};
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      declare_parameter<std::string>("px4_local_position_topic",
                                     "/fmu/out/vehicle_local_position_v1"),
      sensor_qos, [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
        onLocalPosition(*message);
      });
  raw_snapshot_sub_ = create_subscription<msg::RawObstacleSnapshot>(
      declare_parameter<std::string>("raw_obstacle_snapshot_topic",
                                     "/drone_city_nav/raw_obstacle_snapshot"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](msg::RawObstacleSnapshot::ConstSharedPtr message) {
        onRawObstacleSnapshot(std::move(message));
      });
  memory_snapshot_sub_ = create_subscription<msg::ObstacleMemorySnapshot>(
      declare_parameter<std::string>("obstacle_memory_snapshot_topic",
                                     "/drone_city_nav/obstacle_memory_snapshot"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::ObstacleMemorySnapshot::SharedPtr message) {
        onMemorySnapshot(*message);
      });
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic", "/drone_city_nav/mppi/path"),
      rclcpp::QoS{1}.reliable());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      declare_parameter<std::string>("markers_topic", "/drone_city_nav/mppi/markers"),
      rclcpp::QoS{1}.best_effort());
  status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/drone_city_nav/mppi/status"),
      rclcpp::QoS{10}.best_effort());
  execution_horizon_pub_ = create_publisher<msg::MppiTrajectoryHorizon>(
      declare_parameter<std::string>("execution_horizon_topic",
                                     "/drone_city_nav/mppi/execution_horizon"),
      rclcpp::QoS{2}.reliable());
  esdf_worker_ =
      std::jthread([this](const std::stop_token token) { esdfWorker(token); });
  planning_timer_ = create_wall_timer(
      std::chrono::duration<double>{1.0 / tick_rate_hz_}, [this]() { planningTick(); });
  RCLCPP_INFO(get_logger(),
              "Production MPPI ready: rollouts=%zu steps=%zu rate=%.1fHz "
              "deadline=%.1fms known_solids=%zu",
              mppi_config_.rollouts, mppi_config_.steps, tick_rate_hz_, deadline_ms_,
              known_solids_.size());
}

ProductionMppiNode::~ProductionMppiNode() {
  if (esdf_worker_.joinable()) {
    esdf_worker_.request_stop();
    raw_queue_condition_.notify_all();
    esdf_worker_.join();
  }
  publishSummary();
}

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ProductionMppiNode>());
  rclcpp::shutdown();
  return 0;
}
