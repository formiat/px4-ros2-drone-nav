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

const char*
productionMppiPlanningStateName(const ProductionMppiPlanningState state) noexcept {
  switch (state) {
    case ProductionMppiPlanningState::kPlanned:
      return "planned";
    case ProductionMppiPlanningState::kNoGuideBrakingHold:
      return "no_guide_braking_hold";
    case ProductionMppiPlanningState::kStaleWorldBrakingHold:
      return "stale_world_braking_hold";
    case ProductionMppiPlanningState::kMissionGoalPositionHold:
      return "mission_goal_position_hold";
  }
  return "unknown";
}

ProductionMppiNode::ProductionMppiNode()
    : Node{"production_mppi_node"} {
  tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 50.0);
  rviz_rate_hz_ = declare_parameter<double>("rviz_rate_hz", 10.0);
  diagnostics_info_rate_hz_ =
      declare_parameter<double>("diagnostics_info_rate_hz", 5.0);
  deadline_ms_ = declare_parameter<double>("deadline_ms", 20.0);
  maximum_pose_age_ms_ = declare_parameter<double>("maximum_pose_age_ms", 150.0);
  maximum_esdf_age_ms_ = declare_parameter<double>("maximum_esdf_age_ms", 1000.0);
  maximum_control_feedback_age_ms_ =
      declare_parameter<double>("maximum_control_feedback_age_ms", 200.0);
  semantic_route_config_.crossing_lateral_margin_m =
      declare_parameter<double>("portal_crossing_lateral_margin_m", 0.5);
  semantic_route_config_.minimum_normal_alignment =
      declare_parameter<double>("portal_minimum_route_normal_alignment", 0.35);
  passage_speed_policy_.use_static_map =
      declare_parameter<bool>("use_static_map", true);
  no_static_guide_lookahead_m_ =
      declare_parameter<double>("no_static_guide_lookahead_m", 30.0);
  passage_speed_policy_.static_limit_mps = static_cast<float>(
      declare_parameter<double>("static_passage_speed_limit_mps", 10.0));
  passage_speed_policy_.no_static_limit_mps = static_cast<float>(
      declare_parameter<double>("no_static_passage_speed_limit_mps", 5.0));
  target_mode_ = declare_parameter<std::string>("target_mode", "active_route_guide");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  diagnostics_output_dir_ =
      declare_parameter<std::string>("diagnostics_output_dir", "log/mppi");
  px4_local_origin_.x = declare_parameter<double>("px4_local_origin_x_m", 54.0);
  px4_local_origin_.y = declare_parameter<double>("px4_local_origin_y_m", 54.0);
  mission_start_.x = declare_parameter<double>("start_x_m", 54.0);
  mission_start_.y = declare_parameter<double>("start_y_m", 54.0);
  mission_start_.z = declare_parameter<double>("start_z_m", 0.0);
  mission_goal_.x = declare_parameter<double>("goal_x_m", 216.0);
  mission_goal_.y = declare_parameter<double>("goal_y_m", 378.0);
  mission_goal_.z = declare_parameter<double>("goal_z_m", 18.0);
  mission_goal_capture_config_.capture_radius_m =
      declare_parameter<double>("mission_goal_capture_radius_m", 2.0);
  mppi_config_.rollouts =
      static_cast<std::size_t>(declare_parameter<int>("rollouts", 8192));
  mppi_config_.dynamics.dt_s =
      static_cast<float>(declare_parameter<double>("dt_s", 0.05));
  const double static_horizon_duration_s =
      declare_parameter<double>("static_horizon_duration_s", 6.0);
  const double no_static_horizon_duration_s =
      declare_parameter<double>("no_static_horizon_duration_s", 4.0);
  const double active_horizon_duration_s = passage_speed_policy_.use_static_map
                                               ? static_horizon_duration_s
                                               : no_static_horizon_duration_s;
  mppi_config_.steps = static_cast<std::size_t>(
      std::ceil(active_horizon_duration_s / mppi_config_.dynamics.dt_s));
  MppiSpeedPolicyConfig static_speed_policy_config;
  static_speed_policy_config.horizon_duration_s = static_horizon_duration_s;
  static_speed_policy_config.cruise_speed_mps =
      declare_parameter<double>("static_cruise_speed_mps", 20.0);
  static_speed_policy_config.absolute_speed_limit_mps =
      declare_parameter<double>("static_absolute_speed_limit_mps", 20.0);
  static_speed_policy_config.maximum_lateral_acceleration_mps2 =
      declare_parameter<double>("static_maximum_lateral_acceleration_mps2", 5.0);
  static_speed_policy_config.maximum_braking_acceleration_mps2 =
      declare_parameter<double>("static_maximum_braking_acceleration_mps2", 8.0);
  static_speed_policy_config.reaction_latency_s =
      declare_parameter<double>("static_speed_reaction_latency_s", 0.10);
  static_speed_policy_config.observation_distance_m =
      declare_parameter<double>("static_observation_distance_m", 30.0);
  static_speed_policy_config.observation_margin_m =
      declare_parameter<double>("static_observation_margin_m", 3.0);
  static_speed_policy_config.goal_margin_m =
      declare_parameter<double>("static_goal_braking_margin_m", 2.0);
  static_speed_policy_config.curvature_preview_distance_m =
      declare_parameter<double>("static_curvature_preview_distance_m", 100.0);
  static_speed_policy_config.minimum_target_lookahead_m =
      declare_parameter<double>("static_minimum_target_lookahead_m", 30.0);
  static_speed_policy_config.maximum_target_lookahead_m =
      declare_parameter<double>("static_maximum_target_lookahead_m", 100.0);
  const double static_maximum_horizontal_acceleration_mps2 =
      declare_parameter<double>("static_maximum_horizontal_acceleration_mps2", 8.0);
  const double static_maximum_control_jerk_mps3 =
      declare_parameter<double>("static_maximum_control_jerk_mps3", 20.0);
  MppiSpeedPolicyConfig no_static_speed_policy_config = static_speed_policy_config;
  no_static_speed_policy_config.horizon_duration_s = no_static_horizon_duration_s;
  no_static_speed_policy_config.cruise_speed_mps =
      declare_parameter<double>("no_static_cruise_speed_mps", 10.0);
  no_static_speed_policy_config.absolute_speed_limit_mps =
      declare_parameter<double>("no_static_absolute_speed_limit_mps", 10.0);
  const double no_static_maximum_horizontal_acceleration_mps2 =
      declare_parameter<double>("no_static_maximum_horizontal_acceleration_mps2", 4.0);
  const double no_static_maximum_control_jerk_mps3 =
      declare_parameter<double>("no_static_maximum_control_jerk_mps3", 12.0);
  no_static_speed_policy_config.maximum_lateral_acceleration_mps2 =
      no_static_maximum_horizontal_acceleration_mps2;
  no_static_speed_policy_config.maximum_braking_acceleration_mps2 =
      no_static_maximum_horizontal_acceleration_mps2;
  no_static_speed_policy_config.curvature_preview_distance_m =
      declare_parameter<double>("no_static_curvature_preview_distance_m", 60.0);
  no_static_speed_policy_config.minimum_target_lookahead_m =
      no_static_guide_lookahead_m_;
  no_static_speed_policy_config.maximum_target_lookahead_m =
      no_static_guide_lookahead_m_;
  speed_policy_config_ = passage_speed_policy_.use_static_map
                             ? static_speed_policy_config
                             : no_static_speed_policy_config;
  mppi_config_.dynamics.maximum_horizontal_speed_mps =
      static_cast<float>(speed_policy_config_.absolute_speed_limit_mps);
  mppi_config_.dynamics.maximum_horizontal_acceleration_mps2 =
      static_cast<float>(passage_speed_policy_.use_static_map
                             ? static_maximum_horizontal_acceleration_mps2
                             : no_static_maximum_horizontal_acceleration_mps2);
  mppi_config_.dynamics.maximum_control_jerk_mps3 = static_cast<float>(
      passage_speed_policy_.use_static_map ? static_maximum_control_jerk_mps3
                                           : no_static_maximum_control_jerk_mps3);
  mppi_config_.dynamics.maximum_vertical_acceleration_mps2 = static_cast<float>(
      declare_parameter<double>("maximum_vertical_acceleration_mps2", 4.0));
  mppi_config_.dynamics.maximum_vertical_speed_mps =
      static_cast<float>(declare_parameter<double>("maximum_vertical_speed_mps", 5.0));
  passage_coordinator_config_.vertical_clearance_margin_m =
      declare_parameter<double>("passage_vertical_clearance_margin_m", 1.0);
  passage_coordinator_config_.vertical_capture_hysteresis_m =
      declare_parameter<double>("passage_vertical_capture_hysteresis_m", 0.25);
  passage_coordinator_config_.preferred_z_capture_tolerance_m =
      declare_parameter<double>("passage_preferred_z_capture_tolerance_m", 0.5);
  passage_coordinator_config_.maximum_capture_vertical_speed_mps =
      declare_parameter<double>("passage_vertical_capture_maximum_speed_mps", 0.5);
  const std::int64_t passage_capture_stable_cycles =
      declare_parameter<std::int64_t>("passage_capture_stable_cycles", 3);
  const std::int64_t passage_retention_violation_cycles =
      declare_parameter<std::int64_t>("passage_retention_violation_cycles", 3);
  if (passage_capture_stable_cycles <= 0 || passage_retention_violation_cycles <= 0) {
    throw std::invalid_argument{"passage stability cycles must be positive"};
  }
  passage_coordinator_config_.capture_stable_cycles =
      static_cast<std::size_t>(passage_capture_stable_cycles);
  passage_coordinator_config_.retention_violation_cycles =
      static_cast<std::size_t>(passage_retention_violation_cycles);
  passage_coordinator_config_.alignment_time_margin_s =
      declare_parameter<double>("passage_alignment_time_margin_s", 0.5);
  passage_coordinator_config_.minimum_stationary_trigger_distance_m =
      declare_parameter<double>("passage_stationary_trigger_minimum_distance_m", 2.0);
  passage_coordinator_config_.maximum_vertical_acceleration_mps2 =
      mppi_config_.dynamics.maximum_vertical_acceleration_mps2;
  passage_coordinator_config_.maximum_vertical_speed_mps =
      mppi_config_.dynamics.maximum_vertical_speed_mps;
  passage_coordinator_config_.maximum_horizontal_braking_acceleration_mps2 =
      speed_policy_config_.maximum_braking_acceleration_mps2;
  passage_coordinator_config_.reaction_latency_s =
      speed_policy_config_.reaction_latency_s;
  passage_coordinator_config_.exit_station_hysteresis_m =
      declare_parameter<double>("portal_exit_station_hysteresis_m", 0.5);
  mppi_config_.costs.head_progress_horizon_s =
      static_cast<float>(declare_parameter<double>("head_progress_horizon_s", 0.4));
  mppi_config_.costs.head_progress_weight =
      static_cast<float>(declare_parameter<double>("head_progress_weight", 8.0));
  const double static_speed_tracking_weight =
      declare_parameter<double>("static_speed_tracking_weight", 1.0);
  const double no_static_speed_tracking_weight =
      declare_parameter<double>("no_static_speed_tracking_weight", 1.0);
  mppi_config_.costs.speed_tracking_weight = static_cast<float>(
      passage_speed_policy_.use_static_map ? static_speed_tracking_weight
                                           : no_static_speed_tracking_weight);
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
  const double static_lattice_distance =
      declare_parameter<double>("static_global_lattice_window_m", 180.0);
  const double no_static_lattice_distance =
      declare_parameter<double>("no_static_global_lattice_window_m", 60.0);
  lattice_config_.receding_goal_distance_m = passage_speed_policy_.use_static_map
                                                 ? static_lattice_distance
                                                 : no_static_lattice_distance;
  const std::int64_t static_lattice_expansions = declare_parameter<std::int64_t>(
      "static_global_lattice_maximum_expansions", 120000);
  const std::int64_t no_static_lattice_expansions = declare_parameter<std::int64_t>(
      "no_static_global_lattice_maximum_expansions", 60000);
  lattice_config_.maximum_expansions = static_cast<std::size_t>(
      passage_speed_policy_.use_static_map ? static_lattice_expansions
                                           : no_static_lattice_expansions);
  lattice_config_.collision_radius_m = mppi_config_.risk.collision_radius_m;
  lattice_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  lattice_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  lattice_config_.portal_lateral_margin_m =
      semantic_route_config_.crossing_lateral_margin_m;
  lattice_config_.portal_entry_capture_distance_m =
      declare_parameter<double>("portal_lattice_entry_capture_distance_m", 6.0);
  lattice_config_.portal_exit_extension_m =
      declare_parameter<double>("portal_lattice_exit_extension_m", 4.0);
  lattice_config_.portal_maximum_heading_delta_bins = static_cast<int>(
      declare_parameter<std::int64_t>("portal_lattice_maximum_heading_delta_bins", 4));
  active_guide_config_.collision_radius_m = mppi_config_.risk.collision_radius_m;
  active_guide_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  active_guide_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  active_guide_config_.validation_sample_step_m =
      declare_parameter<double>("global_guide_validation_sample_step_m", 0.5);
  const double static_guide_replan_remaining_m =
      declare_parameter<double>("static_global_guide_replan_remaining_m", 45.0);
  const double no_static_guide_replan_remaining_m =
      declare_parameter<double>("no_static_global_guide_replan_remaining_m", 15.0);
  active_guide_config_.minimum_remaining_m = passage_speed_policy_.use_static_map
                                                 ? static_guide_replan_remaining_m
                                                 : no_static_guide_replan_remaining_m;
  active_guide_config_.maximum_cross_track_m =
      declare_parameter<double>("global_guide_maximum_cross_track_m", 15.0);
  active_guide_config_.velocity_heading_low_speed_mps =
      declare_parameter<double>("global_guide_heading_low_speed_mps", 0.5);
  active_guide_config_.velocity_heading_high_speed_mps =
      declare_parameter<double>("global_guide_heading_high_speed_mps", 1.5);
  guide_progress_config_.observation_window_s =
      declare_parameter<double>("global_guide_stall_observation_window_s", 1.0);
  guide_progress_config_.minimum_progress_m =
      declare_parameter<double>("global_guide_stall_minimum_progress_m", 0.5);
  guide_progress_config_.minimum_predicted_head_progress_m = declare_parameter<double>(
      "global_guide_stall_minimum_predicted_head_progress_m", 0.5);
  guide_progress_config_.persistent_safety_rejection_window_s =
      declare_parameter<double>("persistent_safety_rejection_window_s", 1.0);
  mppi_config_.early_exit_on_collision = true;
  safety_config_.collision_radius_m = mppi_config_.risk.collision_radius_m;
  safety_config_.reaction_latency_s =
      declare_parameter<double>("safety_reaction_latency_s", 0.10);
  safety_config_.maximum_braking_acceleration_mps2 = std::min(
      declare_parameter<double>("safety_maximum_braking_acceleration_mps2", 8.0),
      static_cast<double>(mppi_config_.dynamics.maximum_horizontal_acceleration_mps2));
  safety_config_.minimum_time_to_collision_s =
      declare_parameter<double>("safety_minimum_time_to_collision_s", 0.50);
  const double static_safety_fallback_duration_s =
      declare_parameter<double>("static_safety_fallback_duration_s", 3.0);
  const double no_static_safety_fallback_duration_s =
      declare_parameter<double>("no_static_safety_fallback_duration_s", 3.0);
  const double configured_fallback_duration_s =
      passage_speed_policy_.use_static_map ? static_safety_fallback_duration_s
                                           : no_static_safety_fallback_duration_s;
  const double minimum_fallback_duration_s =
      safety_config_.reaction_latency_s +
      speed_policy_config_.absolute_speed_limit_mps /
          std::max(1.0e-3, safety_config_.maximum_braking_acceleration_mps2) +
      mppi_config_.dynamics.dt_s;
  safety_config_.fallback_duration_s =
      std::max(configured_fallback_duration_s, minimum_fallback_duration_s);
  safety_config_.dt_s = mppi_config_.dynamics.dt_s;
  liveness_config_.enabled = declare_parameter<bool>("liveness_enabled", true);
  liveness_config_.observation_window_s =
      declare_parameter<double>("liveness_observation_window_s", 1.0);
  liveness_config_.minimum_actual_displacement_m =
      declare_parameter<double>("liveness_minimum_actual_displacement_m", 0.5);
  liveness_config_.minimum_predicted_terminal_progress_m =
      declare_parameter<double>("liveness_minimum_predicted_terminal_progress_m", 5.0);
  rviz_period_ns_ = static_cast<std::int64_t>(1.0e9 / std::max(0.1, rviz_rate_hz_));
  diagnostics_info_period_ns_ =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics_info_rate_hz_));

  if (!(tick_rate_hz_ > 0.0) || !(rviz_rate_hz_ > 0.0) ||
      !(diagnostics_info_rate_hz_ > 0.0) || !(deadline_ms_ > 0.0) ||
      !(maximum_control_feedback_age_ms_ > 0.0) ||
      !std::isfinite(passage_speed_policy_.static_limit_mps) ||
      passage_speed_policy_.static_limit_mps < 0.0F ||
      !std::isfinite(passage_speed_policy_.no_static_limit_mps) ||
      passage_speed_policy_.no_static_limit_mps < 0.0F ||
      !(semantic_route_config_.crossing_lateral_margin_m >= 0.0) ||
      !(semantic_route_config_.minimum_normal_alignment >= 0.0) ||
      !(semantic_route_config_.minimum_normal_alignment <= 1.0)) {
    throw std::invalid_argument{"invalid production MPPI configuration"};
  }

  liveness_supervisor_ = std::make_unique<MppiLivenessSupervisor>(liveness_config_);
  active_guide_lifecycle_ =
      std::make_unique<ActiveGlobalGuideLifecycle>(active_guide_config_);
  guide_progress_tracker_ =
      std::make_unique<GlobalGuideProgressTracker>(guide_progress_config_);
  mission_goal_capture_latch_ =
      std::make_unique<MissionGoalCaptureLatch>(mission_goal_capture_config_);
  passage_coordinator_ =
      std::make_unique<PassageCoordinator>(passage_coordinator_config_);
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
    semantic_portal_primitives_ = semanticPortalPrimitives(*known_passages_);
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
  applied_control_sub_ = create_subscription<msg::MppiControlFeedback>(
      declare_parameter<std::string>("applied_control_feedback_topic",
                                     "/drone_city_nav/mppi/applied_control"),
      rclcpp::QoS{10}.reliable(),
      [this](const msg::MppiControlFeedback::SharedPtr message) {
        onAppliedControl(*message);
      });
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic", "/drone_city_nav/mppi/path"),
      rclcpp::QoS{1}.reliable());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      declare_parameter<std::string>("markers_topic", "/drone_city_nav/mppi/markers"),
      rclcpp::QoS{1}.reliable());
  status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/drone_city_nav/mppi/status"),
      rclcpp::QoS{10}.best_effort());
  execution_horizon_pub_ = create_publisher<msg::MppiTrajectoryHorizon>(
      declare_parameter<std::string>("execution_horizon_topic",
                                     "/drone_city_nav/mppi/execution_horizon"),
      rclcpp::QoS{2}.reliable());
  diagnostics_worker_ =
      std::jthread([this](const std::stop_token token) { diagnosticsWorker(token); });
  esdf_worker_ =
      std::jthread([this](const std::stop_token token) { esdfWorker(token); });
  planning_timer_ = create_wall_timer(
      std::chrono::duration<double>{1.0 / tick_rate_hz_}, [this]() { planningTick(); });
  RCLCPP_INFO(
      get_logger(),
      "Production MPPI ready: rollouts=%zu steps=%zu rate=%.1fHz "
      "deadline=%.1fms known_solids=%zu static_map=%s "
      "horizon=%.1fs guide_window=%.1fm cruise=%.1fmps speed_cap=%.1fmps "
      "acceleration_cap=%.1fmps2 jerk_cap=%.1fmps3 speed_tracking_weight=%.2f "
      "passage_speed_limit=%.1fmps head_progress=%.2fs liveness=%s "
      "sticky_guide=true guide_replan_remaining=%.1fm "
      "guide_heading_blend=(%.1f,%.1f)mps passage_vertical_margin=%.2fm "
      "passage_capture_hysteresis=%.2fm passage_capture_max_vz=%.2fmps",
      mppi_config_.rollouts, mppi_config_.steps, tick_rate_hz_, deadline_ms_,
      known_solids_.size(), passage_speed_policy_.use_static_map ? "true" : "false",
      static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s,
      lattice_config_.receding_goal_distance_m, speed_policy_config_.cruise_speed_mps,
      mppi_config_.dynamics.maximum_horizontal_speed_mps,
      mppi_config_.dynamics.maximum_horizontal_acceleration_mps2,
      mppi_config_.dynamics.maximum_control_jerk_mps3,
      mppi_config_.costs.speed_tracking_weight,
      activePassageSpeedLimitMps(passage_speed_policy_),
      mppi_config_.costs.head_progress_horizon_s,
      liveness_config_.enabled ? "true" : "false",
      active_guide_config_.minimum_remaining_m,
      active_guide_config_.velocity_heading_low_speed_mps,
      active_guide_config_.velocity_heading_high_speed_mps,
      passage_coordinator_config_.vertical_clearance_margin_m,
      passage_coordinator_config_.vertical_capture_hysteresis_m,
      passage_coordinator_config_.maximum_capture_vertical_speed_mps);
}

ProductionMppiNode::~ProductionMppiNode() {
  if (diagnostics_worker_.joinable()) {
    diagnostics_worker_.request_stop();
    diagnostics_mailbox_.notifyAll();
    diagnostics_worker_.join();
  }
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
