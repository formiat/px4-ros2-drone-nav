#include <chrono>
#include <exception>
#include <string>
#include <utility>

#include "navigation_diagnostics_sink.hpp"
#include "production_mppi_node.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

void ProductionMppiNode::initializeRuntimeInterfaces(
    StaticWorldResources3D&& static_world_resources) {
  diagnostics_sink_ = std::make_unique<NavigationDiagnosticsSink>(
      NavigationDiagnosticsSinkConfig{
          .output_directory = diagnostics_output_dir_,
          .file_period_ns = diagnostics_file_period_ns_,
          .flush_period = std::chrono::duration<double>{diagnostics_flush_period_s_},
          .error_ring_capacity = diagnostics_error_ring_capacity_,
          .configured_rollouts = mppi_config_.rollouts,
          .deadline_ms = deadline_ms_,
      },
      [this](const ProductionMppiDiagnosticsSnapshot& snapshot) {
        processDiagnostics(snapshot);
      },
      [this](const std::exception_ptr& failure) {
        try {
          std::rethrow_exception(failure);
        } catch (const std::exception& error) {
          RCLCPP_ERROR(get_logger(), "NAVIGATION_DIAGNOSTICS failure: %s",
                       error.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(),
                       "NAVIGATION_DIAGNOSTICS failure: unknown exception");
        }
      });
  const WorldPipeline3D::ProcessingFailureHandler world_failure_handler =
      [this](const std::exception_ptr& failure) {
        try {
          std::rethrow_exception(failure);
        } catch (const std::exception& error) {
          RCLCPP_ERROR(get_logger(), "WORLD_PIPELINE3D failure: %s", error.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "WORLD_PIPELINE3D failure: unknown exception");
        }
      };
  if (use_static_map_) {
    world_pipeline_ = std::make_unique<WorldPipeline3D>(
        StaticWorldRuntime3D{
            .builder_config =
                StaticWorldBuilderConfig3D{
                    .resources = std::move(static_world_resources),
                    .route_lookahead_m = static_esdf_route_lookahead_m_,
                    .roi_halo_m = 40.0,
                    .maximum_distance_m =
                        static_cast<double>(mppi_config_.risk.preferred_distance_m) +
                        20.0,
                    .worker_pool = planning_worker_pool_.get(),
                },
            .request_provider =
                [this](const StaticWorldRefreshRequest3D& refresh) {
                  return makeStaticWorldBuildRequest3D(refresh);
                },
            .commit_context_provider =
                [this]() { return makeStaticWorldCommitContext3D(); },
            .uploader =
                [this](const WorldEsdfUploadRequest3D& request) {
                  const mppi::EsdfUploadResult upload = engine_->updateEsdf(
                      mppi::EsdfSnapshot{request.grid, request.distances_m,
                                         request.revision, request.dirty_regions});
                  return WorldEsdfUploadResult3D{
                      .accepted = upload.accepted,
                      .upload_ms = upload.upload_ms,
                      .revision = upload.revision,
                  };
                },
            .update_handler =
                [this](const StaticWorldUpdate3D& update) {
                  handleStaticWorldUpdate3D(update);
                },
        },
        world_failure_handler);
  } else {
    world_pipeline_ = std::make_unique<WorldPipeline3D>(
        ObservedWorldRuntime3D{
            .builder_config =
                ObservedWorldBuilderConfig3D{
                    .local_window = no_static_3d_esdf_window_,
                    .footprint = physical_footprint_config_,
                    .preferred_distance_m =
                        static_cast<double>(mppi_config_.risk.preferred_distance_m),
                    .update_rate_hz = no_static_3d_esdf_update_rate_hz_,
                    .incremental_maximum_rebuild_ratio =
                        no_static_3d_esdf_incremental_maximum_rebuild_ratio_,
                    .full_audit_interval_builds =
                        no_static_3d_esdf_full_audit_interval_builds_,
                    .worker_pool = planning_worker_pool_.get(),
                },
            .request_provider =
                [this](std::shared_ptr<const ProductionMppiRawWorld3D> raw_world) {
                  return makeObservedWorldBuildRequest3D(std::move(raw_world));
                },
            .uploader =
                [this](const WorldEsdfUploadRequest3D& request) {
                  const mppi::EsdfUploadResult upload = engine_->updateEsdf(
                      mppi::EsdfSnapshot{request.grid, request.distances_m,
                                         request.revision, request.dirty_regions});
                  return WorldEsdfUploadResult3D{
                      .accepted = upload.accepted,
                      .upload_ms = upload.upload_ms,
                      .revision = upload.revision,
                  };
                },
            .evidence_handler =
                [this](const ObservedWorldEvidenceChange3D& change) {
                  handleObservedWorldEvidenceChange3D(change);
                },
            .update_handler =
                [this](const ObservedWorldUpdate3D& update) {
                  handleObservedWorldUpdate3D(update);
                },
        },
        world_failure_handler);
  }

  input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Large lidar evidence messages must not compete with continuously ready PX4
  // navigation inputs. Evidence admission has its own commit mutex and may run
  // concurrently with the lightweight input callbacks.
  lidar_evidence_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  world_input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  planning_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input_subscription_options;
  input_subscription_options.callback_group = input_callback_group_;
  rclcpp::SubscriptionOptions lidar_evidence_subscription_options;
  lidar_evidence_subscription_options.callback_group = lidar_evidence_callback_group_;
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

  const std::string raw_snapshot_3d_topic = declare_parameter<std::string>(
      "raw_obstacle_snapshot_3d_topic", "/drone_city_nav/raw_obstacle_snapshot_3d");
  const std::string raw_delta_3d_topic = declare_parameter<std::string>(
      "raw_obstacle_delta_3d_topic", "/drone_city_nav/raw_obstacle_delta_3d");
  if (!use_static_map_) {
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
      lidar_evidence_subscription_options);
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
  planner_health_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("planner_health_topic",
                                     "/drone_city_nav/mppi/planner_alive"),
      rclcpp::QoS{1}.reliable().transient_local());
  navigation_health_pub_ = create_publisher<msg::NavigationHealth>(
      declare_parameter<std::string>("navigation_health_topic",
                                     "/drone_city_nav/mppi/navigation_health"),
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
  std_msgs::msg::Bool planner_alive;
  planner_alive.data = true;
  planner_health_pub_->publish(planner_alive);
  planner_health_timer_ = create_wall_timer(std::chrono::milliseconds{250}, [this]() {
    std_msgs::msg::Bool heartbeat;
    heartbeat.data = true;
    planner_health_pub_->publish(heartbeat);
  });

  diagnostics_sink_->start();
  world_pipeline_->start();
  route_planning_worker_ =
      std::jthread([this](const std::stop_token token) { routePlanningWorker(token); });
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
