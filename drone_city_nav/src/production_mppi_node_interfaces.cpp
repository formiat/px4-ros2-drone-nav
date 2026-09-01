#include "drone_city_nav/mppi/static_route_handoff.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <exception>
#include <string>
#include <utility>

#include "execution_horizon_assembler_3d.hpp"
#include "mppi_controller_3d.hpp"
#include "navigation_diagnostics_sink.hpp"
#include "planning_cycle_coordinator_3d.hpp"
#include "production_mppi_node.hpp"
#include "route_activation_coordinator_3d.hpp"
#include "route_materializer_3d.hpp"
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
                  const mppi::EsdfUploadResult upload = mppi_controller_->updateEsdf(
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
                  const mppi::EsdfUploadResult upload = mppi_controller_->updateEsdf(
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
  RouteMaterializerConfig3D route_materializer_config{
      .route_envelope = route_envelope_config_,
      .future_route_connector = future_route_connector_config_,
      .route_geometry = static_route_geometry_config_,
      .physical_footprint = physical_footprint_config_,
      .flight_envelope = flight_envelope_config_,
      .route_extension = static_route_extension_config_,
      .passage_volume = cooperative_passage_volume_config_,
      .cooperative_passage_route = cooperative_passage_route_config_,
      .critical_distance_m = static_cast<double>(mppi_config_.risk.critical_distance_m),
      .preferred_distance_m =
          static_cast<double>(mppi_config_.risk.preferred_distance_m),
      .worker_pool = planning_worker_pool_.get(),
      .cooperative_traffic_enabled = cooperative_traffic_enabled_,
  };
  RouteActivationCoordinatorConfig3D route_activation_config{
      .trajectory_compiler =
          RouteTrajectoryCompilerConfig3D{
              .trajectory =
                  TrajectoryCompilerConfig3D{
                      .unconstrained_speed_mps = speed_policy_config_.cruise_speed_mps,
                      .constrained_speed_mps = constrained_route_speed_limit_mps_,
                      .maximum_lateral_acceleration_mps2 =
                          speed_policy_config_.maximum_lateral_acceleration_mps2,
                      .minimum_continuous_turn_alignment =
                          future_route_connector_config_
                              .minimum_continuous_turn_alignment,
                      .time_model =
                          FlightTimeModel3D{
                              .maximum_horizontal_speed_mps = std::min(
                                  {speed_policy_config_.cruise_speed_mps,
                                   speed_policy_config_.absolute_speed_limit_mps,
                                   static_cast<double>(
                                       mppi_config_.dynamics
                                           .maximum_horizontal_speed_mps)}),
                              .maximum_vertical_speed_mps = static_cast<double>(
                                  mppi_config_.dynamics.maximum_vertical_speed_mps),
                              .maximum_translational_speed_mps = static_cast<double>(
                                  mppi_config_.dynamics
                                      .maximum_translational_speed_mps),
                              .maximum_horizontal_acceleration_mps2 =
                                  static_cast<double>(
                                      mppi_config_.dynamics
                                          .maximum_horizontal_acceleration_mps2),
                              .maximum_vertical_acceleration_mps2 = static_cast<double>(
                                  mppi_config_.dynamics
                                      .maximum_vertical_acceleration_mps2),
                              .maximum_control_jerk_mps3 = static_cast<double>(
                                  mppi_config_.dynamics.maximum_control_jerk_mps3),
                              .maximum_yaw_acceleration_radps2 = static_cast<double>(
                                  mppi_config_.dynamics
                                      .maximum_yaw_acceleration_radps2),
                              .maximum_yaw_rate_radps = static_cast<double>(
                                  mppi_config_.dynamics.maximum_yaw_rate_radps),
                          },
                      .physical_footprint = physical_footprint_config_,
                      .tracking_error_tube = tracking_error_tube_config_,
                  },
              .passage_volume = cooperative_passage_volume_config_,
          },
      .route_extension = static_route_extension_config_,
      .successor_improvement = route_successor_improvement_config_,
      .flight_envelope = flight_envelope_config_,
      .route_tracking = route_tracking_policy_,
      .physical_footprint = physical_footprint_config_,
      .certified_splice = certified_route_splice_config_,
      .validation_policy = execution_validation_policy_,
      .route_risk =
          RouteRiskPolicy3D{
              .critical_distance_m =
                  static_cast<double>(mppi_config_.risk.critical_distance_m),
              .preferred_distance_m =
                  static_cast<double>(mppi_config_.risk.preferred_distance_m),
          },
      .dynamic_handoff_validator =
          mppi::makeMppiDynamicHandoffValidator3D(mppi_config_),
      .cruise_speed_mps = speed_policy_config_.cruise_speed_mps,
      .maximum_control_feedback_age_ms = maximum_control_feedback_age_ms_,
  };
  planning_cycle_coordinator_ = std::make_unique<PlanningCycleCoordinator3D>(
      execution_supervisor_,
      PlanningCycleCoordinatorConfig3D{
          .route_execution =
              RouteExecutionSelectorConfig3D{
                  .physical_footprint = physical_footprint_config_,
                  .flight_envelope = flight_envelope_config_,
                  .route_tracking = route_tracking_policy_,
                  .route_cross_track_constraints_enabled =
                      optional_constraints_.route_cross_track_constraints_enabled,
                  .route_tracking_tube_constraints_enabled =
                      optional_constraints_.route_tracking_tube_constraints_enabled,
              },
          .liveness = liveness_config_,
          .route_progress =
              route_stall_recovery_enabled_
                  ? std::optional<RouteProgressConfig3D>{route_progress_config_}
                  : std::nullopt,
          .goal_capture = mission_goal_capture_config_,
          .direct_tracking = direct_tracking_maneuver_config_,
          .route_envelope = route_envelope_config_,
          .constrained_route_control = constrained_route_control_config_,
          .speed_policy = speed_policy_config_,
          .rollout_budget = rollout_budget_config_,
          .cooperative_timing = cooperative_passage_timing_config_,
          .cooperative_yield = cooperative_passage_yield_config_,
          .noncooperative_avoidance = noncooperative_avoidance_config_,
          .flight_envelope = flight_envelope_config_,
          .dynamics = mppi_config_.dynamics,
          .vehicle_id = vehicle_id_,
          .horizon_steps = mppi_config_.steps,
          .tracking_capture_radius_m = tracking_capture_radius_m_,
          .route_constraint_diagnostics_distance_m =
              route_constraint_diagnostics_distance_m_,
          .cooperative_traffic_enabled = cooperative_traffic_enabled_,
          .noncooperative_avoidance_enabled = noncooperative_avoidance_enabled_,
          .route_progress_replan_enabled =
              optional_constraints_.route_progress_replan_enabled,
          .route_cross_track_constraints_enabled =
              optional_constraints_.route_cross_track_constraints_enabled,
          .stochastic_trajectory_selection_enabled =
              optional_constraints_.stochastic_trajectory_selection_enabled,
      });
  execution_horizon_assembler_ =
      std::make_unique<ExecutionHorizonAssembler3D>(ExecutionHorizonAssemblerConfig3D{
          .flight_envelope = flight_envelope_config_,
          .finite_horizon = finite_horizon_config_,
          .direct_tracking_validation_policy = execution_validation_policy_,
      });
  route_lifecycle_coordinator_ = std::make_unique<RouteLifecycleCoordinator3D>(
      execution_supervisor_,
      RouteLifecycleCoordinatorConfig3D{
          .planner =
              RoutePlannerConfig3D{
                  .planner = persistent_planner_config_,
                  .extension = static_route_extension_config_,
                  .route_sampling_step_m = route_sampling_step_m_,
                  .cruise_speed_mps = speed_policy_config_.cruise_speed_mps,
              },
          .materializer = route_materializer_config,
          .activation = std::move(route_activation_config),
          .extension = static_route_extension_config_,
          .search_retry = static_route_search_retry_config_,
          .flight_envelope = flight_envelope_config_,
          .static_route_lookahead_m = static_esdf_route_lookahead_m_,
          .tracking_world_refresh_margin_m = static_tracking_esdf_refresh_margin_m_,
          .observed_world = !use_static_map_,
          .vehicle_state_provider =
              [this]() {
                const std::scoped_lock lock{input_mutex_};
                return RoutePlannerVehicleState3D{
                    .position = Point3{navigation_.state.x, navigation_.state.y,
                                       navigation_.state.z},
                    .velocity = Vec3{navigation_.state.vx, navigation_.state.vy,
                                     navigation_.state.vz},
                    .valid = navigation_.valid,
                };
              },
          .activation_snapshot_provider =
              [this]() { return captureRouteActivationSnapshot3D(); },
          .activation_commit_boundary =
              [this](PreparedRouteActivation3D prepared,
                     const RouteActivationCommitOperation3D& commit) {
                RouteActivationCommitResult3D committed;
                {
                  const std::scoped_lock lock{execution_evidence_commit_mutex_};
                  WorldPipeline3D::ResidentLease resident =
                      world_pipeline_->lockResident();
                  committed =
                      commit(std::move(prepared),
                             RouteActivationCommitContext3D{
                                 .resident_world = resident.world(),
                                 .objective = navigationObjective(),
                                 .raw_world = world_pipeline_->latestRawWorld(),
                                 .minimum_tracking_route_mission_epoch =
                                     minimum_tracking_route_mission_epoch_.load(
                                         std::memory_order_acquire),
                                 .minimum_tracking_route_sample_sequence =
                                     minimum_tracking_route_sample_sequence_.load(
                                         std::memory_order_acquire),
                             });
                }
                latest_route_pipeline_event_.store(
                    std::make_shared<const ProductionRouteActivationResult3D>(
                        committed.result),
                    std::memory_order_release);
                return committed;
              },
          .tracking_context_provider =
              [this]() {
                return RouteLifecycleTrackingContext3D{
                    .objective = navigationObjective(),
                    .minimum_route_mission_epoch =
                        minimum_tracking_route_mission_epoch_.load(
                            std::memory_order_acquire),
                    .minimum_route_sample_sequence =
                        minimum_tracking_route_sample_sequence_.load(
                            std::memory_order_acquire),
                };
              },
          .replan_snapshot_provider =
              [this]() {
                RouteLifecycleReplanSnapshot3D snapshot;
                {
                  const std::scoped_lock input_lock{input_mutex_};
                  snapshot.navigation = navigation_;
                }
                snapshot.objective = navigationObjective();
                const std::shared_ptr<const ExecutionPlan3D> execution =
                    execution_supervisor_.plan();
                snapshot.committed_route_generation =
                    execution != nullptr ? execution->routeGenerationHighWater() : 0U;
                {
                  WorldPipeline3D::ResidentLease resident =
                      world_pipeline_->lockResident();
                  snapshot.resident_world = resident.world();
                  snapshot.world_telemetry = resident.telemetry();
                  if (snapshot.resident_world != nullptr) {
                    snapshot.resident_planner_world =
                        captureResidentPlannerWorld3D(*snapshot.resident_world);
                  }
                }
                snapshot.latest_raw_world = world_pipeline_->latestRawWorld();
                snapshot.blocked_raw_revision =
                    observed_route_blocked_raw_revision_.load(
                        std::memory_order_acquire);
                snapshot.minimum_route_mission_epoch =
                    minimum_tracking_route_mission_epoch_.load(
                        std::memory_order_acquire);
                snapshot.minimum_route_sample_sequence =
                    minimum_tracking_route_sample_sequence_.load(
                        std::memory_order_acquire);
                snapshot.stamp_ns = get_clock()->now().nanoseconds();
                return snapshot;
              },
          .world_refresh_requester =
              [this](const std::uint64_t base_generation,
                     const RouteLifecycleWorldRefreshPurpose3D purpose) {
                const StaticWorldRefreshRequest3D refresh =
                    world_pipeline_->requestStaticRefresh(
                        base_generation,
                        purpose ==
                                RouteLifecycleWorldRefreshPurpose3D::kTrackingObjective
                            ? StaticWorldRefreshPurpose3D::kTrackingObjective
                            : StaticWorldRefreshPurpose3D::kRouteExtension);
                return RouteLifecycleWorldRefreshResult3D{
                    .sequence = refresh.sequence,
                    .base_route_generation = refresh.base_route_generation,
                };
              },
          .stamp_provider = [this]() { return get_clock()->now().nanoseconds(); },
          .update_handler =
              [this](RouteLifecycleUpdate3D update) {
                processRouteSearch3D(std::move(update));
              },
          .rejection_handler =
              [this](const RoutePlanningRejection3D& rejection) {
                handleRoutePlanningRejection3D(rejection);
              },
          .replan_outcome_handler =
              [this](const RouteLifecycleReplanOutcome3D& outcome) {
                logRouteLifecycleReplanOutcome3D(outcome);
              },
          .tracking_followup_handler =
              [this](const RouteLifecycleTrackingFollowup3D& followup) {
                RCLCPP_INFO(get_logger(),
                            "STATIC_ROUTE_SHADOW status=followup_required "
                            "required_epoch=%" PRIu64 " required_sample=%" PRIu64
                            " resident_epoch=%" PRIu64 " resident_sample=%" PRIu64,
                            followup.required_mission_epoch,
                            followup.required_sample_sequence,
                            followup.resident_objective.mission_epoch,
                            followup.resident_objective.sample_sequence);
                requestRouteRelease(RouteReleaseReason3D::kObjectiveChanged);
              },
          .failure_handler =
              [this](const std::exception_ptr failure) {
                try {
                  std::rethrow_exception(failure);
                } catch (const std::exception& error) {
                  RCLCPP_ERROR(get_logger(),
                               "ROUTE_LIFECYCLE_COORDINATOR3D failure: %s",
                               error.what());
                } catch (...) {
                  RCLCPP_ERROR(
                      get_logger(),
                      "ROUTE_LIFECYCLE_COORDINATOR3D failure: unknown exception");
                }
              },
      });

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
  route_lifecycle_coordinator_->start();
  world_pipeline_->start();
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
