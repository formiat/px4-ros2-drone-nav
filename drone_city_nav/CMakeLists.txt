cmake_minimum_required(VERSION 3.20)
project(drone_city_nav VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(DRONE_CITY_NAV_ENABLE_CUDA_MPPI
       "Build the CUDA MPPI controller and benchmark" ON)
set(DRONE_CITY_NAV_CUDA_ARCHITECTURES
    "75;86"
    CACHE STRING "CUDA architectures for MPPI targets")

if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  include(CheckLanguage)
  check_language(CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    message(WARNING "CUDA compiler unavailable; CUDA MPPI targets disabled")
    set(DRONE_CITY_NAV_ENABLE_CUDA_MPPI OFF)
  else()
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ObstacleMemoryHitObservation.msg"
  "msg/ObstacleMemoryCellProvenance.msg"
  "msg/ObstacleMemoryProvenance.msg"
  "msg/ObstacleMemorySnapshot.msg"
  "msg/ObstacleMemoryStatus.msg"
  "msg/LatestLidarObstacleScan.msg"
  "msg/RawObstacleDelta.msg"
  "msg/RawObstacleSnapshot.msg"
  "msg/ObservedObstacleChunk3D.msg"
  "msg/RawObstacleDelta3D.msg"
  "msg/RawObstacleSnapshot3D.msg"
  "msg/VehicleDestroyed.msg"
  "msg/MppiHorizonPoint.msg"
  "msg/MppiTrajectoryHorizon.msg"
  "msg/MppiControlFeedback.msg"
  "msg/NavigationHealth.msg"
  "msg/MissionWaypointAcknowledgement.msg"
  "msg/NavigationObjective.msg"
  "msg/RadarDetection.msg"
  "msg/RadarScan.msg"
  "msg/RadarTrackModeCommand.msg"
  "msg/TargetTrack.msg"
  "msg/TargetTrackArray.msg"
  "msg/TargetAssignment.msg"
  "msg/InterceptTargetStatus.msg"
  "msg/InterceptMissionCommand.msg"
  "msg/SpectatorTarget.msg"
  "msg/SimulationTruthAlignment.msg"
  "msg/SimulationTruthState.msg"
  "msg/VehicleNavigationState.msg"
  "msg/CooperativeTrajectoryPoint.msg"
  "msg/CooperativeConflictResourceUse.msg"
  "msg/CooperativePassageIntent.msg"
  "msg/CooperativeFlightIntent.msg"
  "msg/CooperativePeerTrajectory.msg"
  "msg/CooperativeManeuverCommand.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              rosidl_typesupport_cpp)

function(enable_project_warnings target_name)
  target_compile_options(
    ${target_name}
    PRIVATE
      -Wall
      -Wextra
      -Wpedantic
      -Wconversion
      -Wsign-conversion
      -Wshadow
      -Wold-style-cast)
endfunction()

if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  add_library(
    drone_city_nav_mppi_cuda
    src/mppi/mppi_acquisition_diagnostics.cpp
    src/mppi/mppi_engine.cu
    src/mppi/mppi_control_sequence.cpp
    src/mppi/mppi_input_validation.cpp
    src/mppi/mppi_noncooperative_acquisition.cpp
    src/mppi/static_route_handoff.cpp)
  target_sources(drone_city_nav_mppi_cuda PRIVATE
                 src/mppi/mppi_separation_acquisition.cpp
                 src/mppi/mppi_separation_acquisition_coordinator.cpp)
  target_compile_features(drone_city_nav_mppi_cuda PUBLIC cxx_std_20)
  set_target_properties(
    drone_city_nav_mppi_cuda
    PROPERTIES CUDA_ARCHITECTURES "${DRONE_CITY_NAV_CUDA_ARCHITECTURES}"
               CUDA_STANDARD 20
               CUDA_STANDARD_REQUIRED ON
               POSITION_INDEPENDENT_CODE ON)
  target_include_directories(
    drone_city_nav_mppi_cuda PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(drone_city_nav_mppi_cuda PUBLIC CUDA::cudart)
  target_compile_options(
    drone_city_nav_mppi_cuda
    PRIVATE
      "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion;-Wshadow;-Wold-style-cast>"
      $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)

  add_executable(
    mppi_cuda_benchmark
    tools/mppi_cuda_benchmark.cpp
    src/mppi/mppi_cuda.cu
    src/mppi/mppi_cuda_host.cpp)
  target_compile_features(mppi_cuda_benchmark PRIVATE cxx_std_20)
  set_target_properties(
    mppi_cuda_benchmark
    PROPERTIES CUDA_ARCHITECTURES "${DRONE_CITY_NAV_CUDA_ARCHITECTURES}"
               CUDA_STANDARD 20
               CUDA_STANDARD_REQUIRED ON)
  target_include_directories(mppi_cuda_benchmark PRIVATE include)
  target_link_libraries(mppi_cuda_benchmark PRIVATE drone_city_nav_mppi_cuda
                                                    CUDA::cudart)
  target_compile_options(
    mppi_cuda_benchmark
    PRIVATE
      "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion;-Wshadow;-Wold-style-cast>"
      $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)
  install(
    TARGETS drone_city_nav_mppi_cuda mppi_cuda_benchmark
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

function(add_drone_city_nav_layer target_name)
  add_library(${target_name} SHARED ${ARGN})
  target_compile_features(${target_name} PUBLIC cxx_std_20)
  target_include_directories(
    ${target_name}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>")
  target_link_options(${target_name} PRIVATE "LINKER:--no-undefined")
  enable_project_warnings(${target_name})
endfunction()

# Enforced dependency graph. Two contract targets branch directly from model and
# are consumed lower down without dragging their consumers upward:
#
#   model -> control_contracts        (controller-neutral motion DTOs)
#   model -> route_contracts          (route geometry and release contracts)
#   model + route_contracts -> world -> collision
#   collision + control_contracts -> finite_execution
#   collision -> planning -> trajectory
#   trajectory + finite_execution -> execution -> control -> runtime
#
# finite_execution holds the controller-neutral validation algorithms that need
# collision evidence; control_contracts stays a pure DTO target so model-level
# motion code can depend on it. No target may link a layer above itself.
add_drone_city_nav_layer(
  drone_city_nav_model
  src/bounded_worker_pool.cpp
  src/flight_time_model_3d.cpp
  src/footprint_geometry_3d.cpp
  src/navigation_angular_derivative.cpp
  src/navigation_pose.cpp
  src/navigation_recovery_episode_tracker.cpp
  src/navigation_state_prediction.cpp
  src/producer_epoch_admission.cpp
  src/producer_instance_id.cpp
  src/px4_ros_time_mapper.cpp
  src/radar_cadence.cpp
  src/radar_model.cpp
  src/simulation_truth_alignment.cpp
  src/stopping_distance.cpp
  src/world_generation.cpp)

add_drone_city_nav_layer(
  drone_city_nav_control_contracts
  src/finite_motion_horizon_3d.cpp
  src/motion_dynamics_3d.cpp)
target_link_libraries(drone_city_nav_control_contracts PUBLIC drone_city_nav_model)

add_drone_city_nav_layer(
  drone_city_nav_route_contracts
  src/route_3d.cpp
  src/route_3d_passage.cpp
  src/route_3d_spans.cpp)
target_link_libraries(drone_city_nav_route_contracts PUBLIC drone_city_nav_model)

add_drone_city_nav_layer(
  drone_city_nav_world
  src/ambiguous_lidar_hit_tracker.cpp
  src/collision_voxelizer.cpp
  src/debug_image.cpp
  src/distance_field.cpp
  src/distance_field_3d.cpp
  src/esdf_query.cpp
  src/grid_config.cpp
  src/grid_overlay.cpp
  src/known_obstacle_distance_3d.cpp
  src/known_static_lidar_hit_classifier.cpp
  src/latest_lidar_obstacle_scan.cpp
  src/lidar_acquisition_pose.cpp
  src/lidar_beam_observation.cpp
  src/lidar_debug_renderer.cpp
  src/lidar_debug_snapshot_pipeline.cpp
  src/lidar_ingestion_confidence.cpp
  src/lidar_ingestion_decision.cpp
  src/lidar_memory_hit_diagnostics.cpp
  src/lidar_pose_history.cpp
  src/lidar_projection.cpp
  src/lidar_scan_3d.cpp
  src/lidar_self_filter.cpp
  src/lidar_snapshot_writer.cpp
  src/local_esdf_2d.cpp
  src/mapping_lifecycle.cpp
  src/obstacle_memory.cpp
  src/obstacle_memory_3d.cpp
  src/obstacle_memory_transport_policy_3d.cpp
  src/observed_esdf_3d.cpp
  src/observed_esdf_3d_fingerprint.cpp
  src/observed_esdf_3d_metadata.cpp
  src/observed_esdf_3d_window.cpp
  src/observed_occupancy_grid_3d.cpp
  src/occupancy_grid.cpp
  src/occupancy_grid_3d.cpp
  src/static_esdf_cache.cpp
  src/tracked_agent_lidar_filter.cpp)
target_link_libraries(
  drone_city_nav_world
  PUBLIC drone_city_nav_model drone_city_nav_route_contracts
  PRIVATE PkgConfig::ZSTD)

add_drone_city_nav_layer(
  drone_city_nav_collision
  src/flight_envelope.cpp
  src/launch_support_contact_3d.cpp
  src/observed_esdf_footprint_3d.cpp
  src/observed_world_content_hash_3d.cpp
  src/occupied_collision_oracle_3d.cpp
  src/sensor_braking_contract_3d.cpp
  src/swept_footprint.cpp
  src/swept_footprint_2d.cpp
  src/swept_footprint_esdf.cpp
  src/versioned_world_evidence_3d.cpp)
target_link_libraries(drone_city_nav_collision PUBLIC drone_city_nav_world)

add_drone_city_nav_layer(drone_city_nav_finite_execution
                         src/finite_execution_path_3d.cpp)
target_link_libraries(drone_city_nav_finite_execution
                      PUBLIC drone_city_nav_collision
                             drone_city_nav_control_contracts)

add_drone_city_nav_layer(
  drone_city_nav_planning
  src/free_space_topology_3d.cpp
  src/free_space_topology_extractor_3d.cpp
  src/free_space_topology_extractor_3d_api.cpp
  src/free_space_topology_extractor_3d_geometry.cpp
  src/free_space_topology_extractor_3d_graph.cpp
  src/passage_traversal_selection_3d.cpp
  src/persistent_dstar_lite_planner_3d.cpp
  src/persistent_dstar_lite_planner_3d_feasible_search.cpp
  src/persistent_dstar_lite_planner_3d_lattice.cpp
  src/persistent_dstar_lite_planner_3d_search.cpp
  src/persistent_dstar_lite_planner_3d_time_search.cpp
  src/persistent_dstar_lite_planner_3d_world.cpp
  src/route_planning_3d.cpp
  src/route_risk_annotation_3d.cpp
  src/static_route_geometry.cpp
  src/tracking_objective.cpp)
target_link_libraries(drone_city_nav_planning PUBLIC drone_city_nav_collision)

add_drone_city_nav_layer(
  drone_city_nav_trajectory
  src/compiled_trajectory_3d.cpp
  src/compiled_trajectory_validation_3d.cpp
  src/compiled_trajectory_views_3d.cpp
  src/cooperative_passage_coordination.cpp
  src/cooperative_passage_route.cpp
  src/cooperative_space_time.cpp
  src/cooperative_traffic.cpp
  src/passage_volume.cpp
  src/route_execution_contract_3d.cpp
  src/route_decoration_compiler_3d.cpp
  src/route_decorations_3d.cpp
  src/route_time_parameterization.cpp
  src/static_route_extension.cpp
  src/tracking_error_tube_3d.cpp
  src/trajectory_compiler_3d.cpp)
target_link_libraries(drone_city_nav_trajectory PUBLIC drone_city_nav_planning)

add_drone_city_nav_layer(
  drone_city_nav_execution
  src/applied_control_admission.cpp
  src/certified_route_splice_3d.cpp
  src/committed_execution_authority_3d.cpp
  src/direct_tracking_maneuver_lifecycle.cpp
  src/execution_evidence_3d.cpp
  src/execution_horizon_admission.cpp
  src/execution_horizon_timing.cpp
  src/execution_horizon_witness.cpp
  src/execution_plan_3d.cpp
  src/execution_plan_reducer_3d.cpp
  src/execution_publication_currentness_3d.cpp
  src/execution_route_snapshot_3d.cpp
  src/execution_route_snapshot_3d_certificates.cpp
  src/execution_route_snapshot_3d_certification.cpp
  src/execution_route_snapshot_3d_continuation_validation.cpp
  src/execution_route_snapshot_3d_finite_execution.cpp
  src/execution_route_snapshot_3d_finite_execution_diagnostics.cpp
  src/execution_route_snapshot_3d_geometry.cpp
  src/execution_route_snapshot_3d_hold.cpp
  src/execution_route_snapshot_3d_progress.cpp
  src/execution_route_snapshot_3d_tracking_tube.cpp
  src/execution_route_snapshot_3d_transitions.cpp
  src/execution_route_snapshot_3d_validation.cpp
  src/execution_supervisor_3d.cpp
  src/execution_supervisor_3d_hold.cpp
  src/execution_supervisor_3d_horizon.cpp
  src/execution_supervisor_3d_retention.cpp
  src/mission_goal_capture.cpp
  src/mission_waypoint_acknowledgement_admission.cpp
  src/mission_waypoint_capture_gate.cpp
  src/mission_waypoint_sequence.cpp
  src/offboard_session_admission.cpp
  src/pending_certified_route_3d.cpp
  src/rolling_route_telemetry_3d.cpp
  src/route_execution_manager_3d.cpp
  src/trajectory_control_reference_3d.cpp
  src/route_lifecycle_3d.cpp
  src/route_progress_3d.cpp
  src/route_successor_improvement_3d.cpp
  src/vehicle_destruction_disarm_lifecycle.cpp)
target_link_libraries(drone_city_nav_execution
                      PUBLIC drone_city_nav_trajectory
                             drone_city_nav_finite_execution)

add_drone_city_nav_layer(
  drone_city_nav_control
  src/cooperative_mppi_adapter.cpp
  src/cooperative_passage_execution.cpp
  src/mppi/mppi_reference.cpp
  src/mppi/mppi_post_update_classification.cpp
  src/mppi_liveness.cpp
  src/mppi_nominal_reseed.cpp
  src/mppi_rollout_budget.cpp
  src/mppi_speed_policy.cpp
  src/noncooperative_collision_avoidance.cpp)
target_sources(drone_city_nav_control
               PRIVATE src/mppi/trajectory_reference_adapter_3d.cpp)
target_link_libraries(drone_city_nav_control PUBLIC drone_city_nav_execution)

add_drone_city_nav_layer(
  drone_city_nav_runtime
  src/cooperative_traffic_mission.cpp
  src/dynamic_agent_lidar_state.cpp
  src/intercept_guidance.cpp
  src/intercept_mission.cpp
  src/navigation_health_supervisor.cpp
  src/radar_target_tracker.cpp
  src/radar_visibility.cpp
  src/spectator_selection.cpp
  src/target_assignment.cpp)
target_link_libraries(drone_city_nav_runtime PUBLIC drone_city_nav_control)

# Header ownership. Every hand-written public header belongs to exactly one
# target, and may include only headers owned by that target or by one of its
# declared predecessors. The lists are exhaustive: a new header that is not
# named here fails test_navigation_dependency_contract.py rather than silently
# inheriting whatever include path happens to be available.
set(DRONE_CITY_NAV_MODEL_HEADERS
    include/drone_city_nav/bounded_worker_pool.hpp
    include/drone_city_nav/flight_time_model_3d.hpp
    include/drone_city_nav/footprint_geometry_3d.hpp
    include/drone_city_nav/json_output.hpp
    include/drone_city_nav/latest_processing_queue.hpp
    include/drone_city_nav/latest_value_mailbox.hpp
    include/drone_city_nav/motion_state_3d.hpp
    include/drone_city_nav/navigation_angular_derivative.hpp
    include/drone_city_nav/navigation_pose.hpp
    include/drone_city_nav/navigation_recovery_episode_tracker.hpp
    include/drone_city_nav/navigation_state_prediction.hpp
    include/drone_city_nav/producer_epoch_admission.hpp
    include/drone_city_nav/producer_instance_id.hpp
    include/drone_city_nav/px4_map_frame_transform.hpp
    include/drone_city_nav/px4_ros_time_mapper.hpp
    include/drone_city_nav/radar_cadence.hpp
    include/drone_city_nav/radar_model.hpp
    include/drone_city_nav/simulation_truth_alignment.hpp
    include/drone_city_nav/stopping_capability.hpp
    include/drone_city_nav/stopping_distance.hpp
    include/drone_city_nav/strong_string_id.hpp
    include/drone_city_nav/timed_vehicle_state.hpp
    include/drone_city_nav/types.hpp
    include/drone_city_nav/vehicle_state_3d.hpp
    include/drone_city_nav/world_generation.hpp)

set(DRONE_CITY_NAV_CONTROL_CONTRACTS_HEADERS
    include/drone_city_nav/control_contracts_3d.hpp
    include/drone_city_nav/control_route_projection_3d.hpp
    include/drone_city_nav/finite_motion_horizon_3d.hpp
    include/drone_city_nav/motion_altitude_envelope_3d.hpp
    include/drone_city_nav/motion_dynamics_3d.hpp)

set(DRONE_CITY_NAV_ROUTE_CONTRACTS_HEADERS
    include/drone_city_nav/passage_ids.hpp
    include/drone_city_nav/portal_graph.hpp
    include/drone_city_nav/route_3d.hpp
    include/drone_city_nav/route_release_3d.hpp
    include/drone_city_nav/route_risk_policy_3d.hpp
    include/drone_city_nav/route_risk_tier_3d.hpp)

set(DRONE_CITY_NAV_WORLD_HEADERS
    include/drone_city_nav/ambiguous_lidar_hit_tracker.hpp
    include/drone_city_nav/collision_voxelizer.hpp
    include/drone_city_nav/debug_image.hpp
    include/drone_city_nav/distance_field.hpp
    include/drone_city_nav/distance_field_3d.hpp
    include/drone_city_nav/esdf_grid_3d.hpp
    include/drone_city_nav/esdf_query.hpp
    include/drone_city_nav/grid_config.hpp
    include/drone_city_nav/grid_overlay.hpp
    include/drone_city_nav/known_obstacle_distance_3d.hpp
    include/drone_city_nav/known_static_lidar_hit_classifier.hpp
    include/drone_city_nav/latest_lidar_obstacle_scan.hpp
    include/drone_city_nav/lidar_acquisition_pose.hpp
    include/drone_city_nav/lidar_beam_observation.hpp
    include/drone_city_nav/lidar_debug_renderer.hpp
    include/drone_city_nav/lidar_debug_snapshot_pipeline.hpp
    include/drone_city_nav/lidar_ingestion_decision.hpp
    include/drone_city_nav/lidar_memory_hit_diagnostics.hpp
    include/drone_city_nav/lidar_motion_compensation.hpp
    include/drone_city_nav/lidar_pose_history.hpp
    include/drone_city_nav/lidar_projection.hpp
    include/drone_city_nav/lidar_scan_3d.hpp
    include/drone_city_nav/lidar_self_filter.hpp
    include/drone_city_nav/lidar_snapshot_writer.hpp
    include/drone_city_nav/local_esdf_2d.hpp
    include/drone_city_nav/map_to_sdf_transform.hpp
    include/drone_city_nav/mapping_lifecycle.hpp
    include/drone_city_nav/observed_esdf_3d.hpp
    include/drone_city_nav/observed_occupancy_grid_3d.hpp
    include/drone_city_nav/obstacle_memory.hpp
    include/drone_city_nav/obstacle_memory_3d.hpp
    include/drone_city_nav/obstacle_memory_transport_policy_3d.hpp
    include/drone_city_nav/occupancy_grid.hpp
    include/drone_city_nav/occupancy_grid_3d.hpp
    include/drone_city_nav/static_esdf_cache.hpp
    include/drone_city_nav/tracked_agent_lidar_filter.hpp
    include/drone_city_nav/world_snapshot_3d.hpp)

set(DRONE_CITY_NAV_COLLISION_HEADERS
    include/drone_city_nav/derived_clearance_3d.hpp
    include/drone_city_nav/flight_envelope.hpp
    include/drone_city_nav/launch_support_contact_3d.hpp
    include/drone_city_nav/observed_esdf_footprint_3d.hpp
    include/drone_city_nav/observed_world_content_hash_3d.hpp
    include/drone_city_nav/occupied_collision_oracle_3d.hpp
    include/drone_city_nav/sensor_braking_contract_3d.hpp
    include/drone_city_nav/swept_footprint.hpp
    include/drone_city_nav/versioned_world_evidence_3d.hpp)

set(DRONE_CITY_NAV_FINITE_EXECUTION_HEADERS
    include/drone_city_nav/finite_execution_path_3d.hpp)

set(DRONE_CITY_NAV_PLANNING_HEADERS
    include/drone_city_nav/free_space_topology_3d.hpp
    include/drone_city_nav/free_space_topology_extractor_3d.hpp
    include/drone_city_nav/free_space_topology_format_limits.hpp
    include/drone_city_nav/passage_traversal_selection_3d.hpp
    include/drone_city_nav/persistent_dstar_lite_planner_3d.hpp
    include/drone_city_nav/route_planning_3d.hpp
    include/drone_city_nav/route_risk_annotation_3d.hpp
    include/drone_city_nav/static_route_geometry.hpp
    include/drone_city_nav/tracking_objective.hpp)

set(DRONE_CITY_NAV_TRAJECTORY_HEADERS
    include/drone_city_nav/compiled_trajectory_3d.hpp
    include/drone_city_nav/compiled_trajectory_views_3d.hpp
    include/drone_city_nav/cooperative_passage_coordination.hpp
    include/drone_city_nav/cooperative_passage_route.hpp
    include/drone_city_nav/cooperative_space_time.hpp
    include/drone_city_nav/cooperative_traffic.hpp
    include/drone_city_nav/passage_volume.hpp
    include/drone_city_nav/route_decoration_compiler_3d.hpp
    include/drone_city_nav/route_decorations_3d.hpp
    include/drone_city_nav/route_execution_contract_3d.hpp
    include/drone_city_nav/route_time_parameterization.hpp
    include/drone_city_nav/static_route_extension.hpp
    include/drone_city_nav/tracking_error_tube_3d.hpp
    include/drone_city_nav/tracking_error_tube_handoff_3d.hpp
    include/drone_city_nav/trajectory_compiler_3d.hpp)

set(DRONE_CITY_NAV_EXECUTION_HEADERS
    include/drone_city_nav/applied_control_admission.hpp
    include/drone_city_nav/certified_route_splice_3d.hpp
    include/drone_city_nav/committed_execution_authority_3d.hpp
    include/drone_city_nav/direct_tracking_maneuver_lifecycle.hpp
    include/drone_city_nav/dynamic_handoff_validator_3d.hpp
    include/drone_city_nav/execution_evidence_3d.hpp
    include/drone_city_nav/execution_hold_3d.hpp
    include/drone_city_nav/execution_horizon_admission.hpp
    include/drone_city_nav/execution_horizon_commit_3d.hpp
    include/drone_city_nav/execution_horizon_timing.hpp
    include/drone_city_nav/execution_horizon_witness.hpp
    include/drone_city_nav/execution_plan_3d.hpp
    include/drone_city_nav/execution_publication_currentness_3d.hpp
    include/drone_city_nav/execution_retention_3d.hpp
    include/drone_city_nav/execution_route_certificates_3d.hpp
    include/drone_city_nav/execution_route_certification_3d.hpp
    include/drone_city_nav/execution_route_model_3d.hpp
    include/drone_city_nav/execution_route_store_3d.hpp
    include/drone_city_nav/execution_route_transitions_3d.hpp
    include/drone_city_nav/execution_supervisor_3d.hpp
    include/drone_city_nav/mission_goal_capture.hpp
    include/drone_city_nav/mission_waypoint_acknowledgement_admission.hpp
    include/drone_city_nav/mission_waypoint_capture_gate.hpp
    include/drone_city_nav/mission_waypoint_sequence.hpp
    include/drone_city_nav/offboard_session_admission.hpp
    include/drone_city_nav/pending_certified_route_3d.hpp
    include/drone_city_nav/rolling_route_telemetry_3d.hpp
    include/drone_city_nav/route_lifecycle_3d.hpp
    include/drone_city_nav/route_progress_3d.hpp
    include/drone_city_nav/route_successor_improvement_3d.hpp
    include/drone_city_nav/trajectory_control_reference_3d.hpp
    include/drone_city_nav/vehicle_destruction_disarm_lifecycle.hpp)

set(DRONE_CITY_NAV_CONTROL_HEADERS
    include/drone_city_nav/cooperative_mppi_adapter.hpp
    include/drone_city_nav/cooperative_passage_execution.hpp
    include/drone_city_nav/mppi_liveness.hpp
    include/drone_city_nav/mppi_nominal_reseed.hpp
    include/drone_city_nav/mppi_rollout_budget.hpp
    include/drone_city_nav/mppi_speed_policy.hpp
    include/drone_city_nav/noncooperative_collision_avoidance.hpp)

set(DRONE_CITY_NAV_RUNTIME_HEADERS
    include/drone_city_nav/cooperative_traffic_mission.hpp
    include/drone_city_nav/dynamic_agent_lidar_state.hpp
    include/drone_city_nav/intercept_guidance.hpp
    include/drone_city_nav/intercept_mission.hpp
    include/drone_city_nav/navigation_health_supervisor.hpp
    include/drone_city_nav/radar_target_tracker.hpp
    include/drone_city_nav/radar_visibility.hpp
    include/drone_city_nav/spectator_diagnostics_selection.hpp
    include/drone_city_nav/spectator_selection.hpp
    include/drone_city_nav/target_assignment.hpp)

set(DRONE_CITY_NAV_ROS_HEADERS
    include/drone_city_nav/cooperative_traffic_agent_node.hpp
    include/drone_city_nav/cooperative_traffic_ros.hpp
    include/drone_city_nav/execution_horizon_contract_ros.hpp
    include/drone_city_nav/intercept_diagnostics_mux_node.hpp
    include/drone_city_nav/intercept_spectator_node.hpp
    include/drone_city_nav/interceptor_guidance_node.hpp
    include/drone_city_nav/latest_lidar_obstacle_scan_ros.hpp
    include/drone_city_nav/lidar_debug_node_config.hpp
    include/drone_city_nav/lidar_debug_pointclouds.hpp
    include/drone_city_nav/mppi_debug_markers.hpp
    include/drone_city_nav/obstacle_memory_provenance_ros.hpp
    include/drone_city_nav/px4_offboard_setpoint_io.hpp
    include/drone_city_nav/radar_target_tracker_node.hpp
    include/drone_city_nav/raw_obstacle_3d_ros.hpp
    include/drone_city_nav/raw_obstacle_delta.hpp
    include/drone_city_nav/ros_conversions.hpp
    include/drone_city_nav/spectator_diagnostics_selection_ros.hpp
    include/drone_city_nav/static_map_debug.hpp
    include/drone_city_nav/target_assignment_coordinator_node.hpp
    include/drone_city_nav/visualization_marker_helpers.hpp
    include/drone_city_nav/world_visualization_node.hpp)

set(DRONE_CITY_NAV_INTERNAL_LAYERS
    drone_city_nav_model
    drone_city_nav_control_contracts
    drone_city_nav_route_contracts
    drone_city_nav_world
    drone_city_nav_collision
    drone_city_nav_finite_execution
    drone_city_nav_planning
    drone_city_nav_trajectory
    drone_city_nav_execution
    drone_city_nav_control
    drone_city_nav_runtime)

# A layer may link exactly the predecessors named here: every one of them, and
# no other internal layer. Direct predecessors are declared, not inferred, so a
# new edge has to be argued for in this file instead of appearing silently.
function(assert_drone_city_nav_layer_dependency target_name)
  set(expected_predecessors ${ARGN})
  get_target_property(layer_links ${target_name} LINK_LIBRARIES)
  if(NOT layer_links)
    set(layer_links "")
  endif()
  foreach(layer_link IN LISTS layer_links)
    list(FIND DRONE_CITY_NAV_INTERNAL_LAYERS "${layer_link}" internal_index)
    if(internal_index EQUAL -1)
      continue()
    endif()
    list(FIND expected_predecessors "${layer_link}" expected_index)
    if(expected_index EQUAL -1)
      message(
        FATAL_ERROR
          "${target_name} has forbidden internal dependency ${layer_link}; expected only ${expected_predecessors}"
      )
    endif()
  endforeach()
  foreach(expected_predecessor IN LISTS expected_predecessors)
    list(FIND layer_links "${expected_predecessor}" predecessor_index)
    if(predecessor_index EQUAL -1)
      message(
        FATAL_ERROR
          "${target_name} must link its predecessor ${expected_predecessor}")
    endif()
  endforeach()
endfunction()

assert_drone_city_nav_layer_dependency(drone_city_nav_model)
assert_drone_city_nav_layer_dependency(drone_city_nav_control_contracts
                                       drone_city_nav_model)
assert_drone_city_nav_layer_dependency(drone_city_nav_route_contracts
                                       drone_city_nav_model)
assert_drone_city_nav_layer_dependency(drone_city_nav_world drone_city_nav_model
                                       drone_city_nav_route_contracts)
assert_drone_city_nav_layer_dependency(drone_city_nav_collision
                                       drone_city_nav_world)
assert_drone_city_nav_layer_dependency(drone_city_nav_finite_execution
                                       drone_city_nav_collision
                                       drone_city_nav_control_contracts)
assert_drone_city_nav_layer_dependency(drone_city_nav_planning
                                       drone_city_nav_collision)
assert_drone_city_nav_layer_dependency(drone_city_nav_trajectory
                                       drone_city_nav_planning)
assert_drone_city_nav_layer_dependency(drone_city_nav_execution
                                       drone_city_nav_trajectory
                                       drone_city_nav_finite_execution)
assert_drone_city_nav_layer_dependency(drone_city_nav_control
                                       drone_city_nav_execution)
assert_drone_city_nav_layer_dependency(drone_city_nav_runtime
                                       drone_city_nav_control)

# Compatibility umbrella while downstream targets migrate to the narrow layer
# they actually consume.
add_library(drone_city_nav_core INTERFACE)
target_compile_features(drone_city_nav_core INTERFACE cxx_std_20)
target_include_directories(
  drone_city_nav_core
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_link_libraries(drone_city_nav_core INTERFACE drone_city_nav_runtime)

add_executable(generate_static_esdf_cache tools/generate_static_esdf_cache.cpp)
target_compile_features(generate_static_esdf_cache PRIVATE cxx_std_20)
target_link_libraries(generate_static_esdf_cache drone_city_nav_core)
enable_project_warnings(generate_static_esdf_cache)

add_executable(free_space_topology_compiler
               tools/free_space_topology_compiler.cpp)
target_compile_features(free_space_topology_compiler PRIVATE cxx_std_20)
target_link_libraries(free_space_topology_compiler drone_city_nav_core)
enable_project_warnings(free_space_topology_compiler)

add_executable(voxelize_sdf_collisions tools/voxelize_sdf_collisions.cpp)
target_compile_features(voxelize_sdf_collisions PRIVATE cxx_std_20)
target_link_libraries(
  voxelize_sdf_collisions
  PRIVATE drone_city_nav_core gz-common5::graphics sdformat14::sdformat14)
enable_project_warnings(voxelize_sdf_collisions)

if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  target_link_libraries(drone_city_nav_mppi_cuda PUBLIC drone_city_nav_control)
endif()

function(add_drone_city_nav_private_runtime target_name)
  add_library(${target_name} STATIC ${ARGN})
  target_compile_features(${target_name} PUBLIC cxx_std_20)
  set_target_properties(${target_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(
    ${target_name}
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  enable_project_warnings(${target_name})
endfunction()

function(assert_drone_city_nav_target_links target_name)
  set(options)
  set(one_value_arguments)
  set(multi_value_arguments REQUIRED FORBIDDEN)
  cmake_parse_arguments(ASSERT_LINKS "${options}" "${one_value_arguments}"
                        "${multi_value_arguments}" ${ARGN})
  get_target_property(actual_links ${target_name} LINK_LIBRARIES)
  if(NOT actual_links)
    set(actual_links "")
  endif()
  foreach(required_link IN LISTS ASSERT_LINKS_REQUIRED)
    list(FIND actual_links "${required_link}" required_index)
    if(required_index EQUAL -1)
      message(
        FATAL_ERROR
          "${target_name} must link its declared boundary ${required_link}")
    endif()
  endforeach()
  foreach(forbidden_link IN LISTS ASSERT_LINKS_FORBIDDEN)
    list(FIND actual_links "${forbidden_link}" forbidden_index)
    if(NOT forbidden_index EQUAL -1)
      message(
        FATAL_ERROR
          "${target_name} has forbidden direct dependency ${forbidden_link}")
    endif()
  endforeach()
endfunction()

# Package-private application services. These targets make the production
# component a ROS composition adapter instead of a second domain-layer
# umbrella. Their public include root is build-only because none of these APIs
# is installed as a package contract.
set(DRONE_CITY_NAV_WORLD_RUNTIME_SOURCES
    src/world/observed_world_builder_3d.cpp
    src/world/production_mppi_raw_world.cpp
    src/world/production_mppi_route_world.cpp
    src/world/static_world_builder_3d.cpp
    src/world/world_pipeline_3d.cpp)
add_drone_city_nav_private_runtime(drone_city_nav_world_runtime
                                   ${DRONE_CITY_NAV_WORLD_RUNTIME_SOURCES})
target_include_directories(
  drone_city_nav_world_runtime
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/world>")
# World runtime owns world evidence only. It must not reach the route or
# execution layers; the versioned world evidence it publishes lives in
# drone_city_nav_world.
target_link_libraries(drone_city_nav_world_runtime PUBLIC drone_city_nav_planning)

set(DRONE_CITY_NAV_ROUTE_RUNTIME_SOURCES
    src/route_application/production_mppi_route_execution.cpp
    src/route_application/production_mppi_route_helpers.cpp
    src/route_application/route_activation_coordinator_3d.cpp
    src/route_application/route_activation_preparation_3d.cpp
    src/route_application/route_lifecycle_coordinator_3d.cpp
    src/route_application/route_planner_3d.cpp
    src/route_application/route_planning_coordinator_3d.cpp
    src/route_application/route_materializer_3d.cpp
    src/route_application/route_trajectory_compiler_3d.cpp)
add_drone_city_nav_private_runtime(drone_city_nav_route_runtime
                                   ${DRONE_CITY_NAV_ROUTE_RUNTIME_SOURCES})
target_include_directories(
  drone_city_nav_route_runtime
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/route_application>")
# Route runtime is the first application service that owns execution state, so
# it names the execution layer directly instead of inheriting it through the
# world runtime.
target_link_libraries(drone_city_nav_route_runtime
                      PUBLIC drone_city_nav_world_runtime
                             drone_city_nav_execution)

if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  set(DRONE_CITY_NAV_MPPI_RUNTIME_SOURCES
      src/runtime/execution_horizon_assembler_3d.cpp
      src/runtime/mppi_controller_3d.cpp
      src/runtime/navigation_diagnostics_sink.cpp
      src/runtime/planning_cycle_coordinator_3d.cpp
      src/runtime/production_mppi_config.cpp
      src/runtime/production_mppi_cooperative_diagnostics.cpp
      src/runtime/production_mppi_execution_control.cpp
      src/runtime/production_mppi_execution_diagnostics.cpp
      src/runtime/production_mppi_node_names.cpp
      src/runtime/production_mppi_noncooperative_diagnostics.cpp)
  add_drone_city_nav_private_runtime(drone_city_nav_mppi_runtime
                                     ${DRONE_CITY_NAV_MPPI_RUNTIME_SOURCES})
  target_include_directories(
    drone_city_nav_mppi_runtime
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/runtime>")
  target_link_libraries(drone_city_nav_mppi_runtime
                        PUBLIC drone_city_nav_route_runtime
                               drone_city_nav_runtime
                               drone_city_nav_mppi_cuda)
endif()

assert_drone_city_nav_target_links(
  drone_city_nav_world_runtime
  REQUIRED drone_city_nav_planning
  FORBIDDEN drone_city_nav_execution drone_city_nav_control drone_city_nav_core
            drone_city_nav_route_runtime drone_city_nav_mppi_runtime
            drone_city_nav_runtime drone_city_nav_ros_adapters
            drone_city_nav_mppi_cuda)
assert_drone_city_nav_target_links(
  drone_city_nav_route_runtime
  REQUIRED drone_city_nav_world_runtime drone_city_nav_execution
  FORBIDDEN drone_city_nav_core drone_city_nav_mppi_runtime
            drone_city_nav_runtime drone_city_nav_control
            drone_city_nav_ros_adapters drone_city_nav_mppi_cuda)
if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  assert_drone_city_nav_target_links(
    drone_city_nav_mppi_runtime
    REQUIRED drone_city_nav_route_runtime drone_city_nav_runtime
             drone_city_nav_mppi_cuda
    FORBIDDEN drone_city_nav_core drone_city_nav_ros_adapters)
endif()

add_library(drone_contact_system SHARED src/drone_contact_system.cpp)
target_compile_features(drone_contact_system PRIVATE cxx_std_20)
target_link_libraries(
  drone_contact_system
  PRIVATE gz-sim8::gz-sim8 gz-transport13::gz-transport13
          gz-msgs10::gz-msgs10)
enable_project_warnings(drone_contact_system)

add_library(
  drone_city_nav_ros_adapters
  src/cooperative_traffic_ros.cpp
  src/execution_horizon_contract_ros.cpp
  src/latest_lidar_obstacle_scan_ros.cpp
  src/lidar_debug_node_config.cpp
  src/memory_provenance_audit_tracker.cpp
  src/obstacle_memory_provenance_ros.cpp
  src/px4_offboard_setpoint_io.cpp
  src/lidar_debug_pointclouds.cpp
  src/mppi_debug_markers.cpp
  src/ros_conversions.cpp
  src/raw_obstacle_delta.cpp
  src/raw_obstacle_3d_ros.cpp
  src/spectator_diagnostics_selection_ros.cpp
  src/static_map_debug.cpp
  src/visualization_marker_helpers.cpp)
target_compile_features(drone_city_nav_ros_adapters PUBLIC cxx_std_20)
target_link_libraries(drone_city_nav_ros_adapters drone_city_nav_runtime)
target_link_libraries(drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(
  drone_city_nav_ros_adapters
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs)
enable_project_warnings(drone_city_nav_ros_adapters)

add_executable(obstacle_memory_node src/obstacle_memory_node.cpp
                                    src/obstacle_memory_node_helpers.cpp
                                    src/obstacle_memory_transport.cpp
                                    src/obstacle_memory_transport_3d.cpp
                                    src/raw_world_snapshot.cpp)
target_compile_features(obstacle_memory_node PRIVATE cxx_std_20)
target_link_libraries(obstacle_memory_node drone_city_nav_core
                      drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(
  obstacle_memory_node
  ament_index_cpp
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs)
enable_project_warnings(obstacle_memory_node)

add_executable(obstacle_memory_3d_node src/obstacle_memory_3d_node.cpp
                                       src/obstacle_memory_3d_worker.cpp
                                       src/obstacle_memory_node_helpers.cpp
                                       src/obstacle_memory_transport_3d.cpp)
target_compile_features(obstacle_memory_3d_node PRIVATE cxx_std_20)
target_link_libraries(obstacle_memory_3d_node drone_city_nav_core
                      drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(
  obstacle_memory_3d_node
  ament_index_cpp
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs)
enable_project_warnings(obstacle_memory_3d_node)

add_executable(mppi_offboard_node src/mppi_offboard_node.cpp)
target_compile_features(mppi_offboard_node PRIVATE cxx_std_20)
target_link_libraries(mppi_offboard_node drone_city_nav_core
                      drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(
  mppi_offboard_node
  geometry_msgs
  px4_msgs
  rclcpp
  std_msgs
  tf2_ros
  visualization_msgs)
enable_project_warnings(mppi_offboard_node)

add_executable(collision_crash_node src/collision_crash_node.cpp)
target_compile_features(collision_crash_node PRIVATE cxx_std_20)
target_link_libraries(collision_crash_node drone_city_nav_core
                      ${cpp_typesupport_target})
ament_target_dependencies(
  collision_crash_node
  geometry_msgs
  px4_msgs
  rclcpp
  ros_gz_interfaces)
enable_project_warnings(collision_crash_node)

add_executable(mission_monitor_node src/mission_monitor_node.cpp)
target_compile_features(mission_monitor_node PRIVATE cxx_std_20)
target_link_libraries(mission_monitor_node drone_city_nav_core
                      drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(
  mission_monitor_node
  ament_index_cpp
  px4_msgs
  rclcpp)
enable_project_warnings(mission_monitor_node)

add_executable(intercept_mission_referee_node
               src/intercept_mission_referee_node.cpp
               src/intercept_mission_referee_lifecycle.cpp
               src/intercept_ground_truth_boundary.cpp
               src/intercept_referee_support.cpp)
target_compile_features(intercept_mission_referee_node PRIVATE cxx_std_20)
target_link_libraries(intercept_mission_referee_node drone_city_nav_core
                      drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(intercept_mission_referee_node rclcpp std_msgs)
enable_project_warnings(intercept_mission_referee_node)

add_executable(cooperative_traffic_referee_node
               src/cooperative_traffic_referee_main.cpp
               src/cooperative_traffic_referee_node.cpp
               src/cooperative_traffic_referee_lifecycle.cpp
               src/intercept_ground_truth_boundary.cpp)
target_compile_features(cooperative_traffic_referee_node PRIVATE cxx_std_20)
target_link_libraries(cooperative_traffic_referee_node drone_city_nav_core
                      drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(cooperative_traffic_referee_node rclcpp std_msgs
                          visualization_msgs)
enable_project_warnings(cooperative_traffic_referee_node)

add_executable(radar_simulator_node src/radar_simulator_node.cpp)
target_compile_features(radar_simulator_node PRIVATE cxx_std_20)
target_link_libraries(radar_simulator_node drone_city_nav_core
                      ${cpp_typesupport_target})
ament_target_dependencies(radar_simulator_node ament_index_cpp rclcpp)
enable_project_warnings(radar_simulator_node)

add_executable(simulation_truth_adapter_node
               src/simulation_truth_adapter_node.cpp)
target_compile_features(simulation_truth_adapter_node PRIVATE cxx_std_20)
target_link_libraries(
  simulation_truth_adapter_node
  drone_city_nav_core
  ${cpp_typesupport_target}
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
ament_target_dependencies(simulation_truth_adapter_node rclcpp)
enable_project_warnings(simulation_truth_adapter_node)

add_library(
  drone_city_nav_interceptor_tracking_components SHARED
  src/radar_target_tracker_node.cpp
  src/target_assignment_coordinator_node.cpp
  src/interceptor_guidance_node.cpp)
target_compile_features(drone_city_nav_interceptor_tracking_components
                        PRIVATE cxx_std_20)
target_link_libraries(drone_city_nav_interceptor_tracking_components
                      drone_city_nav_core ${cpp_typesupport_target})
ament_target_dependencies(
  drone_city_nav_interceptor_tracking_components
  rclcpp
  rclcpp_components
  std_msgs)
rclcpp_components_register_nodes(
  drone_city_nav_interceptor_tracking_components
  "drone_city_nav::RadarTargetTrackerNode"
  "drone_city_nav::TargetAssignmentCoordinatorNode"
  "drone_city_nav::InterceptorGuidanceNode")
enable_project_warnings(drone_city_nav_interceptor_tracking_components)

add_library(
  drone_city_nav_cooperative_components SHARED
  src/cooperative_traffic_agent_node.cpp)
target_compile_features(drone_city_nav_cooperative_components PRIVATE cxx_std_20)
target_link_libraries(
  drone_city_nav_cooperative_components
  drone_city_nav_core
  drone_city_nav_ros_adapters
  ${cpp_typesupport_target})
ament_target_dependencies(
  drone_city_nav_cooperative_components
  rclcpp
  rclcpp_components
  std_msgs)
rclcpp_components_register_nodes(
  drone_city_nav_cooperative_components
  "drone_city_nav::CooperativeTrafficAgentNode")
enable_project_warnings(drone_city_nav_cooperative_components)

add_executable(cooperative_traffic_agent_node
               src/cooperative_traffic_agent_main.cpp)
target_compile_features(cooperative_traffic_agent_node PRIVATE cxx_std_20)
target_link_libraries(cooperative_traffic_agent_node
                      drone_city_nav_cooperative_components)
ament_target_dependencies(cooperative_traffic_agent_node rclcpp)
enable_project_warnings(cooperative_traffic_agent_node)

add_executable(radar_target_tracker_node src/radar_target_tracker_main.cpp)
target_compile_features(radar_target_tracker_node PRIVATE cxx_std_20)
target_link_libraries(radar_target_tracker_node
                      drone_city_nav_interceptor_tracking_components)
ament_target_dependencies(radar_target_tracker_node rclcpp)
enable_project_warnings(radar_target_tracker_node)

add_executable(interceptor_guidance_node src/interceptor_guidance_main.cpp)
target_compile_features(interceptor_guidance_node PRIVATE cxx_std_20)
target_link_libraries(interceptor_guidance_node
                      drone_city_nav_interceptor_tracking_components)
ament_target_dependencies(interceptor_guidance_node rclcpp)
enable_project_warnings(interceptor_guidance_node)

add_library(
  drone_city_nav_diagnostics_components SHARED
  src/world_visualization_node.cpp
  src/intercept_spectator_node.cpp
  src/intercept_diagnostics_mux_node.cpp
  src/lidar_debug_component.cpp
  src/lidar_debug_node_callbacks.cpp
  src/lidar_debug_node_lifecycle.cpp
  src/lidar_debug_node_points.cpp
  src/lidar_debug_node_snapshot.cpp)
target_compile_features(drone_city_nav_diagnostics_components PRIVATE cxx_std_20)
target_link_libraries(
  drone_city_nav_diagnostics_components
  drone_city_nav_core
  drone_city_nav_ros_adapters
  ${cpp_typesupport_target})
ament_target_dependencies(
  drone_city_nav_diagnostics_components
  ament_index_cpp
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs)
rclcpp_components_register_nodes(
  drone_city_nav_diagnostics_components
  "drone_city_nav::WorldVisualizationNode"
  "drone_city_nav::InterceptSpectatorNode"
  "drone_city_nav::InterceptDiagnosticsMuxNode"
  "drone_city_nav::LidarDebugNode")
enable_project_warnings(drone_city_nav_diagnostics_components)

add_executable(world_visualization_node src/world_visualization_main.cpp)
target_compile_features(world_visualization_node PRIVATE cxx_std_20)
target_link_libraries(world_visualization_node
                      drone_city_nav_diagnostics_components)
ament_target_dependencies(world_visualization_node rclcpp)
enable_project_warnings(world_visualization_node)

add_executable(intercept_spectator_node src/intercept_spectator_main.cpp)
target_compile_features(intercept_spectator_node PRIVATE cxx_std_20)
target_link_libraries(intercept_spectator_node
                      drone_city_nav_diagnostics_components)
ament_target_dependencies(intercept_spectator_node rclcpp)
enable_project_warnings(intercept_spectator_node)

add_executable(intercept_diagnostics_mux_node
               src/intercept_diagnostics_mux_main.cpp)
target_compile_features(intercept_diagnostics_mux_node PRIVATE cxx_std_20)
target_link_libraries(intercept_diagnostics_mux_node
                      drone_city_nav_diagnostics_components)
ament_target_dependencies(intercept_diagnostics_mux_node rclcpp)
enable_project_warnings(intercept_diagnostics_mux_node)

add_executable(lidar_debug_node src/lidar_debug_node.cpp)
target_compile_features(lidar_debug_node PRIVATE cxx_std_20)
target_link_libraries(lidar_debug_node drone_city_nav_diagnostics_components)
ament_target_dependencies(lidar_debug_node rclcpp)
enable_project_warnings(lidar_debug_node)

if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  add_library(
    drone_city_nav_production_mppi_component SHARED
    src/runtime/ros/raw_world_ingress_ros_3d.cpp
    src/runtime/ros/production_mppi_config_ros.cpp
    src/runtime/ros/production_mppi_node.cpp
    src/runtime/ros/production_mppi_node_interfaces.cpp
    src/runtime/ros/production_mppi_node_health.cpp
    src/runtime/ros/production_mppi_node_mission.cpp
    src/runtime/ros/production_mppi_node_cooperative.cpp
    src/runtime/ros/production_mppi_node_noncooperative.cpp
    src/runtime/ros/production_mppi_node_esdf.cpp
    src/runtime/ros/production_mppi_node_observed_esdf.cpp
    src/runtime/ros/production_mppi_node_observed_evidence.cpp
    src/runtime/ros/production_mppi_node_inputs.cpp
    src/runtime/ros/production_mppi_node_navigation_input.cpp
    src/runtime/ros/production_mppi_node_control_feedback.cpp
    src/runtime/ros/production_mppi_node_inputs_3d.cpp
    src/runtime/ros/production_mppi_node_raw_input.cpp
    src/runtime/ros/production_mppi_node_planning_tick.cpp
    src/runtime/ros/production_mppi_node_planning_controller.cpp
    src/runtime/ros/production_mppi_node_world_generation.cpp
    src/runtime/ros/production_mppi_node_planning_tick_finalize.cpp
    src/runtime/ros/production_mppi_node_planning_tick_rearm.cpp
    src/runtime/ros/production_mppi_node_runtime.cpp
    src/runtime/ros/production_mppi_route_activation.cpp
    src/runtime/ros/production_mppi_node_static_extension.cpp
    src/runtime/ros/production_mppi_node_route_planning.cpp
    src/runtime/ros/production_mppi_node_route_lifecycle.cpp
    src/runtime/ros/production_mppi_node_diagnostics.cpp
    src/runtime/ros/production_mppi_node_diagnostics_events.cpp
    src/runtime/ros/production_mppi_node_diagnostics_runtime.cpp
    src/runtime/ros/production_mppi_node_summary.cpp
    src/runtime/ros/tracking_objective_diagnostics.cpp
    src/runtime/ros/production_mppi_node_execution.cpp
    src/runtime/ros/production_mppi_node_execution_publication.cpp
    src/runtime/ros/production_mppi_node_execution_holds.cpp
    src/runtime/ros/production_mppi_node_execution_retention.cpp)
  target_compile_features(drone_city_nav_production_mppi_component PRIVATE cxx_std_20)
  target_include_directories(
    drone_city_nav_production_mppi_component
    PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/runtime/ros>")
  target_link_libraries(
    drone_city_nav_production_mppi_component
    drone_city_nav_mppi_runtime
    drone_city_nav_ros_adapters
    ${cpp_typesupport_target})
  assert_drone_city_nav_target_links(
    drone_city_nav_production_mppi_component
    REQUIRED drone_city_nav_mppi_runtime drone_city_nav_ros_adapters
    FORBIDDEN drone_city_nav_core drone_city_nav_mppi_cuda
              drone_city_nav_world_runtime drone_city_nav_route_runtime)
  ament_target_dependencies(
    drone_city_nav_production_mppi_component
    ament_index_cpp
    nav_msgs
    px4_msgs
    rclcpp
    rclcpp_components
    std_msgs
    visualization_msgs)
  rclcpp_components_register_nodes(
    drone_city_nav_production_mppi_component
    "drone_city_nav::ProductionMppiNode")
  enable_project_warnings(drone_city_nav_production_mppi_component)

  add_executable(production_mppi_node src/runtime/ros/production_mppi_main.cpp)
  target_compile_features(production_mppi_node PRIVATE cxx_std_20)
  target_link_libraries(production_mppi_node
                        drone_city_nav_production_mppi_component)
  ament_target_dependencies(production_mppi_node rclcpp)
  enable_project_warnings(production_mppi_node)
  install(
    TARGETS production_mppi_node drone_city_nav_production_mppi_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(
  TARGETS
    drone_city_nav_model
    drone_city_nav_control_contracts
    drone_city_nav_route_contracts
    drone_city_nav_world
    drone_city_nav_collision
    drone_city_nav_finite_execution
    drone_city_nav_planning
    drone_city_nav_trajectory
    drone_city_nav_execution
    drone_city_nav_control
    drone_city_nav_runtime
    drone_city_nav_ros_adapters
    drone_contact_system
    collision_crash_node
    lidar_debug_node
    obstacle_memory_node
    obstacle_memory_3d_node
    mppi_offboard_node
    world_visualization_node
    mission_monitor_node
    intercept_mission_referee_node
    cooperative_traffic_referee_node
    radar_simulator_node
    simulation_truth_adapter_node
    radar_target_tracker_node
    interceptor_guidance_node
    cooperative_traffic_agent_node
    drone_city_nav_interceptor_tracking_components
    drone_city_nav_cooperative_components
    drone_city_nav_diagnostics_components
    intercept_spectator_node
    intercept_diagnostics_mux_node
    generate_static_esdf_cache
    free_space_topology_compiler
    voxelize_sdf_collisions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Hand-written navigation headers are package-private contracts between the
# enforced internal layers. ROS messages remain the supported external API and
# are installed/exported by rosidl_generate_interfaces().
install(DIRECTORY config launch models rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Compile every hand-written header as the first and only include in an
  # independent translation unit.
  function(drone_city_nav_header_check_sources output_variable)
    set(generated_sources "")
    foreach(header_path IN LISTS ARGN)
      if(NOT IS_ABSOLUTE "${header_path}")
        set(header_path "${CMAKE_CURRENT_SOURCE_DIR}/${header_path}")
      endif()
      file(RELATIVE_PATH header_relative_path "${CMAKE_CURRENT_SOURCE_DIR}"
           "${header_path}")
      string(MAKE_C_IDENTIFIER "${header_relative_path}" header_identifier)
      set(header_check_source
          "${CMAKE_CURRENT_BINARY_DIR}/header_self_containment/${header_identifier}.cpp"
      )
      file(
        GENERATE
        OUTPUT "${header_check_source}"
        CONTENT "#include \"${header_path}\"\n")
      list(APPEND generated_sources "${header_check_source}")
    endforeach()
    set(${output_variable} "${generated_sources}" PARENT_SCOPE)
  endfunction()

  # Each owning target compiles its own public headers against the include and
  # link closure of that target alone. A header that reaches past its layer at
  # link time fails here; one that reaches past it at include time fails in
  # test_navigation_dependency_contract.py.
  foreach(owned_layer IN LISTS DRONE_CITY_NAV_INTERNAL_LAYERS)
    string(REPLACE "drone_city_nav_" "" layer_suffix "${owned_layer}")
    string(TOUPPER "${layer_suffix}" layer_suffix_upper)
    drone_city_nav_header_check_sources(
      layer_header_sources ${DRONE_CITY_NAV_${layer_suffix_upper}_HEADERS})
    add_library(drone_city_nav_header_owner_${layer_suffix} OBJECT
                ${layer_header_sources})
    target_compile_features(drone_city_nav_header_owner_${layer_suffix}
                            PRIVATE cxx_std_20)
    target_link_libraries(drone_city_nav_header_owner_${layer_suffix}
                          ${owned_layer})
    enable_project_warnings(drone_city_nav_header_owner_${layer_suffix})
    target_compile_options(drone_city_nav_header_owner_${layer_suffix}
                           PRIVATE -Wno-unused-function)
  endforeach()

  # ROS adapters and private application headers keep the umbrella closure:
  # they are the composition root, not a domain layer.
  file(
    GLOB_RECURSE drone_city_nav_private_headers
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp")
  drone_city_nav_header_check_sources(
    drone_city_nav_header_check_sources_list
    ${drone_city_nav_private_headers} ${DRONE_CITY_NAV_ROS_HEADERS})
  add_library(drone_city_nav_header_self_containment OBJECT
              ${drone_city_nav_header_check_sources_list})
  target_compile_features(drone_city_nav_header_self_containment PRIVATE cxx_std_20)
  target_include_directories(
    drone_city_nav_header_self_containment
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${CMAKE_CURRENT_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/world"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/route_application"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/runtime"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/runtime/ros")
  target_link_libraries(
    drone_city_nav_header_self_containment
    drone_city_nav_core drone_city_nav_ros_adapters ${cpp_typesupport_target})
  if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
    target_link_libraries(drone_city_nav_header_self_containment
                          drone_city_nav_mppi_cuda)
  endif()
  ament_target_dependencies(
    drone_city_nav_header_self_containment
    ament_index_cpp
    geometry_msgs
    nav_msgs
    px4_msgs
    rclcpp
    rclcpp_components
    ros_gz_interfaces
    sensor_msgs
    std_msgs
    tf2_ros
    visualization_msgs)
  enable_project_warnings(drone_city_nav_header_self_containment)
  target_compile_options(drone_city_nav_header_self_containment
                         PRIVATE -Wno-unused-function)

  function(add_drone_city_nav_core_gtest target_name source_path)
    ament_add_gtest(${target_name} ${source_path})
    target_compile_features(${target_name} PRIVATE cxx_std_20)
    target_link_libraries(${target_name} drone_city_nav_core)
    enable_project_warnings(${target_name})
  endfunction()

  function(add_drone_city_nav_ros_adapter_gtest target_name source_path)
    ament_add_gtest(${target_name} ${source_path})
    target_compile_features(${target_name} PRIVATE cxx_std_20)
    target_link_libraries(${target_name} drone_city_nav_ros_adapters)
    set_tests_properties(
      ${target_name}
      PROPERTIES ENVIRONMENT
                 "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
    enable_project_warnings(${target_name})
  endfunction()

  function(add_drone_city_nav_private_runtime_gtest target_name source_path
           runtime_target)
    ament_add_gtest(${target_name} ${source_path})
    target_compile_features(${target_name} PRIVATE cxx_std_20)
    target_link_libraries(${target_name} ${runtime_target})
    enable_project_warnings(${target_name})
  endfunction()

  add_drone_city_nav_core_gtest(ambiguous_lidar_hit_tracker_test
                                tests/ambiguous_lidar_hit_tracker_test.cpp)
  add_drone_city_nav_core_gtest(debug_image_test tests/debug_image_test.cpp)
  add_drone_city_nav_ros_adapter_gtest(
    lidar_debug_node_config_test tests/lidar_debug_node_config_test.cpp)
  add_drone_city_nav_ros_adapter_gtest(
    lidar_debug_pointclouds_test tests/lidar_debug_pointclouds_test.cpp)
  add_drone_city_nav_core_gtest(lidar_debug_renderer_test
                                tests/lidar_debug_renderer_test.cpp)
  add_drone_city_nav_core_gtest(
    lidar_debug_snapshot_pipeline_test
    tests/lidar_debug_snapshot_pipeline_test.cpp)
  add_drone_city_nav_core_gtest(lidar_ingestion_decision_test
                                tests/lidar_ingestion_decision_test.cpp)
  add_drone_city_nav_core_gtest(lidar_memory_hit_diagnostics_test
                                tests/lidar_memory_hit_diagnostics_test.cpp)
  add_drone_city_nav_core_gtest(lidar_motion_compensation_test
                                tests/lidar_motion_compensation_test.cpp)
  add_drone_city_nav_core_gtest(lidar_snapshot_writer_test
                                tests/lidar_snapshot_writer_test.cpp)
  add_drone_city_nav_ros_adapter_gtest(
    obstacle_memory_provenance_ros_test
    tests/obstacle_memory_provenance_ros_test.cpp)

  add_drone_city_nav_private_runtime_gtest(
    production_mppi_route_world_test
    tests/production_mppi_route_world_test.cpp drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    world_pipeline_3d_test tests/world_pipeline_3d_test.cpp
    drone_city_nav_world_runtime)
  add_drone_city_nav_private_runtime_gtest(
    production_mppi_route_helpers_test
    tests/production_mppi_route_helpers_test.cpp drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    route_planner_3d_test tests/route_planner_3d_test.cpp
    drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    route_planning_coordinator_3d_test
    tests/route_planning_coordinator_3d_test.cpp drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    route_lifecycle_coordinator_3d_test
    tests/route_lifecycle_coordinator_3d_test.cpp drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    route_materializer_3d_test tests/route_materializer_3d_test.cpp
    drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    route_trajectory_compiler_3d_test
    tests/route_trajectory_compiler_3d_test.cpp drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    route_activation_coordinator_3d_test
    tests/route_activation_coordinator_3d_test.cpp drone_city_nav_route_runtime)
  add_drone_city_nav_private_runtime_gtest(
    route_execution_selector_3d_test
    tests/route_execution_selector_3d_test.cpp drone_city_nav_route_runtime)
  add_drone_city_nav_core_gtest(obstacle_memory_test
                                tests/obstacle_memory_test.cpp)
  add_drone_city_nav_core_gtest(px4_ros_time_mapper_test
                                tests/px4_ros_time_mapper_test.cpp)

  ament_add_gtest(trajectory_reference_adapter_3d_test
                  tests/trajectory_reference_adapter_3d_test.cpp)
  target_compile_features(trajectory_reference_adapter_3d_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_reference_adapter_3d_test
                        drone_city_nav_control)
  enable_project_warnings(trajectory_reference_adapter_3d_test)

  ament_add_gtest(intercept_mission_test tests/intercept_mission_test.cpp)
  target_compile_features(intercept_mission_test PRIVATE cxx_std_20)
  target_link_libraries(intercept_mission_test drone_city_nav_core)
  enable_project_warnings(intercept_mission_test)

  ament_add_gtest(intercept_guidance_test tests/intercept_guidance_test.cpp)
  target_compile_features(intercept_guidance_test PRIVATE cxx_std_20)
  target_link_libraries(intercept_guidance_test drone_city_nav_core)
  enable_project_warnings(intercept_guidance_test)

  ament_add_gtest(simulation_truth_alignment_test
                  tests/simulation_truth_alignment_test.cpp)
  target_compile_features(simulation_truth_alignment_test PRIVATE cxx_std_20)
  target_link_libraries(simulation_truth_alignment_test drone_city_nav_core)
  enable_project_warnings(simulation_truth_alignment_test)

  ament_add_gtest(map_to_sdf_transform_test
                  tests/map_to_sdf_transform_test.cpp)
  target_compile_features(map_to_sdf_transform_test PRIVATE cxx_std_20)
  target_include_directories(map_to_sdf_transform_test PRIVATE include)
  enable_project_warnings(map_to_sdf_transform_test)

  ament_add_gtest(navigation_pose_origin_test
                  tests/navigation_pose_origin_test.cpp)
  target_compile_features(navigation_pose_origin_test PRIVATE cxx_std_20)
  target_link_libraries(navigation_pose_origin_test drone_city_nav_core)
  enable_project_warnings(navigation_pose_origin_test)

  ament_add_gtest(direct_tracking_maneuver_lifecycle_test
                  tests/direct_tracking_maneuver_lifecycle_test.cpp)
  target_compile_features(direct_tracking_maneuver_lifecycle_test PRIVATE cxx_std_20)
  target_link_libraries(direct_tracking_maneuver_lifecycle_test drone_city_nav_core)
  enable_project_warnings(direct_tracking_maneuver_lifecycle_test)

  ament_add_gtest(noncooperative_collision_avoidance_test
                  tests/noncooperative_collision_avoidance_test.cpp)
  target_compile_features(noncooperative_collision_avoidance_test PRIVATE cxx_std_20)
  target_link_libraries(noncooperative_collision_avoidance_test drone_city_nav_core)
  enable_project_warnings(noncooperative_collision_avoidance_test)

  ament_add_gtest(tracking_objective_test tests/tracking_objective_test.cpp)
  target_compile_features(tracking_objective_test PRIVATE cxx_std_20)
  target_link_libraries(tracking_objective_test drone_city_nav_core)
  enable_project_warnings(tracking_objective_test)

  ament_add_gtest(radar_tracking_test tests/radar_tracking_test.cpp)
  target_compile_features(radar_tracking_test PRIVATE cxx_std_20)
  target_link_libraries(radar_tracking_test drone_city_nav_core)
  enable_project_warnings(radar_tracking_test)

  ament_add_gtest(target_assignment_test tests/target_assignment_test.cpp)
  target_compile_features(target_assignment_test PRIVATE cxx_std_20)
  target_link_libraries(target_assignment_test drone_city_nav_core)
  enable_project_warnings(target_assignment_test)

  ament_add_gtest(target_assignment_lifecycle_test
                  tests/target_assignment_lifecycle_test.cpp)
  target_compile_features(target_assignment_lifecycle_test PRIVATE cxx_std_20)
  target_link_libraries(
    target_assignment_lifecycle_test
    drone_city_nav_interceptor_tracking_components
    ${cpp_typesupport_target})
  ament_target_dependencies(target_assignment_lifecycle_test rclcpp)
  set_tests_properties(
    target_assignment_lifecycle_test
    PROPERTIES ENVIRONMENT
               "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
  enable_project_warnings(target_assignment_lifecycle_test)

  ament_add_gtest(tracked_agent_lidar_filter_test
                  tests/tracked_agent_lidar_filter_test.cpp)
  target_compile_features(tracked_agent_lidar_filter_test PRIVATE cxx_std_20)
  target_link_libraries(tracked_agent_lidar_filter_test drone_city_nav_core)
  enable_project_warnings(tracked_agent_lidar_filter_test)

  ament_add_gtest(latest_lidar_obstacle_scan_test
                  tests/latest_lidar_obstacle_scan_test.cpp)
  target_compile_features(latest_lidar_obstacle_scan_test PRIVATE cxx_std_20)
  target_link_libraries(latest_lidar_obstacle_scan_test drone_city_nav_core)
  enable_project_warnings(latest_lidar_obstacle_scan_test)

  ament_add_gtest(mppi_reference_test_current tests/mppi_reference_test.cpp)
  target_compile_features(mppi_reference_test_current PRIVATE cxx_std_20)
  target_include_directories(mppi_reference_test_current PRIVATE include)
  target_link_libraries(mppi_reference_test_current drone_city_nav_core)
  enable_project_warnings(mppi_reference_test_current)

  ament_add_gtest(esdf_query_test tests/esdf_query_test.cpp)
  target_compile_features(esdf_query_test PRIVATE cxx_std_20)
  target_link_libraries(esdf_query_test drone_city_nav_core)
  enable_project_warnings(esdf_query_test)

  ament_add_gtest(local_esdf_2d_test tests/local_esdf_2d_test.cpp)
  target_compile_features(local_esdf_2d_test PRIVATE cxx_std_20)
  target_link_libraries(local_esdf_2d_test drone_city_nav_core)
  enable_project_warnings(local_esdf_2d_test)

  ament_add_gtest(raw_obstacle_delta_test tests/raw_obstacle_delta_test.cpp)
  target_compile_features(raw_obstacle_delta_test PRIVATE cxx_std_20)
  target_link_libraries(raw_obstacle_delta_test drone_city_nav_ros_adapters)
  enable_project_warnings(raw_obstacle_delta_test)

  ament_add_gtest(execution_horizon_contract_ros_test
                  tests/execution_horizon_contract_ros_test.cpp)
  target_compile_features(execution_horizon_contract_ros_test PRIVATE cxx_std_20)
  target_link_libraries(execution_horizon_contract_ros_test
                        drone_city_nav_ros_adapters)
  enable_project_warnings(execution_horizon_contract_ros_test)

  ament_add_gtest(flight_envelope_test tests/flight_envelope_test.cpp)
  target_compile_features(flight_envelope_test PRIVATE cxx_std_20)
  target_link_libraries(flight_envelope_test drone_city_nav_core)
  enable_project_warnings(flight_envelope_test)

  ament_add_gtest(swept_footprint_test tests/swept_footprint_test.cpp)
  target_compile_features(swept_footprint_test PRIVATE cxx_std_20)
  target_link_libraries(swept_footprint_test drone_city_nav_core)
  enable_project_warnings(swept_footprint_test)

  ament_add_gtest(occupied_collision_oracle_3d_test
                  tests/occupied_collision_oracle_3d_test.cpp)
  target_compile_features(occupied_collision_oracle_3d_test PRIVATE cxx_std_20)
  target_link_libraries(occupied_collision_oracle_3d_test drone_city_nav_core)
  enable_project_warnings(occupied_collision_oracle_3d_test)

  ament_add_gtest(finite_execution_path_test
                  tests/finite_execution_path_test.cpp)
  target_compile_features(finite_execution_path_test PRIVATE cxx_std_20)
  target_link_libraries(finite_execution_path_test drone_city_nav_core)
  enable_project_warnings(finite_execution_path_test)

  ament_add_gtest(mppi_liveness_test_current tests/mppi_liveness_test.cpp)
  target_compile_features(mppi_liveness_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_liveness_test_current drone_city_nav_core)
  enable_project_warnings(mppi_liveness_test_current)

  ament_add_gtest(mission_goal_capture_test tests/mission_goal_capture_test.cpp)
  target_compile_features(mission_goal_capture_test PRIVATE cxx_std_20)
  target_link_libraries(mission_goal_capture_test drone_city_nav_core)
  enable_project_warnings(mission_goal_capture_test)

  ament_add_gtest(mission_waypoint_sequence_test
                  tests/mission_waypoint_sequence_test.cpp)
  target_compile_features(mission_waypoint_sequence_test PRIVATE cxx_std_20)
  target_link_libraries(mission_waypoint_sequence_test drone_city_nav_core)
  enable_project_warnings(mission_waypoint_sequence_test)

  ament_add_gtest(mission_waypoint_capture_gate_test
                  tests/mission_waypoint_capture_gate_test.cpp)
  target_compile_features(mission_waypoint_capture_gate_test PRIVATE cxx_std_20)
  target_link_libraries(mission_waypoint_capture_gate_test drone_city_nav_core)
  enable_project_warnings(mission_waypoint_capture_gate_test)

  ament_add_gtest(mission_waypoint_acknowledgement_admission_test
                  tests/mission_waypoint_acknowledgement_admission_test.cpp)
  target_compile_features(mission_waypoint_acknowledgement_admission_test
                          PRIVATE cxx_std_20)
  target_link_libraries(mission_waypoint_acknowledgement_admission_test
                        drone_city_nav_core)
  enable_project_warnings(mission_waypoint_acknowledgement_admission_test)

  ament_add_gtest(mission_waypoint_acknowledgement_transport_test
                  tests/mission_waypoint_acknowledgement_transport_test.cpp)
  target_compile_features(mission_waypoint_acknowledgement_transport_test
                          PRIVATE cxx_std_20)
  target_link_libraries(mission_waypoint_acknowledgement_transport_test
                        drone_city_nav_core ${cpp_typesupport_target})
  ament_target_dependencies(mission_waypoint_acknowledgement_transport_test rclcpp)
  set_tests_properties(
    mission_waypoint_acknowledgement_transport_test
    PROPERTIES ENVIRONMENT
               "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
  enable_project_warnings(mission_waypoint_acknowledgement_transport_test)

  ament_add_gtest(mapping_lifecycle_test tests/mapping_lifecycle_test.cpp)
  target_compile_features(mapping_lifecycle_test PRIVATE cxx_std_20)
  target_link_libraries(mapping_lifecycle_test drone_city_nav_core)
  enable_project_warnings(mapping_lifecycle_test)

  ament_add_gtest(navigation_state_prediction_test
                  tests/navigation_state_prediction_test.cpp)
  target_compile_features(navigation_state_prediction_test PRIVATE cxx_std_20)
  target_link_libraries(navigation_state_prediction_test drone_city_nav_core)
  enable_project_warnings(navigation_state_prediction_test)

  ament_add_gtest(navigation_health_supervisor_test
                  tests/navigation_health_supervisor_test.cpp)
  target_compile_features(navigation_health_supervisor_test PRIVATE cxx_std_20)
  target_link_libraries(navigation_health_supervisor_test drone_city_nav_core)
  enable_project_warnings(navigation_health_supervisor_test)

  ament_add_gtest(json_output_test tests/json_output_test.cpp)
  target_compile_features(json_output_test PRIVATE cxx_std_20)
  target_include_directories(json_output_test PRIVATE include)
  enable_project_warnings(json_output_test)

  ament_add_gtest(navigation_angular_derivative_test
                  tests/navigation_angular_derivative_test.cpp)
  target_compile_features(navigation_angular_derivative_test PRIVATE cxx_std_20)
  target_link_libraries(navigation_angular_derivative_test drone_city_nav_core)
  enable_project_warnings(navigation_angular_derivative_test)

  ament_add_gtest(latest_value_mailbox_test
                  tests/latest_value_mailbox_test.cpp)
  target_compile_features(latest_value_mailbox_test PRIVATE cxx_std_20)
  target_include_directories(latest_value_mailbox_test PRIVATE include)
  enable_project_warnings(latest_value_mailbox_test)

  ament_add_gtest(latest_processing_queue_test
                  tests/latest_processing_queue_test.cpp)
  target_compile_features(latest_processing_queue_test PRIVATE cxx_std_20)
  target_include_directories(latest_processing_queue_test PRIVATE include)
  enable_project_warnings(latest_processing_queue_test)

  ament_add_gtest(world_generation_test tests/world_generation_test.cpp)
  target_compile_features(world_generation_test PRIVATE cxx_std_20)
  target_link_libraries(world_generation_test drone_city_nav_core)
  enable_project_warnings(world_generation_test)

  ament_add_gtest(bounded_worker_pool_test
                  tests/bounded_worker_pool_test.cpp)
  target_compile_features(bounded_worker_pool_test PRIVATE cxx_std_20)
  target_link_libraries(bounded_worker_pool_test drone_city_nav_core)
  enable_project_warnings(bounded_worker_pool_test)

  ament_add_gtest(mppi_post_update_classification_test
                  tests/mppi_post_update_classification_test.cpp)
  target_compile_features(mppi_post_update_classification_test PRIVATE cxx_std_20)
  target_link_libraries(mppi_post_update_classification_test drone_city_nav_core)
  enable_project_warnings(mppi_post_update_classification_test)

  ament_add_gtest(mppi_speed_policy_test_current tests/mppi_speed_policy_test.cpp)
  target_compile_features(mppi_speed_policy_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_speed_policy_test_current drone_city_nav_core)
  enable_project_warnings(mppi_speed_policy_test_current)

  ament_add_gtest(sensor_braking_contract_3d_test
                  tests/sensor_braking_contract_3d_test.cpp)
  target_compile_features(sensor_braking_contract_3d_test PRIVATE cxx_std_20)
  target_link_libraries(sensor_braking_contract_3d_test drone_city_nav_core)
  enable_project_warnings(sensor_braking_contract_3d_test)

  ament_add_gtest(route_time_parameterization_test tests/route_time_parameterization_test.cpp)
  target_compile_features(route_time_parameterization_test PRIVATE cxx_std_20)
  target_link_libraries(route_time_parameterization_test drone_city_nav_core)
  enable_project_warnings(route_time_parameterization_test)

  ament_add_gtest(tracking_error_tube_3d_test tests/tracking_error_tube_3d_test.cpp)
  target_compile_features(tracking_error_tube_3d_test PRIVATE cxx_std_20)
  target_link_libraries(tracking_error_tube_3d_test drone_city_nav_core)
  enable_project_warnings(tracking_error_tube_3d_test)

  ament_add_gtest(mppi_rollout_budget_test tests/mppi_rollout_budget_test.cpp)
  target_compile_features(mppi_rollout_budget_test PRIVATE cxx_std_20)
  target_link_libraries(mppi_rollout_budget_test drone_city_nav_core)
  enable_project_warnings(mppi_rollout_budget_test)

  ament_add_gtest(mppi_horizon_sampling_test
                  tests/mppi_horizon_sampling_test.cpp)
  target_compile_features(mppi_horizon_sampling_test PRIVATE cxx_std_20)
  target_include_directories(mppi_horizon_sampling_test PRIVATE include)
  enable_project_warnings(mppi_horizon_sampling_test)

  if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
    ament_add_gtest(mppi_control_sequence_test_current
                    tests/mppi_control_sequence_test.cpp)
    target_compile_features(mppi_control_sequence_test_current PRIVATE cxx_std_20)
    target_link_libraries(mppi_control_sequence_test_current
                          drone_city_nav_mppi_cuda)
    enable_project_warnings(mppi_control_sequence_test_current)

    ament_add_gtest(mppi_finite_horizon_test
                    tests/mppi_finite_horizon_test.cpp)
    target_compile_features(mppi_finite_horizon_test PRIVATE cxx_std_20)
    target_link_libraries(mppi_finite_horizon_test drone_city_nav_mppi_cuda)
    enable_project_warnings(mppi_finite_horizon_test)

    ament_add_gtest(mppi_noncooperative_acquisition_test
                    tests/mppi_noncooperative_acquisition_test.cpp)
    target_compile_features(mppi_noncooperative_acquisition_test PRIVATE cxx_std_20)
    target_link_libraries(mppi_noncooperative_acquisition_test
                          drone_city_nav_mppi_cuda)
    enable_project_warnings(mppi_noncooperative_acquisition_test)

    ament_add_gtest(static_route_handoff_test
                    tests/static_route_handoff_test.cpp)
    target_compile_features(static_route_handoff_test PRIVATE cxx_std_20)
    target_link_libraries(static_route_handoff_test drone_city_nav_mppi_cuda)
    enable_project_warnings(static_route_handoff_test)

    add_drone_city_nav_private_runtime_gtest(
      planning_cycle_coordinator_3d_test
      tests/planning_cycle_coordinator_3d_test.cpp drone_city_nav_mppi_runtime)
    add_drone_city_nav_private_runtime_gtest(
      execution_horizon_assembler_3d_test
      tests/execution_horizon_assembler_3d_test.cpp drone_city_nav_mppi_runtime)
    add_drone_city_nav_private_runtime_gtest(
      mppi_controller_3d_test tests/mppi_controller_3d_test.cpp
      drone_city_nav_mppi_runtime)
    add_drone_city_nav_private_runtime_gtest(
      navigation_diagnostics_sink_test
      tests/navigation_diagnostics_sink_test.cpp drone_city_nav_mppi_runtime)
    add_drone_city_nav_private_runtime_gtest(
      production_mppi_execution_control_runtime_test
      tests/production_mppi_execution_control_runtime_test.cpp
      drone_city_nav_mppi_runtime)

    ament_add_gtest(raw_world_ingress_ros_3d_test
                    tests/raw_world_ingress_ros_3d_test.cpp)
    target_compile_features(raw_world_ingress_ros_3d_test PRIVATE cxx_std_20)
    target_include_directories(raw_world_ingress_ros_3d_test
                               PRIVATE src/runtime/ros)
    target_link_libraries(raw_world_ingress_ros_3d_test
                          drone_city_nav_production_mppi_component)
    enable_project_warnings(raw_world_ingress_ros_3d_test)

    ament_add_gtest(production_mppi_config_test
                    tests/production_mppi_config_test.cpp)
    target_compile_features(production_mppi_config_test PRIVATE cxx_std_20)
    target_include_directories(production_mppi_config_test
                               PRIVATE src/runtime/ros)
    target_link_libraries(production_mppi_config_test
                          drone_city_nav_production_mppi_component)
    enable_project_warnings(production_mppi_config_test)
  endif()

  ament_add_gtest(route_progress_3d_test
                  tests/route_progress_3d_test.cpp)
  target_compile_features(route_progress_3d_test PRIVATE cxx_std_20)
  target_link_libraries(route_progress_3d_test drone_city_nav_core)
  enable_project_warnings(route_progress_3d_test)

  ament_add_gtest(planning_contract_test tests/planning_contract_test.cpp)
  target_compile_features(planning_contract_test PRIVATE cxx_std_20)
  target_link_libraries(planning_contract_test drone_city_nav_core)
  enable_project_warnings(planning_contract_test)

  ament_add_gtest(vehicle_destruction_disarm_lifecycle_test
                  tests/vehicle_destruction_disarm_lifecycle_test.cpp)
  target_compile_features(vehicle_destruction_disarm_lifecycle_test PRIVATE cxx_std_20)
  target_link_libraries(vehicle_destruction_disarm_lifecycle_test drone_city_nav_core)
  enable_project_warnings(vehicle_destruction_disarm_lifecycle_test)

  ament_add_gtest(distance_field_test_current tests/distance_field_test.cpp)
  target_compile_features(distance_field_test_current PRIVATE cxx_std_20)
  target_link_libraries(distance_field_test_current drone_city_nav_core)
  enable_project_warnings(distance_field_test_current)

  ament_add_gtest(occupancy_grid_3d_test tests/occupancy_grid_3d_test.cpp)
  target_compile_features(occupancy_grid_3d_test PRIVATE cxx_std_20)
  target_link_libraries(occupancy_grid_3d_test drone_city_nav_core)
  target_compile_definitions(
    occupancy_grid_3d_test
    PRIVATE
      TEST_COMPACT_PASSAGE_OCCUPANCY3D_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../environments/fixtures/compact_3d_passage/world.occupancy3d")
  enable_project_warnings(occupancy_grid_3d_test)

  ament_add_gtest(persistent_dstar_lite_planner_3d_test
                  tests/persistent_dstar_lite_planner_3d_test.cpp)
  target_compile_features(persistent_dstar_lite_planner_3d_test PRIVATE cxx_std_20)
  target_link_libraries(persistent_dstar_lite_planner_3d_test drone_city_nav_core)
  enable_project_warnings(persistent_dstar_lite_planner_3d_test)

  ament_add_gtest(
    persistent_planner_acceptance_fixture_test
    tests/persistent_planner_acceptance_fixture.cpp
    tests/persistent_planner_acceptance_fixture_test.cpp)
  target_compile_features(persistent_planner_acceptance_fixture_test PRIVATE cxx_std_20)
  target_link_libraries(persistent_planner_acceptance_fixture_test drone_city_nav_core)
  enable_project_warnings(persistent_planner_acceptance_fixture_test)

  ament_add_gtest(obstacle_memory_3d_test tests/obstacle_memory_3d_test.cpp)
  target_compile_features(obstacle_memory_3d_test PRIVATE cxx_std_20)
  target_link_libraries(obstacle_memory_3d_test drone_city_nav_core)
  enable_project_warnings(obstacle_memory_3d_test)

  ament_add_gtest(obstacle_memory_transport_policy_3d_test
                  tests/obstacle_memory_transport_policy_3d_test.cpp)
  target_compile_features(obstacle_memory_transport_policy_3d_test PRIVATE cxx_std_20)
  target_link_libraries(obstacle_memory_transport_policy_3d_test
                        drone_city_nav_core)
  enable_project_warnings(obstacle_memory_transport_policy_3d_test)

  ament_add_gtest(lidar_scan_3d_test tests/lidar_scan_3d_test.cpp)
  target_compile_features(lidar_scan_3d_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_scan_3d_test drone_city_nav_core)
  enable_project_warnings(lidar_scan_3d_test)

  ament_add_gtest(lidar_self_filter_test tests/lidar_self_filter_test.cpp)
  target_compile_features(lidar_self_filter_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_self_filter_test drone_city_nav_core)
  enable_project_warnings(lidar_self_filter_test)

  ament_add_gtest(raw_obstacle_3d_ros_test tests/raw_obstacle_3d_ros_test.cpp)
  target_compile_features(raw_obstacle_3d_ros_test PRIVATE cxx_std_20)
  target_link_libraries(raw_obstacle_3d_ros_test drone_city_nav_ros_adapters)
  enable_project_warnings(raw_obstacle_3d_ros_test)

  ament_add_gtest(free_space_topology_3d_test
                  tests/free_space_topology_3d_test.cpp)
  target_compile_features(free_space_topology_3d_test PRIVATE cxx_std_20)
  target_link_libraries(free_space_topology_3d_test drone_city_nav_core)
  target_compile_definitions(
    free_space_topology_3d_test
    PRIVATE
      TEST_OCCUPANCY3D_PATH="${CMAKE_CURRENT_SOURCE_DIR}/worlds/generated_city.occupancy3d"
      TEST_FREE_SPACE_TOPOLOGY3D_PATH="${CMAKE_CURRENT_SOURCE_DIR}/worlds/generated_city.topology3d")
  enable_project_warnings(free_space_topology_3d_test)

  ament_add_gtest(
    advanced_passage_fixture_test
    tests/advanced_passage_fixture.cpp
    tests/advanced_passage_fixture_test.cpp)
  target_compile_features(advanced_passage_fixture_test PRIVATE cxx_std_20)
  target_link_libraries(advanced_passage_fixture_test drone_city_nav_core)
  enable_project_warnings(advanced_passage_fixture_test)

  ament_add_gtest(
    free_space_topology_extractor_3d_test
    tests/advanced_passage_fixture.cpp
    tests/free_space_topology_extractor_3d_test.cpp)
  target_compile_features(free_space_topology_extractor_3d_test PRIVATE cxx_std_20)
  target_link_libraries(free_space_topology_extractor_3d_test drone_city_nav_core)
  enable_project_warnings(free_space_topology_extractor_3d_test)

  ament_add_gtest(collision_voxelizer_test tests/collision_voxelizer_test.cpp)
  target_compile_features(collision_voxelizer_test PRIVATE cxx_std_20)
  target_link_libraries(collision_voxelizer_test drone_city_nav_core)
  enable_project_warnings(collision_voxelizer_test)

  ament_add_gtest(passage_volume_test tests/passage_volume_test.cpp)
  target_compile_features(passage_volume_test PRIVATE cxx_std_20)
  target_link_libraries(passage_volume_test drone_city_nav_core)
  enable_project_warnings(passage_volume_test)

  ament_add_gtest(cooperative_passage_route_test
                  tests/cooperative_passage_route_test.cpp)
  target_compile_features(cooperative_passage_route_test PRIVATE cxx_std_20)
  target_link_libraries(cooperative_passage_route_test drone_city_nav_core)
  enable_project_warnings(cooperative_passage_route_test)

  ament_add_gtest(cooperative_passage_execution_test
                  tests/cooperative_passage_execution_test.cpp)
  target_compile_features(cooperative_passage_execution_test PRIVATE cxx_std_20)
  target_link_libraries(cooperative_passage_execution_test drone_city_nav_core)
  enable_project_warnings(cooperative_passage_execution_test)

  ament_add_gtest(cooperative_traffic_test tests/cooperative_traffic_test.cpp)
  target_compile_features(cooperative_traffic_test PRIVATE cxx_std_20)
  target_link_libraries(cooperative_traffic_test drone_city_nav_core)
  enable_project_warnings(cooperative_traffic_test)

  ament_add_gtest(cooperative_space_time_test
                  tests/cooperative_space_time_test.cpp)
  target_compile_features(cooperative_space_time_test PRIVATE cxx_std_20)
  target_link_libraries(cooperative_space_time_test drone_city_nav_core)
  enable_project_warnings(cooperative_space_time_test)

  ament_add_gtest(cooperative_traffic_mission_test
                  tests/cooperative_traffic_mission_test.cpp)
  target_compile_features(cooperative_traffic_mission_test PRIVATE cxx_std_20)
  target_link_libraries(cooperative_traffic_mission_test drone_city_nav_core)
  enable_project_warnings(cooperative_traffic_mission_test)

  ament_add_gtest(cooperative_mppi_adapter_test
                  tests/cooperative_mppi_adapter_test.cpp)
  target_compile_features(cooperative_mppi_adapter_test PRIVATE cxx_std_20)
  target_link_libraries(cooperative_mppi_adapter_test drone_city_nav_core)
  enable_project_warnings(cooperative_mppi_adapter_test)

  ament_add_gtest(distance_field_3d_test tests/distance_field_3d_test.cpp)
  target_compile_features(distance_field_3d_test PRIVATE cxx_std_20)
  target_link_libraries(distance_field_3d_test drone_city_nav_core)
  enable_project_warnings(distance_field_3d_test)

  ament_add_gtest(known_obstacle_distance_3d_test
                  tests/known_obstacle_distance_3d_test.cpp)
  target_compile_features(known_obstacle_distance_3d_test PRIVATE cxx_std_20)
  target_link_libraries(known_obstacle_distance_3d_test drone_city_nav_core)
  enable_project_warnings(known_obstacle_distance_3d_test)

  ament_add_gtest(observed_esdf_3d_test tests/observed_esdf_3d_test.cpp)
  target_compile_features(observed_esdf_3d_test PRIVATE cxx_std_20)
  target_link_libraries(observed_esdf_3d_test drone_city_nav_core)
  enable_project_warnings(observed_esdf_3d_test)

  ament_add_gtest(static_esdf_cache_test tests/static_esdf_cache_test.cpp)
  target_compile_features(static_esdf_cache_test PRIVATE cxx_std_20)
  target_link_libraries(static_esdf_cache_test drone_city_nav_core)
  target_compile_definitions(
    static_esdf_cache_test
    PRIVATE
      TEST_COMPACT_PASSAGE_OCCUPANCY3D_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../environments/fixtures/compact_3d_passage/world.occupancy3d"
      TEST_COMPACT_PASSAGE_ESDF3D_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../environments/fixtures/compact_3d_passage/world.esdf3d")
  enable_project_warnings(static_esdf_cache_test)

  ament_add_gtest(route_3d_test tests/route_3d_test.cpp)
  target_compile_features(route_3d_test PRIVATE cxx_std_20)
  target_include_directories(route_3d_test PRIVATE src)
  target_link_libraries(route_3d_test drone_city_nav_core)
  enable_project_warnings(route_3d_test)

  ament_add_gtest(passage_traversal_selection_3d_test
                  tests/passage_traversal_selection_3d_test.cpp)
  target_compile_features(passage_traversal_selection_3d_test PRIVATE cxx_std_20)
  target_link_libraries(passage_traversal_selection_3d_test drone_city_nav_core)
  enable_project_warnings(passage_traversal_selection_3d_test)

  ament_add_gtest(route_planning_3d_test tests/route_planning_3d_test.cpp)
  target_compile_features(route_planning_3d_test PRIVATE cxx_std_20)
  target_link_libraries(route_planning_3d_test drone_city_nav_core)
  enable_project_warnings(route_planning_3d_test)

  ament_add_gtest(route_lifecycle_3d_test tests/route_lifecycle_3d_test.cpp)
  target_compile_features(route_lifecycle_3d_test PRIVATE cxx_std_20)
  target_link_libraries(route_lifecycle_3d_test drone_city_nav_core)
  enable_project_warnings(route_lifecycle_3d_test)

  ament_add_gtest(route_successor_improvement_3d_test
                  tests/route_successor_improvement_3d_test.cpp)
  target_compile_features(route_successor_improvement_3d_test PRIVATE cxx_std_20)
  target_link_libraries(route_successor_improvement_3d_test drone_city_nav_core)
  enable_project_warnings(route_successor_improvement_3d_test)

  ament_add_gtest(trajectory_compiler_3d_test tests/trajectory_compiler_3d_test.cpp)
  target_compile_features(trajectory_compiler_3d_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_compiler_3d_test drone_city_nav_core)
  enable_project_warnings(trajectory_compiler_3d_test)

  # Each contract is compiled as the first include in an independent
  # translation unit so another header cannot hide missing includes.
  ament_add_gtest(
    execution_route_public_headers_test
    tests/execution_route_public_headers_test.cpp
    tests/execution_route_model_header_compile.cpp
    tests/execution_route_certificates_header_compile.cpp
    tests/execution_plan_header_compile.cpp
    tests/execution_hold_header_compile.cpp
    tests/execution_horizon_commit_header_compile.cpp
    tests/execution_retention_header_compile.cpp
    tests/execution_route_certification_header_compile.cpp
    tests/execution_route_transitions_header_compile.cpp
    tests/execution_route_store_header_compile.cpp
    tests/execution_supervisor_header_compile.cpp)
  target_compile_features(execution_route_public_headers_test PRIVATE cxx_std_20)
  target_link_libraries(execution_route_public_headers_test drone_city_nav_execution)
  enable_project_warnings(execution_route_public_headers_test)

  ament_add_gtest(execution_route_snapshot_3d_test
                  tests/committed_execution_authority_3d_test.cpp
                  tests/execution_plan_reducer_3d_test.cpp
                  tests/execution_route_snapshot_3d_test.cpp
                  tests/execution_route_snapshot_3d_certification_test.cpp
                  tests/execution_route_snapshot_3d_edge_cases_test.cpp
                  tests/execution_route_snapshot_3d_lifecycle_braking_test.cpp
                  tests/execution_route_snapshot_3d_optional_constraints_test.cpp
                  tests/execution_route_snapshot_3d_plan_test.cpp
                  tests/execution_route_snapshot_3d_successor_evidence_test.cpp
                  tests/execution_route_snapshot_3d_tracking_tube_test.cpp
                  tests/execution_route_snapshot_3d_transitions_test.cpp
                  tests/execution_route_snapshot_3d_validation_test.cpp
                  tests/execution_route_snapshot_3d_world_transition_test.cpp)
  target_compile_features(execution_route_snapshot_3d_test PRIVATE cxx_std_20)
  target_link_libraries(execution_route_snapshot_3d_test drone_city_nav_core)
  enable_project_warnings(execution_route_snapshot_3d_test)

  ament_add_gtest(
    route_execution_manager_pending_publication_test
    tests/route_execution_manager_pending_publication_test.cpp)
  target_compile_features(route_execution_manager_pending_publication_test
                          PRIVATE cxx_std_20)
  target_link_libraries(route_execution_manager_pending_publication_test
                        drone_city_nav_core)
  enable_project_warnings(route_execution_manager_pending_publication_test)

  ament_add_gtest(execution_supervisor_3d_test
                  tests/execution_supervisor_3d_test.cpp)
  target_compile_features(execution_supervisor_3d_test PRIVATE cxx_std_20)
  target_link_libraries(execution_supervisor_3d_test drone_city_nav_execution)
  enable_project_warnings(execution_supervisor_3d_test)

  ament_add_gtest(execution_supervisor_retention_3d_test
                  tests/execution_supervisor_retention_3d_test.cpp)
  target_compile_features(execution_supervisor_retention_3d_test PRIVATE cxx_std_20)
  target_link_libraries(execution_supervisor_retention_3d_test
                        drone_city_nav_execution)
  enable_project_warnings(execution_supervisor_retention_3d_test)

  ament_add_gtest(execution_supervisor_hold_3d_test
                  tests/execution_supervisor_hold_3d_test.cpp)
  target_compile_features(execution_supervisor_hold_3d_test PRIVATE cxx_std_20)
  target_link_libraries(execution_supervisor_hold_3d_test
                        drone_city_nav_execution)
  enable_project_warnings(execution_supervisor_hold_3d_test)

  ament_add_gtest(execution_supervisor_horizon_3d_test
                  tests/execution_supervisor_horizon_3d_test.cpp)
  target_compile_features(execution_supervisor_horizon_3d_test PRIVATE cxx_std_20)
  target_link_libraries(execution_supervisor_horizon_3d_test
                        drone_city_nav_execution)
  enable_project_warnings(execution_supervisor_horizon_3d_test)

  ament_add_gtest(execution_evidence_3d_test
                  tests/execution_evidence_3d_test.cpp)
  target_compile_features(execution_evidence_3d_test PRIVATE cxx_std_20)
  target_link_libraries(execution_evidence_3d_test drone_city_nav_core)
  enable_project_warnings(execution_evidence_3d_test)

  ament_add_gtest(execution_horizon_admission_test
                  tests/execution_horizon_admission_test.cpp)
  target_compile_features(execution_horizon_admission_test PRIVATE cxx_std_20)
  target_link_libraries(execution_horizon_admission_test drone_city_nav_core)
  enable_project_warnings(execution_horizon_admission_test)

  ament_add_gtest(execution_horizon_witness_test
                  tests/execution_horizon_witness_test.cpp)
  target_compile_features(execution_horizon_witness_test PRIVATE cxx_std_20)
  target_link_libraries(execution_horizon_witness_test drone_city_nav_core)
  enable_project_warnings(execution_horizon_witness_test)

  ament_add_gtest(applied_control_admission_test
                  tests/applied_control_admission_test.cpp)
  target_compile_features(applied_control_admission_test PRIVATE cxx_std_20)
  target_link_libraries(applied_control_admission_test drone_city_nav_core)
  enable_project_warnings(applied_control_admission_test)

  ament_add_gtest(production_mppi_execution_control_test
                  tests/production_mppi_execution_control_test.cpp)
  target_compile_features(production_mppi_execution_control_test PRIVATE cxx_std_20)
  target_include_directories(production_mppi_execution_control_test
                             PRIVATE src/route_application)
  target_link_libraries(production_mppi_execution_control_test
                        drone_city_nav_execution)
  enable_project_warnings(production_mppi_execution_control_test)

  ament_add_gtest(execution_horizon_timing_test
                  tests/execution_horizon_timing_test.cpp)
  target_compile_features(execution_horizon_timing_test PRIVATE cxx_std_20)
  target_link_libraries(execution_horizon_timing_test drone_city_nav_core)
  enable_project_warnings(execution_horizon_timing_test)

  ament_add_gtest(producer_instance_id_test
                  tests/producer_instance_id_test.cpp)
  target_compile_features(producer_instance_id_test PRIVATE cxx_std_20)
  target_link_libraries(producer_instance_id_test drone_city_nav_core)
  enable_project_warnings(producer_instance_id_test)

  ament_add_gtest(producer_epoch_admission_test
                  tests/producer_epoch_admission_test.cpp)
  target_compile_features(producer_epoch_admission_test PRIVATE cxx_std_20)
  target_link_libraries(producer_epoch_admission_test drone_city_nav_core)
  enable_project_warnings(producer_epoch_admission_test)

  ament_add_gtest(offboard_session_admission_test
                  tests/offboard_session_admission_test.cpp)
  target_compile_features(offboard_session_admission_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_session_admission_test drone_city_nav_core)
  enable_project_warnings(offboard_session_admission_test)

  ament_add_gtest(execution_publication_currentness_3d_test
                  tests/execution_publication_currentness_3d_test.cpp)
  target_compile_features(execution_publication_currentness_3d_test
                          PRIVATE cxx_std_20)
  target_link_libraries(execution_publication_currentness_3d_test
                        drone_city_nav_core)
  enable_project_warnings(execution_publication_currentness_3d_test)

  ament_add_gtest(rolling_route_telemetry_3d_test
                  tests/rolling_route_telemetry_3d_test.cpp)
  target_compile_features(rolling_route_telemetry_3d_test PRIVATE cxx_std_20)
  target_link_libraries(rolling_route_telemetry_3d_test drone_city_nav_core)
  enable_project_warnings(rolling_route_telemetry_3d_test)

  ament_add_gtest(passage_traversal_evidence_test
                  tests/passage_traversal_evidence_test.cpp)
  target_compile_features(passage_traversal_evidence_test PRIVATE cxx_std_20)
  target_link_libraries(passage_traversal_evidence_test drone_city_nav_core)
  enable_project_warnings(passage_traversal_evidence_test)

  ament_add_gtest(static_route_extension_test tests/static_route_extension_test.cpp)
  target_compile_features(static_route_extension_test PRIVATE cxx_std_20)
  target_link_libraries(static_route_extension_test drone_city_nav_core)
  enable_project_warnings(static_route_extension_test)

  ament_add_gtest(certified_route_splice_3d_test
                  tests/certified_route_splice_3d_test.cpp)
  target_compile_features(certified_route_splice_3d_test PRIVATE cxx_std_20)
  target_link_libraries(certified_route_splice_3d_test drone_city_nav_core)
  enable_project_warnings(certified_route_splice_3d_test)

  ament_add_gtest(static_route_geometry_test tests/static_route_geometry_test.cpp)
  target_compile_features(static_route_geometry_test PRIVATE cxx_std_20)
  target_link_libraries(static_route_geometry_test drone_city_nav_core)
  enable_project_warnings(static_route_geometry_test)

  ament_add_gtest(static_map_debug_3d_test tests/static_map_debug_3d_test.cpp)
  target_compile_features(static_map_debug_3d_test PRIVATE cxx_std_20)
  target_link_libraries(static_map_debug_3d_test drone_city_nav_ros_adapters)
  enable_project_warnings(static_map_debug_3d_test)

  ament_add_gtest(grid_overlay_test_current tests/grid_overlay_test.cpp)
  target_compile_features(grid_overlay_test_current PRIVATE cxx_std_20)
  target_link_libraries(grid_overlay_test_current drone_city_nav_core)
  enable_project_warnings(grid_overlay_test_current)

  ament_add_gtest(px4_offboard_setpoint_io_test_current
                  tests/px4_offboard_setpoint_io_test.cpp)
  target_compile_features(px4_offboard_setpoint_io_test_current PRIVATE cxx_std_20)
  target_link_libraries(px4_offboard_setpoint_io_test_current
                        drone_city_nav_ros_adapters)
  ament_target_dependencies(px4_offboard_setpoint_io_test_current px4_msgs)
  enable_project_warnings(px4_offboard_setpoint_io_test_current)


  ament_add_gtest(mppi_debug_markers_test_current
                  tests/mppi_debug_markers_test.cpp)
  target_compile_features(mppi_debug_markers_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_debug_markers_test_current
                        drone_city_nav_ros_adapters)
  enable_project_warnings(mppi_debug_markers_test_current)

  ament_add_gtest(lidar_debug_heading_gate_test
                  tests/lidar_debug_heading_gate_test.cpp)
  target_compile_features(lidar_debug_heading_gate_test PRIVATE cxx_std_20)
  target_include_directories(lidar_debug_heading_gate_test PRIVATE include)
  target_link_libraries(lidar_debug_heading_gate_test drone_city_nav_core)
  enable_project_warnings(lidar_debug_heading_gate_test)

  ament_add_gtest(lidar_projection_test_current tests/lidar_projection_test.cpp)
  target_compile_features(lidar_projection_test_current PRIVATE cxx_std_20)
  target_link_libraries(lidar_projection_test_current drone_city_nav_core)
  enable_project_warnings(lidar_projection_test_current)

  ament_add_gtest(lidar_pose_history_test_current
                  tests/lidar_pose_history_test.cpp)
  target_compile_features(lidar_pose_history_test_current PRIVATE cxx_std_20)
  target_link_libraries(lidar_pose_history_test_current drone_city_nav_core)
  enable_project_warnings(lidar_pose_history_test_current)

  ament_add_gtest(lidar_acquisition_pose_test
                  tests/lidar_acquisition_pose_test.cpp)
  target_compile_features(lidar_acquisition_pose_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_acquisition_pose_test drone_city_nav_core)
  enable_project_warnings(lidar_acquisition_pose_test)

  ament_add_gtest(spectator_diagnostics_selection_test
                  tests/spectator_diagnostics_selection_test.cpp)
  target_compile_features(spectator_diagnostics_selection_test PRIVATE cxx_std_20)
  target_include_directories(spectator_diagnostics_selection_test PRIVATE include)
  enable_project_warnings(spectator_diagnostics_selection_test)

  ament_add_gtest(spectator_selection_test tests/spectator_selection_test.cpp)
  target_compile_features(spectator_selection_test PRIVATE cxx_std_20)
  target_link_libraries(spectator_selection_test drone_city_nav_core)
  enable_project_warnings(spectator_selection_test)

  ament_add_gtest(spectator_reselection_lifecycle_test
                  tests/spectator_reselection_lifecycle_test.cpp)
  target_compile_features(spectator_reselection_lifecycle_test PRIVATE cxx_std_20)
  target_link_libraries(
    spectator_reselection_lifecycle_test
    drone_city_nav_diagnostics_components
    ${cpp_typesupport_target})
  ament_target_dependencies(spectator_reselection_lifecycle_test rclcpp)
  set_tests_properties(
    spectator_reselection_lifecycle_test
    PROPERTIES ENVIRONMENT
               "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
  enable_project_warnings(spectator_reselection_lifecycle_test)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
