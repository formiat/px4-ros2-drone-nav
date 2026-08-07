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
  "msg/RawObstacleSnapshot.msg"
  "msg/VehicleDestroyed.msg"
  "msg/MppiHorizonPoint.msg"
  "msg/MppiTrajectoryHorizon.msg"
  "msg/MppiControlFeedback.msg"
  "msg/NavigationObjective.msg"
  "msg/RadarDetection.msg"
  "msg/RadarScan.msg"
  "msg/RadarTrackModeCommand.msg"
  "msg/TargetTrack.msg"
  "msg/InterceptMissionCommand.msg"
  "msg/SpectatorTarget.msg"
  "msg/VehicleNavigationState.msg"
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
    src/mppi/mppi_engine.cu
    src/mppi/mppi_control_sequence.cpp
    src/mppi/mppi_reference.cpp)
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
    EXPORT drone_city_nav_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

add_library(
  drone_city_nav_core
  src/active_global_guide.cpp
  src/ambiguous_lidar_hit_tracker.cpp
  src/bounded_worker_pool.cpp
  src/vehicle_destruction_disarm_lifecycle.cpp
  src/debug_image.cpp
  src/distance_field.cpp
  src/distance_field_3d.cpp
  src/esdf_query.cpp
  src/flight_envelope.cpp
  src/global_guide_candidate.cpp
  src/grid_config.cpp
  src/grid_overlay.cpp
  src/intercept_guidance.cpp
  src/intercept_mission.cpp
  src/known_static_lidar_hit_classifier.cpp
  src/lidar_debug_renderer.cpp
  src/lidar_debug_snapshot_pipeline.cpp
  src/lidar_beam_observation.cpp
  src/lidar_ingestion_decision.cpp
  src/lidar_ingestion_confidence.cpp
  src/lidar_memory_hit_diagnostics.cpp
  src/local_esdf_2d.cpp
  src/mapping_lifecycle.cpp
  src/mission_goal_capture.cpp
  src/lidar_pose_history.cpp
  src/lidar_projection.cpp
  src/px4_ros_time_mapper.cpp
  src/radar_cadence.cpp
  src/radar_model.cpp
  src/radar_target_tracker.cpp
  src/raw_guide_validation.cpp
  src/risk_aware_lattice.cpp
  src/risk_aware_lattice_geometry.cpp
  src/risk_aware_lattice_names.cpp
  src/risk_aware_lattice_successors.cpp
  src/risk_aware_lattice_3d.cpp
  src/risk_aware_lattice_3d_continuation.cpp
  src/risk_aware_lattice_3d_geometry.cpp
  src/risk_aware_lattice_3d_result.cpp
  src/route_3d.cpp
  src/static_route_extension.cpp
  src/static_route_geometry.cpp
  src/static_esdf_cache.cpp
  src/successor_profiling_diagnostics.cpp
  src/swept_footprint.cpp
  src/tracked_agent_lidar_filter.cpp
  src/tracking_objective.cpp
  src/lidar_snapshot_writer.cpp
  src/mppi/mppi_post_update_classification.cpp
  src/mppi_horizon_safety.cpp
  src/mppi_liveness.cpp
  src/mppi_nominal_reseed.cpp
  src/mppi_risk_escalation.cpp
  src/mppi_speed_policy.cpp
  src/navigation_state_prediction.cpp
  src/no_static_route_cycle.cpp
  src/navigation_pose.cpp
  src/obstacle_memory.cpp
  src/occupancy_grid.cpp
  src/occupancy_grid_3d.cpp
  )
target_compile_features(drone_city_nav_core PUBLIC cxx_std_20)
target_include_directories(
  drone_city_nav_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
enable_project_warnings(drone_city_nav_core)
target_link_libraries(drone_city_nav_core PRIVATE PkgConfig::ZSTD)

add_executable(generate_static_esdf_cache tools/generate_static_esdf_cache.cpp)
target_compile_features(generate_static_esdf_cache PRIVATE cxx_std_20)
target_link_libraries(generate_static_esdf_cache drone_city_nav_core)
enable_project_warnings(generate_static_esdf_cache)

if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  target_link_libraries(drone_city_nav_mppi_cuda PUBLIC drone_city_nav_core)
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
  src/lidar_debug_node_config.cpp
  src/memory_provenance_audit_tracker.cpp
  src/obstacle_memory_provenance_ros.cpp
  src/px4_offboard_setpoint_io.cpp
  src/lidar_debug_pointclouds.cpp
  src/mppi_debug_markers.cpp
  src/ros_conversions.cpp
  src/static_map_debug.cpp
  src/visualization_marker_helpers.cpp)
target_compile_features(drone_city_nav_ros_adapters PUBLIC cxx_std_20)
target_link_libraries(drone_city_nav_ros_adapters drone_city_nav_core)
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
                      ${cpp_typesupport_target})
ament_target_dependencies(
  mission_monitor_node
  ament_index_cpp
  px4_msgs
  rclcpp)
enable_project_warnings(mission_monitor_node)

add_executable(intercept_mission_referee_node
               src/intercept_mission_referee_node.cpp)
target_compile_features(intercept_mission_referee_node PRIVATE cxx_std_20)
target_link_libraries(intercept_mission_referee_node drone_city_nav_core
                      ${cpp_typesupport_target})
ament_target_dependencies(intercept_mission_referee_node rclcpp std_msgs)
enable_project_warnings(intercept_mission_referee_node)

add_executable(radar_simulator_node src/radar_simulator_node.cpp)
target_compile_features(radar_simulator_node PRIVATE cxx_std_20)
target_link_libraries(radar_simulator_node drone_city_nav_core
                      ${cpp_typesupport_target})
ament_target_dependencies(radar_simulator_node rclcpp)
enable_project_warnings(radar_simulator_node)

add_library(
  drone_city_nav_interceptor_tracking_components SHARED
  src/radar_target_tracker_node.cpp
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
  "drone_city_nav::InterceptorGuidanceNode")
enable_project_warnings(drone_city_nav_interceptor_tracking_components)

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
    src/production_mppi_node.cpp
    src/production_mppi_node_esdf.cpp
    src/production_mppi_node_inputs.cpp
    src/production_mppi_node_planning_tick.cpp
    src/production_mppi_node_raw_world.cpp
    src/production_mppi_node_runtime.cpp
    src/production_mppi_route_helpers.cpp
    src/production_mppi_node_static_extension.cpp
    src/production_mppi_node_static_guide.cpp
    src/production_mppi_node_diagnostics.cpp
    src/production_mppi_node_diagnostics_runtime.cpp
    src/production_mppi_node_summary.cpp
    src/tracking_objective_diagnostics.cpp
    src/production_mppi_node_execution.cpp)
  target_compile_features(drone_city_nav_production_mppi_component PRIVATE cxx_std_20)
  target_link_libraries(
    drone_city_nav_production_mppi_component
    drone_city_nav_core
    drone_city_nav_ros_adapters
    drone_city_nav_mppi_cuda
    ${cpp_typesupport_target})
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

  add_executable(production_mppi_node src/production_mppi_main.cpp)
  target_compile_features(production_mppi_node PRIVATE cxx_std_20)
  target_link_libraries(production_mppi_node
                        drone_city_nav_production_mppi_component)
  ament_target_dependencies(production_mppi_node rclcpp)
  enable_project_warnings(production_mppi_node)
  install(
    TARGETS production_mppi_node drone_city_nav_production_mppi_component
    EXPORT drone_city_nav_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(
  TARGETS
    drone_city_nav_core
    drone_city_nav_ros_adapters
    drone_contact_system
    collision_crash_node
    lidar_debug_node
    obstacle_memory_node
    mppi_offboard_node
    world_visualization_node
    mission_monitor_node
    intercept_mission_referee_node
    radar_simulator_node
    radar_target_tracker_node
    interceptor_guidance_node
    drone_city_nav_interceptor_tracking_components
    drone_city_nav_diagnostics_components
    intercept_spectator_node
    intercept_diagnostics_mux_node
    generate_static_esdf_cache
  EXPORT drone_city_nav_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(intercept_mission_test tests/intercept_mission_test.cpp)
  target_compile_features(intercept_mission_test PRIVATE cxx_std_20)
  target_link_libraries(intercept_mission_test drone_city_nav_core)
  enable_project_warnings(intercept_mission_test)

  ament_add_gtest(intercept_guidance_test tests/intercept_guidance_test.cpp)
  target_compile_features(intercept_guidance_test PRIVATE cxx_std_20)
  target_link_libraries(intercept_guidance_test drone_city_nav_core)
  enable_project_warnings(intercept_guidance_test)

  ament_add_gtest(tracking_objective_test tests/tracking_objective_test.cpp)
  target_compile_features(tracking_objective_test PRIVATE cxx_std_20)
  target_link_libraries(tracking_objective_test drone_city_nav_core)
  enable_project_warnings(tracking_objective_test)

  ament_add_gtest(radar_tracking_test tests/radar_tracking_test.cpp)
  target_compile_features(radar_tracking_test PRIVATE cxx_std_20)
  target_link_libraries(radar_tracking_test drone_city_nav_core)
  enable_project_warnings(radar_tracking_test)

  ament_add_gtest(tracked_agent_lidar_filter_test
                  tests/tracked_agent_lidar_filter_test.cpp)
  target_compile_features(tracked_agent_lidar_filter_test PRIVATE cxx_std_20)
  target_link_libraries(tracked_agent_lidar_filter_test drone_city_nav_core)
  enable_project_warnings(tracked_agent_lidar_filter_test)

  ament_add_gtest(mppi_reference_test_current tests/mppi_reference_test.cpp
                  src/mppi/mppi_reference.cpp)
  target_compile_features(mppi_reference_test_current PRIVATE cxx_std_20)
  target_include_directories(mppi_reference_test_current PRIVATE include)
  target_link_libraries(mppi_reference_test_current drone_city_nav_core)
  enable_project_warnings(mppi_reference_test_current)

  ament_add_gtest(mppi_horizon_safety_test_current
                  tests/mppi_horizon_safety_test.cpp)
  target_compile_features(mppi_horizon_safety_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_horizon_safety_test_current drone_city_nav_core)
  enable_project_warnings(mppi_horizon_safety_test_current)

  ament_add_gtest(esdf_query_test tests/esdf_query_test.cpp)
  target_compile_features(esdf_query_test PRIVATE cxx_std_20)
  target_link_libraries(esdf_query_test drone_city_nav_core)
  enable_project_warnings(esdf_query_test)

  ament_add_gtest(local_esdf_2d_test tests/local_esdf_2d_test.cpp)
  target_compile_features(local_esdf_2d_test PRIVATE cxx_std_20)
  target_link_libraries(local_esdf_2d_test drone_city_nav_core)
  enable_project_warnings(local_esdf_2d_test)

  ament_add_gtest(flight_envelope_test tests/flight_envelope_test.cpp)
  target_compile_features(flight_envelope_test PRIVATE cxx_std_20)
  target_link_libraries(flight_envelope_test drone_city_nav_core)
  enable_project_warnings(flight_envelope_test)

  ament_add_gtest(swept_footprint_test tests/swept_footprint_test.cpp)
  target_compile_features(swept_footprint_test PRIVATE cxx_std_20)
  target_link_libraries(swept_footprint_test drone_city_nav_core)
  enable_project_warnings(swept_footprint_test)

  ament_add_gtest(mppi_liveness_test_current tests/mppi_liveness_test.cpp)
  target_compile_features(mppi_liveness_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_liveness_test_current drone_city_nav_core)
  enable_project_warnings(mppi_liveness_test_current)

  ament_add_gtest(mppi_risk_escalation_test
                  tests/mppi_risk_escalation_test.cpp)
  target_compile_features(mppi_risk_escalation_test PRIVATE cxx_std_20)
  target_link_libraries(mppi_risk_escalation_test drone_city_nav_core)
  enable_project_warnings(mppi_risk_escalation_test)

  ament_add_gtest(mission_goal_capture_test tests/mission_goal_capture_test.cpp)
  target_compile_features(mission_goal_capture_test PRIVATE cxx_std_20)
  target_link_libraries(mission_goal_capture_test drone_city_nav_core)
  enable_project_warnings(mission_goal_capture_test)

  ament_add_gtest(mapping_lifecycle_test tests/mapping_lifecycle_test.cpp)
  target_compile_features(mapping_lifecycle_test PRIVATE cxx_std_20)
  target_link_libraries(mapping_lifecycle_test drone_city_nav_core)
  enable_project_warnings(mapping_lifecycle_test)

  ament_add_gtest(navigation_state_prediction_test
                  tests/navigation_state_prediction_test.cpp)
  target_compile_features(navigation_state_prediction_test PRIVATE cxx_std_20)
  target_link_libraries(navigation_state_prediction_test drone_city_nav_core)
  enable_project_warnings(navigation_state_prediction_test)

  ament_add_gtest(no_static_route_cycle_test
                  tests/no_static_route_cycle_test.cpp)
  target_compile_features(no_static_route_cycle_test PRIVATE cxx_std_20)
  target_link_libraries(no_static_route_cycle_test drone_city_nav_core)
  enable_project_warnings(no_static_route_cycle_test)

  ament_add_gtest(latest_value_mailbox_test
                  tests/latest_value_mailbox_test.cpp)
  target_compile_features(latest_value_mailbox_test PRIVATE cxx_std_20)
  target_include_directories(latest_value_mailbox_test PRIVATE include)
  enable_project_warnings(latest_value_mailbox_test)

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

  if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
    ament_add_gtest(mppi_control_sequence_test_current
                    tests/mppi_control_sequence_test.cpp)
    target_compile_features(mppi_control_sequence_test_current PRIVATE cxx_std_20)
    target_link_libraries(mppi_control_sequence_test_current
                          drone_city_nav_mppi_cuda)
    enable_project_warnings(mppi_control_sequence_test_current)
  endif()

  ament_add_gtest(risk_aware_lattice_test
                  tests/risk_aware_lattice_test.cpp)
  target_compile_features(risk_aware_lattice_test PRIVATE cxx_std_20)
  target_link_libraries(risk_aware_lattice_test drone_city_nav_core)
  enable_project_warnings(risk_aware_lattice_test)

  ament_add_gtest(active_global_guide_test
                  tests/active_global_guide_test.cpp)
  target_compile_features(active_global_guide_test PRIVATE cxx_std_20)
  target_link_libraries(active_global_guide_test drone_city_nav_core)
  enable_project_warnings(active_global_guide_test)

  ament_add_gtest(global_guide_candidate_test
                  tests/global_guide_candidate_test.cpp)
  target_compile_features(global_guide_candidate_test PRIVATE cxx_std_20)
  target_link_libraries(global_guide_candidate_test drone_city_nav_core)
  enable_project_warnings(global_guide_candidate_test)

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
    PRIVATE TEST_OCCUPANCY3D_PATH="${CMAKE_CURRENT_SOURCE_DIR}/worlds/generated_city.occupancy3d")
  enable_project_warnings(occupancy_grid_3d_test)

  ament_add_gtest(distance_field_3d_test tests/distance_field_3d_test.cpp)
  target_compile_features(distance_field_3d_test PRIVATE cxx_std_20)
  target_link_libraries(distance_field_3d_test drone_city_nav_core)
  enable_project_warnings(distance_field_3d_test)

  ament_add_gtest(static_esdf_cache_test tests/static_esdf_cache_test.cpp)
  target_compile_features(static_esdf_cache_test PRIVATE cxx_std_20)
  target_link_libraries(static_esdf_cache_test drone_city_nav_core)
  enable_project_warnings(static_esdf_cache_test)

  ament_add_gtest(route_3d_test tests/route_3d_test.cpp)
  target_compile_features(route_3d_test PRIVATE cxx_std_20)
  target_link_libraries(route_3d_test drone_city_nav_core)
  enable_project_warnings(route_3d_test)

  ament_add_gtest(static_route_extension_test tests/static_route_extension_test.cpp)
  target_compile_features(static_route_extension_test PRIVATE cxx_std_20)
  target_link_libraries(static_route_extension_test drone_city_nav_core)
  enable_project_warnings(static_route_extension_test)

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
endif()

ament_export_include_directories(include)
ament_export_targets(drone_city_nav_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
