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
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ObstacleMemoryHitObservation.msg"
  "msg/ObstacleMemoryCellProvenance.msg"
  "msg/ObstacleMemoryProvenance.msg"
  "msg/ObstacleMemorySnapshot.msg"
  "msg/RawObstacleSnapshot.msg"
  "msg/CrashState.msg"
  "msg/MppiHorizonPoint.msg"
  "msg/MppiTrajectoryHorizon.msg"
  "msg/MppiControlFeedback.msg"
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
  src/ambiguous_lidar_hit_tracker.cpp
  src/debug_image.cpp
  src/distance_field.cpp
  src/grid_config.cpp
  src/grid_overlay.cpp
  src/known_passage_map.cpp
  src/known_passage_geometry.cpp
  src/known_passage_solid_volumes.cpp
  src/known_static_lidar_hit_classifier.cpp
  src/lidar_debug_renderer.cpp
  src/lidar_debug_snapshot_pipeline.cpp
  src/lidar_beam_observation.cpp
  src/lidar_ingestion_decision.cpp
  src/lidar_ingestion_confidence.cpp
  src/lidar_memory_hit_diagnostics.cpp
  src/lidar_pose_history.cpp
  src/lidar_projection.cpp
  src/px4_ros_time_mapper.cpp
  src/risk_aware_lattice.cpp
  src/lidar_snapshot_writer.cpp
  src/mppi_horizon_safety.cpp
  src/mppi_liveness.cpp
  src/mppi_speed_policy.cpp
  src/navigation_pose.cpp
  src/obstacle_memory.cpp
  src/occupancy_grid.cpp
  src/passage_route_selection.cpp
  src/passage_traversal_monitor.cpp
  src/static_city_map.cpp)
target_compile_features(drone_city_nav_core PUBLIC cxx_std_20)
target_include_directories(
  drone_city_nav_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
enable_project_warnings(drone_city_nav_core)

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
  src/known_passage_debug_markers.cpp
  src/static_map_debug.cpp
  src/static_map_source.cpp
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

add_executable(world_visualization_node src/world_visualization_node.cpp)
target_compile_features(world_visualization_node PRIVATE cxx_std_20)
target_link_libraries(world_visualization_node drone_city_nav_core
                      drone_city_nav_ros_adapters ${cpp_typesupport_target})
ament_target_dependencies(
  world_visualization_node
  ament_index_cpp
  nav_msgs
  rclcpp
  sensor_msgs
  visualization_msgs)
enable_project_warnings(world_visualization_node)

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

add_executable(
  lidar_debug_node
  src/lidar_debug_node.cpp
  src/lidar_debug_node_callbacks.cpp
  src/lidar_debug_node_lifecycle.cpp
  src/lidar_debug_node_points.cpp
  src/lidar_debug_node_snapshot.cpp)
target_compile_features(lidar_debug_node PRIVATE cxx_std_20)
target_link_libraries(lidar_debug_node drone_city_nav_core
                      drone_city_nav_ros_adapters)
ament_target_dependencies(
  lidar_debug_node
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs)
enable_project_warnings(lidar_debug_node)

if(DRONE_CITY_NAV_ENABLE_CUDA_MPPI)
  add_executable(
    production_mppi_node
    src/production_mppi_node.cpp
    src/production_mppi_node_inputs.cpp
    src/production_mppi_node_runtime.cpp
    src/production_mppi_node_diagnostics.cpp
    src/production_mppi_node_execution.cpp)
  target_compile_features(production_mppi_node PRIVATE cxx_std_20)
  target_link_libraries(
    production_mppi_node
    drone_city_nav_core
    drone_city_nav_ros_adapters
    drone_city_nav_mppi_cuda
    ${cpp_typesupport_target})
  ament_target_dependencies(
    production_mppi_node
    ament_index_cpp
    nav_msgs
    px4_msgs
    rclcpp
    std_msgs
    visualization_msgs)
  enable_project_warnings(production_mppi_node)
  install(TARGETS production_mppi_node
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
  EXPORT drone_city_nav_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mppi_reference_test_current tests/mppi_reference_test.cpp
                  src/mppi/mppi_reference.cpp)
  target_compile_features(mppi_reference_test_current PRIVATE cxx_std_20)
  target_include_directories(mppi_reference_test_current PRIVATE include)
  enable_project_warnings(mppi_reference_test_current)

  ament_add_gtest(mppi_horizon_safety_test_current
                  tests/mppi_horizon_safety_test.cpp)
  target_compile_features(mppi_horizon_safety_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_horizon_safety_test_current drone_city_nav_core)
  enable_project_warnings(mppi_horizon_safety_test_current)

  ament_add_gtest(mppi_liveness_test_current tests/mppi_liveness_test.cpp)
  target_compile_features(mppi_liveness_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_liveness_test_current drone_city_nav_core)
  enable_project_warnings(mppi_liveness_test_current)

  ament_add_gtest(mppi_speed_policy_test_current tests/mppi_speed_policy_test.cpp)
  target_compile_features(mppi_speed_policy_test_current PRIVATE cxx_std_20)
  target_link_libraries(mppi_speed_policy_test_current drone_city_nav_core)
  enable_project_warnings(mppi_speed_policy_test_current)

  ament_add_gtest(passage_route_selection_test
                  tests/passage_route_selection_test.cpp)
  target_compile_features(passage_route_selection_test PRIVATE cxx_std_20)
  target_link_libraries(passage_route_selection_test drone_city_nav_core)
  enable_project_warnings(passage_route_selection_test)

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

  ament_add_gtest(distance_field_test_current tests/distance_field_test.cpp)
  target_compile_features(distance_field_test_current PRIVATE cxx_std_20)
  target_link_libraries(distance_field_test_current drone_city_nav_core)
  enable_project_warnings(distance_field_test_current)

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

  ament_add_gtest(static_map_debug_test_current
                  tests/static_map_debug_test.cpp)
  target_compile_features(static_map_debug_test_current PRIVATE cxx_std_20)
  target_link_libraries(static_map_debug_test_current
                        drone_city_nav_ros_adapters)
  enable_project_warnings(static_map_debug_test_current)

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
endif()

ament_export_include_directories(include)
ament_export_targets(drone_city_nav_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
