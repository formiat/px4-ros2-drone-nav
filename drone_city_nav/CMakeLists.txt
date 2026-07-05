cmake_minimum_required(VERSION 3.20)
project(drone_city_nav VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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

add_library(
  drone_city_nav_core
  src/astar_planner.cpp
  src/clearance_field.cpp
  src/control_projection_smoothing.cpp
  src/corridor_samples_io.cpp
  src/corridor.cpp
  src/current_lidar_overlay.cpp
  src/debug_image.cpp
  src/distance_field.cpp
  src/final_trajectory_debug_io.cpp
  src/grid_config.cpp
  src/grid_overlay.cpp
  src/lidar_debug_renderer.cpp
  src/lidar_debug_snapshot_pipeline.cpp
  src/lidar_projection.cpp
  src/lidar_snapshot_writer.cpp
  src/navigation_pose.cpp
  src/offboard_blackbox.cpp
  src/offboard_path_follower.cpp
  src/offboard_velocity_follower.cpp
  src/obstacle_memory.cpp
  src/occupancy_grid.cpp
  src/path_smoothing.cpp
  src/planning_grid_builder.cpp
  src/planner_core.cpp
  src/planner_diagnostics_format.cpp
  src/planner_path_publication.cpp
  src/planner_runtime_state.cpp
  src/route_diagnostics.cpp
  src/static_city_map.cpp
  src/terminal_capture_state_machine.cpp
  src/trajectory_optimizer.cpp
  src/trajectory_optimizer_geometry.cpp
  src/trajectory_optimizer_active_windows.cpp
  src/trajectory_optimizer_scoring.cpp
  src/trajectory_optimizer_candidates.cpp
  src/trajectory_optimizer_dp.cpp
  src/trajectory_optimizer_workers.cpp
  src/trajectory.cpp
  src/trajectory_diagnostics.cpp
  src/trajectory_diagnostics_io.cpp
  src/trajectory_diagnostics_io_csv.cpp
  src/trajectory_diagnostics_io_json_fields.cpp
  src/trajectory_diagnostics_io_json_summary.cpp
  src/trajectory_diagnostics_io_parser.cpp
  src/trajectory_planner.cpp
  src/trajectory_refinement_scheduler.cpp
  src/trajectory_shape_cleanup.cpp
  src/trajectory_speed_planner.cpp
  src/trajectory_update_continuity.cpp
  src/turn_smoothing.cpp
  src/turn_smoothing_geometry.cpp
  src/turn_smoothing_candidates.cpp
  src/turn_smoothing_metrics.cpp
  src/turn_smoothing_diagnostics.cpp
  src/velocity_command_planner.cpp
  src/velocity_control_config.cpp
  src/velocity_smoother.cpp)
target_compile_features(drone_city_nav_core PUBLIC cxx_std_20)
target_include_directories(
  drone_city_nav_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
enable_project_warnings(drone_city_nav_core)

add_library(
  drone_city_nav_ros_adapters
  src/lidar_debug_node_config.cpp
  src/offboard_debug_markers.cpp
  src/offboard_trajectory_state.cpp
  src/planner_node_config.cpp
  src/px4_offboard_node_config.cpp
  src/px4_offboard_setpoint_io.cpp
  src/lidar_debug_pointclouds.cpp
  src/ros_conversions.cpp
  src/static_map_debug.cpp
  src/static_map_source.cpp
  src/trajectory_debug_markers.cpp)
target_compile_features(drone_city_nav_ros_adapters PUBLIC cxx_std_20)
target_link_libraries(drone_city_nav_ros_adapters drone_city_nav_core)
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

add_executable(
  planner_node
  src/planner_node.cpp
  src/planner_node_inputs.cpp
  src/planner_node_lifecycle.cpp
  src/planner_node_publish.cpp
  src/planner_node_trajectory_publication.cpp
  src/planner_node_refinement.cpp
  src/planner_node_safety.cpp
  src/planner_node_lidar_overlay.cpp
  src/planner_node_debug_publication.cpp
  src/planner_node_dumps.cpp
  src/planner_node_runtime.cpp)
target_compile_features(planner_node PRIVATE cxx_std_20)
target_link_libraries(planner_node drone_city_nav_core drone_city_nav_ros_adapters)
ament_target_dependencies(
  planner_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs
  std_msgs)
enable_project_warnings(planner_node)

add_executable(obstacle_memory_node src/obstacle_memory_node.cpp)
target_compile_features(obstacle_memory_node PRIVATE cxx_std_20)
target_link_libraries(obstacle_memory_node drone_city_nav_core)
ament_target_dependencies(
  obstacle_memory_node
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs)
enable_project_warnings(obstacle_memory_node)

add_executable(
  px4_offboard_node
  src/px4_offboard_node.cpp
  src/px4_offboard_node_control.cpp
  src/px4_offboard_node_inputs.cpp
  src/px4_offboard_node_lifecycle.cpp
  src/px4_offboard_node_telemetry.cpp
  src/px4_offboard_node_trajectory.cpp)
target_compile_features(px4_offboard_node PRIVATE cxx_std_20)
target_link_libraries(px4_offboard_node drone_city_nav_core drone_city_nav_ros_adapters)
ament_target_dependencies(
  px4_offboard_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  std_msgs
  visualization_msgs)
enable_project_warnings(px4_offboard_node)

add_executable(mission_monitor_node src/mission_monitor_node.cpp)
target_compile_features(mission_monitor_node PRIVATE cxx_std_20)
target_link_libraries(mission_monitor_node drone_city_nav_core)
ament_target_dependencies(
  mission_monitor_node
  px4_msgs
  rclcpp
  std_msgs)
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

install(
  TARGETS
    drone_city_nav_core
    drone_city_nav_ros_adapters
    lidar_debug_node
    obstacle_memory_node
    planner_node
    px4_offboard_node
    mission_monitor_node
  EXPORT drone_city_nav_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(debug_image_test tests/debug_image_test.cpp)
  target_compile_features(debug_image_test PRIVATE cxx_std_20)
  target_link_libraries(debug_image_test drone_city_nav_core)
  enable_project_warnings(debug_image_test)

  ament_add_gtest(lidar_debug_renderer_test
                  tests/lidar_debug_renderer_test.cpp)
  target_compile_features(lidar_debug_renderer_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_debug_renderer_test drone_city_nav_core)
  enable_project_warnings(lidar_debug_renderer_test)

  ament_add_gtest(lidar_snapshot_writer_test
                  tests/lidar_snapshot_writer_test.cpp)
  target_compile_features(lidar_snapshot_writer_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_snapshot_writer_test drone_city_nav_core)
  enable_project_warnings(lidar_snapshot_writer_test)

  ament_add_gtest(lidar_debug_snapshot_pipeline_test
                  tests/lidar_debug_snapshot_pipeline_test.cpp)
  target_compile_features(lidar_debug_snapshot_pipeline_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_debug_snapshot_pipeline_test drone_city_nav_core)
  enable_project_warnings(lidar_debug_snapshot_pipeline_test)

  ament_add_gtest(lidar_debug_node_config_test
                  tests/lidar_debug_node_config_test.cpp)
  target_compile_features(lidar_debug_node_config_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_debug_node_config_test drone_city_nav_ros_adapters)
  ament_target_dependencies(lidar_debug_node_config_test rclcpp)
  enable_project_warnings(lidar_debug_node_config_test)

  ament_add_gtest(final_trajectory_debug_io_test
                  tests/final_trajectory_debug_io_test.cpp)
  target_compile_features(final_trajectory_debug_io_test PRIVATE cxx_std_20)
  target_link_libraries(final_trajectory_debug_io_test drone_city_nav_core)
  enable_project_warnings(final_trajectory_debug_io_test)

  ament_add_gtest(clearance_field_test tests/clearance_field_test.cpp)
  target_compile_features(clearance_field_test PRIVATE cxx_std_20)
  target_link_libraries(clearance_field_test drone_city_nav_core)
  enable_project_warnings(clearance_field_test)

  ament_add_gtest(distance_field_test tests/distance_field_test.cpp)
  target_compile_features(distance_field_test PRIVATE cxx_std_20)
  target_link_libraries(distance_field_test drone_city_nav_core)
  enable_project_warnings(distance_field_test)

  ament_add_gtest(planner_core_test tests/planner_core_test.cpp)
  target_compile_features(planner_core_test PRIVATE cxx_std_20)
  target_compile_definitions(
    planner_core_test
    PRIVATE DRONE_CITY_NAV_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(planner_core_test drone_city_nav_core)
  enable_project_warnings(planner_core_test)

  ament_add_gtest(current_lidar_overlay_test
                  tests/current_lidar_overlay_test.cpp)
  target_compile_features(current_lidar_overlay_test PRIVATE cxx_std_20)
  target_link_libraries(current_lidar_overlay_test drone_city_nav_core)
  enable_project_warnings(current_lidar_overlay_test)

  ament_add_gtest(lidar_motion_compensation_test
                  tests/lidar_motion_compensation_test.cpp)
  target_compile_features(lidar_motion_compensation_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_motion_compensation_test drone_city_nav_core)
  enable_project_warnings(lidar_motion_compensation_test)

  ament_add_gtest(planning_grid_builder_test
                  tests/planning_grid_builder_test.cpp)
  target_compile_features(planning_grid_builder_test PRIVATE cxx_std_20)
  target_link_libraries(planning_grid_builder_test drone_city_nav_core)
  enable_project_warnings(planning_grid_builder_test)

  ament_add_gtest(static_city_map_test tests/static_city_map_test.cpp)
  target_compile_features(static_city_map_test PRIVATE cxx_std_20)
  target_link_libraries(static_city_map_test drone_city_nav_core)
  enable_project_warnings(static_city_map_test)

  ament_add_gtest(grid_overlay_test tests/grid_overlay_test.cpp)
  target_compile_features(grid_overlay_test PRIVATE cxx_std_20)
  target_link_libraries(grid_overlay_test drone_city_nav_core)
  enable_project_warnings(grid_overlay_test)

  ament_add_gtest(obstacle_memory_test tests/obstacle_memory_test.cpp)
  target_compile_features(obstacle_memory_test PRIVATE cxx_std_20)
  target_link_libraries(obstacle_memory_test drone_city_nav_core)
  enable_project_warnings(obstacle_memory_test)

  ament_add_gtest(trajectory_test tests/trajectory_test.cpp)
  target_compile_features(trajectory_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_test drone_city_nav_core)
  enable_project_warnings(trajectory_test)

  ament_add_gtest(trajectory_diagnostics_test
                  tests/trajectory_diagnostics_test.cpp)
  target_compile_features(trajectory_diagnostics_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_diagnostics_test drone_city_nav_core)
  enable_project_warnings(trajectory_diagnostics_test)

  ament_add_gtest(
    trajectory_diagnostics_io_test
    tests/trajectory_diagnostics_io_csv_test.cpp
    tests/trajectory_diagnostics_io_json_fields_test.cpp
    tests/trajectory_diagnostics_io_roundtrip_test.cpp)
  target_compile_features(trajectory_diagnostics_io_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_diagnostics_io_test drone_city_nav_core)
  enable_project_warnings(trajectory_diagnostics_io_test)

  ament_add_gtest(trajectory_debug_markers_test
                  tests/trajectory_debug_markers_test.cpp
                  src/trajectory_debug_markers.cpp)
  target_compile_features(trajectory_debug_markers_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_debug_markers_test drone_city_nav_core)
  ament_target_dependencies(trajectory_debug_markers_test geometry_msgs std_msgs
                            visualization_msgs)
  enable_project_warnings(trajectory_debug_markers_test)

  ament_add_gtest(corridor_test tests/corridor_test.cpp)
  target_compile_features(corridor_test PRIVATE cxx_std_20)
  target_link_libraries(corridor_test drone_city_nav_core)
  enable_project_warnings(corridor_test)

  ament_add_gtest(corridor_samples_io_test tests/corridor_samples_io_test.cpp)
  target_compile_features(corridor_samples_io_test PRIVATE cxx_std_20)
  target_link_libraries(corridor_samples_io_test drone_city_nav_core)
  enable_project_warnings(corridor_samples_io_test)

  ament_add_gtest(planner_path_publication_test
                  tests/planner_path_publication_test.cpp)
  target_compile_features(planner_path_publication_test PRIVATE cxx_std_20)
  target_link_libraries(planner_path_publication_test drone_city_nav_core)
  enable_project_warnings(planner_path_publication_test)

  ament_add_gtest(planner_diagnostics_format_test
                  tests/planner_diagnostics_format_test.cpp)
  target_compile_features(planner_diagnostics_format_test PRIVATE cxx_std_20)
  target_link_libraries(planner_diagnostics_format_test drone_city_nav_core)
  enable_project_warnings(planner_diagnostics_format_test)

  ament_add_gtest(planner_runtime_state_test
                  tests/planner_runtime_state_test.cpp)
  target_compile_features(planner_runtime_state_test PRIVATE cxx_std_20)
  target_link_libraries(planner_runtime_state_test drone_city_nav_core)
  enable_project_warnings(planner_runtime_state_test)

  ament_add_gtest(trajectory_optimizer_test tests/trajectory_optimizer_test.cpp)
  target_compile_features(trajectory_optimizer_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_optimizer_test drone_city_nav_core)
  enable_project_warnings(trajectory_optimizer_test)

  ament_add_gtest(turn_smoothing_test tests/turn_smoothing_test.cpp)
  target_compile_features(turn_smoothing_test PRIVATE cxx_std_20)
  target_link_libraries(turn_smoothing_test drone_city_nav_core)
  enable_project_warnings(turn_smoothing_test)

  ament_add_gtest(trajectory_planner_test tests/trajectory_planner_test.cpp)
  target_compile_features(trajectory_planner_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_planner_test drone_city_nav_core)
  enable_project_warnings(trajectory_planner_test)

  ament_add_gtest(trajectory_refinement_scheduler_test
                  tests/trajectory_refinement_scheduler_test.cpp)
  target_compile_features(trajectory_refinement_scheduler_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_refinement_scheduler_test
                        drone_city_nav_core)
  enable_project_warnings(trajectory_refinement_scheduler_test)

  ament_add_gtest(terminal_capture_state_machine_test
                  tests/terminal_capture_state_machine_test.cpp)
  target_compile_features(terminal_capture_state_machine_test PRIVATE cxx_std_20)
  target_link_libraries(terminal_capture_state_machine_test drone_city_nav_core)
  enable_project_warnings(terminal_capture_state_machine_test)

  ament_add_gtest(trajectory_update_continuity_test
                  tests/trajectory_update_continuity_test.cpp)
  target_compile_features(trajectory_update_continuity_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_update_continuity_test drone_city_nav_core)
  enable_project_warnings(trajectory_update_continuity_test)

  ament_add_gtest(offboard_path_follower_test
                  tests/offboard_path_follower_test.cpp)
  target_compile_features(offboard_path_follower_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_path_follower_test drone_city_nav_core)
  enable_project_warnings(offboard_path_follower_test)

  ament_add_gtest(
    offboard_velocity_follower_test
    tests/offboard_velocity_follower_speed_profile_test.cpp
    tests/offboard_velocity_follower_lateral_control_test.cpp
    tests/offboard_velocity_follower_terminal_capture_test.cpp
    tests/offboard_velocity_follower_invalid_input_test.cpp)
  target_compile_features(offboard_velocity_follower_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_velocity_follower_test drone_city_nav_core)
  enable_project_warnings(offboard_velocity_follower_test)

  ament_add_gtest(trajectory_speed_planner_test
                  tests/trajectory_speed_planner_test.cpp)
  target_compile_features(trajectory_speed_planner_test PRIVATE cxx_std_20)
  target_link_libraries(trajectory_speed_planner_test drone_city_nav_core)
  enable_project_warnings(trajectory_speed_planner_test)

  ament_add_gtest(velocity_command_planner_test
                  tests/velocity_command_planner_test.cpp)
  target_compile_features(velocity_command_planner_test PRIVATE cxx_std_20)
  target_link_libraries(velocity_command_planner_test drone_city_nav_core)
  enable_project_warnings(velocity_command_planner_test)

  ament_add_gtest(velocity_control_config_test
                  tests/velocity_control_config_test.cpp)
  target_compile_features(velocity_control_config_test PRIVATE cxx_std_20)
  target_link_libraries(velocity_control_config_test drone_city_nav_core)
  enable_project_warnings(velocity_control_config_test)

  ament_add_gtest(velocity_smoother_test tests/velocity_smoother_test.cpp)
  target_compile_features(velocity_smoother_test PRIVATE cxx_std_20)
  target_link_libraries(velocity_smoother_test drone_city_nav_core)
  enable_project_warnings(velocity_smoother_test)

  ament_add_gtest(px4_offboard_config_test tests/px4_offboard_config_test.cpp)
  target_compile_features(px4_offboard_config_test PRIVATE cxx_std_20)
  target_compile_definitions(
    px4_offboard_config_test
    PRIVATE DRONE_CITY_NAV_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  enable_project_warnings(px4_offboard_config_test)

  ament_add_gtest(px4_offboard_node_config_test
                  tests/px4_offboard_node_config_test.cpp)
  target_compile_features(px4_offboard_node_config_test PRIVATE cxx_std_20)
  target_link_libraries(px4_offboard_node_config_test drone_city_nav_ros_adapters)
  ament_target_dependencies(px4_offboard_node_config_test rclcpp)
  enable_project_warnings(px4_offboard_node_config_test)

  ament_add_gtest(offboard_blackbox_test tests/offboard_blackbox_test.cpp)
  target_compile_features(offboard_blackbox_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_blackbox_test drone_city_nav_core)
  enable_project_warnings(offboard_blackbox_test)

  ament_add_gtest(px4_offboard_setpoint_io_test
                  tests/px4_offboard_setpoint_io_test.cpp)
  target_compile_features(px4_offboard_setpoint_io_test PRIVATE cxx_std_20)
  target_link_libraries(px4_offboard_setpoint_io_test
                        drone_city_nav_ros_adapters)
  ament_target_dependencies(px4_offboard_setpoint_io_test px4_msgs)
  enable_project_warnings(px4_offboard_setpoint_io_test)

  ament_add_gtest(offboard_debug_markers_test
                  tests/offboard_debug_markers_test.cpp)
  target_compile_features(offboard_debug_markers_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_debug_markers_test drone_city_nav_ros_adapters)
  ament_target_dependencies(offboard_debug_markers_test std_msgs
                            visualization_msgs)
  enable_project_warnings(offboard_debug_markers_test)

  ament_add_gtest(offboard_trajectory_state_test
                  tests/offboard_trajectory_state_test.cpp)
  target_compile_features(offboard_trajectory_state_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_trajectory_state_test
                        drone_city_nav_ros_adapters)
  ament_target_dependencies(offboard_trajectory_state_test geometry_msgs nav_msgs)
  enable_project_warnings(offboard_trajectory_state_test)

  ament_add_gtest(lidar_projection_test tests/lidar_projection_test.cpp)
  target_compile_features(lidar_projection_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_projection_test drone_city_nav_core)
  enable_project_warnings(lidar_projection_test)

  ament_add_gtest(planner_node_config_test
                  tests/planner_node_config_test.cpp)
  target_compile_features(planner_node_config_test PRIVATE cxx_std_20)
  target_link_libraries(planner_node_config_test drone_city_nav_ros_adapters)
  ament_target_dependencies(planner_node_config_test rclcpp)
  enable_project_warnings(planner_node_config_test)

  ament_add_gtest(ros_conversions_test tests/ros_conversions_test.cpp)
  target_compile_features(ros_conversions_test PRIVATE cxx_std_20)
  target_link_libraries(ros_conversions_test drone_city_nav_ros_adapters)
  ament_target_dependencies(ros_conversions_test nav_msgs std_msgs)
  enable_project_warnings(ros_conversions_test)

  ament_add_gtest(static_map_source_test tests/static_map_source_test.cpp)
  target_compile_features(static_map_source_test PRIVATE cxx_std_20)
  target_link_libraries(static_map_source_test drone_city_nav_ros_adapters)
  enable_project_warnings(static_map_source_test)

  ament_add_gtest(static_map_debug_test tests/static_map_debug_test.cpp)
  target_compile_features(static_map_debug_test PRIVATE cxx_std_20)
  target_link_libraries(static_map_debug_test drone_city_nav_ros_adapters)
  ament_target_dependencies(static_map_debug_test nav_msgs sensor_msgs std_msgs)
  enable_project_warnings(static_map_debug_test)

  ament_add_gtest(lidar_debug_pointclouds_test
                  tests/lidar_debug_pointclouds_test.cpp)
  target_compile_features(lidar_debug_pointclouds_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_debug_pointclouds_test drone_city_nav_ros_adapters)
  ament_target_dependencies(lidar_debug_pointclouds_test nav_msgs sensor_msgs)
  enable_project_warnings(lidar_debug_pointclouds_test)
endif()

ament_export_include_directories(include)
ament_export_targets(drone_city_nav_targets HAS_LIBRARY_TARGET)
ament_package()
