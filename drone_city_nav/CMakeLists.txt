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
  src/corner_rounding.cpp
  src/current_lidar_overlay.cpp
  src/debug_image.cpp
  src/grid_config.cpp
  src/grid_overlay.cpp
  src/lidar_debug_renderer.cpp
  src/lidar_projection.cpp
  src/lidar_snapshot_writer.cpp
  src/navigation_pose.cpp
  src/offboard_path_follower.cpp
  src/offboard_velocity_follower.cpp
  src/offboard_target_continuity.cpp
  src/obstacle_memory.cpp
  src/occupancy_grid.cpp
  src/path_smoothing.cpp
  src/planning_grid_builder.cpp
  src/planner_core.cpp
  src/static_city_map.cpp
  src/trajectory.cpp)
target_compile_features(drone_city_nav_core PUBLIC cxx_std_20)
target_include_directories(
  drone_city_nav_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
enable_project_warnings(drone_city_nav_core)

add_library(
  drone_city_nav_ros_adapters
  src/planner_node_config.cpp
  src/ros_conversions.cpp
  src/static_map_debug.cpp
  src/static_map_source.cpp)
target_compile_features(drone_city_nav_ros_adapters PUBLIC cxx_std_20)
target_link_libraries(drone_city_nav_ros_adapters drone_city_nav_core)
ament_target_dependencies(
  drone_city_nav_ros_adapters
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs)
enable_project_warnings(drone_city_nav_ros_adapters)

add_executable(planner_node src/planner_node.cpp)
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

add_executable(px4_offboard_node src/px4_offboard_node.cpp)
target_compile_features(px4_offboard_node PRIVATE cxx_std_20)
target_link_libraries(px4_offboard_node drone_city_nav_core drone_city_nav_ros_adapters)
ament_target_dependencies(
  px4_offboard_node
  nav_msgs
  px4_msgs
  rclcpp
  std_msgs)
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

add_executable(lidar_debug_node src/lidar_debug_node.cpp src/lidar_radar_markers.cpp)
target_compile_features(lidar_debug_node PRIVATE cxx_std_20)
target_link_libraries(lidar_debug_node drone_city_nav_core)
ament_target_dependencies(
  lidar_debug_node
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs
  visualization_msgs)
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

  ament_add_gtest(clearance_field_test tests/clearance_field_test.cpp)
  target_compile_features(clearance_field_test PRIVATE cxx_std_20)
  target_link_libraries(clearance_field_test drone_city_nav_core)
  enable_project_warnings(clearance_field_test)

  ament_add_gtest(lidar_radar_markers_test tests/lidar_radar_markers_test.cpp
                  src/lidar_radar_markers.cpp)
  target_compile_features(lidar_radar_markers_test PRIVATE cxx_std_20)
  target_link_libraries(lidar_radar_markers_test drone_city_nav_core)
  ament_target_dependencies(lidar_radar_markers_test visualization_msgs)
  enable_project_warnings(lidar_radar_markers_test)

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

  ament_add_gtest(corner_rounding_test tests/corner_rounding_test.cpp)
  target_compile_features(corner_rounding_test PRIVATE cxx_std_20)
  target_link_libraries(corner_rounding_test drone_city_nav_core)
  enable_project_warnings(corner_rounding_test)

  ament_add_gtest(offboard_path_follower_test
                  tests/offboard_path_follower_test.cpp)
  target_compile_features(offboard_path_follower_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_path_follower_test drone_city_nav_core)
  enable_project_warnings(offboard_path_follower_test)

  ament_add_gtest(offboard_velocity_follower_test
                  tests/offboard_velocity_follower_test.cpp)
  target_compile_features(offboard_velocity_follower_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_velocity_follower_test drone_city_nav_core)
  enable_project_warnings(offboard_velocity_follower_test)

  ament_add_gtest(offboard_target_continuity_test
                  tests/offboard_target_continuity_test.cpp)
  target_compile_features(offboard_target_continuity_test PRIVATE cxx_std_20)
  target_link_libraries(offboard_target_continuity_test drone_city_nav_core)
  enable_project_warnings(offboard_target_continuity_test)

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
endif()

ament_export_include_directories(include)
ament_export_targets(drone_city_nav_targets HAS_LIBRARY_TARGET)
ament_package()
