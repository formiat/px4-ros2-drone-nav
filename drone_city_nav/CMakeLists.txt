cmake_minimum_required(VERSION 3.20)
project(drone_city_nav VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
  src/occupancy_grid.cpp
  src/path_smoothing.cpp)
target_compile_features(drone_city_nav_core PUBLIC cxx_std_20)
target_include_directories(
  drone_city_nav_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
enable_project_warnings(drone_city_nav_core)

add_executable(planner_node src/planner_node.cpp)
target_compile_features(planner_node PRIVATE cxx_std_20)
target_link_libraries(planner_node drone_city_nav_core)
ament_target_dependencies(
  planner_node
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  sensor_msgs)
enable_project_warnings(planner_node)

add_executable(px4_offboard_node src/px4_offboard_node.cpp)
target_compile_features(px4_offboard_node PRIVATE cxx_std_20)
target_link_libraries(px4_offboard_node drone_city_nav_core)
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

install(
  TARGETS
    drone_city_nav_core
    planner_node
    px4_offboard_node
    mission_monitor_node
  EXPORT drone_city_nav_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(planner_core_test tests/planner_core_test.cpp)
  target_compile_features(planner_core_test PRIVATE cxx_std_20)
  target_link_libraries(planner_core_test drone_city_nav_core)
  enable_project_warnings(planner_core_test)
endif()

ament_export_include_directories(include)
ament_export_targets(drone_city_nav_targets HAS_LIBRARY_TARGET)
ament_package()
