#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${BAG_OUTPUT_DIR:-${DRONE_GAZEBO_LOG_DIR:-${repo_root}/log}/rosbag_city_debug}"

mkdir -p "$(dirname "${output_dir}")"

ros2 bag record \
  --output "${output_dir}" \
  /scan \
  /drone_city_nav/lidar_debug_points \
  /drone_city_nav/remembered_lidar_points \
  /drone_city_nav/raw_memory_obstacle_points \
  /drone_city_nav/prohibited_obstacle_points \
  /drone_city_nav/static_building_markers \
  /drone_city_nav/known_passage_markers \
  /drone_city_nav/obstacle_memory_grid \
  /drone_city_nav/obstacle_memory_provenance \
  /drone_city_nav/obstacle_memory_snapshot \
  /drone_city_nav/prohibited_grid \
  /drone_city_nav/path \
  /drone_city_nav/current_waypoint \
  /drone_city_nav/emergency_stop \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_status_v1
