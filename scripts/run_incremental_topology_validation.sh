#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
validation_id="${INCREMENTAL_TOPOLOGY_VALIDATION_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
validation_root="${DRONE_GAZEBO_VALIDATION_LOG_ROOT:-${repo_root}/log/incremental_topology_validation/${validation_id}}"

run_stage() {
  local stage="$1"
  shift

  local log_dir="${validation_root}/${stage}"
  printf '\nINCREMENTAL_TOPOLOGY_VALIDATION_STAGE stage=%s log_dir=%s\n' \
    "${stage}" "${log_dir}"
  env DRONE_GAZEBO_LOG_DIR="${log_dir}" "$@"
  printf 'INCREMENTAL_TOPOLOGY_VALIDATION_STAGE_RESULT stage=%s success=true\n' \
    "${stage}"
  sleep 2
}

cd "${repo_root}"

common_no_static=(
  ENABLE_STATIC_MAP=false
  LIDAR_PROFILE=3d
  REQUIRE_INCREMENTAL_TOPOLOGY_EVIDENCE=true
  MAXIMUM_NO_EXECUTABLE_ROUTE_AGE_MS="${MAXIMUM_NO_EXECUTABLE_ROUTE_AGE_MS:-10000}"
)

run_stage manhattan_low_altitude_smoke \
  env -u MISSION_GOALS_XYZ_M \
  "${common_no_static[@]}" \
  MISSION_TYPE=point_to_point \
  POINT_TO_POINT_SCENARIO_PATH=drone_city_nav/config/manhattan_low_altitude_point_to_point_scenario.json \
  REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING=true \
  OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M=42,147,1.5,66,177,8.5 \
  SMOKE_DURATION_S="${MANHATTAN_LOW_ALTITUDE_TIMEOUT_S:-300}" \
  make sim-headless

run_stage manhattan_four_waypoints \
  env -u POINT_TO_POINT_SCENARIO_PATH \
  "${common_no_static[@]}" \
  MISSION_TYPE=point_to_point \
  MISSION_GOALS_XYZ_M='216,378,18;216,54,18;54,378,18;54,54,18' \
  SMOKE_DURATION_S="${MANHATTAN_POINT_TO_POINT_TIMEOUT_S:-1800}" \
  make sim-headless

run_stage manhattan_cooperative \
  "${common_no_static[@]}" \
  MISSION_TYPE=cooperative_traffic \
  MULTI_VEHICLE_SCENARIO_PATH=drone_city_nav/config/cooperative_traffic_scenario.json \
  COOPERATIVE_MISSION_TIMEOUT_S="${MANHATTAN_COOPERATIVE_MISSION_TIMEOUT_S:-1200}" \
  SMOKE_DURATION_S="${MANHATTAN_COOPERATIVE_TIMEOUT_S:-1300}" \
  make sim-cooperative-traffic-headless

run_stage urban_point_to_point \
  SMOKE_DURATION_S="${URBAN_POINT_TO_POINT_TIMEOUT_S:-600}" \
  make sim-urban-point-to-point-headless

run_stage urban_cooperative \
  COOPERATIVE_MISSION_TIMEOUT_S="${URBAN_COOPERATIVE_MISSION_TIMEOUT_S:-780}" \
  SMOKE_DURATION_S="${URBAN_COOPERATIVE_TIMEOUT_S:-900}" \
  make sim-cooperative-traffic-urban-headless

printf '\nINCREMENTAL_TOPOLOGY_VALIDATION_RESULT success=true stages=5 log_root=%s\n' \
  "${validation_root}"
