#!/usr/bin/env bash

# Runtime helpers shared by every finite multi-vehicle scenario.

load_multi_vehicle_sim_scenario() {
  local requested_mission_type="$1"
  local scenario_override="$2"
  local default_scenario="drone_city_nav/config/intercept_scenario.json"
  local scenario_metadata_tsv=""
  local scenario_tsv=""

  multi_vehicle_mission="false"
  cooperative_traffic_mission="false"
  case "${requested_mission_type}" in
  point_to_point) ;;
  intercept | multi_intercept)
    multi_vehicle_mission="true"
    ;;
  cooperative_traffic)
    multi_vehicle_mission="true"
    cooperative_traffic_mission="true"
    default_scenario="drone_city_nav/config/cooperative_traffic_scenario.json"
    ;;
  *)
    echo "Unsupported MISSION_TYPE=${requested_mission_type}; expected point_to_point, intercept, multi_intercept, or cooperative_traffic" >&2
    return 1
    ;;
  esac
  if [[ "${requested_mission_type}" == "multi_intercept" ]]; then
    default_scenario="drone_city_nav/config/multi_intercept_2v2_scenario.json"
  fi
  multi_vehicle_scenario_path="$(
    make_abs_path "${scenario_override:-${default_scenario}}"
  )"
  multi_vehicle_ids=()
  multi_vehicle_roles=()
  multi_vehicle_px4_namespaces=()
  multi_vehicle_px4_model_targets=()
  multi_vehicle_gazebo_model_names=()
  multi_vehicle_map_start_poses=()
  multi_vehicle_gazebo_spawn_poses=()
  multi_vehicle_world_name="generated_city"
  multi_vehicle_initial_altitude_m="18.0"
  multi_vehicle_minimum_target_z_m="1.0"
  multi_vehicle_maximum_target_z_m="32.0"
  if ! bool_is_true "${multi_vehicle_mission}"; then
    return 0
  fi
  if ! scenario_metadata_tsv="$(
    python3 "${repo_root}/drone_city_nav/launch/intercept_scenario.py" \
      --scenario "${multi_vehicle_scenario_path}" --format metadata-tsv
  )"; then
    echo "Failed to resolve multi-vehicle scenario metadata: ${multi_vehicle_scenario_path}" >&2
    return 1
  fi
  IFS=$'\t' read -r multi_vehicle_world_name \
    multi_vehicle_initial_altitude_m multi_vehicle_minimum_target_z_m \
    multi_vehicle_maximum_target_z_m <<< "${scenario_metadata_tsv}"
  if ! scenario_tsv="$(
    python3 "${repo_root}/drone_city_nav/launch/intercept_scenario.py" \
      --scenario "${multi_vehicle_scenario_path}" --format tsv
  )"; then
    echo "Failed to resolve multi-vehicle scenario: ${multi_vehicle_scenario_path}" >&2
    return 1
  fi
  while IFS=$'\t' read -r vehicle_id vehicle_role px4_namespace \
    vehicle_px4_model_target gazebo_model_name map_x map_y map_z \
    gazebo_x gazebo_y gazebo_z yaw_rad; do
    [[ -n "${vehicle_id}" ]] || continue
    multi_vehicle_ids+=("${vehicle_id}")
    multi_vehicle_roles+=("${vehicle_role}")
    multi_vehicle_px4_namespaces+=("${px4_namespace}")
    multi_vehicle_px4_model_targets+=("${vehicle_px4_model_target}")
    multi_vehicle_gazebo_model_names+=("${gazebo_model_name}")
    multi_vehicle_map_start_poses+=("${map_x},${map_y},${map_z},0,0,${yaw_rad}")
    multi_vehicle_gazebo_spawn_poses+=(
      "${gazebo_x},${gazebo_y},${gazebo_z},0,0,${yaw_rad}"
    )
  done <<< "${scenario_tsv}"
  if [[ "${#multi_vehicle_ids[@]}" -lt 2 ]]; then
    echo "Multi-vehicle scenario must resolve at least two vehicles" >&2
    return 1
  fi
}

load_point_to_point_sim_scenario() {
  local scenario_override="$1"
  local scenario_tsv=""

  point_to_point_scenario_path=""
  point_to_point_world_name=""
  point_to_point_px4_model_target=""
  point_to_point_gazebo_model_name=""
  point_to_point_map_start_pose=""
  point_to_point_gazebo_spawn_pose=""
  if [[ -z "${scenario_override}" ]]; then
    return 0
  fi
  point_to_point_scenario_path="$(make_abs_path "${scenario_override}")"
  if ! scenario_tsv="$(
    python3 "${repo_root}/drone_city_nav/launch/point_to_point_scenario.py" \
      --scenario "${point_to_point_scenario_path}" --format runtime-tsv
  )"; then
    echo "Failed to resolve point-to-point scenario: ${point_to_point_scenario_path}" >&2
    return 1
  fi
  local map_x map_y map_z gazebo_x gazebo_y gazebo_z yaw_rad
  IFS=$'\t' read -r point_to_point_world_name point_to_point_px4_model_target \
    point_to_point_gazebo_model_name map_x map_y map_z gazebo_x gazebo_y gazebo_z \
    yaw_rad <<< "${scenario_tsv}"
  point_to_point_map_start_pose="${map_x},${map_y},${map_z},0,0,${yaw_rad}"
  point_to_point_gazebo_spawn_pose="${gazebo_x},${gazebo_y},${gazebo_z},0,0,${yaw_rad}"
}

resolve_point_to_point_runtime() {
  local scenario_override="$1"

  load_point_to_point_sim_scenario "${scenario_override}"
  scenario_world_name="${point_to_point_world_name:-generated_city}"
}

resolve_point_to_point_gazebo_spawn() {
  local start_x_m start_y_m start_z_m unused_roll unused_pitch start_yaw_rad

  IFS=',' read -r start_x_m start_y_m start_z_m unused_roll unused_pitch \
    start_yaw_rad <<< "${point_to_point_gazebo_spawn_pose:--171.0,-81.0,0.3,0,0,0}"
  point_gazebo_spawn_x_m="${SIM_START_X_M:-${start_x_m}}"
  point_gazebo_spawn_y_m="${SIM_START_Y_M:-${start_y_m}}"
  point_gazebo_spawn_z_m="${SIM_START_Z_M:-${start_z_m}}"
  point_gazebo_spawn_yaw_rad="${SIM_START_YAW_RAD:-${start_yaw_rad}}"
}

prepare_multi_vehicle_model_resources() {
  local instance
  local evader_model_name

  if ! bool_is_true "${multi_vehicle_mission}"; then
    return 0
  fi
  for instance in "${!multi_vehicle_ids[@]}"; do
    [[ "${multi_vehicle_roles[instance]}" == "evader" ]] || continue
    evader_model_name="${multi_vehicle_px4_model_targets[instance]#gz_}"
    if [[ -e "${runtime_models_dir}/${evader_model_name}" ]]; then
      continue
    fi
    cp -a "${repo_root}/drone_city_nav/models/x500_lidar_2d" \
      "${runtime_models_dir}/${evader_model_name}"
    python3 "${repo_root}/scripts/configure_drone_marker_color.py" \
      "${runtime_models_dir}/${evader_model_name}" \
      --model-name "${evader_model_name}"
  done
}

print_log_tail() {
  local label="$1"
  local file="$2"
  echo "---- ${label}: ${file} ----" >&2
  perl -pe 's{\e\[[0-9;?]*[ -/]*[@-~]}{}g; s/\r/\n/g' "${file}" \
    | sed '/^pxh> *$/d' \
    | tail -n 80 >&2 || true
}

check_headless_run() {
  local validation_args=(
    --ros-log "${ros_log_file}"
    --px4-log "${px4_log_file}"
    --mission-type "${mission_type}"
    --expected-static "${expected_static_map}"
    --expected-memory "${expected_obstacle_memory}"
    --enable-lidar-debug "${enable_lidar_debug}"
    --expect-noncooperative-avoidance "${intercept_noncooperative_avoidance_enabled}"
  )
  if bool_is_true "${multi_vehicle_mission}"; then
    validation_args+=(--expected-vehicles "${#multi_vehicle_ids[@]}")
    for instance in "${!multi_vehicle_ids[@]}"; do
      [[ "${instance}" -eq 0 ]] ||
        validation_args+=(--px4-log "${multi_vehicle_px4_logs[instance]}")
    done
  fi
  if [[ -n "${mission_check}" ]] || bool_is_true "${multi_vehicle_mission}"; then
    validation_args+=(--mission-check)
  fi
  if bool_is_true "${allow_mission_failure}"; then
    validation_args+=(--allow-mission-failure)
  fi

  if ! python3 "${repo_root}/scripts/validate_drone_nav_headless.py" \
    "${validation_args[@]}"; then
    print_log_tail "Gazebo" "${gz_log_file}"
    print_log_tail "PX4 SITL" "${px4_log_file}"
    if bool_is_true "${multi_vehicle_mission}"; then
      for instance in "${!multi_vehicle_ids[@]}"; do
        [[ "${instance}" -eq 0 ]] || print_log_tail \
          "${multi_vehicle_ids[instance]} PX4 SITL" \
          "${multi_vehicle_px4_logs[instance]}"
      done
    fi
    print_log_tail "ROS launch" "${ros_log_file}"
    return 1
  fi
  return 0
}
