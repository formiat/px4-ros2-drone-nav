#!/usr/bin/env bash

# Runtime helpers shared by every finite interceptor scenario.

load_intercept_sim_scenario() {
  local requested_mission_type="$1"
  local scenario_override="$2"
  local default_scenario="drone_city_nav/config/intercept_scenario.json"
  local scenario_tsv=""

  intercept_mission="false"
  case "${requested_mission_type}" in
  point_to_point) ;;
  intercept | multi_intercept) intercept_mission="true" ;;
  *)
    echo "Unsupported MISSION_TYPE=${requested_mission_type}; expected point_to_point, intercept, or multi_intercept" >&2
    return 1
    ;;
  esac
  if [[ "${requested_mission_type}" == "multi_intercept" ]]; then
    default_scenario="drone_city_nav/config/multi_intercept_2v2_scenario.json"
  fi
  intercept_scenario_path="$(
    make_abs_path "${scenario_override:-${default_scenario}}"
  )"
  intercept_vehicle_ids=()
  intercept_vehicle_roles=()
  intercept_px4_namespaces=()
  intercept_px4_model_targets=()
  intercept_gazebo_model_names=()
  intercept_map_start_poses=()
  intercept_gazebo_spawn_poses=()
  if ! bool_is_true "${intercept_mission}"; then
    return 0
  fi
  if ! scenario_tsv="$(
    python3 "${repo_root}/drone_city_nav/launch/intercept_scenario.py" \
      --scenario "${intercept_scenario_path}" --format tsv
  )"; then
    echo "Failed to resolve intercept scenario: ${intercept_scenario_path}" >&2
    return 1
  fi
  while IFS=$'\t' read -r vehicle_id vehicle_role px4_namespace \
    vehicle_px4_model_target gazebo_model_name map_x map_y map_z \
    gazebo_x gazebo_y gazebo_z yaw_rad; do
    [[ -n "${vehicle_id}" ]] || continue
    intercept_vehicle_ids+=("${vehicle_id}")
    intercept_vehicle_roles+=("${vehicle_role}")
    intercept_px4_namespaces+=("${px4_namespace}")
    intercept_px4_model_targets+=("${vehicle_px4_model_target}")
    intercept_gazebo_model_names+=("${gazebo_model_name}")
    intercept_map_start_poses+=("${map_x},${map_y},${map_z},0,0,${yaw_rad}")
    intercept_gazebo_spawn_poses+=(
      "${gazebo_x},${gazebo_y},${gazebo_z},0,0,${yaw_rad}"
    )
  done <<< "${scenario_tsv}"
  if [[ "${#intercept_vehicle_ids[@]}" -lt 2 ]]; then
    echo "Intercept scenario must resolve at least two vehicles" >&2
    return 1
  fi
}

prepare_intercept_evader_model_resources() {
  local instance
  local evader_model_name

  if ! bool_is_true "${intercept_mission}"; then
    return 0
  fi
  for instance in "${!intercept_vehicle_ids[@]}"; do
    [[ "${intercept_vehicle_roles[instance]}" == "evader" ]] || continue
    evader_model_name="${intercept_px4_model_targets[instance]#gz_}"
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
