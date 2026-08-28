#!/usr/bin/env bash

initialize_runtime_evidence_paths() {
  runtime_artifact_dir="${run_log_dir}/runs/${run_id}"
  runtime_manifest_path="${runtime_artifact_dir}/manifest.json"
  raw_snapshot_capture_log_file="${runtime_artifact_dir}/raw_snapshot_capture.log"
}

prepare_runtime_evidence() {
  local runtime_scenario_path=""
  local -a runtime_manifest_args
  mkdir -p "${runtime_artifact_dir}"
  if bool_is_true "${multi_vehicle_mission}"; then
    runtime_scenario_path="${multi_vehicle_scenario_path}"
  elif [[ -n "${point_to_point_scenario_path}" ]]; then
    runtime_scenario_path="${point_to_point_scenario_path}"
  fi
  runtime_manifest_args=(
    --output "${runtime_manifest_path}"
    --repository "${repo_root}"
    --run-id "${run_id}"
    --config "${city_nav_params_file}"
    --world "${gazebo_world_sdf_path}"
    --mission-type "${mission_type}"
    --mission-goals "${mission_goal_sequence_xyz_m}"
    --lidar-profile "${lidar_profile}"
    --static-map-enabled "${active_static_map}"
  )
  if [[ -n "${runtime_scenario_path}" ]]; then
    runtime_manifest_args+=(--scenario "${runtime_scenario_path}")
  fi
  if [[ -n "${observed_3d_route_volume_bounds_m}" ]]; then
    runtime_manifest_args+=(
      --route-volume-bounds "${observed_3d_route_volume_bounds_m}"
    )
  fi
  python3 "${repo_root}/scripts/runtime_manifest.py" "${runtime_manifest_args[@]}"
}

start_runtime_evidence_capture() {
  echo "Runtime manifest: ${runtime_manifest_path}"
  if bool_is_true "${multi_vehicle_mission}" ||
    bool_is_true "${active_static_map}" || [[ "${lidar_profile}" != "3d" ]] ||
    [[ -z "${observed_3d_route_volume_bounds_m}" ]]; then
    return
  fi
  python3 "${repo_root}/scripts/capture_raw_snapshot_3d.py" \
    --topic /drone_city_nav/raw_obstacle_snapshot_3d \
    --output-directory "${runtime_artifact_dir}" \
    --manifest "${runtime_manifest_path}" \
    --bounds "${observed_3d_route_volume_bounds_m}" \
    > "${raw_snapshot_capture_log_file}" 2>&1 &
}
