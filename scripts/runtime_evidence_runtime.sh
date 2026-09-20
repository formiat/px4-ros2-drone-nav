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
  if [[ -n "${raw_snapshot_bounds_m}" ]]; then
    runtime_manifest_args+=(--raw-snapshot-bounds "${raw_snapshot_bounds_m}")
  fi
  # The launch overrides that change what was flown. Without them the manifest
  # binds a configuration file two runs share while their speed limits differ.
  local override_name
  for override_name in \
    CRUISE_SPEED_MPS ABSOLUTE_SPEED_LIMIT_MPS MAXIMUM_HORIZONTAL_ACCELERATION_MPS2 \
    MAXIMUM_VERTICAL_ACCELERATION_MPS2 MAXIMUM_CONTROL_JERK_MPS3 \
    ENABLE_STATIC_MAP LIDAR_PROFILE LOCALIZATION_PROFILE HEADLESS SMOKE_DURATION_S \
    CAMERA_PROFILE NAVIGATION_SENSOR_PROFILE \
    MISSION_GOALS_XYZ_M POINT_TO_POINT_SCENARIO_PATH CITY_NAV_PARAMS_FILE \
    OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M RAW_SNAPSHOT_BOUNDS_M; do
    if [[ -n "${!override_name:-}" ]]; then
      runtime_manifest_args+=(
        --effective-override "${override_name}=${!override_name}"
      )
    fi
  done
  python3 "${repo_root}/scripts/runtime_manifest.py" "${runtime_manifest_args[@]}"
}

start_runtime_evidence_capture() {
  echo "Runtime manifest: ${runtime_manifest_path}"
  # What every process of the flight consumes, for the resource-budget checks
  # and the budget document; every flight records it, whatever it flies.
  python3 "${repo_root}/scripts/capture_process_resources.py" \
    "${runtime_artifact_dir}/resources.csv" \
    "${runtime_artifact_dir}/resources_host.json" \
    --world "${world_name}" \
    > "${runtime_artifact_dir}/resources_capture.log" 2>&1 &
  if bool_is_true "${multi_vehicle_mission}" ||
    bool_is_true "${active_static_map}" || [[ "${lidar_profile}" != "3d" ]] ||
    [[ -z "${raw_snapshot_bounds_m}" ]]; then
    return
  fi
  python3 "${repo_root}/scripts/capture_raw_snapshot_3d.py" \
    --topic /drone_city_nav/raw_obstacle_snapshot_3d \
    --output-directory "${runtime_artifact_dir}" \
    --manifest "${runtime_manifest_path}" \
    --bounds "${raw_snapshot_bounds_m}" \
    > "${raw_snapshot_capture_log_file}" 2>&1 &
  # The controller-dynamics records the mission check reads: the setpoints
  # against the local position, and the true pose. Every headless flight
  # records them; a GUI flight on request.
  if [[ -n "${headless}" ]] || bool_is_true "${DRONE_GAZEBO_CAPTURE_DYNAMICS:-false}"; then
    python3 "${repo_root}/scripts/capture_tracking_setpoints.py" \
      "${runtime_artifact_dir}/tracking.npz" \
      > "${runtime_artifact_dir}/tracking_capture.log" 2>&1 &
    python3 "${repo_root}/scripts/capture_gazebo_pose.py" \
      "${runtime_artifact_dir}/gz_pose.csv" \
      --world "${world_name}" --model "${default_gazebo_follow_target}" \
      > "${runtime_artifact_dir}/gz_pose.log" 2>&1 &
    # The lidar-inertial estimate, when the localization profile runs it;
    # the mission check holds it against the true pose.
    python3 "${repo_root}/scripts/capture_lidar_inertial_estimate.py" \
      "${runtime_artifact_dir}/lio_estimate.csv" \
      > "${runtime_artifact_dir}/lio_estimate.log" 2>&1 &
    # The visual-inertial estimate, likewise.
    python3 "${repo_root}/scripts/capture_lidar_inertial_estimate.py" \
      "${runtime_artifact_dir}/vio_estimate.csv" \
      --topic /drone_city_nav/visual_inertial_odometry/pose \
      > "${runtime_artifact_dir}/vio_estimate.log" 2>&1 &
  fi
}
