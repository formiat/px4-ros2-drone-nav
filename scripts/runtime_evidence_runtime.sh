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
    --unreachable-goal-injected "${mission_goal_unreachable}"
    --truth-occupancy-3d "${truth_occupancy_3d_path}"
  )
  if [[ -n "${runtime_scenario_path}" ]]; then
    runtime_manifest_args+=(--scenario "${runtime_scenario_path}")
  fi
  if [[ -n "${observed_3d_route_volume_bounds_m}" ]]; then
    runtime_manifest_args+=(
      --route-volume-bounds="${observed_3d_route_volume_bounds_m}"
    )
  fi
  if [[ -n "${raw_snapshot_bounds_m}" ]]; then
    runtime_manifest_args+=(--raw-snapshot-bounds="${raw_snapshot_bounds_m}")
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
    OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M RAW_SNAPSHOT_BOUNDS_M \
    BLANK_PANELS REAL_TIME_FACTOR LIGHT_FAULTS LIGHT_FAULT_SEED \
    LIGHT_BATTERY_S RETURN_HOME_EXPECTED STREAM_FAULTS \
    STREAM_FAULT_SEED RETURN_HOME_ALLOWED RETURN_TRIGGERS_ALLOWED VEHICLE_INTACT_EXPECTED \
    IMPERFECTIONS IMPERFECTION_SEED IMU_FAULTS IMU_FAULT_SEED \
    HEAT_SOURCES SMOKE_ZONES SMOKE_SEED MPPI_ROLLOUTS SMOKE_FACE FLIGHT_CPUSET; do
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
  # Roadmap item 27 block 1 (specification K29): the pilot, an evaluation
  # component, puts each vehicle in offboard and arms it; nothing of the stack
  # does. PILOT_EVENTS schedules its and the operator's actions, PILOT_RC a
  # radio heard from the start.
  local pilot_namespaces=""
  if bool_is_true "${multi_vehicle_mission:-false}"; then
    pilot_namespaces="${multi_vehicle_px4_namespaces[*]}"
  fi
  python3 "${repo_root}/scripts/sim_pilot.py" --namespaces "${pilot_namespaces}" \
    --events "${PILOT_EVENTS:-}" $(bool_is_true "${PILOT_RC:-false}" && echo --rc) \
    --output "${runtime_artifact_dir}/pilot.csv" \
    > "${runtime_artifact_dir}/pilot.log" 2>&1 &
  # Roadmap item 17 stage 5: the light the camera vehicle carries, its
  # battery (the charge is published) and its injected failures (never
  # announced); the schedule and every change land beside the manifest.
  if [[ "${CAMERA_PROFILE:-}" == "stereo_tof" ]] && ! bool_is_true "${multi_vehicle_mission}"; then
    python3 "${repo_root}/scripts/carried_light.py" \
      --world "${world_name}" --profile "${LIGHT_FAULTS:-none}" \
      --seed "${LIGHT_FAULT_SEED:-0}" ${LIGHT_BATTERY_S:+--battery-s "${LIGHT_BATTERY_S}"} \
      ${LIGHT_ZONES:+--zones="${LIGHT_ZONES}"} \
      --model "${default_gazebo_follow_target}" \
      --output "${runtime_artifact_dir}/carried_light.csv" \
      > "${runtime_artifact_dir}/carried_light.log" 2>&1 &
    # The camera stream's failures, the same kind of component: the camera
    # driver reads the pair from its relay (run_drone_nav_sim.sh). Under the
    # imperfections profile (roadmap item 35, K52) the relay also moves every
    # frame's stamp by +-2 ms, the sign from the seed.
    if [[ ( -n "${STREAM_FAULTS:-}" && "${STREAM_FAULTS}" != none ) ]] ||
      { imperfection_family_on stamps && [[ -z "${SMOKE_ZONES:-}" ]]; }; then
      local frame_offset_ms=0
      if imperfection_family_on stamps; then
        frame_offset_ms=$(( ${IMPERFECTION_SEED:-0} % 2 == 0 ? 2 : -2 ))
      fi
      python3 "${repo_root}/scripts/camera_stream_faults.py" \
        --world "${world_name}" --model "${default_gazebo_follow_target}" \
        --profile "${STREAM_FAULTS:-none}" --seed "${STREAM_FAULT_SEED:-0}" \
        --stamp-offset-ms "${frame_offset_ms}" \
        --output "${runtime_artifact_dir}/camera_stream.csv" \
        > "${runtime_artifact_dir}/camera_stream.log" 2>&1 &
    fi
  fi
  if ! bool_is_true "${multi_vehicle_mission}"; then
    # Specification K28: the autopilot's IMU stream failing, the same kind of
    # component: the estimators read its samples from the relay (the holes of
    # K28, or the jitter and gaps of the imperfections profile, K52).
    if [[ -n "${IMU_FAULTS:-}" && "${IMU_FAULTS}" != none ]]; then
      python3 "${repo_root}/scripts/imu_stream_faults.py" \
        --profile "${IMU_FAULTS}" --seed "${IMU_FAULT_SEED:-0}" \
        --output "${runtime_artifact_dir}/imu_stream.csv" \
        > "${runtime_artifact_dir}/imu_stream.log" 2>&1 &
    fi
    # Roadmap item 35 (K52): the lidar's sweeps delivered with a real driver's
    # cadence spread; the bridge reads the relay (run_drone_nav_sim.sh).
    if imperfection_family_on stamps && [[ "${NAVIGATION_SENSOR_PROFILE:-}" == lidar &&
      -z "${SMOKE_ZONES:-}" ]]; then
      python3 "${repo_root}/scripts/lidar_sweep_jitter.py" \
        --world "${world_name}" --model "${default_gazebo_follow_target}" \
        --seed "${IMPERFECTION_SEED:-0}" \
        --output "${runtime_artifact_dir}/lidar_sweeps.csv" \
        > "${runtime_artifact_dir}/lidar_sweeps.log" 2>&1 &
    fi
  fi
  # The evaluation overlay of RViz: the vehicle's decisions in words, its
  # estimate against the truth, and the carried light. Only where RViz is
  # open; a headless acceptance flight carries nothing of it.
  if bool_is_true "${enable_rviz}" && ! bool_is_true "${multi_vehicle_mission}"; then
    python3 "${repo_root}/scripts/evaluation_overlay.py" \
      --world "${world_name}" --model "${default_gazebo_follow_target}" \
      > "${runtime_artifact_dir}/evaluation_overlay.log" 2>&1 &
  fi
  # A flight rendered afterwards (RENDER_BAG=1, scripts/render_flight_video.py):
  # what both RViz views show, on the simulation's clock, for RViz to play back
  # after the flight; the true pose and the light are the run's own records.
  if bool_is_true "${RENDER_BAG:-false}"; then
    ros2 bag record --use-sim-time -s sqlite3 -o "${runtime_artifact_dir}/render_bag" --topics \
      /tf /tf_static /drone_city_nav/drone_marker /drone_city_nav/mppi/markers \
      /drone_city_nav/mppi/path /drone_city_nav/raw_memory_darkness_points_3d \
      /drone_city_nav/raw_memory_obstacle_points /drone_city_nav/raw_memory_obstacle_points_3d \
      /drone_city_nav/raw_occupied_cells /drone_city_nav/remembered_lidar_points \
      /drone_city_nav/static_building_markers /drone_city_nav/static_map_points \
      /drone_city_nav/vehicle_directions /drone_city_nav/current_lidar_returns_3d \
      /drone_city_nav/raw_lidar_hit_points_3d /evaluation/markers_follow /evaluation/markers_top \
      /stereo/left/image \
      > "${runtime_artifact_dir}/render_bag.log" 2>&1 &
  fi
  if bool_is_true "${multi_vehicle_mission}" ||
    bool_is_true "${active_static_map}"; then
    return
  fi
  # The snapshot only where the scenario states its volume; the records
  # below in every headless flight (the dark hall states none and flew
  # without its true pose, r1466, r1471).
  if [[ -n "${raw_snapshot_bounds_m}" ]]; then
    python3 "${repo_root}/scripts/capture_raw_snapshot_3d.py" \
      --topic /drone_city_nav/raw_obstacle_snapshot_3d \
      --output-directory "${runtime_artifact_dir}" \
      --manifest "${runtime_manifest_path}" \
      --bounds="${raw_snapshot_bounds_m}" \
      > "${raw_snapshot_capture_log_file}" 2>&1 &
  fi
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
    gz_pose_capture_pid=$!
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
