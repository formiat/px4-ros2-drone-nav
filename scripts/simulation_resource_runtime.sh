#!/usr/bin/env bash

# The launch arguments of the sensor relays (appended to ros_launch_args). Roadmap item
# 17 stage 5: the camera stream's failures, injected by a relay the camera driver then
# reads from (runtime_evidence_runtime.sh). Roadmap item 35 (K52): the real vehicle's
# imperfections: the IMU stream's jitter rides the K28 relay, the frames' stamp offset the
# stream relay, the lidar's sweep cadence its own relay; the model's part is the
# preparation's (configure_imperfections.py below).
append_sensor_relay_launch_args() {
  if imperfection_family_on stamps; then
    export IMU_FAULTS="${IMU_FAULTS:-jitter}"
    export IMU_FAULT_SEED="${IMU_FAULT_SEED:-${IMPERFECTION_SEED:-0}}"
    if [[ -z "${SMOKE_ZONES:-}" && "${NAVIGATION_SENSOR_PROFILE:-}" == lidar ]]; then
      ros_launch_args+=(lidar_scan_topic_suffix:=/relayed)
    fi
  fi
  if [[ ( -n "${STREAM_FAULTS:-}" && "${STREAM_FAULTS}" != none ) ]] ||
    { imperfection_family_on stamps && [[ -z "${SMOKE_ZONES:-}" ]]; }; then
    ros_launch_args+=(stereo_image_topic_suffix:=/relayed)
  fi
}

prepare_runtime_resources() {
  rm -rf "${runtime_dir}"
  mkdir -p "${runtime_models_dir}" "${runtime_worlds_dir}"
  if [[ -z "${custom_world_sdf_path}" ]]; then
    install -D "${repo_root}/drone_city_nav/worlds/${world_name}.sdf" \
      "${runtime_worlds_dir}/${world_name}.sdf"
    gazebo_world_sdf_path="${runtime_worlds_dir}/${world_name}.sdf"
  else
    gazebo_world_sdf_path="${custom_world_sdf_path}"
  fi

  local px4_model
  local model_name
  for px4_model in "${px4_models_dir}"/*; do
    [[ -d "${px4_model}" ]] || continue
    model_name="$(basename "${px4_model}")"
    if [[ "${model_name}" == "x500_lidar_3d" ||
      "${model_name}" == "lidar_3d_v1" ]]; then
      continue
    fi
    ln -s "${px4_model}" "${runtime_models_dir}/${model_name}"
  done

  local runtime_drone_model_name="x500_lidar_3d"
  local runtime_sensor_model_name="lidar_3d_v1"
  cp -a "${repo_root}/drone_city_nav/models/x500_lidar_3d" \
    "${runtime_models_dir}/${runtime_drone_model_name}"
  python3 "${repo_root}/scripts/configure_drone_lidar_model.py" \
    "${runtime_models_dir}/${runtime_drone_model_name}" \
    --model-name "${runtime_drone_model_name}" \
    --camera-profile "${camera_profile}" \
    $([[ "${navigation_sensor_profile}" == "stereo_tof" ]] && printf '%s' '--without-lidar')
  cp -a "${repo_root}/drone_city_nav/models/${runtime_sensor_model_name}" \
    "${runtime_models_dir}/${runtime_sensor_model_name}"
  if [[ "${camera_profile}" == "stereo_tof" ]]; then
    cp -a "${repo_root}/drone_city_nav/models/stereo_tof_v1" \
      "${runtime_models_dir}/stereo_tof_v1"
  fi

  local lidar_visibility_mode="no-static-3d"
  if bool_is_true "${active_static_map}"; then
    lidar_visibility_mode="static"
  fi
  python3 "${repo_root}/scripts/configure_lidar_visibility.py" \
    "${runtime_models_dir}/${runtime_sensor_model_name}/model.sdf" \
    --mode "${lidar_visibility_mode}" \
    --enabled true
  # Roadmap item 35 (K52): the real vehicle's imperfections in the runtime
  # model alone: the IMU's bias walk, the rotors' vibration, the sensor set's
  # pose turned by the calibration's residual.
  local model_families=()
  local family
  for family in imu vibration extrinsics; do
    imperfection_family_on "${family}" && model_families+=("${family}")
  done
  if [[ ${#model_families[@]} -gt 0 ]]; then
    python3 "${repo_root}/scripts/configure_imperfections.py" \
      "${runtime_models_dir}" \
      --seed "${IMPERFECTION_SEED:-0}" \
      --px4-models-dir "${px4_models_dir}" \
      --families "$(IFS=,; echo "${model_families[*]}")"
  fi
}
