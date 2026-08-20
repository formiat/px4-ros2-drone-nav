#!/usr/bin/env bash

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
    if [[ "${model_name}" == "x500_lidar_2d" ||
      "${model_name}" == "lidar_2d_v2" ||
      "${model_name}" == "lidar_3d_v1" ]]; then
      continue
    fi
    ln -s "${px4_model}" "${runtime_models_dir}/${model_name}"
  done

  local runtime_drone_model_name="x500_lidar_2d"
  local runtime_sensor_model_name="lidar_2d_v2"
  local materialized_lidar_profile="2d"
  if [[ "${lidar_profile}" == "3d" ]]; then
    runtime_drone_model_name="x500_lidar_3d"
    runtime_sensor_model_name="lidar_3d_v1"
    materialized_lidar_profile="3d"
  fi
  cp -a "${repo_root}/drone_city_nav/models/x500_lidar_2d" \
    "${runtime_models_dir}/${runtime_drone_model_name}"
  python3 "${repo_root}/scripts/configure_drone_lidar_model.py" \
    "${runtime_models_dir}/${runtime_drone_model_name}" \
    --model-name "${runtime_drone_model_name}" \
    --lidar-profile "${materialized_lidar_profile}"
  cp -a "${repo_root}/drone_city_nav/models/${runtime_sensor_model_name}" \
    "${runtime_models_dir}/${runtime_sensor_model_name}"
  prepare_multi_vehicle_model_resources

  local lidar_visibility_mode="no-static-2d"
  if bool_is_true "${active_static_map}"; then
    lidar_visibility_mode="static"
  elif [[ "${lidar_profile}" == "3d" ]]; then
    lidar_visibility_mode="no-static-3d"
  fi
  python3 "${repo_root}/scripts/configure_lidar_visibility.py" \
    "${runtime_models_dir}/${runtime_sensor_model_name}/model.sdf" \
    --mode "${lidar_visibility_mode}" \
    --enabled "$([[ "${lidar_profile}" == "none" ]] && printf 'false' || printf 'true')"
}
