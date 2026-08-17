#!/usr/bin/env bash

start_gazebo_gui_camera_logger() {
  local repo_root="$1"
  local run_log_dir="$2"
  local gui_log_file="$3"
  local output_file="${GZ_GUI_CAMERA_LOG_FILE:-${run_log_dir}/gz_gui_camera.jsonl}"
  local interval_s="${GZ_GUI_CAMERA_LOG_INTERVAL_S:-1}"

  mkdir -p "$(dirname "${output_file}")"
  python3 "${repo_root}/scripts/gazebo_gui_camera_logger.py" \
    --output "${output_file}" \
    --interval-s "${interval_s}" \
    >> "${gui_log_file}" 2>&1 &
  GAZEBO_GUI_CAMERA_LOGGER_PID=$!
}

configure_gazebo_gui_follow_camera() {
  local repo_root="$1"
  local world_name="$2"
  local target="$3"
  local offset="$4"
  local wait_s="$5"
  python3 "${repo_root}/scripts/gazebo_gui_control.py" \
    follow-camera --world "${world_name}" --target "${target}" \
    --offset "${offset}" --wait-s "${wait_s}"
}

wait_for_gazebo_scene_entity() {
  local repo_root="$1"
  local world_name="$2"
  local target="$3"
  local wait_s="$4"
  python3 "${repo_root}/scripts/gazebo_gui_control.py" \
    wait-for-entity --world "${world_name}" --target "${target}" --wait-s "${wait_s}"
}

configure_gazebo_world_running() {
  local repo_root="$1"
  local world_name="$2"
  local wait_s="$3"
  python3 "${repo_root}/scripts/gazebo_gui_control.py" \
    world-running --world "${world_name}" --wait-s "${wait_s}"
}

capture_gazebo_scene_diagnostics() {
  local repo_root="$1"
  local world_name="$2"
  local target="$3"
  local output_dir="$4"
  python3 "${repo_root}/scripts/capture_gazebo_scene_diagnostics.py" \
    --world "${world_name}" --target "${target}" --output-dir "${output_dir}"
}
