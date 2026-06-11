#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_AUTOPILOT_DIR:-${repo_root}/external/PX4-Autopilot}"
ros_distro="${ROS_DISTRO:-jazzy}"
world_name="generated_city"
px4_model_target="${PX4_MODEL_TARGET:-gz_x500_lidar_2d}"
startup_sleep_s="${STARTUP_SLEEP_S:-8}"
smoke_duration_s="${SMOKE_DURATION_S:-0}"
px4_log_file="${PX4_LOG_FILE:-${repo_root}/log/px4_city_mvp.log}"
uxrce_log_file="${UXRCE_AGENT_LOG_FILE:-${repo_root}/log/uxrce_agent_city_mvp.log}"
ros_log_file="${ROS_LOG_FILE:-${repo_root}/log/ros_city_mvp.log}"
gz_log_file="${GZ_LOG_FILE:-${repo_root}/log/gz_city_mvp.log}"
px4_param_delay_s="${PX4_PARAM_DELAY_S:-6}"
mission_check="${MISSION_CHECK:-}"
headless="${HEADLESS:-}"
spawn_x_m="${SIM_START_X_M:--75}"
spawn_y_m="${SIM_START_Y_M:--45}"
spawn_z_m="${SIM_START_Z_M:-0.3}"
spawn_yaw_rad="${SIM_START_YAW_RAD:-0}"
runtime_dir="${repo_root}/build/gazebo_city_mvp"
runtime_models_dir="${runtime_dir}/models"
runtime_worlds_dir="${runtime_dir}/worlds"
px4_models_dir="${px4_dir}/Tools/simulation/gz/models"
px4_plugins_dir="${px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
px4_server_config="${px4_dir}/src/modules/simulation/gz_bridge/server.config"

if [[ ! -d "${px4_dir}" ]]; then
  echo "PX4-Autopilot was not found at ${px4_dir}" >&2
  echo "Run scripts/setup_px4_autopilot.sh first or set PX4_AUTOPILOT_DIR." >&2
  exit 1
fi

set +u
source "/opt/ros/${ros_distro}/setup.bash"
source /opt/px4_msgs_ws/install/setup.bash
set -u

prepare_runtime_resources() {
  rm -rf "${runtime_dir}"
  mkdir -p "${runtime_models_dir}" "${runtime_worlds_dir}"
  install -D "${repo_root}/drone_city_nav/worlds/${world_name}.sdf" \
    "${runtime_worlds_dir}/${world_name}.sdf"

  local px4_model
  local model_name
  for px4_model in "${px4_models_dir}"/*; do
    [[ -d "${px4_model}" ]] || continue
    model_name="$(basename "${px4_model}")"
    if [[ -n "${headless}" &&
      ( "${model_name}" == "x500_lidar_2d" ||
        "${model_name}" == "lidar_2d_v2" ) ]]; then
      continue
    fi
    ln -s "${px4_model}" "${runtime_models_dir}/${model_name}"
  done

  if [[ -n "${headless}" ]]; then
    ln -s "${repo_root}/drone_city_nav/models/x500_lidar_2d" \
      "${runtime_models_dir}/x500_lidar_2d"
    ln -s "${repo_root}/drone_city_nav/models/lidar_2d_v2" \
      "${runtime_models_dir}/lidar_2d_v2"
  fi
}

prepare_runtime_resources
mkdir -p "$(dirname "${px4_log_file}")"
mkdir -p "$(dirname "${uxrce_log_file}")"
mkdir -p "$(dirname "${ros_log_file}")"
mkdir -p "$(dirname "${gz_log_file}")"
: > "${px4_log_file}"
: > "${uxrce_log_file}"
: > "${ros_log_file}"
: > "${gz_log_file}"

cd "${repo_root}"
colcon build --packages-select drone_city_nav --symlink-install
set +u
source install/setup.bash
set -u

cleanup() {
  local pids
  pids="$(jobs -pr || true)"
  if [[ -z "${pids}" ]]; then
    return
  fi

  kill ${pids} 2>/dev/null || true
  for _ in {1..20}; do
    local live_pid=""
    for pid in ${pids}; do
      if kill -0 "${pid}" 2>/dev/null; then
        live_pid="${pid}"
        break
      fi
    done
    [[ -z "${live_pid}" ]] && break
    sleep 0.25
  done

  kill -KILL ${pids} 2>/dev/null || true
  wait ${pids} 2>/dev/null || true
}
trap cleanup EXIT INT TERM

gz_resource_path="${runtime_models_dir}:${runtime_worlds_dir}"
if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
  gz_resource_path="${gz_resource_path}:${GZ_SIM_RESOURCE_PATH}"
fi

export PX4_GZ_MODELS="${runtime_models_dir}"
export PX4_GZ_WORLDS="${runtime_worlds_dir}"
export PX4_GZ_PLUGINS="${px4_plugins_dir}"
export PX4_GZ_SERVER_CONFIG="${px4_server_config}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_SIM_RESOURCE_PATH="${gz_resource_path}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_GZ_PLUGINS}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_GZ_SERVER_CONFIG}"

echo "Gazebo log: ${gz_log_file}"
echo "Gazebo resources: ${runtime_dir}"
(
  gz_args=(--verbose="${GZ_VERBOSE:-1}" -r -s)
  if [[ -n "${headless}" ]]; then
    gz_args+=(--headless-rendering)
  fi
  gz sim "${gz_args[@]}" "${runtime_worlds_dir}/${world_name}.sdf" &
  gz_server_pid=$!

  if [[ -z "${headless}" ]]; then
    gz sim -g > /dev/null 2>&1 &
    gz_gui_pid=$!
    wait "${gz_server_pid}" "${gz_gui_pid}"
  else
    wait "${gz_server_pid}"
  fi
) > "${gz_log_file}" 2>&1 &

echo "MicroXRCEAgent log: ${uxrce_log_file}"
MicroXRCEAgent udp4 -p 8888 > "${uxrce_log_file}" 2>&1 &

echo "PX4 SITL log: ${px4_log_file}"
echo "PX4 Gazebo spawn pose: ${spawn_x_m},${spawn_y_m},${spawn_z_m},0,0,${spawn_yaw_rad}"
(
  {
    sleep "${px4_param_delay_s}"
    echo "param set CBRK_SUPPLY_CHK 894281"
    echo "param set NAV_DLL_ACT 0"
    while true; do
      sleep 3600
    done
  } | PX4_GZ_WORLD="${world_name}" \
      PX4_GZ_STANDALONE=1 \
      PX4_GZ_MODEL_POSE="${spawn_x_m},${spawn_y_m},${spawn_z_m},0,0,${spawn_yaw_rad}" \
      HEADLESS="${headless}" \
        make -C "${px4_dir}" px4_sitl "${px4_model_target}"
) > "${px4_log_file}" 2>&1 &
px4_pid=$!

sleep "${startup_sleep_s}"

if ! kill -0 "${px4_pid}" 2>/dev/null; then
  echo "PX4 SITL exited before ROS launch. Last PX4 log lines:" >&2
  tail -n 80 "${px4_log_file}" >&2
  exit 1
fi

print_log_tail() {
  local label="$1"
  local file="$2"
  echo "---- ${label}: ${file} ----" >&2
  perl -pe 's{\e\[[0-9;?]*[ -/]*[@-~]}{}g; s/\r/\n/g' "${file}" \
    | sed '/^pxh> *$/d' \
    | tail -n 80 >&2 || true
}

require_log_pattern() {
  local label="$1"
  local file="$2"
  local pattern="$3"
  if grep -Eq "${pattern}" "${file}"; then
    echo "OK: ${label}"
    return 0
  fi

  echo "FAIL: ${label}" >&2
  return 1
}

check_headless_run() {
  local failed=0

  require_log_pattern "Gazebo world is ready" "${px4_log_file}" \
    "Gazebo world is ready" || failed=1
  require_log_pattern "PX4 local position is valid" "${ros_log_file}" \
    "First valid PX4 local position" || failed=1
  require_log_pattern "lidar scans are received" "${ros_log_file}" \
    "First lidar scan" || failed=1
  require_log_pattern "planner publishes a path" "${ros_log_file}" \
    "Published path: waypoints=[1-9]" || failed=1
  require_log_pattern "offboard command is sent" "${ros_log_file}" \
    "Sent PX4 command: VEHICLE_CMD_DO_SET_MODE" || failed=1
  require_log_pattern "arm command is sent" "${ros_log_file}" \
    "Sent PX4 command: VEHICLE_CMD_COMPONENT_ARM_DISARM" || failed=1
  require_log_pattern "vehicle reaches armed offboard state" "${ros_log_file}" \
    "Offboard summary: .*armed=true.*offboard=true" || failed=1

  if grep -Eq "MISSION_RESULT success=false" "${ros_log_file}"; then
    echo "FAIL: mission monitor reported failure" >&2
    failed=1
  fi

  if [[ -n "${mission_check}" ]]; then
    require_log_pattern "mission monitor verifies complete A-to-B flight" \
      "${ros_log_file}" "MISSION_RESULT success=true" || failed=1
  fi

  if grep -Eqi \
    "Sensor [0-9]+ missing|Accel Sensor [0-9]+ missing|Gyro Sensor [0-9]+ missing|barometer [0-9]+ missing|Found 0 compass|Timed out waiting for Gazebo world|gz_bridge failed|Attitude failure" \
    "${px4_log_file}"; then
    echo "FAIL: PX4 log contains critical simulator/preflight errors" >&2
    failed=1
  else
    echo "OK: no critical PX4 simulator/preflight errors found"
  fi

  if [[ "${failed}" -ne 0 ]]; then
    print_log_tail "Gazebo" "${gz_log_file}"
    print_log_tail "PX4 SITL" "${px4_log_file}"
    print_log_tail "ROS launch" "${ros_log_file}"
    return 1
  fi

  return 0
}

echo "ROS launch log: ${ros_log_file}"
if [[ "${smoke_duration_s}" != "0" ]]; then
  timeout "${smoke_duration_s}" ros2 launch drone_city_nav city_nav.launch.py \
    params_file:="${repo_root}/drone_city_nav/config/urban_mvp.yaml" \
    enable_gazebo_bridge:=true \
    enable_mission_monitor:=true \
    > "${ros_log_file}" 2>&1 || {
    exit_code=$?
    if [[ "${exit_code}" -eq 124 ]]; then
      echo "Headless run reached ${smoke_duration_s}s timeout."
      if [[ -n "${headless}" ]]; then
        check_headless_run
      else
        echo "Smoke run reached ${smoke_duration_s}s timeout successfully."
      fi
      exit 0
    fi
    print_log_tail "ROS launch" "${ros_log_file}"
    exit "${exit_code}"
  }
else
  ros2 launch drone_city_nav city_nav.launch.py \
    params_file:="${repo_root}/drone_city_nav/config/urban_mvp.yaml" \
    enable_gazebo_bridge:=true \
    enable_mission_monitor:=true \
    2>&1 | tee "${ros_log_file}"
fi
