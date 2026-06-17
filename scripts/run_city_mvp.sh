#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

make_abs_path() {
  local path="$1"
  case "${path}" in
    /*) printf '%s\n' "${path}" ;;
    *) printf '%s/%s\n' "${repo_root}" "${path}" ;;
  esac
}

guard_against_root_owned_workspace_writes() {
  local repo_owner_uid
  repo_owner_uid="$(stat -c '%u' "${repo_root}")"
  if [[ "${EUID}" -eq 0 && "${repo_owner_uid}" -ne 0 &&
    "${ALLOW_ROOT_WORKSPACE_WRITE:-}" != "1" ]]; then
    cat >&2 <<EOF
Refusing to run as root in a non-root-owned workspace because this script writes
build, install, log, and runtime files.
Run through ./scripts/dev_shell.sh or docker run with:
  --user "\$(id -u):\$(id -g)"

Set ALLOW_ROOT_WORKSPACE_WRITE=1 only for intentional maintenance.
EOF
    exit 1
  fi
}

guard_against_root_owned_workspace_writes

normalize_bool() {
  local value="${1}"
  case "${value,,}" in
    1|true|yes|on)
      printf 'true'
      ;;
    0|false|no|off)
      printf 'false'
      ;;
    *)
      printf '%s' "${value}"
      ;;
  esac
}

px4_dir="${PX4_AUTOPILOT_DIR:-${repo_root}/external/PX4-Autopilot}"
if [[ -n "${PX4_BUILD_DIR+x}" ]]; then
  px4_build_dir="$(make_abs_path "${PX4_BUILD_DIR}")"
else
  px4_build_dir="${px4_dir}/build/px4_sitl_default"
fi
ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup_file="${ROS_SETUP_FILE:-/opt/ros/${ros_distro}/setup.bash}"
px4_msgs_setup_file="${PX4_MSGS_SETUP_FILE:-/opt/px4_msgs_ws/install/setup.bash}"
colcon_build_base="$(make_abs_path "${COLCON_BUILD_BASE:-build}")"
colcon_install_base="$(make_abs_path "${COLCON_INSTALL_BASE:-install}")"
colcon_log_base="$(make_abs_path "${COLCON_LOG_BASE:-log}")"
run_log_dir="$(make_abs_path "${DRONE_GAZEBO_LOG_DIR:-log}")"
world_name="generated_city"
px4_model_target="${PX4_MODEL_TARGET:-gz_x500_lidar_2d}"
startup_sleep_s="${STARTUP_SLEEP_S:-8}"
smoke_duration_s="${SMOKE_DURATION_S:-0}"
px4_log_file="${PX4_LOG_FILE:-${run_log_dir}/px4_city_mvp.log}"
uxrce_log_file="${UXRCE_AGENT_LOG_FILE:-${run_log_dir}/uxrce_agent_city_mvp.log}"
ros_log_file="${ROS_LOG_FILE:-${run_log_dir}/ros_city_mvp.log}"
gz_log_file="${GZ_LOG_FILE:-${run_log_dir}/gz_city_mvp.log}"
lidar_debug_dir="${LIDAR_DEBUG_DIR:-${run_log_dir}/lidar_debug}"
default_city_nav_params_file="${repo_root}/drone_city_nav/config/urban_mvp.yaml"
city_nav_params_file="${CITY_NAV_PARAMS_FILE:-${default_city_nav_params_file}}"
enable_lidar_debug="$(normalize_bool "${ENABLE_LIDAR_DEBUG:-true}")"
enable_static_map_override=""
enable_obstacle_memory_override=""
enable_current_lidar_override=""
if [[ -n "${ENABLE_STATIC_MAP+x}" ]]; then
  enable_static_map_override="$(normalize_bool "${ENABLE_STATIC_MAP}")"
fi
if [[ -n "${ENABLE_OBSTACLE_MEMORY+x}" ]]; then
  enable_obstacle_memory_override="$(normalize_bool "${ENABLE_OBSTACLE_MEMORY}")"
fi
if [[ -n "${ENABLE_CURRENT_LIDAR+x}" ]]; then
  enable_current_lidar_override="$(normalize_bool "${ENABLE_CURRENT_LIDAR}")"
fi
static_city_map_path="${STATIC_CITY_MAP_PATH:-${repo_root}/drone_city_nav/worlds/${world_name}.map2d}"
static_city_map_path_override=false
if [[ -n "${STATIC_CITY_MAP_PATH+x}" ]]; then
  static_city_map_path_override=true
fi
px4_param_delay_s="${PX4_PARAM_DELAY_S:-6}"
mission_check="${MISSION_CHECK:-}"
allow_mission_failure="$(normalize_bool "${ALLOW_MISSION_FAILURE:-false}")"
headless="${HEADLESS:-}"
if [[ -n "${ENABLE_RVIZ+x}" ]]; then
  enable_rviz="${ENABLE_RVIZ}"
elif [[ -n "${headless}" ]]; then
  enable_rviz="false"
else
  enable_rviz="true"
fi
spawn_x_m="${SIM_START_X_M:--57}"
spawn_y_m="${SIM_START_Y_M:--27}"
spawn_z_m="${SIM_START_Z_M:-0.3}"
spawn_yaw_rad="${SIM_START_YAW_RAD:-0}"
runtime_dir="${colcon_build_base}/gazebo_city_mvp"
runtime_models_dir="${runtime_dir}/models"
runtime_worlds_dir="${runtime_dir}/worlds"
px4_models_dir="${px4_dir}/Tools/simulation/gz/models"
px4_plugins_dir="${px4_build_dir}/src/modules/simulation/gz_plugins"
px4_server_config="${px4_dir}/src/modules/simulation/gz_bridge/server.config"

if [[ ! -d "${px4_dir}" ]]; then
  echo "PX4-Autopilot was not found at ${px4_dir}" >&2
  echo "Run scripts/setup_px4_autopilot.sh first or set PX4_AUTOPILOT_DIR." >&2
  exit 1
fi
if [[ ! -f "${ros_setup_file}" ]]; then
  echo "ROS setup file was not found: ${ros_setup_file}" >&2
  echo "Set ROS_SETUP_FILE or run inside ./scripts/dev_shell.sh." >&2
  exit 1
fi
if [[ ! -f "${px4_msgs_setup_file}" ]]; then
  echo "px4_msgs setup file was not found: ${px4_msgs_setup_file}" >&2
  echo "Set PX4_MSGS_SETUP_FILE or run inside ./scripts/dev_shell.sh." >&2
  exit 1
fi
if [[ ! -f "${city_nav_params_file}" ]]; then
  echo "City navigation params file was not found: ${city_nav_params_file}" >&2
  exit 1
fi

format_override_value() {
  local value="$1"
  if [[ -n "${value}" ]]; then
    printf '%s' "${value}"
  else
    printf 'from_params'
  fi
}

params_are_default=false
canonical_city_nav_params_file="$(realpath -m "${city_nav_params_file}")"
canonical_default_city_nav_params_file="$(realpath -m "${default_city_nav_params_file}")"
if [[ "${canonical_city_nav_params_file}" == "${canonical_default_city_nav_params_file}" ]]; then
  params_are_default=true
fi

expected_static_map="${enable_static_map_override}"
expected_obstacle_memory="${enable_obstacle_memory_override}"
expected_current_lidar="${enable_current_lidar_override}"
if [[ "${params_are_default}" == "true" ]]; then
  expected_static_map="${expected_static_map:-true}"
  expected_obstacle_memory="${expected_obstacle_memory:-true}"
  expected_current_lidar="${expected_current_lidar:-true}"
fi

bool_is_true() {
  [[ "$1" == "true" || "$1" == "1" ]]
}

set +u
source "${ros_setup_file}"
source "${px4_msgs_setup_file}"
set -u

prepare_runtime_resources() {
  rm -rf "${runtime_dir}"
  mkdir -p "${runtime_models_dir}" "${runtime_worlds_dir}"
  install -D "${repo_root}/drone_city_nav/worlds/${world_name}.sdf" \
    "${runtime_worlds_dir}/${world_name}.sdf"
  install -D "${repo_root}/drone_city_nav/worlds/${world_name}.map2d" \
    "${runtime_worlds_dir}/${world_name}.map2d"

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
if [[ "${enable_lidar_debug}" == "true" || "${enable_lidar_debug}" == "1" ]]; then
  rm -rf "${lidar_debug_dir}"
  mkdir -p "${lidar_debug_dir}"
fi
: > "${px4_log_file}"
: > "${uxrce_log_file}"
: > "${ros_log_file}"
: > "${gz_log_file}"

cd "${repo_root}"
colcon --log-base "${colcon_log_base}" build \
  --packages-select drone_city_nav --symlink-install \
  --build-base "${colcon_build_base}" \
  --install-base "${colcon_install_base}" \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
set +u
source "${colcon_install_base}/setup.bash"
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

run_px4_sitl() {
  if [[ -n "${PX4_BUILD_DIR+x}" ]]; then
    local cmake_generator="${PX4_CMAKE_GENERATOR:-Ninja}"
    local python_executable="${PYTHON_EXECUTABLE:-$(command -v python3)}"
    local ninja_file="${px4_build_dir}/build.ninja"
    if [[ ! -f "${px4_build_dir}/CMakeCache.txt" ]]; then
      cmake -S "${px4_dir}" -B "${px4_build_dir}" -G "${cmake_generator}" \
        -DCONFIG=px4_sitl_default \
        -DPYTHON_EXECUTABLE="${python_executable}" \
        -DPython3_EXECUTABLE="${python_executable}"
    fi
    if [[ -f "${ninja_file}" ]] &&
      grep -Eq -- '-std=(gnu\+\+|c\+\+)14' "${ninja_file}"; then
      echo "PX4 host build: switching generated Ninja C++ standard flags to C++17 for conda protobuf compatibility"
      perl -0pi -e 's/-std=gnu\+\+14/-std=gnu++17/g; s/-std=c\+\+14/-std=c++17/g' \
        "${ninja_file}"
    fi
    cmake --build "${px4_build_dir}" --target "${px4_model_target}"
    return
  fi

  make -C "${px4_dir}" px4_sitl "${px4_model_target}"
}

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
echo "Lidar debug dir: ${lidar_debug_dir} (enabled=${enable_lidar_debug})"
echo "RViz debug view: enabled=${enable_rviz}"
echo "City navigation params: ${city_nav_params_file}"
echo "Obstacle source overrides: static=$(format_override_value "${enable_static_map_override}") memory=$(format_override_value "${enable_obstacle_memory_override}") current_lidar=$(format_override_value "${enable_current_lidar_override}")"
echo "Expected obstacle sources for checks: static=$(format_override_value "${expected_static_map}") memory=$(format_override_value "${expected_obstacle_memory}") current_lidar=$(format_override_value "${expected_current_lidar}")"
echo "Static city map: ${static_city_map_path}"
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
        run_px4_sitl
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

check_headless_run() {
  local validation_args=(
    --ros-log "${ros_log_file}"
    --px4-log "${px4_log_file}"
    --expected-static "${expected_static_map}"
    --expected-memory "${expected_obstacle_memory}"
    --expected-current-lidar "${expected_current_lidar}"
    --enable-lidar-debug "${enable_lidar_debug}"
  )
  if [[ -n "${mission_check}" ]]; then
    validation_args+=(--mission-check)
  fi
  if bool_is_true "${allow_mission_failure}"; then
    validation_args+=(--allow-mission-failure)
  fi

  if ! python3 "${repo_root}/scripts/validate_city_mvp_headless.py" \
    "${validation_args[@]}"; then
    print_log_tail "Gazebo" "${gz_log_file}"
    print_log_tail "PX4 SITL" "${px4_log_file}"
    print_log_tail "ROS launch" "${ros_log_file}"
    return 1
  fi

  return 0
}

ros_launch_args=(
  params_file:="${city_nav_params_file}"
  lidar_debug_output_dir:="${lidar_debug_dir}"
  enable_gazebo_bridge:=true
  enable_mission_monitor:=true
  enable_lidar_debug:="${enable_lidar_debug}"
  enable_rviz:="${enable_rviz}"
)
if [[ -n "${enable_static_map_override}" ]]; then
  ros_launch_args+=(use_static_map:="${enable_static_map_override}")
fi
if [[ -n "${enable_obstacle_memory_override}" ]]; then
  ros_launch_args+=(use_obstacle_memory:="${enable_obstacle_memory_override}")
fi
if [[ -n "${enable_current_lidar_override}" ]]; then
  ros_launch_args+=(use_current_lidar_obstacles:="${enable_current_lidar_override}")
fi
if [[ "${static_city_map_path_override}" == "true" ]]; then
  ros_launch_args+=(static_map_path:="${static_city_map_path}")
fi

echo "ROS launch log: ${ros_log_file}"
if [[ "${smoke_duration_s}" != "0" ]]; then
  timeout "${smoke_duration_s}" ros2 launch drone_city_nav city_nav.launch.py \
    "${ros_launch_args[@]}" \
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
    "${ros_launch_args[@]}" \
    2>&1 | tee "${ros_log_file}"
fi
