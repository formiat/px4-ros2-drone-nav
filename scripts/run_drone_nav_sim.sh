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

bool_is_true() {
  [[ "$1" == "true" || "$1" == "1" ]]
}

format_mission_goal_sequence() {
  local sequence="$1"
  python3 - "${sequence}" <<'PY'
import math
import sys

raw = sys.argv[1].strip()
if not raw:
    raise SystemExit("MISSION_GOALS_XYZ_M must not be empty when provided")
values = []
for waypoint_index, waypoint in enumerate(raw.split(";")):
    components = [component.strip() for component in waypoint.split(",")]
    if len(components) != 3 or any(not component for component in components):
        raise SystemExit(
            "MISSION_GOALS_XYZ_M must use x,y,z;x,y,z syntax; "
            f"invalid waypoint {waypoint_index}"
        )
    try:
        numeric = [float(component) for component in components]
    except ValueError as error:
        raise SystemExit(
            f"MISSION_GOALS_XYZ_M contains a non-numeric waypoint {waypoint_index}"
        ) from error
    if not all(math.isfinite(component) for component in numeric):
        raise SystemExit(
            f"MISSION_GOALS_XYZ_M contains a non-finite waypoint {waypoint_index}"
        )
    values.extend(numeric)
print("[" + ", ".join(f"{value:.12g}" for value in values) + "]")
PY
}

# shellcheck source=multi_vehicle_sim_runtime.sh
source "${repo_root}/scripts/multi_vehicle_sim_runtime.sh"
# shellcheck source=gazebo_gui_camera_runtime.sh
source "${repo_root}/scripts/gazebo_gui_camera_runtime.sh"

run_with_cpu_affinity() {
  local cpu_list="$1"
  shift
  if [[ -n "${cpu_list}" ]]; then
    taskset --cpu-list "${cpu_list}" "$@"
  else
    "$@"
  fi
}

exec_with_cpu_affinity() {
  local cpu_list="$1"
  shift
  if [[ -n "${cpu_list}" ]]; then
    exec taskset --cpu-list "${cpu_list}" "$@"
  else
    exec "$@"
  fi
}

px4_dir="${PX4_AUTOPILOT_DIR:-${repo_root}/external/PX4-Autopilot}"
px4_build_dir="${px4_dir}/build/px4_sitl_default"
ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup_file="${ROS_SETUP_FILE:-/opt/ros/${ros_distro}/setup.bash}"
px4_msgs_setup_file="${PX4_MSGS_SETUP_FILE:-/opt/px4_msgs_ws/install/setup.bash}"
colcon_build_base="$(make_abs_path "${COLCON_BUILD_BASE:-build}")"
colcon_install_base="$(make_abs_path "${COLCON_INSTALL_BASE:-install}")"
colcon_log_base="$(make_abs_path "${COLCON_LOG_BASE:-log}")"
run_log_dir="$(make_abs_path "${DRONE_GAZEBO_LOG_DIR:-log}")"
run_id="${DRONE_GAZEBO_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)-$$}"
mission_type="${MISSION_TYPE:-point_to_point}"
multi_vehicle_scenario_override="${MULTI_VEHICLE_SCENARIO_PATH:-${INTERCEPT_SCENARIO_PATH:-}}"
point_to_point_scenario_override="${POINT_TO_POINT_SCENARIO_PATH:-}"
load_multi_vehicle_sim_scenario "${mission_type}" "${multi_vehicle_scenario_override}"
scenario_world_name="${multi_vehicle_world_name:-generated_city}"
bool_is_true "${multi_vehicle_mission}" ||
  resolve_point_to_point_runtime "${point_to_point_scenario_override}"
world_name="${SIM_WORLD_NAME:-${scenario_world_name}}"
if bool_is_true "${multi_vehicle_mission}" &&
  [[ "${world_name}" != "${scenario_world_name}" ]]; then
  echo "SIM_WORLD_NAME=${world_name} does not match scenario world ${scenario_world_name}" >&2
  exit 1
fi
multi_vehicle_spectator_initial_vehicle_id=""
multi_vehicle_spectator_initial_model=""
multi_vehicle_spectator_reselection_policy="${MULTI_VEHICLE_SPECTATOR_RESELECTION_POLICY:-${INTERCEPT_SPECTATOR_RESELECTION_POLICY:-first_living}}"
multi_vehicle_spectator_reselection_delay_s="${MULTI_VEHICLE_SPECTATOR_RESELECTION_DELAY_S:-${INTERCEPT_SPECTATOR_RESELECTION_DELAY_S:-3.0}}"
if bool_is_true "${multi_vehicle_mission}"; then
  multi_vehicle_spectator_initial_vehicle_id="${MULTI_VEHICLE_SPECTATOR_INITIAL_VEHICLE_ID:-${INTERCEPT_SPECTATOR_INITIAL_VEHICLE_ID:-${multi_vehicle_ids[0]}}}"
  case "${multi_vehicle_spectator_reselection_policy}" in
  first_living | next_living) ;;
  *)
    echo "Invalid spectator reselection policy ${multi_vehicle_spectator_reselection_policy}; expected first_living or next_living" >&2
    exit 1
    ;;
  esac
  for instance in "${!multi_vehicle_ids[@]}"; do
    if [[ "${multi_vehicle_ids[instance]}" == "${multi_vehicle_spectator_initial_vehicle_id}" ]]; then
      multi_vehicle_spectator_initial_model="${multi_vehicle_gazebo_model_names[instance]}"
      break
    fi
  done
  if [[ -z "${multi_vehicle_spectator_initial_model}" ]]; then
    echo "Spectator vehicle ${multi_vehicle_spectator_initial_vehicle_id} is not present in ${multi_vehicle_scenario_path}" >&2
    exit 1
  fi
fi
enable_subsystem_cpu_affinity="$(
  normalize_bool "${ENABLE_SUBSYSTEM_CPU_AFFINITY:-true}"
)"
control_cpu_list=""
planning_cpu_list=""
diagnostics_cpu_list=""
if bool_is_true "${enable_subsystem_cpu_affinity}" &&
  command -v taskset >/dev/null 2>&1; then
  cpu_count="$(nproc)"
  if ((cpu_count >= 4)); then
    control_cpu_list="${CONTROL_CPU_LIST:-0-$((cpu_count / 2 - 1))}"
    planning_cpu_list="${PLANNING_CPU_LIST:-$((cpu_count / 4))-$((cpu_count - 1))}"
    diagnostics_cpu_list="${DIAGNOSTICS_CPU_LIST:-$((3 * cpu_count / 4))-$((cpu_count - 1))}"
    if ! taskset --cpu-list "${control_cpu_list}" true ||
      ! taskset --cpu-list "${planning_cpu_list}" true ||
      ! taskset --cpu-list "${diagnostics_cpu_list}" true; then
      echo "WARNING: subsystem CPU affinity masks are unavailable; disabling affinity." >&2
      control_cpu_list=""
      planning_cpu_list=""
      diagnostics_cpu_list=""
    fi
  fi
fi
px4_model_target="${PX4_MODEL_TARGET:-${point_to_point_px4_model_target:-gz_x500_lidar_2d}}"
if bool_is_true "${multi_vehicle_mission}"; then
  px4_model_target="${multi_vehicle_px4_model_targets[0]}"
  for vehicle_px4_model_target in "${multi_vehicle_px4_model_targets[@]}"; do
    if [[ ! "${vehicle_px4_model_target}" =~ ^gz_[A-Za-z0-9_]+$ ]]; then
      echo "Invalid multi-vehicle PX4 model target: ${vehicle_px4_model_target}" >&2
      exit 1
    fi
  done
fi
startup_sleep_s="${STARTUP_SLEEP_S:-8}"
smoke_duration_s="${SMOKE_DURATION_S:-0}"
px4_log_file="${PX4_LOG_FILE:-${run_log_dir}/px4_drone_nav.log}"
multi_vehicle_px4_logs=()
if bool_is_true "${multi_vehicle_mission}"; then
  for instance in "${!multi_vehicle_ids[@]}"; do
    if [[ "${instance}" -eq 0 ]]; then
      multi_vehicle_px4_logs+=("${px4_log_file}")
      continue
    fi
    log_env_name="$(printf '%s' "${multi_vehicle_ids[instance]}" |
      tr '[:lower:]-' '[:upper:]_')_PX4_LOG_FILE"
    log_override="${!log_env_name:-}"
    multi_vehicle_px4_logs+=(
      "${log_override:-${run_log_dir}/px4_${multi_vehicle_ids[instance]}_nav.log}"
    )
  done
fi
uxrce_log_file="${UXRCE_AGENT_LOG_FILE:-${run_log_dir}/uxrce_agent_drone_nav.log}"
ros_log_file="${ROS_LOG_FILE:-${run_log_dir}/ros_drone_nav.log}"
gz_log_file="${GZ_LOG_FILE:-${run_log_dir}/gz_drone_nav.log}"
gz_gui_log_file="${GZ_GUI_LOG_FILE:-${run_log_dir}/gz_gui_drone_nav.log}"
gz_spectator_log_file="${GZ_SPECTATOR_LOG_FILE:-${run_log_dir}/gz_spectator_follow.log}"
gz_scene_diagnostics_dir="${GZ_SCENE_DIAGNOSTICS_DIR:-${run_log_dir}/gazebo_scene_debug}"
lidar_debug_dir="${LIDAR_DEBUG_DIR:-${run_log_dir}/lidar_debug/${run_id}}"
lidar_memory_hit_dump_path="${LIDAR_MEMORY_HIT_DUMP_PATH:-${run_log_dir}/lidar_memory_hits/${run_id}.jsonl}"
default_city_nav_params_file="${repo_root}/drone_city_nav/config/urban_mvp.yaml"
city_nav_params_file="${CITY_NAV_PARAMS_FILE:-${default_city_nav_params_file}}"
static_global_lattice_deadline_ms="${STATIC_GLOBAL_LATTICE_DEADLINE_MS:-}"
cruise_speed_override="${CRUISE_SPEED_MPS:-}"
static_route_tracking_margin_override="${STATIC_ROUTE_TRACKING_MARGIN_M:-}"
speed_limit_override="${ABSOLUTE_SPEED_LIMIT_MPS:-}"
horizontal_acceleration_override="${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-}"
enable_lidar_debug_override=""
if [[ -n "${ENABLE_LIDAR_DEBUG+x}" ]]; then
  enable_lidar_debug_override="$(normalize_bool "${ENABLE_LIDAR_DEBUG}")"
fi
enable_2d_lidar="$(normalize_bool "${ENABLE_2D_LIDAR:-true}")"
case "${enable_2d_lidar}" in
true | false) ;;
*)
  echo "ENABLE_2D_LIDAR must be a boolean, got '${enable_2d_lidar}'" >&2
  exit 1
  ;;
esac
enable_obstacle_memory_override=""
if [[ -n "${ENABLE_OBSTACLE_MEMORY+x}" ]]; then
  enable_obstacle_memory_override="$(normalize_bool "${ENABLE_OBSTACLE_MEMORY}")"
fi
enable_gz_scene_diagnostics="$(
  normalize_bool "${ENABLE_GZ_SCENE_DIAGNOSTICS:-true}"
)"
enable_static_map_override=""
if [[ -n "${ENABLE_STATIC_MAP+x}" ]]; then
  enable_static_map_override="$(normalize_bool "${ENABLE_STATIC_MAP}")"
fi
px4_param_delay_s="${PX4_PARAM_DELAY_S:-6}"
mission_check="${MISSION_CHECK:-}"
allow_mission_failure="$(normalize_bool "${ALLOW_MISSION_FAILURE:-false}")"
headless="${HEADLESS:-}"
mission_goal_sequence_xyz_m=""
if [[ -n "${MISSION_GOALS_XYZ_M+x}" ]]; then
  mission_goal_sequence_xyz_m="$(format_mission_goal_sequence "${MISSION_GOALS_XYZ_M}")"
fi
if [[ -n "${MULTI_VEHICLE_SHUTDOWN_ON_TERMINAL_OUTCOME+x}" ]]; then
  multi_vehicle_shutdown_on_terminal_outcome="$(
    normalize_bool "${MULTI_VEHICLE_SHUTDOWN_ON_TERMINAL_OUTCOME}"
  )"
elif [[ -n "${INTERCEPT_SHUTDOWN_ON_TERMINAL_OUTCOME+x}" ]]; then
  multi_vehicle_shutdown_on_terminal_outcome="$(
    normalize_bool "${INTERCEPT_SHUTDOWN_ON_TERMINAL_OUTCOME}"
  )"
elif [[ -n "${headless}" ]]; then
  multi_vehicle_shutdown_on_terminal_outcome="true"
else
  multi_vehicle_shutdown_on_terminal_outcome="false"
fi
if [[ -n "${POINT_TO_POINT_SHUTDOWN_ON_MISSION_RESULT+x}" ]]; then
  point_to_point_shutdown_on_mission_result="$(
    normalize_bool "${POINT_TO_POINT_SHUTDOWN_ON_MISSION_RESULT}"
  )"
elif [[ -n "${headless}" ]]; then
  point_to_point_shutdown_on_mission_result="true"
else
  point_to_point_shutdown_on_mission_result="false"
fi
case "${point_to_point_shutdown_on_mission_result}" in
true | false) ;;
*)
  echo "POINT_TO_POINT_SHUTDOWN_ON_MISSION_RESULT must be a boolean" >&2
  exit 1
  ;;
esac
if [[ -n "${ENABLE_RVIZ+x}" ]]; then
  enable_rviz="${ENABLE_RVIZ}"
elif [[ -n "${headless}" ]]; then
  enable_rviz="false"
else
  enable_rviz="true"
fi
evader_speed_scale="${EVADER_SPEED_SCALE:-1.0}"
intercept_directional_hypotheses_enabled="$(
  normalize_bool "${INTERCEPT_DIRECTIONAL_HYPOTHESES_ENABLED:-false}"
)"
intercept_noncooperative_avoidance_enabled="$(
  normalize_bool "${INTERCEPT_NONCOOPERATIVE_AVOIDANCE_ENABLED:-false}"
)"
cooperative_desired_minimum_separation_m="${COOPERATIVE_DESIRED_MINIMUM_SEPARATION_M:-5.0}"
cooperative_release_separation_m="${COOPERATIVE_RELEASE_SEPARATION_M:-7.0}"
cooperative_prediction_horizon_s="${COOPERATIVE_PREDICTION_HORIZON_S:-5.0}"
cooperative_mission_timeout_s="${COOPERATIVE_MISSION_TIMEOUT_S:-240.0}"
enable_gazebo_gui_follow_camera="$(
  normalize_bool "${ENABLE_GZ_GUI_FOLLOW_CAMERA:-true}"
)"
enable_rviz_follow_camera="$(
  normalize_bool "${ENABLE_RVIZ_FOLLOW_CAMERA:-true}"
)"
if bool_is_true "${enable_rviz_follow_camera}"; then
  rviz_config_file="${RVIZ_CONFIG_FILE:-${repo_root}/drone_city_nav/rviz/city_nav_debug.rviz}"
else
  rviz_config_file="${RVIZ_CONFIG_FILE:-${repo_root}/drone_city_nav/rviz/city_nav_debug_top_down.rviz}"
fi
rviz_drone_follow_tf_enabled="${enable_rviz_follow_camera}"
if ! bool_is_true "${enable_rviz}"; then
  rviz_drone_follow_tf_enabled="false"
fi
gazebo_gui_follow_target="${GZ_GUI_FOLLOW_TARGET:-${multi_vehicle_spectator_initial_model:-x500_lidar_2d_0}}"
gazebo_gui_follow_offset="${GZ_GUI_FOLLOW_OFFSET:--12 0 6}"
gazebo_gui_follow_wait_s="${GZ_GUI_FOLLOW_WAIT_S:-60}"
gazebo_world_unpause_wait_s="${GZ_WORLD_UNPAUSE_WAIT_S:-60}"
clean_stale_gazebo_processes_enabled="$(
  normalize_bool "${DRONE_GAZEBO_CLEAN_STALE_PROCESSES:-true}"
)"
clean_stale_gazebo_processes_dry_run="$(
  normalize_bool "${DRONE_GAZEBO_CLEAN_STALE_DRY_RUN:-false}"
)"
resolve_point_to_point_gazebo_spawn
runtime_dir="${colcon_build_base}/gazebo_drone_nav"
runtime_models_dir="${runtime_dir}/models"
runtime_worlds_dir="${runtime_dir}/worlds"
custom_world_sdf_path=""
if [[ -n "${SIM_WORLD_SDF_PATH:-}" ]]; then
  custom_world_sdf_path="$(make_abs_path "${SIM_WORLD_SDF_PATH}")"
fi
custom_world_resource_path=""
if [[ -n "${SIM_WORLD_RESOURCE_PATH:-}" ]]; then
  custom_world_resource_path="$(make_abs_path "${SIM_WORLD_RESOURCE_PATH}")"
fi
static_occupancy_3d_path_override=""
if [[ -n "${STATIC_OCCUPANCY_3D_PATH:-}" ]]; then
  static_occupancy_3d_path_override="$(make_abs_path "${STATIC_OCCUPANCY_3D_PATH}")"
fi
static_esdf_3d_cache_path_override=""
if [[ -n "${STATIC_ESDF_3D_CACHE_PATH:-}" ]]; then
  static_esdf_3d_cache_path_override="$(make_abs_path "${STATIC_ESDF_3D_CACHE_PATH}")"
fi
static_free_space_topology_3d_path_override=""
if [[ -n "${STATIC_FREE_SPACE_TOPOLOGY_3D_PATH:-}" ]]; then
  static_free_space_topology_3d_path_override="$(
    make_abs_path "${STATIC_FREE_SPACE_TOPOLOGY_3D_PATH}"
  )"
fi
px4_models_dir="${px4_dir}/Tools/simulation/gz/models"
px4_plugins_dir="${px4_build_dir}/src/modules/simulation/gz_plugins"
px4_server_config="${px4_dir}/src/modules/simulation/gz_bridge/server.config"

if [[ ! -d "${px4_dir}" ]]; then
  echo "PX4-Autopilot was not found at ${px4_dir}" >&2
  echo "Run scripts/setup_px4_autopilot.sh first or set PX4_AUTOPILOT_DIR." >&2
  exit 1
fi
if bool_is_true "${multi_vehicle_mission}" &&
  [[ ! -x "${px4_build_dir}/bin/px4" ]]; then
  echo "PX4 SITL binary was not found: ${px4_build_dir}/bin/px4" >&2
  echo "Build PX4 SITL before running a multi-vehicle mission." >&2
  exit 1
fi
if [[ ! -f "${ros_setup_file}" ]]; then
  echo "ROS setup file was not found: ${ros_setup_file}" >&2
  echo "Run inside ./scripts/dev_shell.sh or set ROS_SETUP_FILE." >&2
  exit 1
fi
if [[ ! -f "${px4_msgs_setup_file}" ]]; then
  echo "px4_msgs setup file was not found: ${px4_msgs_setup_file}" >&2
  echo "Run inside ./scripts/dev_shell.sh or set PX4_MSGS_SETUP_FILE." >&2
  exit 1
fi
if [[ ! -f "${city_nav_params_file}" ]]; then
  echo "City navigation params file was not found: ${city_nav_params_file}" >&2
  exit 1
fi
if [[ -n "${custom_world_sdf_path}" && ! -f "${custom_world_sdf_path}" ]]; then
  echo "Custom Gazebo world was not found: ${custom_world_sdf_path}" >&2
  exit 1
fi

read_ros_float_parameter() {
  local node_name="$1"
  local parameter_name="$2"
  python3 - "${city_nav_params_file}" "${node_name}" "${parameter_name}" <<'PY'
import math
import sys

import yaml

params_path, node_name, parameter_name = sys.argv[1:]
with open(params_path, encoding="utf-8") as stream:
    document = yaml.safe_load(stream)
try:
    value = float(document[node_name]["ros__parameters"][parameter_name])
except (KeyError, TypeError, ValueError) as exc:
    raise SystemExit(
        f"Missing numeric ROS parameter {node_name}.{parameter_name} in {params_path}"
    ) from exc
if not math.isfinite(value) or value < 0.0:
    raise SystemExit(
        f"Invalid ROS parameter {node_name}.{parameter_name}={value} in {params_path}"
    )
print(value)
PY
}

read_ros_bool_parameter() {
  local node_name="$1"
  local parameter_name="$2"
  python3 - "${city_nav_params_file}" "${node_name}" "${parameter_name}" <<'PY'
import sys

import yaml

params_path, node_name, parameter_name = sys.argv[1:]
with open(params_path, encoding="utf-8") as stream:
    document = yaml.safe_load(stream)
try:
    value = document[node_name]["ros__parameters"][parameter_name]
except (KeyError, TypeError) as exc:
    raise SystemExit(
        f"Missing boolean ROS parameter {node_name}.{parameter_name} in {params_path}"
    ) from exc
if not isinstance(value, bool):
    raise SystemExit(
        f"Invalid ROS parameter {node_name}.{parameter_name}={value} in {params_path}"
    )
print("true" if value else "false")
PY
}

px4_max_climb_speed_mps="$(
    read_ros_float_parameter production_mppi_node maximum_vertical_speed_mps
)"
px4_max_descent_speed_mps="${px4_max_climb_speed_mps}"
configured_static_map="$(
    read_ros_bool_parameter production_mppi_node use_static_map
)"
active_static_map="${enable_static_map_override:-${configured_static_map}}"
if bool_is_true "${active_static_map}"; then
  for static_artifact in \
    "${static_occupancy_3d_path_override}" \
    "${static_esdf_3d_cache_path_override}" \
    "${static_free_space_topology_3d_path_override}"; do
    if [[ -n "${static_artifact}" && ! -f "${static_artifact}" ]]; then
      echo "Static world artifact was not found: ${static_artifact}" >&2
      exit 1
    fi
  done
fi
if [[ -n "${enable_lidar_debug_override}" ]]; then
  enable_lidar_debug="${enable_lidar_debug_override}"
elif ! bool_is_true "${enable_2d_lidar}"; then
  enable_lidar_debug="false"
elif bool_is_true "${active_static_map}" && [[ -n "${headless}" ]]; then
  enable_lidar_debug="false"
else
  enable_lidar_debug="true"
fi
if [[ -n "${enable_obstacle_memory_override}" ]]; then
  enable_obstacle_memory="${enable_obstacle_memory_override}"
elif ! bool_is_true "${active_static_map}" || bool_is_true "${enable_lidar_debug}" ||
  [[ -z "${headless}" ]]; then
  enable_obstacle_memory="true"
else
  enable_obstacle_memory="false"
fi
if ! bool_is_true "${active_static_map}" &&
  ! bool_is_true "${enable_obstacle_memory}"; then
  echo "No-static navigation requires ENABLE_OBSTACLE_MEMORY=true" >&2
  exit 1
fi
if ! bool_is_true "${active_static_map}" &&
  ! bool_is_true "${enable_2d_lidar}"; then
  echo "No-static navigation requires ENABLE_2D_LIDAR=true" >&2
  exit 1
fi
if bool_is_true "${enable_lidar_debug}" &&
  ! bool_is_true "${enable_obstacle_memory}"; then
  echo "Lidar debug requires ENABLE_OBSTACLE_MEMORY=true" >&2
  exit 1
fi
if bool_is_true "${enable_lidar_debug}" &&
  ! bool_is_true "${enable_2d_lidar}"; then
  echo "Lidar debug requires ENABLE_2D_LIDAR=true" >&2
  exit 1
fi
px4_active_max_horizontal_speed_mps="${speed_limit_override:-$(
    read_ros_float_parameter production_mppi_node absolute_speed_limit_mps
)}"
px4_active_cruise_speed_mps="${cruise_speed_override:-$(
    read_ros_float_parameter production_mppi_node cruise_speed_mps
)}"
px4_active_max_horizontal_acceleration_mps2="${horizontal_acceleration_override:-$(
    read_ros_float_parameter \
      production_mppi_node maximum_horizontal_acceleration_mps2
)}"
px4_active_maximum_jerk_mps3="$(
    read_ros_float_parameter production_mppi_node maximum_control_jerk_mps3
)"
evader_px4_max_horizontal_speed_mps="$(
  python3 -c 'import sys; print(float(sys.argv[1]) * float(sys.argv[2]))' \
    "${px4_active_max_horizontal_speed_mps}" "${evader_speed_scale}"
)"
evader_px4_cruise_speed_mps="$(
  python3 -c 'import sys; print(float(sys.argv[1]) * float(sys.argv[2]))' \
    "${px4_active_cruise_speed_mps}" "${evader_speed_scale}"
)"

format_override_value() {
  local value="$1"
  if [[ -n "${value}" ]]; then
    printf '%s' "${value}"
  else
    printf 'from_params'
  fi
}

expected_static_map="${active_static_map}"
expected_obstacle_memory="${enable_obstacle_memory}"

clean_stale_gazebo_processes() {
  if ! bool_is_true "${clean_stale_gazebo_processes_enabled}"; then
    echo "WARNING: stale Gazebo process cleanup is disabled"
    return 0
  fi

  local cleanup_args=(
    --self-pid "$$"
    --protect-pid "${BASHPID}"
  )
  if bool_is_true "${clean_stale_gazebo_processes_dry_run}"; then
    cleanup_args+=(--dry-run)
  fi

  python3 "${repo_root}/scripts/gazebo_process_cleanup.py" "${cleanup_args[@]}" \
    --repo-root "${repo_root}" \
    --project-marker "/workspace"
}

mkdir -p "$(dirname "${gz_log_file}")"
mkdir -p "$(dirname "${gz_gui_log_file}")"
: > "${gz_log_file}"
: > "${gz_gui_log_file}"
clean_stale_gazebo_processes | tee -a "${gz_log_file}"

set +u
source "${ros_setup_file}"
source "${px4_msgs_setup_file}"
set -u

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
      "${model_name}" == "lidar_2d_v2" ]]; then
      continue
    fi
    ln -s "${px4_model}" "${runtime_models_dir}/${model_name}"
  done

  ln -s "${repo_root}/drone_city_nav/models/x500_lidar_2d" \
    "${runtime_models_dir}/x500_lidar_2d"
  prepare_multi_vehicle_model_resources
  cp -a "${repo_root}/drone_city_nav/models/lidar_2d_v2" \
    "${runtime_models_dir}/lidar_2d_v2"

  local lidar_visibility_mode="no-static"
  if bool_is_true "${active_static_map}"; then
    lidar_visibility_mode="static"
  fi
  python3 "${repo_root}/scripts/configure_lidar_visibility.py" \
    "${runtime_models_dir}/lidar_2d_v2/model.sdf" \
    --mode "${lidar_visibility_mode}" --enabled "${enable_2d_lidar}"
}

prepare_runtime_resources
mkdir -p "$(dirname "${px4_log_file}")"
mkdir -p "$(dirname "${uxrce_log_file}")"
mkdir -p "$(dirname "${ros_log_file}")"
if [[ "${enable_lidar_debug}" == "true" || "${enable_lidar_debug}" == "1" ]]; then
  mkdir -p "${lidar_debug_dir}"
fi
if [[ "${enable_gz_scene_diagnostics}" == "true" ||
  "${enable_gz_scene_diagnostics}" == "1" ]]; then
  rm -rf "${gz_scene_diagnostics_dir}"
  mkdir -p "${gz_scene_diagnostics_dir}"
fi
: > "${px4_log_file}"
if bool_is_true "${multi_vehicle_mission}"; then
  for vehicle_log in "${multi_vehicle_px4_logs[@]}"; do
    mkdir -p "$(dirname "${vehicle_log}")"
    : > "${vehicle_log}"
  done
fi
: > "${uxrce_log_file}"
: > "${ros_log_file}"

cd "${repo_root}"
colcon --log-base "${colcon_log_base}" build \
  --packages-select drone_city_nav --symlink-install \
  --build-base "${colcon_build_base}" \
  --install-base "${colcon_install_base}" \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
set +u
source "${colcon_install_base}/setup.bash"
set -u

collect_descendant_pids() {
  local root_pid="$1"
  local child_pid

  for child_pid in $(pgrep -P "${root_pid}" 2>/dev/null || true); do
    printf '%s\n' "${child_pid}"
    collect_descendant_pids "${child_pid}"
  done
}

terminate_pids_bounded() {
  local pid
  local pids=("$@")
  [[ "${#pids[@]}" -eq 0 ]] && return 0

  kill "${pids[@]}" 2>/dev/null || true
  for _ in {1..20}; do
    local live_pid=""
    for pid in "${pids[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        live_pid="${pid}"
        break
      fi
    done
    [[ -z "${live_pid}" ]] && return 0
    sleep 0.1
  done
  kill -KILL "${pids[@]}" 2>/dev/null || true
}

cleanup_started=false
cleanup() {
  local job_pids
  local pids=()
  local pid
  local child_pids

  if [[ "${cleanup_started}" == "true" ]]; then
    return
  fi
  cleanup_started=true
  trap - EXIT INT TERM

  job_pids="$(jobs -pr || true)"
  if [[ -z "${job_pids}" ]]; then
    return
  fi

  for pid in ${job_pids}; do
    pids+=("${pid}")
    child_pids="$(collect_descendant_pids "${pid}")"
    if [[ -n "${child_pids}" ]]; then
      while read -r pid; do
        [[ -n "${pid}" ]] && pids+=("${pid}")
      done <<< "${child_pids}"
    fi
  done

  terminate_pids_bounded "${pids[@]}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

run_px4_sitl() {
  run_with_cpu_affinity "${control_cpu_list}" \
    make -C "${px4_dir}" px4_sitl "${px4_model_target}"
}

run_px4_instance() {
  local instance="$1"
  cd "${px4_dir}"
  exec_with_cpu_affinity "${control_cpu_list}" \
    "${px4_build_dir}/bin/px4" -i "${instance}"
}

reset_px4_instance_state() {
  local instance="$1"
  local rootfs_dir="${px4_build_dir}/rootfs/${instance}"

  # SITL state must not carry sensor calibration or a stored mission between runs.
  rm -f -- \
    "${rootfs_dir}/parameters.bson" \
    "${rootfs_dir}/parameters_backup.bson" \
    "${rootfs_dir}/dataman"
}

gz_resource_path="${runtime_models_dir}:${runtime_worlds_dir}"
if [[ -n "${custom_world_resource_path}" ]]; then
  gz_resource_path="${gz_resource_path}:${custom_world_resource_path}"
fi
if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
  gz_resource_path="${gz_resource_path}:${GZ_SIM_RESOURCE_PATH}"
fi

export PX4_GZ_MODELS="${runtime_models_dir}"
export PX4_GZ_WORLDS="${runtime_worlds_dir}"
export PX4_GZ_PLUGINS="${px4_plugins_dir}"
export PX4_GZ_SERVER_CONFIG="${px4_server_config}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export GZ_SIM_RESOURCE_PATH="${gz_resource_path}"
drone_city_nav_plugin_dir="${colcon_install_base}/drone_city_nav/lib"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${drone_city_nav_plugin_dir}:${PX4_GZ_PLUGINS}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_GZ_SERVER_CONFIG}"

echo "Gazebo log: ${gz_log_file}"
echo "Gazebo GUI log: ${gz_gui_log_file}"
echo "Gazebo scene diagnostics: enabled=${enable_gz_scene_diagnostics} dir=${gz_scene_diagnostics_dir}"
echo "Lidar debug dir: ${lidar_debug_dir} (enabled=${enable_lidar_debug})"
echo "Obstacle memory: enabled=${enable_obstacle_memory}"
echo "2D lidar: enabled=${enable_2d_lidar}"
echo "RViz debug view: enabled=${enable_rviz}"
echo "RViz follow camera: enabled=${enable_rviz_follow_camera} tf=${rviz_drone_follow_tf_enabled} config=${rviz_config_file}"
echo "Gazebo GUI follow camera: enabled=${enable_gazebo_gui_follow_camera} target=${gazebo_gui_follow_target} offset='${gazebo_gui_follow_offset}'" |
  tee -a "${gz_log_file}"
if bool_is_true "${multi_vehicle_mission}"; then
  echo "Multi-vehicle spectator: initial_vehicle_id=${multi_vehicle_spectator_initial_vehicle_id} reselection_policy=${multi_vehicle_spectator_reselection_policy} reselection_delay_s=${multi_vehicle_spectator_reselection_delay_s}" |
    tee -a "${gz_log_file}"
fi
echo "Gazebo world unpause wait: ${gazebo_world_unpause_wait_s}s"
echo "Gazebo stale cleanup: enabled=${clean_stale_gazebo_processes_enabled} dry_run=${clean_stale_gazebo_processes_dry_run}"
if ! bool_is_true "${multi_vehicle_mission}"; then
  echo "Point-to-point waypoint sequence: ${mission_goal_sequence_xyz_m:-configured_single_goal}"
  echo "Point-to-point shutdown on mission result: ${point_to_point_shutdown_on_mission_result}"
fi
echo "City navigation params: ${city_nav_params_file}"
echo "Obstacle source overrides: static=$(format_override_value "${enable_static_map_override}") memory=always"
echo "Expected obstacle sources for checks: static=$(format_override_value "${expected_static_map}") memory=$(format_override_value "${expected_obstacle_memory}")"
echo "Gazebo world: name=${world_name} sdf=${gazebo_world_sdf_path}"
echo "Static world: occupancy=${static_occupancy_3d_path_override:-from_params} esdf=${static_esdf_3d_cache_path_override:-from_params} topology=${static_free_space_topology_3d_path_override:-from_params}"
echo "Gazebo resources: ${runtime_dir}"
echo "CPU affinity: enabled=${enable_subsystem_cpu_affinity} control='${control_cpu_list:-all}' planning='${planning_cpu_list:-all}' diagnostics='${diagnostics_cpu_list:-all}'"
(
  gz_server_pid=""
  gz_gui_pid=""
  gz_camera_logger_pid=""
  gz_follow_pid=""
  gz_unpause_pid=""
  gazebo_cleanup_started=false

  cleanup_gazebo_children() {
    local child_pids
    local pid
    local pids=()
    if [[ "${gazebo_cleanup_started}" == "true" ]]; then
      return
    fi
    gazebo_cleanup_started=true
    trap - EXIT INT TERM

    [[ -n "${gz_unpause_pid}" ]] && pids+=("${gz_unpause_pid}")
    [[ -n "${gz_follow_pid}" ]] && pids+=("${gz_follow_pid}")
    [[ -n "${gz_camera_logger_pid}" ]] && pids+=("${gz_camera_logger_pid}")
    [[ -n "${gz_gui_pid}" ]] && pids+=("${gz_gui_pid}")
    [[ -n "${gz_server_pid}" ]] && pids+=("${gz_server_pid}")
    [[ "${#pids[@]}" -eq 0 ]] && return 0

    for pid in "${pids[@]}"; do
      child_pids="$(collect_descendant_pids "${pid}")"
      if [[ -n "${child_pids}" ]]; then
        while read -r pid; do
          [[ -n "${pid}" ]] && pids+=("${pid}")
        done <<< "${child_pids}"
      fi
    done
    terminate_pids_bounded "${pids[@]}"
  }
  trap cleanup_gazebo_children EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  gz_args=(--verbose="${GZ_VERBOSE:-1}" -r -s)
  if [[ -n "${headless}" ]]; then
    gz_args+=(--headless-rendering)
  fi
  run_with_cpu_affinity "${control_cpu_list}" \
    gz sim "${gz_args[@]}" "${gazebo_world_sdf_path}" &
  gz_server_pid=$!

  if [[ -z "${headless}" ]]; then
    if ! wait_for_gazebo_scene_entity "${repo_root}" "${world_name}" \
      "${gazebo_gui_follow_target}" \
      "${gazebo_gui_follow_wait_s}"; then
      echo "WARNING: launching Gazebo GUI without the requested drone entity."
    fi
    run_with_cpu_affinity "${diagnostics_cpu_list}" \
      gz sim -g >> "${gz_gui_log_file}" 2>&1 &
    gz_gui_pid=$!
    start_gazebo_gui_camera_logger "${repo_root}" "${run_log_dir}" "${gz_gui_log_file}"
    gz_camera_logger_pid="${GAZEBO_GUI_CAMERA_LOGGER_PID}"
    configure_gazebo_world_running "${repo_root}" "${world_name}" \
      "${gazebo_world_unpause_wait_s}" &
    gz_unpause_pid=$!
    if bool_is_true "${enable_gazebo_gui_follow_camera}" &&
      ! bool_is_true "${multi_vehicle_mission}"; then
      configure_gazebo_gui_follow_camera "${repo_root}" "${world_name}" \
        "${gazebo_gui_follow_target}" \
        "${gazebo_gui_follow_offset}" \
        "${gazebo_gui_follow_wait_s}" &
      gz_follow_pid=$!
    fi
    wait "${gz_server_pid}" "${gz_gui_pid}"
  else
    wait "${gz_server_pid}"
  fi
) >> "${gz_log_file}" 2>&1 &

echo "MicroXRCEAgent log: ${uxrce_log_file}"
run_with_cpu_affinity "${control_cpu_list}" \
  MicroXRCEAgent udp4 -p 8888 > "${uxrce_log_file}" 2>&1 &

px4_parameter_stream() {
  local cruise_speed="$1"
  local maximum_speed="$2"
  sleep "${px4_param_delay_s}"
  echo "param set CBRK_SUPPLY_CHK 894281"
  echo "param set NAV_DLL_ACT 0"
  echo "param set MPC_Z_VEL_MAX_UP ${px4_max_climb_speed_mps}"
  echo "param set MPC_Z_VEL_MAX_DN ${px4_max_descent_speed_mps}"
  echo "param set MPC_XY_CRUISE ${cruise_speed}"
  echo "param set MPC_XY_VEL_MAX ${maximum_speed}"
  echo "param set MPC_ACC_HOR_MAX ${px4_active_max_horizontal_acceleration_mps2}"
  echo "param set MPC_ACC_HOR ${px4_active_max_horizontal_acceleration_mps2}"
  echo "param set MPC_JERK_AUTO ${px4_active_maximum_jerk_mps3}"
  echo "param show MPC_XY_CRUISE"
  echo "param show MPC_XY_VEL_MAX"
  echo "param show MPC_ACC_HOR_MAX"
  echo "param show MPC_ACC_HOR"
  echo "param show MPC_JERK_AUTO"
  echo "param show MPC_Z_VEL_MAX_UP"
  echo "param show MPC_Z_VEL_MAX_DN"
  while true; do
    sleep 3600
  done
}

echo "PX4 SITL log: ${px4_log_file}"
if bool_is_true "${multi_vehicle_mission}"; then
  echo "Multi-vehicle scenario: ${multi_vehicle_scenario_path}"
  for instance in "${!multi_vehicle_ids[@]}"; do
    echo "MULTI_VEHICLE_COORDINATE_CONTRACT vehicle_id='${multi_vehicle_ids[instance]}' map_start='${multi_vehicle_map_start_poses[instance]}' gazebo_spawn='${multi_vehicle_gazebo_spawn_poses[instance]}' gazebo_model='${multi_vehicle_gazebo_model_names[instance]}'"
    echo "PX4 SITL log for ${multi_vehicle_ids[instance]}: ${multi_vehicle_px4_logs[instance]}"
  done
  multi_vehicle_px4_cruise_speeds=()
  multi_vehicle_px4_maximum_speeds=()
  for instance in "${!multi_vehicle_ids[@]}"; do
    if [[ "${multi_vehicle_roles[instance]}" == "evader" ]]; then
      multi_vehicle_px4_cruise_speeds+=("${evader_px4_cruise_speed_mps}")
      multi_vehicle_px4_maximum_speeds+=("${evader_px4_max_horizontal_speed_mps}")
    else
      multi_vehicle_px4_cruise_speeds+=("${px4_active_cruise_speed_mps}")
      multi_vehicle_px4_maximum_speeds+=("${px4_active_max_horizontal_speed_mps}")
    fi
  done
  multi_vehicle_px4_pids=()
  for instance in "${!multi_vehicle_ids[@]}"; do
    reset_px4_instance_state "${instance}"
    (
      px4_parameter_stream "${multi_vehicle_px4_cruise_speeds[instance]}" \
        "${multi_vehicle_px4_maximum_speeds[instance]}" |
        PX4_GZ_WORLD="${world_name}" \
          PX4_GZ_STANDALONE=1 \
          PX4_GZ_MODEL_POSE="${multi_vehicle_gazebo_spawn_poses[instance]}" \
          PX4_SIM_MODEL="${multi_vehicle_px4_model_targets[instance]}" \
          PX4_UXRCE_DDS_NS="${multi_vehicle_px4_namespaces[instance]}" \
          PX4_SYS_AUTOSTART=4013 \
          HEADLESS="${headless}" \
          run_px4_instance "${instance}"
    ) > "${multi_vehicle_px4_logs[instance]}" 2>&1 &
    multi_vehicle_px4_pids+=("$!")
  done
  px4_pid="${multi_vehicle_px4_pids[0]}"
else
  echo "PX4 Gazebo spawn pose: ${point_gazebo_spawn_x_m},${point_gazebo_spawn_y_m},${point_gazebo_spawn_z_m},0,0,${point_gazebo_spawn_yaw_rad}"
  reset_px4_instance_state 0
  (
    px4_parameter_stream "${px4_active_cruise_speed_mps}" \
      "${px4_active_max_horizontal_speed_mps}" |
      PX4_GZ_WORLD="${world_name}" \
        PX4_GZ_STANDALONE=1 \
        PX4_GZ_MODEL_POSE="${point_gazebo_spawn_x_m},${point_gazebo_spawn_y_m},${point_gazebo_spawn_z_m},0,0,${point_gazebo_spawn_yaw_rad}" \
        HEADLESS="${headless}" \
        run_px4_sitl
  ) > "${px4_log_file}" 2>&1 &
  px4_pid=$!
fi

sleep "${startup_sleep_s}"

if ! kill -0 "${px4_pid}" 2>/dev/null; then
  echo "PX4 SITL exited before ROS launch. Last PX4 log lines:" >&2
  tail -n 80 "${px4_log_file}" >&2
  exit 1
fi
if bool_is_true "${multi_vehicle_mission}"; then
  for instance in "${!multi_vehicle_ids[@]}"; do
    if ! kill -0 "${multi_vehicle_px4_pids[instance]}" 2>/dev/null; then
      echo "${multi_vehicle_px4_namespaces[instance]} PX4 SITL exited before ROS launch. Last log lines:" >&2
      tail -n 80 "${multi_vehicle_px4_logs[instance]}" >&2
      exit 1
    fi
  done
fi

if bool_is_true "${enable_gz_scene_diagnostics}"; then
  if ! capture_gazebo_scene_diagnostics "${repo_root}" "${world_name}" \
    "${gazebo_gui_follow_target}" "${gz_scene_diagnostics_dir}" |
    tee -a "${gz_log_file}"; then
    echo "WARNING: Gazebo scene diagnostics capture failed" | tee -a "${gz_log_file}"
  fi
fi

launch_file="city_nav.launch.py"
if bool_is_true "${multi_vehicle_mission}"; then
  scenario_argument="intercept_scenario_path"
  launch_file="intercept.launch.py"
  if bool_is_true "${cooperative_traffic_mission}"; then
    scenario_argument="cooperative_traffic_scenario_path"
    launch_file="cooperative_traffic.launch.py"
  fi
  ros_launch_args=(
    params_file:="${city_nav_params_file}"
    "${scenario_argument}:=${multi_vehicle_scenario_path}"
    enable_lidar_debug:="${enable_lidar_debug}"
    enable_2d_lidar:="${enable_2d_lidar}"
    enable_obstacle_memory:="${enable_obstacle_memory}"
    enable_rviz:="${enable_rviz}"
    evader_speed_scale:="${evader_speed_scale}"
    intercept_directional_hypotheses_enabled:="${intercept_directional_hypotheses_enabled}"
    intercept_noncooperative_avoidance_enabled:="${intercept_noncooperative_avoidance_enabled}"
    spectator_initial_vehicle_id:="${multi_vehicle_spectator_initial_vehicle_id}"
    spectator_reselection_policy:="${multi_vehicle_spectator_reselection_policy}"
    spectator_reselection_delay_s:="${multi_vehicle_spectator_reselection_delay_s}"
    shutdown_on_terminal_outcome:="${multi_vehicle_shutdown_on_terminal_outcome}"
    control_cpu_list:="${control_cpu_list}"
    planning_cpu_list:="${planning_cpu_list}"
    diagnostics_cpu_list:="${diagnostics_cpu_list}"
  )
  if bool_is_true "${cooperative_traffic_mission}"; then
    ros_launch_args+=(
      cooperative_desired_minimum_separation_m:="${cooperative_desired_minimum_separation_m}"
      cooperative_release_separation_m:="${cooperative_release_separation_m}"
      cooperative_prediction_horizon_s:="${cooperative_prediction_horizon_s}"
      cooperative_mission_timeout_s:="${cooperative_mission_timeout_s}"
    )
  fi
else
  ros_launch_args=(
    params_file:="${city_nav_params_file}"
    lidar_debug_output_dir:="${lidar_debug_dir}"
    lidar_memory_hit_dump_path:="${lidar_memory_hit_dump_path}"
    enable_gazebo_bridge:=true
    enable_mission_monitor:=true
    enable_lidar_debug:="${enable_lidar_debug}"
    enable_2d_lidar:="${enable_2d_lidar}"
    enable_obstacle_memory:="${enable_obstacle_memory}"
    enable_rviz:="${enable_rviz}"
    rviz_config:="${rviz_config_file}"
    rviz_drone_follow_tf_enabled:="${rviz_drone_follow_tf_enabled}"
    shutdown_on_mission_result:="${point_to_point_shutdown_on_mission_result}"
  )
  if [[ -n "${point_to_point_scenario_path}" ]]; then
    ros_launch_args+=(
      point_to_point_scenario_path:="${point_to_point_scenario_path}"
    )
  fi
  if [[ -n "${mission_goal_sequence_xyz_m}" ]]; then
    ros_launch_args+=(
      mission_goal_sequence_xyz_m:="${mission_goal_sequence_xyz_m}"
    )
  fi
fi
if [[ -n "${enable_static_map_override}" ]]; then
  ros_launch_args+=(use_static_map:="${enable_static_map_override}")
fi
if [[ -n "${static_global_lattice_deadline_ms}" ]]; then
  ros_launch_args+=(
    static_global_lattice_deadline_ms:="${static_global_lattice_deadline_ms}"
  )
fi
if [[ -n "${static_route_tracking_margin_override}" ]]; then
  ros_launch_args+=(
    static_route_tracking_margin_m:="${static_route_tracking_margin_override}"
  )
fi
if [[ -n "${cruise_speed_override}" ]]; then
  ros_launch_args+=(cruise_speed_mps:="${cruise_speed_override}")
fi
if [[ -n "${speed_limit_override}" ]]; then
  ros_launch_args+=(
    absolute_speed_limit_mps:="${speed_limit_override}"
  )
fi
if [[ -n "${horizontal_acceleration_override}" ]]; then
  ros_launch_args+=(
    maximum_horizontal_acceleration_mps2:="${horizontal_acceleration_override}"
  )
fi
if [[ -n "${static_occupancy_3d_path_override}" ]]; then
  ros_launch_args+=(
    static_occupancy_3d_path:="${static_occupancy_3d_path_override}"
  )
fi
if [[ -n "${static_esdf_3d_cache_path_override}" ]]; then
  ros_launch_args+=(
    static_esdf_3d_cache_path:="${static_esdf_3d_cache_path_override}"
  )
fi
if [[ -n "${static_free_space_topology_3d_path_override}" ]]; then
  ros_launch_args+=(
    static_free_space_topology_3d_path:="${static_free_space_topology_3d_path_override}"
  )
fi
echo "ROS launch log: ${ros_log_file}"
echo "Lidar memory-hit diagnostics: ${lidar_memory_hit_dump_path}"
if bool_is_true "${multi_vehicle_mission}" && [[ -z "${headless}" ]] &&
  bool_is_true "${enable_gazebo_gui_follow_camera}"; then
  python3 "${repo_root}/scripts/gazebo_spectator_follow.py" \
    --world "${world_name}" \
    --offset "${gazebo_gui_follow_offset}" \
    --wait-s "${gazebo_gui_follow_wait_s}" \
    > "${gz_spectator_log_file}" 2>&1 &
fi
if [[ "${smoke_duration_s}" != "0" ]]; then
  timeout "${smoke_duration_s}" ros2 launch drone_city_nav "${launch_file}" \
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
  if [[ -n "${headless}" ]]; then
    check_headless_run
  fi
else
  ros2 launch drone_city_nav "${launch_file}" \
    "${ros_launch_args[@]}" \
    > "${ros_log_file}" 2>&1
fi
