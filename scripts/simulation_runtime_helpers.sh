#!/usr/bin/env bash

# Roadmap item 35 (K52): the real vehicle's imperfections. IMPERFECTIONS=nominal
# turns the whole profile on, IMPERFECTIONS=nominal_<family> one family alone, for
# the series that fly them apart: imu (the bias walk), vibration (the rotors'
# wrench), extrinsics (the sensor set's pose), stamps (the IMU stream's jitter and
# gaps, the frames' offset, the sweeps' cadence).
imperfection_family_on() {
  [[ "${IMPERFECTIONS:-none}" == nominal || "${IMPERFECTIONS:-none}" == "nominal_$1" ]]
}

make_abs_path() {
  local path="$1"
  case "${path}" in
    /*) printf '%s\n' "${path}" ;;
    *) printf '%s/%s\n' "${repo_root}" "${path}" ;;
  esac
}

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

# The three subsystem CPU lists (control, planning, diagnostics) from the CPUs
# this process may run on: the first half, from the first quarter to the end,
# and the last quarter of the allowed list, the proportions of the masks on a
# whole 16-CPU host (0-7, 4-15, 12-15). A flight confined to a core set
# (FLIGHT_CPUSET, the container's cpuset, specification A4) has fewer CPUs and
# other numbers than 0 to nproc - 1: a mask built from nproc would name CPUs
# the container has not got and keep the intersection only. Prints nothing
# under four CPUs.
derive_subsystem_cpu_lists() {
  local allowed part i cpus=()
  allowed="$(taskset -pc $$ 2>/dev/null | sed 's/.*: //')"
  for part in ${allowed//,/ }; do
    if [[ "${part}" == *-* ]]; then
      for ((i = ${part%-*}; i <= ${part#*-}; i++)); do cpus+=("${i}"); done
    else
      cpus+=("${part}")
    fi
  done
  local n="${#cpus[@]}"
  ((n >= 4)) || return 0
  local IFS=,
  printf '%s %s %s\n' "${cpus[*]:0:$((n / 2))}" "${cpus[*]:$((n / 4))}" "${cpus[*]:$((3 * n / 4))}"
}

# The runner's helpers of the process and the parameters (moved whole from
# run_drone_nav_sim.sh on 2026-11-02, the source size contract): they read the
# runner's variables (city_nav_params_file, repo_root, the stale-cleanup switches)
# at the time of the call.
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

format_override_value() {
  local value="$1"
  if [[ -n "${value}" ]]; then
    printf '%s' "${value}"
  else
    printf 'from_params'
  fi
}

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

# The runner's configuration lines, printed once the choices are made (moved whole
# from run_drone_nav_sim.sh on 2026-11-02): every value is the runner's variable.
print_run_configuration() {
  echo "Gazebo log: ${gz_log_file}"
  echo "Gazebo GUI log: ${gz_gui_log_file}"
  echo "Gazebo scene diagnostics: enabled=${enable_gz_scene_diagnostics} dir=${gz_scene_diagnostics_dir}"
  echo "Obstacle memory: enabled=${enable_obstacle_memory}"
  echo "Lidar profile: ${lidar_profile}"
  echo "Camera profile: ${camera_profile}"
  echo "Navigation sensor profile: ${navigation_sensor_profile}"
  echo "Observed 3D route-volume validation: required=${require_observed_3d_route_volume_crossing} bounds_m=${observed_3d_route_volume_bounds_m:-unset}"
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
    echo "Point-to-point waypoint sequence: ${mission_goal_sequence_xyz_m:-configured_parameter_sequence}"
    echo "Point-to-point shutdown on mission result: ${point_to_point_shutdown_on_mission_result}"
  fi
  echo "City navigation params: ${city_nav_params_file}"
  echo "Obstacle source selection: static=${active_static_map} memory=always"
  echo "Expected obstacle sources for checks: static=$(format_override_value "${expected_static_map}") memory=$(format_override_value "${expected_obstacle_memory}")"
  echo "Gazebo world: name=${world_name} sdf=${gazebo_world_sdf_path}"
  echo "Static world: occupancy=${static_occupancy_3d_path_override:-from_params} esdf=${static_esdf_3d_cache_path_override:-from_params} topology=${static_free_space_topology_3d_path_override:-from_params}"
  echo "Gazebo resources: ${runtime_dir}"
  echo "CPU affinity: enabled=${enable_subsystem_cpu_affinity} control='${control_cpu_list:-all}' planning='${planning_cpu_list:-all}' diagnostics='${diagnostics_cpu_list:-all}' cpuset='$(taskset -pc $$ | sed 's/.*: //')' (FLIGHT_CPUSET='${FLIGHT_CPUSET:-}')"
}
