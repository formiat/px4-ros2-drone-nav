#!/usr/bin/env bash
# What a flight refuses to start without: a workspace it may write as this
# user, and the autopilot, the ROS setups, the parameters and the world it was
# asked to fly. Sourced by scripts/run_drone_nav_sim.sh; the checks read that
# script's variables.

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

require_flight_prerequisites() {
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
}
