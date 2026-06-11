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

if [[ ! -d "${px4_dir}" ]]; then
  echo "PX4-Autopilot was not found at ${px4_dir}" >&2
  echo "Run scripts/setup_px4_autopilot.sh first or set PX4_AUTOPILOT_DIR." >&2
  exit 1
fi

set +u
source "/opt/ros/${ros_distro}/setup.bash"
source /opt/px4_msgs_ws/install/setup.bash
set -u

world_dst="${px4_dir}/Tools/simulation/gz/worlds/${world_name}.sdf"
px4_models_dir="${px4_dir}/Tools/simulation/gz/models"
local_models_dir="${repo_root}/drone_city_nav/models"
install -D "${repo_root}/drone_city_nav/worlds/${world_name}.sdf" "${world_dst}"
if [[ -d "${local_models_dir}" ]]; then
  while IFS= read -r -d '' local_model_file; do
    relative_path="${local_model_file#"${local_models_dir}/"}"
    install -D "${local_model_file}" "${px4_models_dir}/${relative_path}"
  done < <(find "${local_models_dir}" -type f -print0)
fi
mkdir -p "$(dirname "${px4_log_file}")"
mkdir -p "$(dirname "${uxrce_log_file}")"
: > "${px4_log_file}"
: > "${uxrce_log_file}"

cd "${repo_root}"
colcon build --packages-select drone_city_nav --symlink-install
set +u
source install/setup.bash
set -u

cleanup() {
  local pids
  pids="$(jobs -pr || true)"
  if [[ -n "${pids}" ]]; then
    kill ${pids} 2>/dev/null || true
    wait ${pids} 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "MicroXRCEAgent log: ${uxrce_log_file}"
MicroXRCEAgent udp4 -p 8888 > "${uxrce_log_file}" 2>&1 &

echo "PX4 SITL log: ${px4_log_file}"
(
  PX4_GZ_WORLD="${world_name}" \
  PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
  HEADLESS="${HEADLESS:-}" \
    make -C "${px4_dir}" px4_sitl "${px4_model_target}"
) > "${px4_log_file}" 2>&1 &
px4_pid=$!

sleep "${startup_sleep_s}"

if ! kill -0 "${px4_pid}" 2>/dev/null; then
  echo "PX4 SITL exited before ROS launch. Last PX4 log lines:" >&2
  tail -n 80 "${px4_log_file}" >&2
  exit 1
fi

if [[ "${smoke_duration_s}" != "0" ]]; then
  timeout "${smoke_duration_s}" ros2 launch drone_city_nav city_nav.launch.py || {
    exit_code=$?
    if [[ "${exit_code}" -eq 124 ]]; then
      echo "Smoke run reached ${smoke_duration_s}s timeout successfully."
      exit 0
    fi
    exit "${exit_code}"
  }
else
  ros2 launch drone_city_nav city_nav.launch.py
fi
