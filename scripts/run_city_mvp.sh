#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
px4_dir="${PX4_AUTOPILOT_DIR:-${repo_root}/external/PX4-Autopilot}"
ros_distro="${ROS_DISTRO:-jazzy}"
world_name="generated_city"

if [[ ! -d "${px4_dir}" ]]; then
  echo "PX4-Autopilot was not found at ${px4_dir}" >&2
  echo "Run scripts/setup_px4_autopilot.sh first or set PX4_AUTOPILOT_DIR." >&2
  exit 1
fi

source "/opt/ros/${ros_distro}/setup.bash"
source /opt/px4_msgs_ws/install/setup.bash

world_dst="${px4_dir}/Tools/simulation/gz/worlds/${world_name}.sdf"
install -D "${repo_root}/drone_city_nav/worlds/${world_name}.sdf" "${world_dst}"

cd "${repo_root}"
colcon build --packages-select drone_city_nav --symlink-install
source install/setup.bash

cleanup() {
  jobs -pr | xargs --no-run-if-empty kill
}
trap cleanup EXIT INT TERM

MicroXRCEAgent udp4 -p 8888 &

PX4_GZ_WORLD="${world_name}" \
PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
  make -C "${px4_dir}" px4_sitl gz_x500_lidar &

sleep 8
ros2 launch drone_city_nav city_nav.launch.py
