#!/usr/bin/env bash
# The stack as a mode of PX4's own (roadmap item 44, specification K91) needs px4-ros2-interface-lib (Auterion,
# BSD-3-Clause): its package px4_ros2_cpp is built in the container beside drone_city_nav by every build of the
# workspace (Makefile, scripts/check_cpp_quality.sh, scripts/run_drone_nav_sim.sh, tools/onboard_image.sh). The
# source is cloned here at one commit: the head of its release/1.17 branch, which pins px4_msgs to release/1.17,
# the messages of external/PX4-Autopilot v1.17.0 (checked by the library's scripts/check-message-compatibility.py).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${PX4_ROS2_INTERFACE_LIB_DIR:-${repo_root}/external/px4-ros2-interface-lib}"
commit="b6d52ad120f68fbe82c9039f85e52c2a8d868083"

if [[ -d "${target_dir}/.git" ]]; then
  if [[ "$(git -C "${target_dir}" rev-parse HEAD)" == "${commit}" ]]; then
    echo "px4-ros2-interface-lib already at ${commit} in ${target_dir}"
    exit 0
  fi
  echo "px4-ros2-interface-lib in ${target_dir} is not at ${commit}: fetching it"
else
  mkdir -p "${target_dir}"
  git -C "${target_dir}" init -q
  git -C "${target_dir}" remote add origin https://github.com/Auterion/px4-ros2-interface-lib.git
fi
git -C "${target_dir}" fetch -q --depth 1 origin "${commit}"
git -C "${target_dir}" checkout -q --detach FETCH_HEAD
echo "px4-ros2-interface-lib at ${commit} in ${target_dir}"
