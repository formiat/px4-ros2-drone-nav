#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target_dir="${PX4_AUTOPILOT_DIR:-${repo_root}/external/PX4-Autopilot}"
px4_ref="${PX4_REF:-v1.17.0}"

if [[ -d "${target_dir}/.git" ]]; then
  echo "PX4-Autopilot already exists at ${target_dir}"
  echo "Using existing checkout; set PX4_AUTOPILOT_DIR to override."
  exit 0
fi

mkdir -p "$(dirname "${target_dir}")"
git clone --recursive --branch "${px4_ref}" https://github.com/PX4/PX4-Autopilot.git "${target_dir}"

echo "PX4-Autopilot cloned to ${target_dir}"
echo "Export this path before running the simulation:"
echo "  export PX4_AUTOPILOT_DIR=${target_dir}"
