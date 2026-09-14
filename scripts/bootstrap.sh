#!/usr/bin/env bash
# One-command setup of a fresh clone: the dev image, the PX4 checkout and its
# SITL build, the workspace build and the urban environment assets, then the
# urban point-to-point simulation. Every step is skipped when its result is
# already in place, so the script is safe to rerun.
#
#   ./scripts/bootstrap.sh              prepare everything, then run the GUI
#                                       urban point-to-point simulation
#   ./scripts/bootstrap.sh --headless   prepare, then run it headless with the
#                                       mission check
#   ./scripts/bootstrap.sh --no-run     prepare only
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

run_mode="gui"
for argument in "$@"; do
  case "${argument}" in
    --headless) run_mode="headless" ;;
    --no-run) run_mode="none" ;;
    -h | --help)
      sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "Unknown argument: ${argument}" >&2
      exit 2
      ;;
  esac
done

image_name="${DRONE_GAZEBO_DEV_IMAGE:-drone-gazebo-dev:latest}"
px4_dir="${PX4_AUTOPILOT_DIR:-${repo_root}/external/PX4-Autopilot}"
px4_binary="${px4_dir}/build/px4_sitl_default/bin/px4"
workspace_setup="${repo_root}/install/setup.bash"
urban_environment_env="${repo_root}/external/environment-artifacts/derived/urban_circuit_practice_01/runtime/environment.env"
urban_scenario="drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json"

step() {
  printf '\n==> %s\n' "$1"
}

step "Host requirements"
for tool in docker git; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "${tool} is required on the host." >&2
    exit 2
  fi
done
if ! docker info >/dev/null 2>&1; then
  echo "Docker is installed but not usable by this user; see docs/installation.md." >&2
  exit 2
fi
if ! command -v nvidia-smi >/dev/null 2>&1 ||
  ! docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
  echo "The NVIDIA container runtime is required: every simulation runs the" >&2
  echo "CUDA MPPI optimiser. Install the NVIDIA driver and nvidia-container-toolkit." >&2
  exit 2
fi
echo "docker, git and the NVIDIA runtime are available."

step "Dev image ${image_name}"
if docker image inspect "${image_name}" >/dev/null 2>&1; then
  echo "present"
else
  echo "building (this downloads ROS 2, Gazebo and px4_msgs; expect a while)"
  "${repo_root}/scripts/build_dev_image.sh"
fi

step "PX4 Autopilot at ${px4_dir}"
if [[ -d "${px4_dir}/.git" ]]; then
  echo "present"
else
  PX4_AUTOPILOT_DIR="${px4_dir}" "${repo_root}/scripts/setup_px4_autopilot.sh"
fi

step "PX4 SITL binary ${px4_binary}"
if [[ -x "${px4_binary}" ]]; then
  echo "present"
else
  # The container mounts only the repository, at /workspace, so the checkout
  # has to live inside it to be built there.
  case "${px4_dir}" in
    "${repo_root}"/*) px4_container_dir="/workspace/${px4_dir#"${repo_root}"/}" ;;
    *)
      echo "PX4_AUTOPILOT_DIR=${px4_dir} lies outside the repository; the dev" >&2
      echo "container cannot build it. Keep PX4 under external/ or build" >&2
      echo "px4_sitl there yourself." >&2
      exit 2
      ;;
  esac
  echo "building px4_sitl inside the dev container (expect 10 to 20 minutes)"
  "${repo_root}/scripts/container_run.sh" make -C "${px4_container_dir}" px4_sitl
fi

step "Workspace build (colcon, drone_city_nav)"
"${repo_root}/scripts/container_run.sh" make build
test -f "${workspace_setup}"

step "Urban environment assets (urban_circuit_practice_01)"
if [[ -f "${urban_environment_env}" ]]; then
  echo "present"
else
  echo "fetching the versioned release assets and deriving the runtime world"
fi
"${repo_root}/scripts/container_run.sh" python3 scripts/prepare_environment_simulation.py \
  --environment urban_circuit_practice_01 --runtime-map-mode no-static \
  --scenario "${urban_scenario}"

step "Ready"
echo "Run the urban point-to-point simulation with:"
echo "  ./scripts/sim_urban_point_to_point_gui.sh        (Gazebo GUI and RViz)"
echo "  ./scripts/sim_urban_point_to_point_headless.sh   (headless, mission check)"
echo "Stop any simulation with ./scripts/stop_sim.sh."

case "${run_mode}" in
  gui) exec "${repo_root}/scripts/sim_urban_point_to_point_gui.sh" ;;
  headless) exec "${repo_root}/scripts/sim_urban_point_to_point_headless.sh" ;;
  none) ;;
esac
