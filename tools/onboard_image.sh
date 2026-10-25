#!/usr/bin/env bash
# onboard_image.sh [--arm64] [--image-only|--tests-only]: roadmap item 27 block 2 (steps 1 to 3). Builds the onboard stage of
# docker/Dockerfile (no Gazebo; CUDA 13 on amd64, none on arm64, which gets CUDA from JetPack) as
# drone-gazebo-onboard:<arch>, then builds the package in it from the mounted workspace with
# -DDRONE_CITY_NAV_ENABLE_SIMULATION=OFF into build-onboard-<arch>/ and runs its unit tests (the CUDA MPPI tests only
# with the host's GPU, --gpus all, on amd64). arm64 runs under qemu-user-static (apt: qemu-user-static
# binfmt-support) through docker buildx, which is slow: hours for the package; ONBOARD_CPUSET confines the package's
# build and tests to those host CPUs (a build beside flights on the flight's set, specification A4). Logs: log/tools/board/.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
arch=amd64; platform=linux/amd64; cuda=on; tests_only=false; image_only=false
for argument in "$@"; do
  case "${argument}" in
    --arm64) arch=arm64; platform=linux/arm64; cuda=off ;;
    --tests-only) tests_only=true ;;
    --image-only) image_only=true ;;
    *) echo "unknown argument ${argument}" >&2; exit 2 ;;
  esac
done
image="drone-gazebo-onboard:${arch}"
mkdir -p log/tools/board; touch log/tools/board/.keep
log="log/tools/board/onboard_${arch}.log"
echo "== onboard image ${image} start $(date -u +%H:%M:%S) commit $(git rev-parse --short HEAD)" | tee -a "${log}"
if [[ "${tests_only}" == false ]]; then
  docker buildx build --load --platform "${platform}" --target onboard --build-arg "CUDA=${cuda}" \
    --file docker/Dockerfile --tag "${image}" . 2>&1 | tail -5 | tee -a "${log}"
  echo "== image built $(date -u +%H:%M:%S): $(docker image inspect "${image}" --format '{{.Architecture}} {{.Size}}')" | tee -a "${log}"
fi
if [[ "${image_only}" == true ]]; then exit 0; fi
cpuset=(); [[ -n "${ONBOARD_CPUSET:-}" ]] && cpuset=(--cpuset-cpus "${ONBOARD_CPUSET}")
gpus=(); [[ "${arch}" == amd64 ]] && gpus=(--gpus all)
cuda_mppi=ON; [[ "${cuda}" == off ]] && cuda_mppi=OFF
docker run --rm --platform "${platform}" "${gpus[@]}" "${cpuset[@]}" --user "$(id -u):$(id -g)" \
  --volume "$(pwd):/workspace" --workdir /workspace "${image}" bash -c "
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash; source /opt/px4_msgs_ws/install/setup.bash
    set -u
    echo \"== build start \$(date -u +%H:%M:%S) on \$(uname -m) nproc \$(nproc)\"
    colcon --log-base log/tools/board/colcon-${arch} build --packages-select drone_city_nav \
      --build-base build-onboard-${arch} --install-base install-onboard-${arch} \
      --cmake-args -DDRONE_CITY_NAV_ENABLE_SIMULATION=OFF -DDRONE_CITY_NAV_ENABLE_CUDA_MPPI=${cuda_mppi} \
      --event-handlers console_direct- 2>&1 | tail -3
    echo \"== build end \$(date -u +%H:%M:%S)\"
    ctest --test-dir build-onboard-${arch}/drone_city_nav --output-on-failure 2>&1 | tail -15
    echo \"== tests end \$(date -u +%H:%M:%S)\"
  " 2>&1 | tee -a "${log}"
echo "== onboard image ${image} end $(date -u +%H:%M:%S)" | tee -a "${log}"
