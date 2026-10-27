#!/usr/bin/env bash
# tools_image.sh [--check]: builds the tool image of roadmap item 25 (docker/Dockerfile.tools: OpenVINS ROS-free
# and FAST-LIO's ROS2 branch on ROS 2 Jazzy) as drone-gazebo-tools:latest, the log in log/tools/part5/tools_image.log
# with both projects' commits; --check runs the image and prints the commits, the OpenVINS library and the FAST-LIO
# executable. The image is never published and nothing of it reaches the flight image.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p log/tools/part5
if [[ "${1:-}" == "--check" ]]; then
  docker run --rm drone-gazebo-tools:latest bash -c 'echo "openvins $(cat /opt/openvins_commit)"; echo "fastlio $(cat /opt/fastlio_commit)"; ls -la /usr/local/lib/libov_msckf_lib.so /opt/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping; source /opt/ros/jazzy/setup.bash; source /opt/fast_lio_ws/install/setup.bash && ros2 pkg executables fast_lio'
  exit 0
fi
echo "tools image build start $(date -u +%FT%TZ)" | tee log/tools/part5/tools_image.log
DOCKER_BUILDKIT=1 docker build --progress=plain -f docker/Dockerfile.tools -t drone-gazebo-tools:latest . >> log/tools/part5/tools_image.log 2>&1 || { echo "tools image build FAILED $(date -u +%FT%TZ)" | tee -a log/tools/part5/tools_image.log; exit 1; }
echo "tools image build end $(date -u +%FT%TZ)" | tee -a log/tools/part5/tools_image.log
docker run --rm drone-gazebo-tools:latest bash -c 'echo "openvins $(cat /opt/openvins_commit)"; echo "fastlio $(cat /opt/fastlio_commit)"' | tee -a log/tools/part5/tools_image.log
