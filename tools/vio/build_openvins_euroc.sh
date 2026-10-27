#!/usr/bin/env bash
# build_openvins_euroc.sh: builds tools/vio/openvins_euroc.cpp inside the tool image (tools/tools_image.sh) against
# the installed OpenVINS library, the binary at log/tools/replay/openvins_euroc; then runs it when SEQUENCE_DIR and
# OUT_CSV are given: build_openvins_euroc.sh [SEQUENCE_DIR OUT_CSV]. The datasets live under /data/gazebo
# (mounted at the same path); the repository at /workspace; TOOLS_IMAGE names another image (the openvins stage).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
mkdir -p log/tools/replay
docker run --rm --user "$(id -u):$(id -g)" -v "$(pwd):/workspace" -v /data/gazebo:/data/gazebo -w /workspace \
  "${TOOLS_IMAGE:-drone-gazebo-tools:latest}" bash -c '
    set -euo pipefail
    if [ ! -x log/tools/replay/openvins_euroc ] || [ tools/vio/openvins_euroc.cpp -nt log/tools/replay/openvins_euroc ]; then
      g++ -O2 -std=c++17 -o log/tools/replay/openvins_euroc tools/vio/openvins_euroc.cpp \
        -I/usr/local/include/open_vins -I/usr/include/eigen3 $(pkg-config --cflags opencv4) \
        -L/usr/local/lib -lov_msckf_lib $(pkg-config --libs opencv4) -lceres -lglog -lboost_system -lboost_filesystem -lpthread
      echo "built log/tools/replay/openvins_euroc"
    fi
    if [ $# -ge 2 ]; then
      LD_LIBRARY_PATH=/usr/local/lib log/tools/replay/openvins_euroc "$1" /opt/open_vins/config/euroc_mav/estimator_config.yaml "$2"
    fi' -- "$@"
