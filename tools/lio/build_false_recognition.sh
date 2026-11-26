#!/usr/bin/env bash
# build_false_recognition.sh: builds tools/lio/false_recognition.cpp inside the container from the tree's sources
# (binary log/tools/replay/false_recognition). Needs no make build: the tool reads the estimator's configuration from
# its header and links the point-to-plane map alone, which depends on Eigen.
set -eu
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
mkdir -p log/tools/replay
./scripts/container_run.sh bash -lc "cd /workspace && g++ -O2 -std=c++20 -pthread -Idrone_city_nav/include -I/usr/include/eigen3 tools/lio/false_recognition.cpp drone_city_nav/src/point_plane_map_3d.cpp -o log/tools/replay/false_recognition && echo BUILT log/tools/replay/false_recognition"
