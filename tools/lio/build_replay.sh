#!/usr/bin/env bash
# build_replay.sh: builds the lidar-inertial replay against the tree's localization library (after make build),
# inside the container. Binary: log/tools/replay/replay_lio.
set -eu
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
./scripts/container_run.sh bash -lc "cd /workspace && g++ -O2 -std=c++20 -Idrone_city_nav/include -I/usr/include/eigen3 tools/lio/replay_lio.cpp -Lbuild/drone_city_nav -ldrone_city_nav_localization -Wl,-rpath,/workspace/build/drone_city_nav -o log/tools/replay/replay_lio"
