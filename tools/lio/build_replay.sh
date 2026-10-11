#!/usr/bin/env bash
# build_replay.sh [REVISION]: builds the lidar-inertial replay inside the container from the tree's sources
# (binary log/tools/replay/replay_lio) or, with a git revision, from that revision's lidar_inertial_odometry and
# point_plane_map_3d sources (binary log/tools/replay/replay_lio_REVISION), so that two revisions of the estimator
# replay the same record. Needs no make build: the two sources depend on Eigen alone.
set -eu
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
rev=${1:-}
if [ -z "$rev" ]; then
  src=drone_city_nav/src; inc=drone_city_nav/include; out=log/tools/replay/replay_lio
else
  stage="log/tools/replay/src_$rev"; mkdir -p "$stage/src" "$stage/include/drone_city_nav"
  for f in lidar_inertial_odometry point_plane_map_3d; do
    git show "$rev:drone_city_nav/src/$f.cpp" > "$stage/src/$f.cpp"
    git show "$rev:drone_city_nav/include/drone_city_nav/$f.hpp" > "$stage/include/drone_city_nav/$f.hpp"
  done
  src=$stage/src; inc=$stage/include; out="log/tools/replay/replay_lio_$rev"
fi
./scripts/container_run.sh bash -lc "cd /workspace && g++ -O2 -std=c++20 -I$inc -I/usr/include/eigen3 tools/lio/replay_lio.cpp $src/lidar_inertial_odometry.cpp $src/point_plane_map_3d.cpp -o $out && echo BUILT $out"
