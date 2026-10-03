#!/usr/bin/env bash
# drag_bench.sh: dark-window replays of r779 and r795 with and without the rotor drag fusion (inside the container).
cd /workspace/log/tools/vio
bash /workspace/tools/vio/build_replay_src.sh /workspace/drone_city_nav/src/stereo_feature_tracker.cpp replay_drag
for run in r779 r795; do
  d=/workspace/log/tools/replay/$run
  for drag in 0 0.106; do
    ./replay_drag $d baro=1 drag=$drag tag=lit_drag$drag > /dev/null 2>&1
    for w in "100 105" "100 110" "140 150" "180 185" "180 190" "220 230" "260 270"; do
      set -- $w
      ./replay_drag $d baro=1 drag=$drag dark_from_s=$1 dark_to_s=$2 tag=dk_${drag}_$1_$2 > /dev/null 2>&1
    done
  done
done
echo BENCH_DONE
