#!/usr/bin/env bash
# stereo_depth_accuracy.sh RUN [KEEP]: specification K7's measurement. An ordinary camera flight on Urban (the dark
# location by the carried light, the pair's noise as it is) whose pair and evaluation depth camera are recorded from
# its 60th second for 420 s (tools/stereo_record_frames.py, every KEEP-th frame, 8 unless given), given the camera
# driver's gain (tools/stereo_gain_copy.py) and matched as the depth node matches (stereo_depth_accuracy, built with
# the package). The records land in log/tools/stereo/RUN and RUNg (a kept directory), the table in
# log/tools/stereo/RUN_accuracy.txt. Not an acceptance flight: the recorder loads the host.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
run=$1; keep=${2:-8}
./scripts/stop_sim.sh > /dev/null 2>&1
(setsid env DRONE_GAZEBO_RUN_ID="$run" SMOKE_DURATION_S=600 ./scripts/dev_shell.sh make sim-urban-point-to-point-headless \
  > "log/tools/run_$run.log" 2>&1 &)
sleep 60
container=$(docker ps -q | head -1)
timeout 600 docker exec "$container" bash -lc "source /opt/ros/jazzy/setup.bash > /dev/null 2>&1; \
  timeout 420 python3 /workspace/tools/stereo_record_frames.py /workspace/log/tools/stereo/$run $keep" \
  > "log/tools/stereo/${run}_record.log" 2>&1
while pgrep -f "[m]ake sim-urban-point-to-point" > /dev/null; do sleep 10; done
./scripts/stop_sim.sh > /dev/null 2>&1
python3 tools/stereo_gain_copy.py "log/tools/stereo/$run" "log/tools/stereo/${run}g"
./scripts/dev_shell.sh > /dev/null 2>&1 <<EOS
cd /workspace && ./build/drone_city_nav/stereo_depth_accuracy log/tools/stereo/${run}g > log/tools/stereo/${run}_accuracy.txt 2>&1
exit
EOS
cat "log/tools/stereo/${run}_accuracy.txt"
