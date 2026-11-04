#!/usr/bin/env bash
# replay_stack.sh RUN [RATE]: the stack of a recorded flight (RECORD_STACK_BAG=true, log/runs/RUN/stack_bag) run
# again in the dev container on the bag alone, no Gazebo and no autopilot (roadmap item 27 block 6): the flight's
# launch arguments (log/runs/RUN/ros_launch_args.txt) with the bridges and the simulated image source off, the
# stack's inputs played on the bag's clock at RATE (0.8 unless given, the camera flights' real-time factor), its
# outputs recorded to log/tools/replay/RUN/stack/replay_bag and compared with the flight's by tools/replay_compare.py
# (log/tools/replay/RUN/stack/compare.txt, REPLAY PASS or FAIL on its last line). Host; --inner runs in the container.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ "${1:-}" != "--inner" ]]; then
  run=$1; rate=${2:-0.8}
  [[ -d "log/runs/$run/stack_bag" ]] || { echo "no stack bag in log/runs/$run"; exit 1; }
  mkdir -p "log/tools/replay/$run/stack"; touch log/tools/replay/.keep
  exec ./scripts/container_run.sh bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash;
    exec bash /workspace/tools/replay_stack.sh --inner $run $rate"
fi
run=$2; rate=$3
bag="/workspace/log/runs/$run/stack_bag"
out="/workspace/log/tools/replay/$run/stack"
navigation=stereo_tof; cameras=stereo_tof
args=()
while IFS= read -r argument; do
  case "$argument" in
    enable_gazebo_bridge:=*) argument="enable_gazebo_bridge:=false" ;;
    mppi_diagnostics_output_dir:=*) argument="mppi_diagnostics_output_dir:=$out/mppi" ;;
    enable_rviz:=*) argument="enable_rviz:=false" ;;
    rviz_config:=*|rviz_drone_follow_tf_enabled:=*|rviz_environment:=*) continue ;;
    navigation_sensor_profile:=*) navigation="${argument#*:=}" ;;
    camera_profile:=*) cameras="${argument#*:=}" ;;
  esac
  args+=("$argument")
done < "/workspace/log/runs/$run/ros_launch_args.txt"
mapfile -t inputs < <(python3 /workspace/scripts/stack_bag_topics.py inputs \
  --navigation-sensor-profile "$navigation" --camera-profile "$cameras" --in-bag "$bag")
mapfile -t outputs < <(python3 /workspace/scripts/stack_bag_topics.py outputs)
echo "REPLAY $run start $(date -u +%H:%M:%S) rate $rate inputs ${#inputs[@]} outputs ${#outputs[@]}" | tee "$out/replay.log"
ros2 launch drone_city_nav city_nav.launch.py "${args[@]}" > "$out/ros_replay.log" 2>&1 &
launch_pid=$!
sleep 30
ros2 bag record -s mcap --max-cache-size 268435456 -o "$out/replay_bag" "${outputs[@]}" > "$out/replay_bag.log" 2>&1 &
record_pid=$!
sleep 5
ros2 bag play --clock 250 -r "$rate" --topics "${inputs[@]}" "$bag" > "$out/play.log" 2>&1
echo "REPLAY $run played $(date -u +%H:%M:%S)" | tee -a "$out/replay.log"
sleep 5
stop() { kill -TERM "$1" 2>/dev/null || true; for _ in $(seq 1 600); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.1; done; kill -KILL "$1" 2>/dev/null || true; }
stop "$record_pid"
stop "$launch_pid"
python3 /workspace/tools/replay_compare.py "$bag" "$out/replay_bag" > "$out/compare.txt" 2>&1
tail -n 6 "$out/compare.txt" | tee -a "$out/replay.log"
echo "REPLAY $run end $(date -u +%H:%M:%S)" | tee -a "$out/replay.log"
