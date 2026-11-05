#!/usr/bin/env bash
# replay_stack.sh RUN|DIR [RATE]: the stack of a recorded flight (RECORD_STACK_BAG=true, log/runs/RUN/stack_bag, or a
# directory kept with the bag and the flight's ros_launch_args.txt, such as log/tools/replay/regression/RUN) run
# again in the dev container on the bag alone, no Gazebo and no autopilot (roadmap item 27 block 6): the flight's
# launch arguments with the bridges and the simulated image source off, the stack's inputs played on the bag's clock
# at RATE (unless given, the set's real-time factor of record: 1.0 for the lidar set, 0.8 for the cameras), its
# outputs recorded to log/tools/replay/RUN/stack/replay_bag and compared with the flight's by tools/replay_compare.py
# (log/tools/replay/RUN/stack/compare.txt, REPLAY PASS or FAIL on its last line; the exit status says which). Host;
# --inner runs in the container.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ "${1:-}" != "--inner" ]]; then
  rate=${2:-}
  if [[ -d "$1/stack_bag" ]]; then dir=${1%/}; run=$(basename "$dir"); else run=$1; dir="log/runs/$run"; fi
  [[ -d "$dir/stack_bag" ]] || { echo "no stack bag in $dir"; exit 1; }
  mkdir -p "log/tools/replay/$run/stack"; touch log/tools/replay/.keep
  exec ./scripts/container_run.sh bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash;
    exec bash /workspace/tools/replay_stack.sh --inner $run /workspace/$dir '$rate'"
fi
run=$2; dir=$3; rate=$4
bag="$dir/stack_bag"
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
done < "$dir/ros_launch_args.txt"
if [[ -z "$rate" ]]; then if [[ "$navigation" == lidar ]]; then rate=1.0; else rate=0.8; fi; fi
mapfile -t inputs < <(python3 /workspace/scripts/stack_bag_topics.py inputs \
  --navigation-sensor-profile "$navigation" --camera-profile "$cameras" --in-bag "$bag")
mapfile -t outputs < <(python3 /workspace/scripts/stack_bag_topics.py outputs)
echo "REPLAY $run start $(date -u +%H:%M:%S) rate $rate inputs ${#inputs[@]} outputs ${#outputs[@]}" | tee "$out/replay.log"
ros2 launch drone_city_nav city_nav.launch.py "${args[@]}" > "$out/ros_replay.log" 2>&1 &
launch_pid=$!
sleep 30
ros2 bag record -s mcap --max-cache-size 268435456 --qos-profile-overrides-path /workspace/scripts/stack_bag_qos.yaml \
  -o "$out/replay_bag" --topics "${outputs[@]}" > "$out/replay_bag.log" 2>&1 &
record_pid=$!
sleep 5
# The bag first: --topics takes every argument after it.
ros2 bag play "$bag" --clock 250 -r "$rate" --topics "${inputs[@]}" > "$out/play.log" 2>&1
echo "REPLAY $run played $(date -u +%H:%M:%S)" | tee -a "$out/replay.log"
sleep 5
stop() { kill -TERM "$1" 2>/dev/null || true; for _ in $(seq 1 600); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.1; done; kill -KILL "$1" 2>/dev/null || true; }
stop "$record_pid"
stop "$launch_pid"
python3 /workspace/tools/replay_compare.py "$bag" "$out/replay_bag" > "$out/compare.txt" 2>&1
tail -n 6 "$out/compare.txt" | tee -a "$out/replay.log"
echo "REPLAY $run end $(date -u +%H:%M:%S)" | tee -a "$out/replay.log"
grep -q '^REPLAY PASS$' "$out/compare.txt"
