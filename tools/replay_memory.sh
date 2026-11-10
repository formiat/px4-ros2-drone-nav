#!/usr/bin/env bash
# replay_memory.sh RUN TAG [RATE]: the stack of a recorded flight (log/runs/RUN/stack_bag) run again on its bag as
# tools/replay_stack.sh runs it, with the obstacle memory's raw snapshots captured into
# log/tools/replay/memory/RUN_TAG (scripts/capture_raw_snapshot_3d.py over the flight's own snapshot bounds): the
# memory the checked-out code builds from the flight's frames and poses, to read against the memory the flight built
# (roadmap item 11: the stereo memory's free rays, K72). Host; --inner runs in the container.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ "${1:-}" != "--inner" ]]; then
  run=$1; tag=$2; rate=${3:-0.8}
  [[ -d "log/runs/$run/stack_bag" ]] || { echo "no stack bag in log/runs/$run"; exit 1; }
  mkdir -p "log/tools/replay/memory/${run}_${tag}"; touch log/tools/replay/.keep
  exec ./scripts/container_run.sh bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash;
    exec bash /workspace/tools/replay_memory.sh --inner $run $tag $rate"
fi
run=$2; tag=$3; rate=$4
dir="/workspace/log/runs/$run"; bag="$dir/stack_bag"; out="/workspace/log/tools/replay/memory/${run}_${tag}"
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
mapfile -t inputs < <(python3 /workspace/scripts/stack_bag_topics.py inputs \
  --navigation-sensor-profile "$navigation" --camera-profile "$cameras" --in-bag "$bag")
bounds=$(python3 -c "import json; print(','.join(str(v) for v in json.load(open('$(ls "$dir"/raw_snapshot_3d_revision_*.json | head -1)'))['selection']['bounds_m']))")
cp "$dir/manifest.json" "$out/manifest.json"
echo "REPLAY MEMORY $run $tag start $(date -u +%H:%M:%S) rate $rate inputs ${#inputs[@]} bounds $bounds commit $(git -C /workspace rev-parse --short HEAD)" | tee "$out/replay.log"
ros2 launch drone_city_nav city_nav.launch.py "${args[@]}" > "$out/ros_replay.log" 2>&1 &
launch_pid=$!
sleep 30
python3 /workspace/scripts/capture_raw_snapshot_3d.py --topic /drone_city_nav/raw_obstacle_snapshot_3d \
  --output-directory "$out" --manifest "$out/manifest.json" --bounds="$bounds" > "$out/capture.log" 2>&1 &
capture_pid=$!
sleep 5
ros2 bag play "$bag" --clock 250 -r "$rate" --delay 5 --topics "${inputs[@]}" > "$out/play.log" 2>&1
echo "REPLAY MEMORY $run $tag played $(date -u +%H:%M:%S)" | tee -a "$out/replay.log"
sleep 5
stop() { kill -TERM "$1" 2>/dev/null || true; for _ in $(seq 1 600); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.1; done; kill -KILL "$1" 2>/dev/null || true; }
stop "$capture_pid"
stop "$launch_pid"
ls "$out"/raw_snapshot_3d_revision_*.json 2>/dev/null | wc -l | xargs echo "snapshots:" | tee -a "$out/replay.log"
echo "REPLAY MEMORY $run $tag end $(date -u +%H:%M:%S)" | tee -a "$out/replay.log"
