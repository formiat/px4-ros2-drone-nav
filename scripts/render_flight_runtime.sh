#!/usr/bin/env bash
# Part 6 of the technical-debt goal, run inside the simulation container by
# scripts/render_flight_video.py: the pictures of a flight rendered afterwards.
# The flight's world in a Gazebo server held paused, the recording's Gazebo
# window following the vehicle model, both RViz views; on the go file the
# vehicle is moved through the run's true poses (render_world_replay.py) and
# the run's bag of what RViz shows is played on the simulation's clock, both at
# RATE, beginning at BEGIN_S of the flight. Every window hands its frames to its
# FIFO in the run's directory (frame_pace_shim.c), as in a live recording.
# render_flight_runtime.sh RUN_DIRECTORY WORLD_SDF RATE BEGIN_S END_S
set -u
run_directory=$1; world_sdf=$2; rate=$3; begin_s=$4; end_s=$5
repo_root=/workspace
source "${repo_root}/scripts/video_recording_runtime.sh"
source "${repo_root}/scripts/gazebo_gui_camera_runtime.sh"
set +u
source /opt/ros/jazzy/setup.bash
source "${repo_root}/install/setup.bash"
set -u
runtime_dir="${repo_root}/build/gazebo_drone_nav"
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="${runtime_dir}/models:${runtime_dir}/worlds:$(dirname "${world_sdf}")"
world=$(python3 -c "import re,sys; print(re.search(r'<world\s+name=\"([^\"]+)\"', open(sys.argv[1]).read()).group(1))" "${world_sdf}")
model=x500_lidar_3d_0
go_file="${run_directory}/render_go"
rm -f "${go_file}"
pids=()
cleanup() {
  kill "${pids[@]}" 2> /dev/null
  sleep 2
  kill -9 "${pids[@]}" 2> /dev/null
}
trap cleanup EXIT

gz sim -s --verbose=1 "${world_sdf}" > "${run_directory}/render_gz_server.log" 2>&1 &
pids+=($!)
wait_for_gazebo_world "${repo_root}" "${world}" 120 || { echo "RENDER none: no world"; exit 1; }
python3 "${repo_root}/scripts/render_world_replay.py" --run-directory "${run_directory}" \
  --world "${world}" --model "${model}" --rate "${rate}" --begin-s "${begin_s}" \
  --end-s "${end_s}" --go-file "${go_file}" --clock-output "${run_directory}/render_world_clock.csv" \
  > "${run_directory}/render_world.log" 2>&1 &
world_pid=$!
pids+=("${world_pid}")
wait_for_gazebo_scene_entity "${repo_root}" "${world}" "${model}" 120 ||
  { echo "RENDER none: no vehicle model"; exit 1; }
recorded_gazebo_gui "${repo_root}" "${run_directory}" > "${run_directory}/render_gz_gui.log" 2>&1 &
pids+=($!)
configure_gazebo_gui_follow_camera "${repo_root}" "${world}" "${model}" "-7 0 3.5" 60 \
  > "${run_directory}/render_follow.log" 2>&1 &
LD_PRELOAD="${run_directory}/frame_pace_shim.so" FRAME_CAPTURE_HZ=15 \
  FRAME_CAPTURE_FIFO="${run_directory}/capture_follow.bgra" \
  rviz2 -d "${run_directory}/rviz_recording_follow.rviz" --ros-args -p use_sim_time:=true \
  > "${run_directory}/render_rviz_follow.log" 2>&1 &
pids+=($!)
recorded_top_view "${run_directory}" > "${run_directory}/render_rviz_top.log" 2>&1 &
pids+=($!)
echo "RENDER windows started world=${world}"

while [ ! -f "${go_file}" ]; do sleep 0.05; done
python3 "${repo_root}/scripts/render_rviz_clock.py" "${run_directory}/render_rviz_clock.csv" \
  > "${run_directory}/render_rviz_clock.log" 2>&1 &
pids+=($!)
# The vehicle's own frame, for the inset of the follow view (K26).
rm -rf "${run_directory}/render_camera"
python3 "${repo_root}/scripts/render_camera_frames.py" "${run_directory}/render_camera" \
  > "${run_directory}/render_camera.log" 2>&1 &
pids+=($!)
# A bag the end of the flight cut off has no metadata; its database holds the messages.
[ -f "${run_directory}/render_bag/metadata.yaml" ] ||
  ros2 bag reindex "${run_directory}/render_bag" -s sqlite3 > "${run_directory}/render_bag_reindex.log" 2>&1
bag_begin=$(python3 -c "
import sqlite3, sys, glob
db = glob.glob(sys.argv[1] + '/*.db3')[0]
print(sqlite3.connect(db).execute('select min(timestamp) from messages').fetchone()[0] / 1e9)" \
  "${run_directory}/render_bag")
offset=$(python3 -c "print(max(0.0, ${begin_s} - ${bag_begin}))")
ros2 bag play "${run_directory}/render_bag" --clock 100 --rate "${rate}" \
  --start-offset "${offset}" > "${run_directory}/render_bag_play.log" 2>&1 &
play_pid=$!
pids+=("${play_pid}")
echo "RENDER playing from ${begin_s} s (bag offset ${offset} s) at ${rate}"
wait "${world_pid}"
sleep 1
echo "RENDER done"
