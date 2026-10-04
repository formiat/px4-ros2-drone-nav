#!/usr/bin/env bash
# Roadmap item 20, sourced by run_drone_nav_sim.sh: what a recorded flight
# (RECORD_VIDEO=1) adds to the headless one. Its pictures are on a display,
# the Gazebo window showing the 3D scene alone and RViz in both of the
# repository's views, and each window hands its frames to the recorder
# (record_flight_video.py) through a FIFO in the run's directory, read where
# the window draws them (frame_pace_shim.c).

# Writes the RViz configurations, the capture shim and the FIFOs into the
# run's directory; sets recording_rviz_config and recording_rviz_environment.
prepare_video_recording() {
  local repo_root="$1"
  local run_directory="$2"
  local view
  mkdir -p "${run_directory}"
  python3 "${repo_root}/scripts/rviz_recording_view.py" follow \
    "${repo_root}/drone_city_nav/rviz/city_nav_debug.rviz" \
    "${run_directory}/rviz_recording_follow.rviz"
  python3 "${repo_root}/scripts/rviz_recording_view.py" top \
    "${repo_root}/drone_city_nav/rviz/city_nav_debug_top_down.rviz" \
    "${run_directory}/rviz_recording_top_down.rviz"
  cc -shared -fPIC -O2 -o "${run_directory}/frame_pace_shim.so" \
    "${repo_root}/scripts/frame_pace_shim.c" -ldl -lpthread
  for view in world follow top; do
    mkfifo "${run_directory}/capture_${view}.bgra"
  done
  recording_rviz_config="${run_directory}/rviz_recording_follow.rviz"
  recording_rviz_environment="LD_PRELOAD=${run_directory}/frame_pace_shim.so;FRAME_CAPTURE_HZ=15;FRAME_CAPTURE_FIFO=${run_directory}/capture_follow.bgra"
}

# The Gazebo window of a recorded flight, in the foreground of the caller.
# It redraws the scene at the display's refresh rate on its own; a recording
# takes 24 frames a second of it. Its top bar, 48 pixels, stays out of the
# picture.
recorded_gazebo_gui() {
  local repo_root="$1"
  local run_directory="$2"
  LD_PRELOAD="${run_directory}/frame_pace_shim.so" FRAME_PACE_HZ=24 \
    FRAME_CAPTURE_TOP=48 \
    FRAME_CAPTURE_FIFO="${run_directory}/capture_world.bgra" \
    gz sim -g --gui-config \
    "${repo_root}/drone_city_nav/config/gazebo_gui_recording.config"
}

# The second view of a recorded flight, beside the one the launch starts.
recorded_top_view() {
  local run_directory="$1"
  LD_PRELOAD="${run_directory}/frame_pace_shim.so" FRAME_CAPTURE_HZ=15 \
    FRAME_CAPTURE_FIFO="${run_directory}/capture_top.bgra" \
    rviz2 -d "${run_directory}/rviz_recording_top_down.rviz" \
    --ros-args -p use_sim_time:=true
}
