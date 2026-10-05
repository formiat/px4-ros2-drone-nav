#!/usr/bin/env bash
# window_rig.sh rate SECONDS [NAME=value ...] | frame [SDF]: the Gazebo window of a recording over a small test world,
# with no flight: a wall, a carrier with the carried light's spot, the recording's GUI configuration and the capture
# shim. Run inside the container, with a display. Leaves its files in log/tools/window_rig/.
#   rate SECONDS [NAME=value ...]  the window for SECONDS, its frames drained; prints the redraws of each second
#                                  (24 is sound; 1 is the slideshow of a window waiting for a blank screen). The
#                                  assignments go into the window's environment, to try a remedy.
#   frame [SDF]                    one frame of the window; prints its bright green pixels (Gazebo's gizmo of a light
#                                  it is told to visualize) and leaves frame.raw. SDF goes inside the <light>, for
#                                  example '<visualize>false</visualize>'.
set -u
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode=${1:?rate or frame}; shift
rig="$repo/log/tools/window_rig"; mkdir -p "$rig"; cd "$rig"
light_sdf=""; seconds=20
if [ "$mode" = frame ]; then light_sdf=${1:-}; set --; else seconds=${1:?seconds}; shift; fi
cat > world.sdf <<SDF
<?xml version="1.0"?>
<sdf version="1.10"><world name="window_rig">
  <physics name="p" type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <scene><ambient>0 0 0 1</ambient><background>0 0 0 1</background></scene>
  <model name="wall"><static>true</static><pose>3 0 1 0 0 0</pose><link name="l"><visual name="v">
    <geometry><box><size>0.1 6 6</size></box></geometry></visual></link></model>
  <model name="carrier"><static>true</static><pose>0 0 1 0 0 0</pose><link name="l">
    <visual name="v"><geometry><box><size>0.2 0.2 0.05</size></box></geometry></visual>
    <light name="carried_light" type="spot"><pose>0.02 0 0 0 0 0</pose><diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular><intensity>2</intensity><direction>1 0 0</direction>
      <attenuation><range>25</range><constant>0.25</constant><linear>0</linear><quadratic>0.06</quadratic></attenuation>
      <spot><inner_angle>2.1</inner_angle><outer_angle>2.4</outer_angle><falloff>1.0</falloff></spot>${light_sdf}</light>
  </link></model>
</world></sdf>
SDF
rm -f cap.bgra cap.bgra.size cap.bgra.rate frame.raw; mkfifo cap.bgra
cc -shared -fPIC -O2 -o shim.so "$repo/scripts/frame_pace_shim.c" -ldl -lpthread
gz sim -s -r --headless-rendering world.sdf > server.log 2>&1 & server=$!
sleep 5
env "$@" LD_PRELOAD="$rig/shim.so" FRAME_PACE_HZ=24 FRAME_CAPTURE_TOP=48 FRAME_CAPTURE_FIFO="$rig/cap.bgra" \
  gz sim -g --gui-config "$repo/drone_city_nav/config/gazebo_gui_recording.config" > gui.log 2>&1 & gui=$!
sleep 12
if [ "$mode" = frame ]; then
  size=$(cat cap.bgra.size)
  python3 - "${size%x*}" "${size#*x}" <<'PY'
import sys
width, height = int(sys.argv[1]), int(sys.argv[2])
with open("cap.bgra", "rb") as fifo:
    for _ in range(30):
        frame = fifo.read(width * height * 4)
open("frame.raw", "wb").write(frame)
green = sum(1 for i in range(0, len(frame), 4)
            if frame[i + 1] > 150 and frame[i] < 110 and frame[i + 2] < 110)
print(f"WINDOW_RIG frame {width}x{height} green_pixels={green}")
PY
else
  timeout "$seconds" cat cap.bgra > /dev/null
  echo "WINDOW_RIG redraws a second: $(awk '{printf "%s ", $2}' cap.bgra.rate)"
fi
kill "$gui" "$server" 2> /dev/null; wait 2> /dev/null
