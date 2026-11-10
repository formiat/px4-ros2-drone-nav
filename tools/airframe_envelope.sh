#!/usr/bin/env bash
# airframe_envelope.sh [SPEED...]: the simulated airframe's envelope by a direct experiment, without the stack
# (roadmap item 39 stage 5): the autopilot's own SITL of the x500 in its default world (flat ground, no wind), a
# pilot's script in place of the stack (tools/airframe_envelope.py: offboard velocity setpoints, as the stack
# commands), and for every SPEED (5 10 12 15 20 m/s unless given) three runs out and back at 30 m: the time and the
# distance to the speed, the speed held, the tilt and the motors' share of their top speed while it is held, then
# the setpoint dropped to nought: the distance and the time to rest, the mean and the peak deceleration, the tilt,
# the height lost. The autopilot's horizontal speed limit is raised to its parameter's top (20 m/s) and its tilt
# limit left at the flights' (MPC_TILTMAX_AIR); nothing else differs from a flight's parameters that the experiment
# reads. The table and the samples go to log/tools/airframe_envelope/<stamp>/. Host; the experiment runs in the
# dev container. Never beside a flight: it is a simulation of its own.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ "${1:-}" != "--inner" ]]; then
  mkdir -p log/tools/airframe_envelope; touch log/tools/airframe_envelope/.keep
  exec ./scripts/container_run.sh bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash;
    exec bash /workspace/tools/airframe_envelope.sh --inner $*"
fi
shift
out=/workspace/log/tools/airframe_envelope/$(date +%Y%m%d_%H%M%S)
mkdir -p "$out"
px4=/workspace/external/PX4-Autopilot
echo "ENVELOPE start $(date -u +%H:%M:%S) speeds ${*:-5 10 12 15 20} -> $out" | tee "$out/envelope.log"
MicroXRCEAgent udp4 -p 8888 > "$out/agent.log" 2>&1 &
agent=$!
(
  cd "$px4" && rm -f build/px4_sitl_default/rootfs/0/parameters.bson build/px4_sitl_default/rootfs/0/parameters_backup.bson
  HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=default PX4_PARAM_SIM_BAT_MIN_PCT=100 \
    exec build/px4_sitl_default/bin/px4 -d -i 0 > "$out/px4.log" 2>&1
) &
autopilot=$!
sleep 15
# The autopilot's parameters, as a flight sets them: no data link, no radio and no power module are a reason to
# refuse the arming or to leave offboard; the horizontal speed limit at the parameter's top.
for pair in "CBRK_SUPPLY_CHK 894281" "NAV_DLL_ACT 0" "COM_RCL_EXCEPT 4" "MPC_XY_VEL_MAX 20" "MPC_XY_CRUISE 20" \
  "MPC_Z_VEL_MAX_UP 5" "MPC_Z_VEL_MAX_DN 5"; do
  # shellcheck disable=SC2086
  "$px4/build/px4_sitl_default/bin/px4-param" --instance 0 set $pair >> "$out/params.log" 2>&1
done
"$px4/build/px4_sitl_default/bin/px4-param" --instance 0 show MPC_TILTMAX_AIR >> "$out/params.log" 2>&1
sleep 10
# The simulator's motor commands, each block stamped with the wall clock the pilot's samples carry too.
gz topic -e -t /x500_0/command/motor_speed 2> /dev/null | python3 -u -c '
import sys, time
block = []
for line in sys.stdin:
    if line.startswith("velocity:"):
        block.append(float(line.split()[1]))
    elif block:
        print(f"{time.time():.3f} {max(abs(v) for v in block):.1f}", flush=True)
        block = []
' > "$out/motors.txt" &
motors=$!
python3 /workspace/tools/airframe_envelope.py --out "$out" "$@" 2>&1 | tee -a "$out/envelope.log"
status=${PIPESTATUS[0]}
kill "$motors" "$autopilot" "$agent" 2> /dev/null
sleep 2   # the simulator the autopilot started for itself ends with the container
python3 /workspace/tools/airframe_envelope.py --out "$out" --report 2>&1 | tee -a "$out/envelope.log"
echo "ENVELOPE end $(date -u +%H:%M:%S) status $status" | tee -a "$out/envelope.log"
exit "$status"
