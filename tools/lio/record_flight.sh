#!/usr/bin/env bash
# record_flight.sh RUN: one flight on the lidar profile through fly_until_valid.sh (one attempt: the quiet-host gate,
# the watchdog and the host's verdict, specification A4 and A7) with a rosbag of the lidar scans and the autopilot's
# IMU recorded inside the simulation container from its start to the end of the flight; the bag is then extracted
# for the replay. Record: log/tools/replay/RUN/ (scans.bin, imu.csv, attitude.csv, baro.csv, bag/); the verdict in log/tools/fly_until_valid.txt.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
run=$1
before=$(docker ps -q | sort)
mkdir -p "log/tools/replay/$run"
tools/fly_until_valid.sh --lidar "${run#r}" 1 > "log/tools/replay/${run}_series.log" 2>&1 &
series=$!
c=""; for _ in $(seq 1 900); do sleep 1; c=$(docker ps --format '{{.ID}}' | sort | comm -13 <(echo "$before") - | head -1); [ -n "$c" ] && break; done
docker exec "$c" bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; sleep 25; exec ros2 bag record -s mcap --max-cache-size 268435456 -o /workspace/log/tools/replay/$run/bag /lidar_3d/points /fmu/out/sensor_combined /fmu/out/timesync_status /fmu/out/vehicle_attitude /barometer/air_pressure" > "log/tools/replay/${run}_bag.log" 2>&1 &
wait "$series"
./scripts/stop_sim.sh > /dev/null 2>&1
bag=$(ls "log/tools/replay/$run"/bag/*.mcap 2>/dev/null | head -1)
./scripts/container_run.sh bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; python3 /workspace/tools/lio/extract_bag.py /workspace/${bag} /workspace/log/tools/replay/$run" >> "log/tools/replay/${run}_bag.log" 2>&1
echo "RECORDED $run" >> "log/tools/replay/${run}_bag.log"
