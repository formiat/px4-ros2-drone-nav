#!/usr/bin/env bash
# record_flight.sh RUN [recorder args]: one flight on the defaults through series.sh with the visual-inertial recorder attached
# inside the simulation container from its start to the end of the flight. Record: log/tools/vio/RUN/.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
run=$1; shift
before=$(docker ps -q | sort)
tools/series2.sh "$run" > "log/tools/vio/series_$run.log" 2>&1 &
series=$!
c=""; for _ in $(seq 1 900); do sleep 1; c=$(docker ps --format '{{.ID}}' | sort | comm -13 <(echo "$before") - | head -1); [ -n "$c" ] && break; done
mkdir -p "log/tools/vio/$run"
docker exec "$c" bash -lc "source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; sleep 25; exec python3 /workspace/tools/vio/record_vio.py /workspace/log/tools/vio/$run $*" > "log/tools/vio/${run}_rec.log" 2>&1 &
docker exec "$c" bash -lc "sleep 25; exec /workspace/log/tools/vio/record_frames /workspace/log/tools/vio/$run" > "log/tools/vio/${run}_frames.log" 2>&1 &
wait "$series"
./scripts/stop_sim.sh > /dev/null 2>&1
python3 tools/vio/trim_record.py "log/tools/vio/$run" >> "log/tools/vio/${run}_rec.log" 2>&1
echo "RECORDED $run" >> "log/tools/vio/${run}_rec.log"
