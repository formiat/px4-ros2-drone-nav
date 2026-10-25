#!/usr/bin/env bash
# cores_series_loaded.sh GROUP FIRST COUNT TAG [LOAD_CPUS]: cores_series.sh under a controlled foreign load on the CPUs
# outside the flight's core set (specification A4, part 0.3 of the root goal of 2026-10-24): stress-ng busies LOAD_CPUS
# (12-15 unless given; one worker a CPU, 100 percent; `all` pins nothing and takes WORKERS workers, 2 unless set) from 30 s before the first flight to the end of the batch, and
# mpstat samples every CPU every 10 s into log/tools/flight_cores/mpstat_TAG.txt as the proof that the load sat where it
# was put. The load is the batch's own process and is killed by its PID at the end; nothing of the owner's is touched.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."
group=$1; first=$2; count=$3; tag=$4; load_cpus=${5:-12-15}
workers=${WORKERS:-$(python3 -c "import sys; s=sys.argv[1]; n=0
for p in s.split(','):
    a,_,b=p.partition('-'); n+= (int(b)-int(a)+1) if b else 1
print(n)" "${load_cpus/all/0-1}")}
mkdir -p log/tools/flight_cores
echo "== load start $(date -u +%H:%M:%S): stress-ng --cpu $workers on $load_cpus for the batch" | tee -a "log/tools/flight_cores/series_${tag}.txt"
if [ "$load_cpus" = all ]; then stress-ng --cpu "$workers" --timeout 6h --quiet & else stress-ng --cpu "$workers" --taskset "$load_cpus" --timeout 6h --quiet & fi
load_pid=$!
mpstat -P ALL 10 > "log/tools/flight_cores/mpstat_${tag}.txt" 2>&1 &
mpstat_pid=$!
sleep 30
./tools/cores_series.sh "$group" "$first" "$count" "$tag"
kill "$load_pid" "$mpstat_pid" 2>/dev/null
sleep 1
echo "== load end $(date -u +%H:%M:%S): stress-ng left $(pgrep -c "[s]tress-ng" || true)" | tee -a "log/tools/flight_cores/series_${tag}.txt"
