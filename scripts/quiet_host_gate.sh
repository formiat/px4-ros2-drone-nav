#!/usr/bin/env bash
# Waits until the host has been quiet for a minute before a simulation flight:
# no rustc, clippy or cargo above 5 percent of a core, a one-minute load below
# 3, and no simulator of a previous run. A flight that shares the host with a
# build is not the flight that was asked for (r592, r593, r619: real-time
# factors of 0.57 to 0.66 and 20 to 40 percent off the mean speed), and the
# project owner's rule of 2026-09-25 is that such a flight is never flown.
# Foreign processes are never touched: the gate waits for them to end.
set -euo pipefail

quiet_s=0
while [[ "${quiet_s}" -lt 60 ]]; do
  busy="$(ps -eo pcpu,comm | awk '$2 ~ /^(rustc|clippy-driver|cargo)$/ && $1 > 5 {count++} END {print count + 0}')"
  load="$(cut -d' ' -f1 /proc/loadavg)"
  simulators="$(pgrep -c -f '[g]z sim' || true)"
  if [[ "${busy}" -eq 0 && "${load%.*}" -lt 3 && "${simulators}" -eq 0 ]]; then
    quiet_s=$((quiet_s + 10))
  else
    quiet_s=0
    printf 'quiet_host_gate: waiting (busy rust processes %s, load %s, simulators %s)\n' \
      "${busy}" "${load}" "${simulators}" >&2
  fi
  sleep 10
done
