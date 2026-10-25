#!/usr/bin/env bash
# Waits until the host has been quiet for a minute before a simulation flight:
# no rustc, clippy or cargo above 5 percent of a core, a one-minute load below
# 3, and no simulator of a previous run. A flight that shares the host with a
# build is not the flight that was asked for (r592, r593, r619: real-time
# factors of 0.57 to 0.66 and 20 to 40 percent off the mean speed), and the
# project owner's rule of 2026-09-25 is that such a flight is never flown.
# Foreign processes are never touched: the gate waits for them to end.
# On the flight's core set (FLIGHT_CPUSET, specification A4) the load that
# counts is the set's own: each of its CPUs at most 10 percent busy over the
# 10 s sample, read from /proc/stat, whatever the other CPUs do.
set -euo pipefail

cpuset_cpus() {
  local part i
  for part in ${FLIGHT_CPUSET//,/ }; do
    if [[ "${part}" == *-* ]]; then
      for ((i = ${part%-*}; i <= ${part#*-}; i++)); do echo "${i}"; done
    else
      echo "${part}"
    fi
  done
}

# The busiest CPU of the set, in percent, between two /proc/stat snapshots.
set_busy_percent() {
  awk -v cpus="$(cpuset_cpus | tr '\n' ' ')" '
    BEGIN { split(cpus, want, " "); for (k in want) wanted["cpu" want[k]] = 1; worst = 0 }
    $1 in wanted {
      total = 0; for (f = 2; f <= NF; f++) total += $f
      idle = $5 + $6
      if (!($1 in t0)) { t0[$1] = total; i0[$1] = idle; next }
      if (total > t0[$1]) busy = 100 * (1 - (idle - i0[$1]) / (total - t0[$1])); else busy = 0
      if (busy > worst) worst = busy
    }
    END { printf "%d\n", worst }' "$1" "$2"
}

quiet_s=0
while [[ "${quiet_s}" -lt 60 ]]; do
  stat_before="$(cat /proc/stat)"
  sleep 10
  busy="$(ps -eo pcpu,comm | awk '$2 ~ /^(rustc|clippy-driver|cargo)$/ && $1 > 5 {count++} END {print count + 0}')"
  simulators="$(pgrep -c -f '[g]z sim' || true)"
  if [[ -n "${FLIGHT_CPUSET:-}" ]]; then
    load="$(set_busy_percent <(printf '%s\n' "${stat_before}") /proc/stat)"
    calm=$(( load <= 10 ))
    load="${load}% busy on ${FLIGHT_CPUSET}"
  else
    load="$(cut -d' ' -f1 /proc/loadavg)"
    calm=$(( ${load%.*} < 3 ))
  fi
  if [[ "${busy}" -eq 0 && "${calm}" -eq 1 && "${simulators}" -eq 0 ]]; then
    quiet_s=$((quiet_s + 10))
  else
    quiet_s=0
    printf 'quiet_host_gate: waiting (busy rust processes %s, load %s, simulators %s)\n' \
      "${busy}" "${load}" "${simulators}" >&2
  fi
done
