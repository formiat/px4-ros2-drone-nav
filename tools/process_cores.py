#!/usr/bin/env python3
"""process_cores.py RUN...: what each process of a flight cost, from the run's resources.csv (one sample a second,
scripts/capture_process_resources.py): cores at the median and the 95th percentile per process, the onboard set's sum
(the processes scripts/resource_budget_evidence.py counts onboard) at p50 and p95, the simulator's real-time factor at
the median, the GPU's utilisation at the median, and the flight's core set when the manifest records one
(FLIGHT_CPUSET, roadmap item 28 step 1, specification A4). One block a run; "python3 -I" on the host, from the
repository's root. Roadmap item 28 step 1, the root goal of 2026-10-24 part 0."""
import collections
import csv
import json
import statistics
import sys
from pathlib import Path

ONBOARD = ("production_mppi_node", "obstacle_memory_3d_node", "stereo_depth_node", "gazebo_stereo_depth_node",
           "lidar_inertial_odometry_node", "mppi_offboard_node", "MicroXRCEAgent")


def quantile(values, share):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * share))]


def main():
    for run in sys.argv[1:]:
        directory = Path("log/runs") / run
        by_process = collections.defaultdict(list)
        onboard_by_stamp = collections.defaultdict(float)
        rtf, gpu = [], []
        for row in csv.DictReader(open(directory / "resources.csv")):
            cores = float(row["cpu_cores"])
            by_process[row["name"]].append(cores)
            if row["name"] in ONBOARD:
                onboard_by_stamp[row["stamp_s"]] += cores
            for column, store in (("real_time_factor", rtf), ("gpu_utilization_percent", gpu)):
                try:
                    value = float(row[column])
                except (TypeError, ValueError):
                    continue
                if value == value:
                    store.append(value)
        cpuset = ""
        try:
            overrides = json.loads((directory / "manifest.json").read_text()).get("effective_overrides", {})
            cpuset = overrides.get("FLIGHT_CPUSET", "")
        except (OSError, ValueError):
            pass
        onboard = list(onboard_by_stamp.values())
        print(f"{run}: onboard set p50 {statistics.median(onboard):.2f} p95 {quantile(onboard, 0.95):.2f} cores; "
              f"rtf p50 {statistics.median(rtf) if rtf else float('nan'):.2f}; "
              f"gpu p50 {statistics.median(gpu) if gpu else float('nan'):.0f}%; cpuset '{cpuset or 'none'}'")
        for name, values in sorted(by_process.items(), key=lambda item: -statistics.median(item[1])):
            if statistics.median(values) < 0.02:
                continue
            tag = "onboard" if name in ONBOARD else "       "
            print(f"  {tag} {name:36s} p50 {statistics.median(values):5.2f} p95 {quantile(values, 0.95):5.2f}")


if __name__ == "__main__":
    main()
