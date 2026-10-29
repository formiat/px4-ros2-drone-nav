#!/usr/bin/env python3
"""node_budget.py [--out FILE] RUN...: the onboard budget measured on counted flights (roadmap item 36, specification
K53): for each run, from `log/runs/RUN/mppi/mppi_ticks.jsonl` the planner's tick (`tick_total_ms`: the whole tick with
its assembly and publication; `controller_ms` the MPPI update alone) and the ages the planner saw (`pose_age_ms`,
`observation_age_ms`, `esdf_content_age_ms`), and from `log/runs/RUN/ros_drone_nav.log` the obstacle memory's
integration of one scan (`integration_ms` of the LIDAR3D_MEMORY lines), the estimator's work (`frame_ms` of the
camera estimator's status, `scan_ms` of the lidar-inertial one) and the stereo match (`match_ms` of STEREO_DEPTH): the
median and the 95th percentile of each, then the same over all the runs of a sensor set. These are the figures the
budget relay delays the nodes' outputs by (the median) and the register keeps (K53); they are measured on the host
of record, never set by hand.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

FIELDS = {
    "planner tick_total_ms": ("ticks", "tick_total_ms"),
    "planner controller_ms": ("ticks", "controller_ms"),
    "planner pose_age_ms": ("ticks", "pose_age_ms"),
    "planner observation_age_ms": ("ticks", "observation_age_ms"),
    "planner esdf_content_age_ms": ("ticks", "esdf_content_age_ms"),
    "memory integration_ms": ("log", r"LIDAR3D_MEMORY accepted=true .*?integration_ms=([0-9.]+)"),
    "camera estimator frame_ms": ("log", r"VISUAL_INERTIAL_ODOMETRY healthy=.*?frame_ms=([0-9.]+)"),
    "lidar estimator scan_ms": ("log", r"LIDAR_INERTIAL_ODOMETRY healthy=.*?scan_ms=([0-9.]+)"),
    "stereo match_ms": ("log", r"STEREO_DEPTH pairs=.*?match_ms=([0-9.]+)"),
}


def samples_of(run: str) -> dict[str, np.ndarray]:
    base = Path("log/runs") / run
    ticks = []
    with (base / "mppi" / "mppi_ticks.jsonl").open() as stream:
        for line in stream:
            try:
                ticks.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    log = (base / "ros_drone_nav.log").read_text(errors="ignore")
    out = {}
    for name, (source, key) in FIELDS.items():
        if source == "ticks":
            values = [t[key] for t in ticks if isinstance(t.get(key), (int, float)) and t[key] >= 0]
        else:
            values = [float(v) for v in re.findall(key, log)]
        if values:
            out[name] = np.array(values, dtype=float)
    return out


def summary(values: np.ndarray) -> str:
    return f"p50 {np.percentile(values, 50):7.1f}  p95 {np.percentile(values, 95):7.1f}  n {len(values):6d}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("runs", nargs="+")
    args = parser.parse_args()
    lines = []
    pooled: dict[str, list[np.ndarray]] = {}
    for run in args.runs:
        try:
            samples = samples_of(run)
        except OSError as error:
            lines.append(f"{run}: {error}")
            continue
        lines.append(f"== {run}")
        for name, values in samples.items():
            lines.append(f"  {name:30s} {summary(values)}")
            pooled.setdefault(name, []).append(values)
    lines.append("== all runs pooled")
    for name, parts in pooled.items():
        lines.append(f"  {name:30s} {summary(np.concatenate(parts))}  runs {len(parts)}")
    text = "\n".join(lines) + "\n"
    sys.stdout.write(text)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
