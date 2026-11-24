#!/usr/bin/env python3
"""refused_stops.py [--table FILE] [--jobs N] RUN...: what the stack did with a moving vehicle when it could not stop
it along a certified trajectory (roadmap item 43, point 1), from each flight's own records.

A revocation of a moving vehicle (the planner's `EXECUTION_REVOCATION moving=true`) hands the vehicle to the
offboard's local hold, which brakes with no world behind it. For each such revocation the tool reads:

- the stop refused just before it (the `STOP_EXECUTION published=false` line within 50 ms) and, when the stop was
  refused at its commit (`status=publication_commit_rejected`: prepared and certified against the world, refused
  only by the commit's checks), the commit's stage (the `EXECUTION_HORIZON_COMMIT committed=false` line within 50 ms
  before it). Such a revocation is one where a certified motion existed;
- what followed: the first horizon the offboard accepted after it (`EXECUTION_HORIZON accepted=true`), its mode and
  how long after;
- by the truth (gz_pose.csv, on the wall clock the log carries): the path flown until then (at most 5 s), and the
  least clearance of the stack's body model (body_truth_clearance.py's cylinder) to the world's truth voxelization
  over that window and half a second more, where the world has one; a crash in the window (`CRASH_EVENT`,
  `cause=physical_collision`);
- whether the flight counts by its host (host_verdict.py, A7);
- every commit refused at `evidence_not_current` with the clause of its latest-evidence revalidation and the view
  it was read on (`revalidation_clause`, `refused_view`; roadmap item 43 point 5, the clauses naipi11's): a path that
  meets the persistent raw world or the newest scan is the world's answer, a sweep that could not be made the tick's;
- since specification K86 (fe88468d) the stop's line names its stage itself and whether it was the second attempt
  (`refreshed`); a certified stop refused for what the tick captured aged (the input, the control evidence, the
  objective) is a case the rule should have answered, and is counted as such (`rule case`);
- the base: the share of the flight (a sample a second, airborne) in which the body is within 0.5 m of a surface
  anyway; in Urban's corridors the 0.55 m body model reads that close often without a contact.

Every line it reads is throttled to one a second in the node: a count of revocations is a lower bound, and a stretch
of revocations within one second is one row. The per-revocation table goes to --table (tab-separated); the summary
(by world, set and the host's verdict; the speed bands; the worst cases; the three answers) to stdout. The batch's
start, total and progress go to stderr. Host (NumPy).
"""
from __future__ import annotations

import argparse
import bisect
import collections
import csv
import json
import math
import multiprocessing
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import host_verdict  # noqa: E402
from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D  # noqa: E402

STAMP = re.compile(r"\] \[(\d+\.\d+)\] \[")
PAIR_S = 0.05
# The refusals K86 answers with a second attempt from the newest input and objective.
AGED = {"execution_input_not_fresh", "control_evidence_not_current", "objective_changed"}
FOLLOW_S = 5.0
# The world's truth voxelization and the offset that carries the map frame of the recorded poses into its frame
# (drone_city_nav/worlds/<world>.world3d.json, map_to_sdf).
GRIDS = {
    "urban_circuit_practice_01": ("log/tools/truth25/urban_r025.occupancy3d", (0.0, 0.0, 0.0)),
    "cave_circuit_practice_01": ("log/tools/truth25/cave_practice_01_r025.occupancy3d", (106.5, -368.0, -52.5)),
    "finals_prize_round_world_07": ("log/tools/truth25/final_event_07_r025.occupancy3d", (6.5, -233.0, -37.5)),
    "open_sky_baylands": ("external/environment-candidates/work/open_sky_baylands_r050.occupancy3d",
                          (-35.0, -179.5, -9.5)),
}
# body_truth_clearance.py's model: a cylinder of 0.55 m, 0.23 m below and 0.35 m above its centre, 0.24 m over the
# model's origin.
RADIUS, BELOW, ABOVE, CENTRE = 0.55, 0.23, 0.35, 0.24
REACH = 1.0
BANDS = ((0.0, 1.0, "< 1"), (1.0, 3.0, "1-3"), (3.0, 8.0, "3-8"), (8.0, math.inf, "> 8"))
_grids: dict[str, Occupancy3D | None] = {}


def grid(world: str) -> tuple[Occupancy3D | None, tuple[float, float, float]]:
    path, offset = GRIDS.get(world, (None, (0.0, 0.0, 0.0)))
    if world not in _grids:
        _grids[world] = Occupancy3D.load(Path(path)) if path and Path(path).is_file() else None
    return _grids[world], offset


def occupied(occupancy: Occupancy3D, i: int, j: int, k: int) -> bool:
    words = occupancy.chunks.get((i // CHUNK_SIZE, j // CHUNK_SIZE, k // CHUNK_SIZE))
    if words is None:
        return False
    bit = i % CHUNK_SIZE + CHUNK_SIZE * (j % CHUNK_SIZE + CHUNK_SIZE * (k % CHUNK_SIZE))
    return (words[bit // 64] >> (bit % 64)) & 1 == 1


def clearance(occupancy: Occupancy3D, x: float, y: float, z: float) -> float:
    """The body's least distance to an occupied voxel, up to REACH."""
    bounds = occupancy.bounds
    res = bounds.resolution_m
    best = REACH
    i0 = math.floor((x - RADIUS - REACH - bounds.origin_x_m) / res)
    i1 = math.floor((x + RADIUS + REACH - bounds.origin_x_m) / res)
    j0 = math.floor((y - RADIUS - REACH - bounds.origin_y_m) / res)
    j1 = math.floor((y + RADIUS + REACH - bounds.origin_y_m) / res)
    k0 = math.floor((z - BELOW - REACH - bounds.origin_z_m) / res)
    k1 = math.floor((z + ABOVE + REACH - bounds.origin_z_m) / res)
    for i in range(i0, i1 + 1):
        lo_x = bounds.origin_x_m + i * res
        dx = max(lo_x - x, 0.0, x - (lo_x + res))
        for j in range(j0, j1 + 1):
            lo_y = bounds.origin_y_m + j * res
            radial = max(0.0, math.hypot(dx, max(lo_y - y, 0.0, y - (lo_y + res))) - RADIUS)
            if radial >= best:
                continue
            for k in range(k0, k1 + 1):
                lo_z = bounds.origin_z_m + k * res
                distance = math.hypot(radial, max(lo_z - (z + ABOVE), 0.0, (z - BELOW) - (lo_z + res)))
                if distance < best and occupied(occupancy, i, j, k):
                    best = distance
    return best


def field(line: str, name: str) -> str | None:
    match = re.search(rf"\b{name}=(\S+)", line)
    return match.group(1) if match else None


def read_run(run: str) -> dict:
    directory = Path("log/runs") / run
    try:
        manifest = json.load(open(directory / "manifest.json"))
    except (OSError, ValueError):
        return {"run": run, "skipped": "no manifest"}
    overrides = manifest.get("effective_overrides", {})
    world_path = manifest.get("world", {}).get("path", "")
    parts = world_path.split("/")
    world = parts[3] if len(parts) > 3 else world_path
    if "MOVING_BODIES" in overrides:
        world += "+body"
    sensor_set = "lidar" if overrides.get("NAVIGATION_SENSOR_PROFILE") == "lidar" else "cameras"
    try:
        # inspect() lists the reasons a flight is void: none, and it counts.
        counted = not host_verdict.inspect(run)
    except (OSError, ValueError, KeyError, StopIteration):
        counted = None
    stops, commits, accepted, revocations, crashes = [], [], [], [], []
    revalidations: collections.Counter = collections.Counter()
    try:
        log = open(directory / "ros_drone_nav.log", errors="replace")
    except OSError:
        return {"run": run, "skipped": "no log"}
    for line in log:
        stamp = STAMP.search(line)
        if stamp is None:
            continue
        t = float(stamp.group(1))
        if "STOP_EXECUTION published=false" in line:
            stops.append((t, field(line, "status") or "?", field(line, "stage"), field(line, "refreshed")))
        elif "EXECUTION_HORIZON_COMMIT committed=false" in line:
            commits.append((t, field(line, "stage") or "?"))
        elif "EXECUTION_HORIZON_COMMIT evidence_currentness=" in line:
            # One line a commit refused at evidence_not_current (unthrottled since roadmap item 43 point 5).
            revalidations[(field(line, "revalidation_clause") or "unnamed", field(line, "refused_view") or "-")] += 1
        elif "EXECUTION_HORIZON accepted=true" in line:
            accepted.append((t, field(line, "mode") or "?"))
        elif "EXECUTION_REVOCATION moving=true" in line:
            revocations.append((t, float(field(line, "speed_mps") or "nan"), field(line, "reason") or "?"))
        elif "CRASH_EVENT" in line or "cause=physical_collision" in line:
            crashes.append(t)
    truth: list[tuple[float, float, float, float]] = []
    try:
        for row in csv.reader(open(directory / "gz_pose.csv")):
            if len(row) >= 9:
                truth.append((float(row[8]), float(row[1]), float(row[2]), float(row[3])))
    except (OSError, ValueError):
        truth = []
    truth_stamps = [row[0] for row in truth]
    occupancy, offset = grid(world.split("+")[0])
    stop_stamps = [s[0] for s in stops]
    commit_stamps = [c[0] for c in commits]
    accepted_stamps = [a[0] for a in accepted]
    rows = []
    for t0, speed, reason in revocations:
        n = bisect.bisect_right(stop_stamps, t0) - 1
        status, stage, refreshed = "unknown", "", None
        if n >= 0 and t0 - stops[n][0] <= PAIR_S:
            status, stage, refreshed = stops[n][1], stops[n][2] or "", stops[n][3]
            if status == "publication_commit_rejected" and not stage:
                c = bisect.bisect_right(commit_stamps, stops[n][0]) - 1
                if c >= 0 and stops[n][0] - commits[c][0] <= PAIR_S:
                    stage = commits[c][1]
                else:
                    stage = "unknown"
        a = bisect.bisect_right(accepted_stamps, t0)
        followed, wait = ("none", math.nan) if a >= len(accepted) else (accepted[a][1], accepted[a][0] - t0)
        window_end = t0 + min(wait if wait == wait else FOLLOW_S, FOLLOW_S)
        first = bisect.bisect_left(truth_stamps, t0)
        last = bisect.bisect_right(truth_stamps, window_end + 0.5)
        window = truth[first:last]
        path = sum(math.dist(p[1:], q[1:]) for p, q in zip(window, window[1:])
                   if q[0] <= window_end)
        least = math.nan
        if occupancy is not None and window:
            least = min(clearance(occupancy, x + offset[0], y + offset[1], z + offset[2] + CENTRE)
                        for _, x, y, z in window[::5])
        crash = any(t0 <= c <= window_end + 0.5 for c in crashes)
        rows.append({"t": t0, "speed": speed, "reason": reason, "stop": status, "stage": stage,
                     "certified": status == "publication_commit_rejected", "refreshed": refreshed,
                     "rule_case": status == "publication_commit_rejected" and stage in AGED,
                     "followed": followed,
                     "wait": wait, "path": path, "least": least, "crash": crash})
    # The base the windows are read against: the share of the flight, a sample a second once it is 1 m over its
    # start, in which the body is as close to a surface without any revocation.
    base = [0, 0]
    if occupancy is not None and truth:
        start_z = truth[0][3]
        airborne = [row for row in truth if row[3] > start_z + 1.0][::50]
        base = [sum(clearance(occupancy, x + offset[0], y + offset[1], z + offset[2] + CENTRE) <= 0.5
                    for _, x, y, z in airborne), len(airborne)]
    return {"run": run, "world": world, "set": sensor_set, "counted": counted, "rows": rows, "base": base,
            "refused_total": max((int(m.group(1)) for line in open(directory / "ros_drone_nav.log", errors="replace")
                                  if (m := re.search(r"refused_total=(\d+)", line))), default=None),
            "revocations_total": max((int(m.group(1)) for line in open(directory / "ros_drone_nav.log", errors="replace")
                                      if "EXECUTION_REVOCATION moving=true" in line
                                      and (m := re.search(r"\btotal=(\d+)", line))), default=None),
            "stops_refused": collections.Counter(s[1] for s in stops), "start": truth_stamps[0] if truth else None,
            "revalidations": revalidations}


def band(speed: float) -> str:
    return next((name for low, high, name in BANDS if low <= speed < high), "?")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--table", type=Path, help="the tab-separated table of every revocation")
    parser.add_argument("--jobs", type=int, default=os.cpu_count())
    args = parser.parse_args()
    print(f"REFUSED_STOPS start {time.strftime('%F %T %z')} total {len(args.runs)}", file=sys.stderr)
    results = []
    with multiprocessing.Pool(max(1, min(args.jobs, len(args.runs)))) as pool:
        for count, result in enumerate(pool.imap_unordered(read_run, args.runs, chunksize=4), 1):
            results.append(result)
            if count % 50 == 0:
                print(f"REFUSED_STOPS done {count} of {len(args.runs)} at {time.strftime('%T')}", file=sys.stderr)
    results.sort(key=lambda r: int(r["run"].lstrip("r")) if r["run"].lstrip("r").isdigit() else 0)
    flights = [r for r in results if "skipped" not in r]
    every = [(f, row) for f in flights for row in f["rows"]]
    if args.table:
        with open(args.table, "w") as out:
            out.write("run\tworld\tset\tcounted\tt_from_start_s\tspeed_mps\treason\tstop\tstage\tcertified\t"
                      "refreshed\trule_case\tfollowed\twait_s\tpath_m\tleast_clearance_m\tcrash\n")
            for f, row in every:
                since = row["t"] - f["start"] if f["start"] is not None else math.nan
                out.write(f"{f['run']}\t{f['world']}\t{f['set']}\t{f['counted']}\t{since:.2f}\t{row['speed']:.2f}\t"
                          f"{row['reason']}\t{row['stop']}\t{row['stage']}\t{row['certified']}\t{row['refreshed']}\t"
                          f"{row['rule_case']}\t{row['followed']}\t"
                          f"{row['wait']:.3f}\t{row['path']:.2f}\t{row['least']:.2f}\t{row['crash']}\n")
    label = {True: "counted", False: "void", None: "verdict unknown"}
    print(f"Flights read {len(flights)} of {len(results)} "
          f"({len(results) - len(flights)} without a manifest or a log); "
          f"counted {sum(f['counted'] is True for f in flights)}, void {sum(f['counted'] is False for f in flights)}, "
          f"verdict unknown {sum(f['counted'] is None for f in flights)}.")
    print("\nRefused stop lines by status, by world, set and verdict:")
    groups = collections.defaultdict(collections.Counter)
    for f in flights:
        groups[(f["world"], f["set"], label[f["counted"]])].update(f["stops_refused"])
    for key in sorted(groups):
        print(f"  {' / '.join(key)} ({sum(groups[key].values())}): "
              + ", ".join(f"{k} {v}" for k, v in groups[key].most_common()))
    print("\nRevocations of a moving vehicle by speed band (m/s), by world, set and verdict "
          "(certified = the stop was refused only at its commit):")
    bands = collections.defaultdict(lambda: collections.Counter())
    for f, row in every:
        key = (f["world"], f["set"], label[f["counted"]])
        bands[key][band(row["speed"])] += 1
        bands[key]["certified"] += row["certified"]
    for key in sorted(bands):
        c = bands[key]
        print(f"  {' / '.join(key)}: " + ", ".join(f"{name} {c[name]}" for _, _, name in BANDS)
              + f"; total {sum(c[name] for _, _, name in BANDS)}, certified {c['certified']}")
    print("\nThe certified ones by the commit's stage (counted flights / void):")
    stages = collections.Counter((row["stage"], label[f["counted"]]) for f, row in every if row["certified"])
    for stage in sorted({s for s, _ in stages}):
        print(f"  {stage}: {stages[(stage, 'counted')]} / {stages[(stage, 'void')]}"
              + (f" (verdict unknown {stages[(stage, 'verdict unknown')]})" if stages[(stage, 'verdict unknown')] else ""))
    print("\nCommits refused at evidence_not_current, by the revalidation's clause and the view it was read on "
          "(roadmap item 43 point 5; a path that meets evidence is the world's answer, the rest the tick's):")
    clauses: collections.Counter = collections.Counter()
    for f in flights:
        clauses.update(f["revalidations"])
    for (clause, view), count in clauses.most_common():
        print(f"  {clause} / {view}: {count}")
    if not clauses:
        print("  none")
    print("\nWhat followed a revocation (the next horizon the offboard accepted), counted flights:")
    followed = collections.Counter(row["followed"] for f, row in every if f["counted"] is True)
    waits = sorted(row["wait"] for f, row in every if f["counted"] is True and row["wait"] == row["wait"])
    print("  " + ", ".join(f"{k} {v}" for k, v in followed.most_common()))
    if waits:
        print(f"  the wait: median {waits[len(waits) // 2]:.2f} s, p95 {waits[int(0.95 * (len(waits) - 1))]:.2f} s, "
              f"longest {waits[-1]:.2f} s")
    print("\nThe twenty worst (the least true clearance, then the speed), any verdict:")
    measured = [(f, row) for f, row in every if row["least"] == row["least"]]
    worst = sorted(measured, key=lambda fr: (fr[1]["least"], -fr[1]["speed"]))[:20]
    print("| run | world | set | verdict | t, s | speed m/s | stop | stage | followed after s | path m | "
          "least clearance m | crash |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for f, row in worst:
        since = row["t"] - f["start"] if f["start"] is not None else math.nan
        print(f"| {f['run']} | {f['world']} | {f['set']} | {label[f['counted']]} | {since:.1f} | {row['speed']:.2f} | "
              f"{row['stop']} | {row['stage'] or '-'} | {row['followed']} {row['wait']:.2f} | {row['path']:.2f} | "
              f"{row['least']:.2f} | {'yes' if row['crash'] else 'no'} |")
    print("\nThe base: the share of airborne seconds within 0.5 m of a surface, counted flights:")
    base = collections.defaultdict(lambda: [0, 0])
    for f in flights:
        if f["counted"] is True and f["base"][1]:
            key = (f["world"], f["set"])
            base[key][0] += f["base"][0]
            base[key][1] += f["base"][1]
    for key in sorted(base):
        print(f"  {' / '.join(key)}: {base[key][0]} of {base[key][1]} s ({100.0 * base[key][0] / base[key][1]:.1f} %)")
    print("\nThe three answers:")
    for name, flag in (("counted flights", True), ("void flights", False), ("verdict unknown", None)):
        rows = [row for f, row in every if f["counted"] is flag]
        if not rows:
            continue
        close = [row for row in rows if (row["least"] == row["least"] and row["least"] <= 0.5) or row["crash"]]
        read = sum(row["least"] == row["least"] for row in rows)
        print(f"  {name}: a moving vehicle handed to the offboard's local hold {len(rows)} times (lower bound); "
              f"a certified motion existed (the stop refused only at its commit) in {sum(r['certified'] for r in rows)}"
              f"; within 0.5 m of a surface by the truth or a crash in {len(close)} "
              f"(of them certified {sum(r['certified'] for r in close)}, crashes {sum(r['crash'] for r in close)}; "
              f"clearance read in {read}, {100.0 * len(close) / read if read else 0.0:.1f} % of them)")
    totals = [f for f in flights if f["revocations_total"] is not None or f["refused_total"] is not None]
    if totals:
        print("\nThe running counts the lines carry since K86 (whole, through the throttle): flight, revocations of a "
              "moving vehicle, stops finally refused at their commit; and the rule cases read from the lines:")
        for f in totals:
            print(f"  {f['run']} {f['world']} / {f['set']} / {label[f['counted']]}: revocations "
                  f"{f['revocations_total'] or 0}, refused at commit {f['refused_total'] or 0}, rule cases "
                  f"{sum(r['rule_case'] for r in f['rows'])}, refreshed refusals "
                  f"{sum(r['refreshed'] == 'true' for r in f['rows'])}")
    print(f"REFUSED_STOPS end {time.strftime('%T')}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
