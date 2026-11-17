#!/usr/bin/env python3
"""flight_sweep.py [--problems-only] RUN...: every flight of a series read for what is wrong with it, at once.

The rule of 2026-11-17 (docs/autonomous_development.md, "Flights"): a series is flown to its first problem, then
every flight so far is swept and everything the sweep shows is repaired in one batch; a defect found by reading
one flight after the whole series was flown costs the series again (roadmap item 21's acceptance was flown three
times for single repairs). One row a flight, from its own records (`log/tools/run_RUN.log`, the run's
`ros_drone_nav.log`, `tools/body_encounters.py` where the flight carried a walking body):

- the mission check's failing lines (the raw snapshot's line of a flight flown with `RAW_SNAPSHOT_BOUNDS_M` is the
  measurement's own and is named apart), the mean speed, where the goal was acknowledged by the truth;
- the stops: published, refused, the longest row of refused ones, and the horizon commits refused (roadmap item
  43: a stop refused seven times in a row flew a vehicle into a wall);
- the bodies the memory read: the scans with a body, with two bodies on one place and two apart; a body read in a
  flight that carried none is a false one;
- with a walking body: the stopping margin, the stale trace cells, the contact.

A problem (the row's last column, and the exit status 1): a failing line that is not the measurement's own, a
contact, a margin not over nought, a stale cell, a false body, five stops refused in a row. The batch's start, total
and count are logged for a long list.
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
from pathlib import Path

REFUSED_IN_A_ROW = 5
SNAPSHOT_LINE = "raw snapshot artifact matches the constrained route volume"
BODY = re.compile(r"\[([\d.\-]+) ([\d.\-]+) [\d.\-]+ r[\d.]+ h[\d.]+ v\S+ n\d+ a[\d.]+ d[\d.]+ (?:warm|size) ")


def verdict_lines(text: str) -> tuple[list[str], float | None, float | None]:
    """The failing lines of a mission check, the mean speed and the goal's distance by the truth."""
    fails = [line[6:].strip() for line in text.splitlines() if line.startswith("FAIL: ")]
    speed = re.search(r"mean flight speed is ([\d.]+) m/s", text)
    goal = re.search(r"true position is (?:inside the goal's capture radius at the acknowledgement \()?([\d.]+) m from",
                     text)
    return fails, float(speed.group(1)) if speed else None, float(goal.group(1)) if goal else None


def stops(lines) -> tuple[int, int, int, int]:
    """Stops published and refused, the longest row of refused ones, the horizon commits refused."""
    published = refused = row = longest = commits = 0
    for line in lines:
        if "STOP_EXECUTION published=true" in line:
            published += 1
            row = 0
        elif "STOP_EXECUTION published=false" in line:
            refused += 1
            row += 1
            longest = max(longest, row)
        elif "EXECUTION_HORIZON_COMMIT committed=false" in line:
            commits += 1
    return published, refused, longest, commits


def bodies(lines) -> tuple[int, int, int]:
    """Scans with a body read, with two on one place (within half a metre), with two apart."""
    read = same = apart = 0
    for line in lines:
        if "FOLLOWED_BODIES" not in line or "followed=0 " in line:
            continue
        read += 1
        places = [(float(m.group(1)), float(m.group(2))) for m in BODY.finditer(line)]
        if len(places) >= 2:
            gap = min(math.dist(a, b) for i, a in enumerate(places) for b in places[i + 1:])
            same += gap < 0.5
            apart += gap >= 0.5
    return read, same, apart


def encounter(run: Path) -> tuple[float | None, int | None, bool]:
    """The stopping margin, the stale cells and the contact of a flight with a walking body."""
    result = subprocess.run([sys.executable, str(Path(__file__).with_name("body_encounters.py")), str(run)],
                            capture_output=True, text=True, check=False)
    rows = [line for line in result.stdout.splitlines() if line.startswith("| r")]
    if not rows:
        return None, None, False
    cells = [cell.strip() for cell in rows[-1].split("|")]
    margin = float(cells[7]) if re.fullmatch(r"[+\-][\d.]+", cells[7]) else None
    stale = re.search(r"\((\d+),", cells[9])
    return margin, int(stale.group(1)) if stale else None, cells[10] != "no"


def sweep(name: str) -> tuple[str, list[str]]:
    run = Path(name) if "/" in name else Path("log/runs") / name
    check = Path("log/tools") / f"run_{run.name}.log"
    fails, speed, goal = verdict_lines(check.read_text(errors="replace") if check.is_file() else "")
    log = run / "ros_drone_nav.log"
    lines = log.read_text(errors="replace").splitlines() if log.is_file() else []
    published, refused, longest, commits = stops(lines)
    read, same, apart = bodies(lines)
    carried = (run / "moving_bodies.csv").is_file()
    problems = [fail[:60] for fail in fails if SNAPSHOT_LINE not in fail]
    if not check.is_file():
        problems.append("no mission check")
    if longest >= REFUSED_IN_A_ROW:
        problems.append(f"{longest} stops refused in a row")
    if same:
        problems.append(f"two bodies on one place in {same} scans")
    margin = stale = None
    if carried:
        margin, stale, contact = encounter(run)
        if contact:
            problems.append("contact with the body")
        if margin is not None and margin <= 0.0:
            problems.append(f"margin {margin:+.2f} m")
        if stale:
            problems.append(f"{stale} stale cells")
    elif read:
        problems.append(f"a body read in {read} scans of a flight without one")
    row = (f"| {run.name} | {'-' if speed is None else f'{speed:.3f}'} | {'-' if goal is None else f'{goal:.2f}'} | "
           f"{published} / {refused} / {longest} / {commits} | {read} / {same} / {apart} | "
           f"{'-' if margin is None else f'{margin:+.2f}'} / {'-' if stale is None else stale} | "
           f"{'; '.join(problems) if problems else 'none'} |")
    return row, problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--problems-only", action="store_true", help="print only the flights with a problem")
    args = parser.parse_args()
    print(f"SWEEP start {time.strftime('%F %T %z')} total {len(args.runs)}", file=sys.stderr)
    print("| run | mean m/s | goal by truth m | stops published / refused / in a row / commits refused | "
          "scans with a body / two on one place / two apart | margin m / stale cells | problems |")
    print("|---|---|---|---|---|---|---|")
    bad = 0
    for count, name in enumerate(args.runs, 1):
        row, problems = sweep(name)
        bad += bool(problems)
        if problems or not args.problems_only:
            print(row, flush=True)
        if count % 10 == 0:
            print(f"SWEEP done {count} of {len(args.runs)} at {time.strftime('%T')}", file=sys.stderr)
    print(f"SWEEP end {time.strftime('%T')}: {bad} of {len(args.runs)} with a problem", file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
