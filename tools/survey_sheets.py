#!/usr/bin/env python3
"""survey_sheets.py RUN... --truth GRID [--reanchored]: every displaced sheet of a survey's map (tools/survey_map_check.py's measure:
the memory's occupied cells farther than half a metre from the truth's, 26-connected components of forty cells or
more, in the flight's last raw snapshot) with when it was laid and what the estimate's error was then (roadmap item
41: which of a map's sheets the estimate's error can have laid).

A sheet is laid in the first of the flight's raw snapshots (log/runs/RUN/raw_snapshot_3d_revision_*.json, in the
order of their stamps) that holds half of its cells. Over the time between that snapshot and the one before it the
flown estimate is set against the truth (mppi/mppi_track.jsonl against gz_pose.csv, as tools/estimate_error.py sets
them, in three dimensions) and the largest error is taken; the true distance from the vehicle to the sheet's centre
at its nearest in that time is printed beside it. A sheet whose median distance off the truth exceeds that error by
more than two cells is not the estimate's: no pose the estimate held while the sheet was laid puts a surface there.
The others may be.

--reanchored adds what the map would hold had the memory followed the closures (specification K92 leaves it as laid):
each occupied cell, laid at the stamp of the first snapshot that holds it, is moved by the share of the estimate's
error that an ideal closure would have taken out of that moment. The closure is the first accepted one after the
cell's moment whose matched keyframe is before it (the log's VIO_LOOP_CLOSURE lines); it is taken to bring the
estimate to the truth (the mean error over the three seconds before it, as flown), and that correction is spread
over the true path between its two keyframes in proportion to the length flown. The sheets of the moved cells are
counted as the map's own are: an upper bound of what re-anchoring the memory by a pose graph can give, read with
the truth. Beside it, the sheets left when every cell is moved by the whole of the estimate's error at its own
moment (the mean over the time since the snapshot before): what no estimate lays, the depth's own. The map frame is the truth's (Urban; a world with an offset between them is refused). Host (SciPy).

  python3 tools/survey_sheets.py r4317 r4319 --truth log/tools/truth25/urban_r025.occupancy3d --reanchored
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))
import survey_coverage  # noqa: E402

DISPLACED_M = 0.5
SHEET_CELLS = 40
LAID_SHARE = 0.5
REST_OFFSET_M = 0.24  # the estimate's origin over the truth's, as tools/estimate_error.py reads it


CLOSURE = re.compile(r"VIO_LOOP_CLOSURE outcome=accepted stamp_s=([0-9.]+) matched_stamp_s=([0-9.]+)")


def errors(run_directory: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The stamps of the flown estimate, its error against the truth (a vector), and the true positions."""
    truth = [[float(v) for v in row] for row in csv.reader(open(run_directory / "gz_pose.csv"))
             if row and row[0][0].isdigit()]
    stamps = [row[0] for row in truth]
    out_stamps, out_errors, out_truth = [], [], []
    for line in open(run_directory / "mppi" / "mppi_track.jsonl"):
        record = json.loads(line)
        t = record["stamp_ns"] / 1.0e9
        i = bisect.bisect_left(stamps, t)
        if i >= len(truth):
            continue
        true = np.array(truth[i][1:4])
        delta = np.array(record["p"]) - true
        delta[2] -= REST_OFFSET_M
        out_stamps.append(t)
        out_errors.append(delta)
        out_truth.append(true)
    return np.array(out_stamps), np.array(out_errors), np.array(out_truth)


def count_sheets(far: np.ndarray) -> tuple[int, int]:
    """The sheets among the displaced cells, and the cells in them."""
    from scipy import ndimage

    labels, _ = ndimage.label(far, structure=np.ones((3, 3, 3)))
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    return int((sizes >= SHEET_CELLS).sum()), int(sizes[sizes >= SHEET_CELLS].sum())


def reanchored(run_directory: Path, snapshots: list, occupied: np.ndarray, distance: np.ndarray, res: float,
               stamps: np.ndarray, error: np.ndarray, true: np.ndarray) -> tuple[int, int, int, int, int]:
    """The sheets and their cells after the ideal re-anchoring, how many cells it moved, and the sheets and their
    cells with the estimate's whole error taken out."""
    first = np.full(occupied.shape, -1, dtype=int)
    for index, (_, path) in enumerate(snapshots):
        then = survey_coverage.load_memory(path, None)[4]
        first[(first < 0) & then & occupied] = index
    along = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(true, axis=0), axis=1))])
    closures = [(float(m.group(1)), float(m.group(2)))
                for m in map(CLOSURE.search, open(run_directory / "ros_drone_nav.log", errors="replace")) if m]
    moved = np.zeros_like(occupied)
    exact = np.zeros_like(occupied)
    count = 0
    for index in range(len(snapshots)):
        cells = np.argwhere(first == index)
        if not len(cells):
            continue
        moment = snapshots[index][0]
        shift = np.zeros(3)
        closure = next(((at, matched) for at, matched in closures if matched < moment < at), None)
        if closure is not None:
            at, matched = closure
            before = (stamps > at - 3.0) & (stamps <= at)
            s_matched, s_moment, s_at = np.interp([matched, moment, at], stamps, along)
            if before.any() and s_at > s_matched:
                shift = -error[before].mean(axis=0) * (s_moment - s_matched) / (s_at - s_matched)
        window = (stamps > (snapshots[index - 1][0] if index > 0 else float("-inf"))) & (stamps <= moment)
        whole = -error[window].mean(axis=0) if window.any() else np.zeros(3)
        for grid, by in ((moved, shift), (exact, whole)):
            target = cells + np.round(by / res).astype(int)
            target = target[np.all((target >= 0) & (target < np.array(occupied.shape)), axis=1)]
            grid[tuple(target.T)] = True
        count += len(cells) if np.any(np.round(shift / res)) else 0
    return (*count_sheets(moved & (distance > DISPLACED_M + 1e-9)), count,
            *count_sheets(exact & (distance > DISPLACED_M + 1e-9)))


def sheets_of(run: str, truth_path: Path, with_reanchored: bool = False) -> tuple[list[dict], tuple | None]:
    from scipy import ndimage

    run_directory = REPOSITORY / "log/runs" / run
    manifest = json.loads((run_directory / "manifest.json").read_text())
    scenario = manifest.get("effective_overrides", {}).get("POINT_TO_POINT_SCENARIO_PATH", "")
    if scenario and any(survey_coverage.scenario_frame(REPOSITORY / scenario)[1]):
        raise ValueError(f"{run}: the map frame is offset from the truth's")
    snapshots = []
    for path in run_directory.glob("raw_snapshot_3d_revision_*.json"):
        stamp = json.loads(path.read_text())["header"]["stamp"]
        snapshots.append((stamp["sec"] + stamp.get("nanosec", 0) * 1.0e-9, path))
    snapshots.sort()
    if not snapshots:
        raise FileNotFoundError(f"{run}: no raw snapshot")
    _, res, origin, _, occupied = survey_coverage.load_memory(snapshots[-1][1], None)
    truth = survey_coverage.load_truth(truth_path, res, origin, occupied.shape)
    distance = ndimage.distance_transform_edt(~truth) * res
    far = occupied & (distance > DISPLACED_M + 1e-9)
    labels, _ = ndimage.label(far, structure=np.ones((3, 3, 3)))
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    members = {int(label): labels == label for label in np.argsort(sizes)[::-1] if sizes[label] >= SHEET_CELLS}
    laid = {}
    for index, (_, path) in enumerate(snapshots):
        _, _, _, _, then = survey_coverage.load_memory(path, None)
        for label, member in members.items():
            if label not in laid and (then & member).sum() >= LAID_SHARE * member.sum():
                laid[label] = index
    stamps, error, true = errors(run_directory)
    out = []
    for label, member in members.items():
        index = laid[label]
        begin = snapshots[index - 1][0] if index > 0 else float("-inf")
        window = (stamps > begin) & (stamps <= snapshots[index][0])
        cells = np.argwhere(member)
        centre = origin + (cells.mean(axis=0) + 0.5) * res
        off = distance[member]
        worst = float(np.linalg.norm(error[window], axis=1).max()) if window.any() else float("nan")
        nearest = float(np.linalg.norm(true[window] - centre, axis=1).min()) if window.any() else float("nan")
        out.append({"cells": int(member.sum()), "off_median_m": float(np.median(off)), "off_max_m": float(off.max()),
                    "centre_m": [float(v) for v in centre], "laid_from_s": begin, "laid_to_s": snapshots[index][0],
                    "estimate_error_m": worst, "nearest_m": nearest,
                    "estimates": bool(np.median(off) <= worst + 2.0 * res)})
    return out, (reanchored(run_directory, snapshots, occupied, distance, res, stamps, error, true)
                 if with_reanchored else None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+")
    parser.add_argument("--truth", type=Path, required=True)
    parser.add_argument("--reanchored", action="store_true")
    arguments = parser.parse_args()
    for run in arguments.runs:
        sheets, moved = sheets_of(run, arguments.truth, arguments.reanchored)
        may = sum(1 for sheet in sheets if sheet["estimates"])
        print(f"{run}: sheets {len(sheets)}, the estimate's error may have laid {may}, not the estimate's {len(sheets) - may}")
        for sheet in sheets:
            centre = sheet["centre_m"]
            print(f"  cells {sheet['cells']:5d}  off the truth median {sheet['off_median_m']:.2f} max "
                  f"{sheet['off_max_m']:.2f} m  centre ({centre[0]:6.1f},{centre[1]:6.1f},{centre[2]:5.1f})  laid "
                  f"{max(sheet['laid_from_s'], 0.0):6.0f} to {sheet['laid_to_s']:6.0f} s  the estimate's error then "
                  f"{sheet['estimate_error_m']:.2f} m  seen from {sheet['nearest_m']:.1f} m  "
                  f"{'may be the estimate' if sheet['estimates'] else 'not the estimate'}")
        if moved is not None:
            print(f"  re-anchored by ideal closures: sheets {moved[0]} of {moved[1]} cells (as laid "
                  f"{len(sheets)} of {sum(sheet['cells'] for sheet in sheets)}), {moved[2]} cells moved; with the "
                  f"estimate's whole error taken out: sheets {moved[3]} of {moved[4]} cells")
    return 0


if __name__ == "__main__":
    sys.exit(main())
