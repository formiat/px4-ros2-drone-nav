#!/usr/bin/env python3
"""The smoke's checks of the truth (headless_runtime_evidence.py re-exports
them): the vehicle never enters dense smoke (the owner's rule, specification
R7) and keeps the standoff from a heat source (roadmap item 32, K49), both
judged on the true pose record of the run."""
from __future__ import annotations

import csv
import math
from pathlib import Path


def true_poses(truth_path: Path) -> list[tuple[float, float, float, float]]:
    """The true pose record whole: (time, x, y, z) a sample."""
    rows = []
    if truth_path.is_file():
        with truth_path.open(newline="", encoding="utf-8") as stream:
            for row in csv.reader(stream):
                if len(row) >= 4:
                    rows.append(tuple(float(value) for value in row[:4]))
    return rows


DENSE_SMOKE_SIGMA_PER_M = 0.5


def validate_no_entry_into_dense_smoke(truth_path: Path, overrides: dict,
                                       errors: list[str]) -> None:
    """The owner's rule (2026-10-18, specification R7): the vehicle never
    enters dense smoke, judged in truth. The zones are the run's own
    (`SMOKE_ZONES` in the manifest, x,y,z,sx,sy,sz,sigma boxes); dense is
    0.5 per metre and more (K37's aerosol line; the pair sees through 0.1).
    The true pose record is read whole: the first sample inside a dense box
    fails the flight with the time and the depth of the entry."""
    zones = []  # (the zone's index in the manifest, its box)
    for index, item in enumerate(str(overrides.get("SMOKE_ZONES", "")).split(";")):
        if item.strip():
            values = [float(value) for value in item.split(",")]
            if len(values) == 7 and values[6] >= DENSE_SMOKE_SIGMA_PER_M:
                zones.append((index, values))
    if not zones:
        print("NOTE: the flight has no dense smoke to keep out of")
        return
    rows = true_poses(truth_path)
    if len(rows) < 2:
        errors.append(f"FAIL: the flight recorded the true pose to judge the smoke by "
                      f"({truth_path.name})")
        return
    for index, (x, y, z, sx, sy, sz, sigma) in zones:
        inside = [(at_s, max(abs(px - x) - sx / 2.0, abs(py - y) - sy / 2.0,
                             abs(pz - z) - sz / 2.0))
                  for at_s, px, py, pz in rows
                  if abs(px - x) <= sx / 2.0 and abs(py - y) <= sy / 2.0 and
                  abs(pz - z) <= sz / 2.0]
        if inside:
            errors.append(
                f"FAIL: the true position stays out of dense smoke (zone {index} of "
                f"{sigma:g}/m entered at {inside[0][0]:.1f} s, {-min(depth for _, depth in inside):.2f} m "
                f"deep at most, {len(inside)} samples)")
            return
    print(f"OK: the true position never entered dense smoke ({len(zones)} zone(s) of "
          f"{DENSE_SMOKE_SIGMA_PER_M:g}/m and more)")


HEAT_STANDOFF_M = 4.0


def validate_standoff_from_heat_sources(truth_path: Path, overrides: dict,
                                        errors: list[str]) -> None:
    """Roadmap item 32 (specification K49): the vehicle keeps the standoff from
    every heat source, judged in truth. The sources are the run's own
    (`HEAT_SOURCES` in the manifest, x,y,z,r,T), the standoff 4.0 m from the
    source's centre; the first true-pose sample within it fails the flight
    with the time and the distance."""
    sources = []
    for index, item in enumerate(str(overrides.get("HEAT_SOURCES", "")).split(";")):
        if item.strip():
            values = [float(value) for value in item.split(",")]
            if len(values) == 5:
                sources.append((index, values[:3]))
    if not sources:
        print("NOTE: the flight has no heat source to keep off")
        return
    rows = true_poses(truth_path)
    if len(rows) < 2:
        errors.append(f"FAIL: the flight recorded the true pose to judge the heat "
                      f"sources by ({truth_path.name})")
        return
    nearest_m = math.inf
    for index, (x, y, z) in sources:
        distances = [(at_s, math.dist((px, py, pz), (x, y, z))) for at_s, px, py, pz in rows]
        nearest_m = min(nearest_m, min(distance for _, distance in distances))
        inside = [(at_s, distance) for at_s, distance in distances if distance < HEAT_STANDOFF_M]
        if inside:
            errors.append(
                f"FAIL: the true position keeps the standoff from heat source {index} "
                f"({HEAT_STANDOFF_M:g} m): within it at {inside[0][0]:.1f} s, "
                f"{min(distance for _, distance in inside):.2f} m at the nearest, "
                f"{len(inside)} samples)")
            return
    print(f"OK: the true position kept the standoff from {len(sources)} heat source(s) "
          f"({HEAT_STANDOFF_M:g} m; the nearest approach {nearest_m:.2f} m)")
