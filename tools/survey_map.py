#!/usr/bin/env python3
"""survey_map.py RUN --out DIR [--truth GRID]: the survey's validated obstacle memory persisted as the environment's
static-map artifact (roadmap item 11). The flight's last raw snapshot of the memory
(log/runs/RUN/raw_snapshot_3d_revision_*.json, the cells of the survey's volume on the memory's 0.25 m lattice,
observed and occupied alike) becomes

  DIR/occupancy.occupancy3d   the occupied cells in the stack's own static format (`DCNOCC3D` v5, 16-cubed chunks of
                              bits; what STATIC_OCCUPANCY_3D_PATH reads; the file carries no "unknown": a cell the
                              map does not hold occupied is free to the planner, as unknown is to the memory, I1)
  DIR/observed.occupancy3d    the observed cells in the same format: the map's evidence of what it saw, read by the
                              check and by nobody in flight
  DIR/map_manifest.json       the environment and the version of its collision geometry (the flight's world SDF and
                              its hash), the run and the commit, the map frame's transform to the world's SDF frame,
                              the resolution, the volume, the coverage by scripts/survey_coverage.py and the result of
                              tools/survey_map_check.py against the truth grid when one is named

The commit is the flight's (the run manifest's repository record), the frame the scenario's canonical world's
`map_to_sdf`. Host (SciPy for the check; the dev container's Python loads none).

  python3 tools/survey_map.py r3168 --out log/tools/wbsm/part2/maps/urban_lidar_r3168 \\
      --truth log/tools/truth25/urban_r025.occupancy3d
"""
from __future__ import annotations

import argparse
import hashlib
import json
import struct
import sys
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))
sys.path.insert(0, str(REPOSITORY / "tools"))
import survey_coverage  # noqa: E402

SCHEMA = "drone_city_nav_survey_map_v1"
MAGIC = b"DCNOCC3D"
VERSION = 5
CHUNK = 16


def last_snapshot(run_directory: Path) -> Path:
    snapshot = survey_coverage.last_snapshot(run_directory)
    if snapshot is None:
        raise FileNotFoundError(f"{run_directory}: no raw snapshot (RAW_SNAPSHOT_BOUNDS_M names the volume)")
    return snapshot


def write_occupancy3d(path: Path, grid: dict, chunks: list[tuple[tuple[int, int, int], list[int]]],
                      fingerprint: int) -> int:
    """The static format the stack reads (occupancy_grid_3d.cpp): the header, then the chunks sorted by index, each
    its index and 64 words of bits, bit (z * 16 + y) * 16 + x of the chunk, the snapshot's own order. Returns the
    occupied voxels."""
    origin = grid["origin_m"]
    shape = grid["shape_cells"]
    kept = [(index, words) for index, words in chunks if any(words)]
    kept.sort(key=lambda item: item[0])
    voxels = 0
    with path.open("wb") as stream:
        stream.write(MAGIC)
        stream.write(struct.pack("<II", VERSION, CHUNK))
        stream.write(struct.pack("<ffff", float(grid["resolution_m"]), *(float(v) for v in origin)))
        stream.write(struct.pack("<III", *(int(v) for v in shape)))
        stream.write(struct.pack("<QI", fingerprint & 0xFFFFFFFFFFFFFFFF, len(kept)))
        for (x, y, z), words in kept:
            stream.write(struct.pack("<iii", x, y, z))
            for word in words:
                stream.write(struct.pack("<Q", word))
                voxels += bin(word).count("1")
    return voxels


def build(run: str, out: Path, truth: Path | None) -> dict:
    run_directory = REPOSITORY / "log/runs" / run
    manifest = json.loads((run_directory / "manifest.json").read_text())
    overrides = manifest.get("effective_overrides", {})
    scenario_path = overrides.get("POINT_TO_POINT_SCENARIO_PATH", "")
    snapshot_path = last_snapshot(run_directory)
    document = json.loads(snapshot_path.read_text())
    grid = document["grid"]
    volume = document["selection"]["bounds_m"]
    if grid["chunk_size_cells"] != CHUNK:
        raise ValueError(f"{snapshot_path}: chunk size {grid['chunk_size_cells']}, the static format holds {CHUNK}")
    fingerprint = int.from_bytes(hashlib.sha256(snapshot_path.read_bytes()).digest()[:8], "little") or 1
    out.mkdir(parents=True, exist_ok=True)
    occupied = write_occupancy3d(
        out / "occupancy.occupancy3d", grid,
        [(tuple(chunk["index"]), [int(w, 16) for w in chunk["occupied_words_hex"]]) for chunk in document["chunks"]],
        fingerprint)
    observed = write_occupancy3d(
        out / "observed.occupancy3d", grid,
        [(tuple(chunk["index"]), [int(w, 16) for w in chunk["observed_words_hex"]]) for chunk in document["chunks"]],
        fingerprint)
    frame = {"frame_id": document["header"]["frame_id"], "map_to_sdf_offset_m": [0.0, 0.0, 0.0],
             "band_m": None, "environment": None}
    if scenario_path:
        band, offset = survey_coverage.scenario_frame(REPOSITORY / scenario_path)
        scenario = json.loads((REPOSITORY / scenario_path).read_text())
        world = json.loads((REPOSITORY / scenario_path).parent.joinpath(scenario["canonical_world"]).read_text())
        frame = {"frame_id": document["header"]["frame_id"], "map_to_sdf_offset_m": list(offset),
                 "band_m": list(band), "environment": world.get("environment", {}).get("manifest_id")}
    try:
        snapshot_record = str(snapshot_path.relative_to(REPOSITORY))
    except ValueError:
        snapshot_record = str(snapshot_path)
    record = {
        "schema": SCHEMA,
        "environment": frame["environment"],
        "collision_geometry": manifest.get("world"),
        "run": run,
        "repository": manifest.get("repository"),
        "mission": manifest.get("mission"),
        "snapshot": {"path": snapshot_record, "revision": document["revision"],
                     "sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest()},
        "frame": frame,
        "resolution_m": grid["resolution_m"],
        "grid": grid,
        "volume_m": volume,
        "occupancy": {"path": "occupancy.occupancy3d", "occupied_voxels": occupied, "fingerprint": fingerprint},
        "observed": {"path": "observed.occupancy3d", "observed_voxels": observed},
        "coverage": None,
        "check": None,
    }
    if truth is not None:
        lidar = overrides.get("NAVIGATION_SENSOR_PROFILE", "") == "lidar" or overrides.get("CAMERA_PROFILE") == "none"
        reach = 35.0 if lidar else 6.4
        figures = survey_coverage.survey_figures(run_directory, truth, [float(v) for v in volume], reach,
                                                 band=tuple(frame["band_m"]) if frame["band_m"] else None,
                                                 truth_offset=tuple(frame["map_to_sdf_offset_m"]))
        figures.pop("observed_arrays", None)
        record["coverage"] = {"sensor_reach_m": reach, **figures}
    # The manifest first: the check reads the map's volume and frame from it, and its result goes into it.
    (out / "map_manifest.json").write_text(json.dumps(record, indent=2, default=str) + "\n")
    if truth is not None:
        import survey_map_check  # noqa: E402  (beside this file)
        record["check"] = survey_map_check.check(out, truth, run_directory)
        (out / "map_manifest.json").write_text(json.dumps(record, indent=2, default=str) + "\n")
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--truth", type=Path)
    args = parser.parse_args()
    record = build(args.run, args.out, args.truth)
    coverage = record["coverage"] or {}
    check = record["check"] or {}
    print(f"{args.run}: {record['occupancy']['occupied_voxels']} occupied voxels, "
          f"{record['observed']['observed_voxels']} observed, volume {record['volume_m']}, "
          f"coverage {coverage.get('volume_coverage')}, check {check.get('verdict')} -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
