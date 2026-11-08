#!/usr/bin/env python3
"""Roadmap item 11: the survey's memory persisted as a static map (tools/survey_map.py) and checked against the
truth (tools/survey_map_check.py), on a synthetic flight: a room's walls in the memory's snapshot, the same walls in
the truth, and a wall the survey never saw."""
from __future__ import annotations

import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))
sys.path.insert(0, str(REPOSITORY / "tools"))
import survey_map  # noqa: E402
import survey_map_check  # noqa: E402
from validate_static_cooperative_scenario import Occupancy3D  # noqa: E402

RES = 0.25
CHUNK = 16
# A memory lattice of two chunks along x (8 x 4 x 4 m), the volume its first 6 m; a floor at z = 0, a wall at x = 4 m
# across the volume with a door, observed from the inside: everything in x < 4 observed, nothing beyond.
SHAPE = (32, 16, 16)
ORIGIN = (0.0, 0.0, 0.0)
VOLUME = [0.0, 0.0, 0.0, 6.0, 4.0, 4.0]


def room() -> tuple[np.ndarray, np.ndarray]:
    occupied = np.zeros(SHAPE, dtype=bool)
    observed = np.zeros(SHAPE, dtype=bool)
    occupied[:, :, 0] = True                        # the floor
    occupied[16, :, :] = True                       # the wall at x = 4 m
    occupied[16, 6:10, 1:9] = False                 # its door
    observed[:17, :, :] = True                      # seen from the inside, the wall's own cells included
    return observed, occupied


def words_of(mask: np.ndarray) -> list[tuple[tuple[int, int, int], list[int]]]:
    chunks = []
    for cx in range(SHAPE[0] // CHUNK):
        for cy in range(SHAPE[1] // CHUNK):
            for cz in range(SHAPE[2] // CHUNK):
                block = mask[cx * CHUNK:(cx + 1) * CHUNK, cy * CHUNK:(cy + 1) * CHUNK, cz * CHUNK:(cz + 1) * CHUNK]
                bits = np.packbits(block.transpose(2, 1, 0).reshape(-1), bitorder="little")
                words = [int.from_bytes(bits[i:i + 8].tobytes(), "little") for i in range(0, len(bits), 8)]
                chunks.append(((cx, cy, cz), words))
    return chunks


def write_run(root: Path, run: str, observed: np.ndarray, occupied: np.ndarray, path: list[tuple[float, ...]]):
    directory = root / "log/runs" / run
    directory.mkdir(parents=True)
    chunks = words_of(observed)
    occupied_chunks = dict(words_of(occupied))
    document = {
        "schema": "drone_city_nav_raw_snapshot_3d_v1", "revision": 7,
        "header": {"frame_id": "map", "stamp": {"sec": 1, "nanosec": 0}},
        "grid": {"origin_m": list(ORIGIN), "resolution_m": RES, "shape_cells": list(SHAPE), "chunk_size_cells": CHUNK},
        "selection": {"bounds_m": VOLUME, "rule": "cell_center_inside_closed_bounds"},
        "chunks": [{"index": list(index), "observed_words_hex": [f"{w:016x}" for w in words],
                    "occupied_words_hex": [f"{w:016x}" for w in occupied_chunks[index]]}
                   for index, words in chunks if any(words)],
    }
    (directory / "raw_snapshot_3d_revision_7.json").write_text(json.dumps(document))
    (directory / "manifest.json").write_text(json.dumps({
        "effective_overrides": {"NAVIGATION_SENSOR_PROFILE": "lidar"},
        "world": {"path": "world.sdf", "sha256": "0" * 64}, "repository": {"commit": "test"}}))
    (directory / "gz_pose.csv").write_text("t,x,y,z\n" + "".join(f"{i},{x},{y},{z}\n" for i, (x, y, z) in enumerate(path)))
    return directory


def write_truth(path: Path, occupied: np.ndarray) -> None:
    grid = {"origin_m": list(ORIGIN), "resolution_m": RES, "shape_cells": list(SHAPE)}
    survey_map.write_occupancy3d(path, grid, words_of(occupied), 11)


class SurveyMapTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.repository = survey_map.REPOSITORY
        survey_map.REPOSITORY = self.root

    def tearDown(self) -> None:
        survey_map.REPOSITORY = self.repository
        self.temporary.cleanup()

    def test_the_map_holds_the_snapshots_occupied_cells_in_the_stacks_format(self) -> None:
        observed, occupied = room()
        write_run(self.root, "r1", observed, occupied, [(2.0, 2.0, 1.5), (2.0, 2.0, 1.5)])
        record = survey_map.build("r1", self.root / "map", None)
        loaded = Occupancy3D.load(self.root / "map/occupancy.occupancy3d")
        self.assertEqual(loaded.bounds.resolution_m, RES)
        self.assertEqual((loaded.bounds.width, loaded.bounds.height, loaded.bounds.depth), SHAPE)
        dense = survey_map_check.dense_in_box(loaded, np.zeros(3, dtype=int), SHAPE)
        np.testing.assert_array_equal(dense, occupied)
        self.assertEqual(record["occupancy"]["occupied_voxels"], int(occupied.sum()))
        self.assertEqual(record["observed"]["observed_voxels"], int(observed.sum()))
        with (self.root / "map/occupancy.occupancy3d").open("rb") as stream:
            self.assertEqual(stream.read(8), b"DCNOCC3D")
            self.assertEqual(struct.unpack("<II", stream.read(8)), (5, 16))
        manifest = json.loads((self.root / "map/map_manifest.json").read_text())
        self.assertEqual(manifest["volume_m"], VOLUME)
        self.assertEqual(manifest["collision_geometry"]["sha256"], "0" * 64)

    def test_the_check_passes_a_map_true_to_its_world_and_fails_one_that_misses_a_wall_by_the_path(self) -> None:
        try:
            import scipy.ndimage  # noqa: F401
        except ImportError as error:
            # The check is the host's; the dev container's NumPy 2 does not load the distribution's SciPy.
            self.skipTest(f"scipy does not load in this interpreter: {error}".splitlines()[0])
        observed, occupied = room()
        path = [(2.0, 2.0, 1.5), (2.0, 2.5, 1.5), (2.5, 2.5, 1.5)]
        write_run(self.root, "r1", observed, occupied, path)
        truth = self.root / "truth.occupancy3d"
        write_truth(truth, occupied)
        survey_map.build("r1", self.root / "map", None)
        check = survey_map_check.check(self.root / "map", truth, self.root / "log/runs/r1")
        self.assertEqual(check["verdict"], "PASS")
        self.assertEqual(check["alignment"]["median_m"], 0.0)
        self.assertEqual(check["collision"]["false_free"], 0)
        self.assertEqual(check["collision"]["false_flyable_by_the_path"], 0)
        self.assertGreater(check["coverage"], 0.99)
        # The truth holds a wall at y = 3 m the survey never saw (its cells observed free), and the path passes it.
        missed = occupied.copy()
        missed[:16, 12, 1:9] = True
        write_truth(truth, missed)
        strict = survey_map_check.check(self.root / "map", truth, self.root / "log/runs/r1")
        self.assertGreater(strict["collision"]["false_free"], 0)
        self.assertGreater(strict["collision"]["false_flyable"], 0)
        self.assertGreater(strict["collision"]["false_flyable_by_the_path"], 0)
        self.assertEqual(strict["verdict"], "FAIL")
        self.assertLess(strict["truth_reachable_cells"], check["truth_reachable_cells"])


if __name__ == "__main__":
    unittest.main()
