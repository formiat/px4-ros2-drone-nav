#!/usr/bin/env python3
"""Roadmap item 34 step 1: the frontiers of the memory and the coverage of the truth, scripts/survey_coverage.py
(read by the mission check and by tools/frontiers.py)."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))
import survey_coverage as FRONTIERS  # noqa: E402

RES = 0.25


def room(door: bool):
    """A room of 8 x 8 x 4 m observed whole, its walls occupied, the air outside unknown; with a door, a gap of
    1.5 m in the east wall whose cells are observed free and the air beyond unknown."""
    shape = (48, 48, 24)
    observed = np.zeros(shape, dtype=bool)
    occupied = np.zeros(shape, dtype=bool)
    observed[8:40, 8:40, 4:20] = True
    occupied[8:40, 8:40, 4:20] = True
    occupied[9:39, 9:39, 5:19] = False
    if door:
        occupied[39, 20:26, 5:13] = False
    return observed, occupied


class FrontiersTest(unittest.TestCase):
    def test_a_closed_room_holds_no_frontier(self) -> None:
        observed, occupied = room(door=False)
        fits = FRONTIERS.body_fits(occupied, RES, 0.55)
        frontier = FRONTIERS.frontier_cells(observed, occupied, fits)
        self.assertEqual(0, int(frontier.sum()))

    def test_a_room_with_a_door_holds_one_frontier_with_a_vantage_inside(self) -> None:
        observed, occupied = room(door=True)
        fits = FRONTIERS.body_fits(occupied, RES, 0.55)
        frontier = FRONTIERS.frontier_cells(observed, occupied, fits)
        # The door's cells next to unknown air beyond the wall the body fits in: not those beside the jambs,
        # the sill or the lintel (24 of the 48, as the survey node counts them).
        self.assertEqual(24, int(frontier.sum()))
        reachable = FRONTIERS.flood(fits, [(24, 24, 12)])
        self.assertTrue(reachable[24, 24, 12])
        self.assertTrue(reachable[45, 24, 12])  # unknown is free: the door lets the flood out (I1)
        clusters = FRONTIERS.clusters(frontier, reachable, 8, RES, 6.4)
        self.assertEqual(1, len(clusters))
        (cluster,) = clusters
        self.assertEqual(24, cluster["size"])
        self.assertIsNotNone(cluster["goal"])
        self.assertTrue(reachable[tuple(cluster["goal"])])
        self.assertLessEqual(cluster["vantage_m"], 1.0)

    def test_the_volume_face_is_not_a_frontier(self) -> None:
        observed = np.ones((8, 8, 8), dtype=bool)
        occupied = np.zeros((8, 8, 8), dtype=bool)
        fits = FRONTIERS.body_fits(occupied, RES, 0.55)
        self.assertEqual(0, int(FRONTIERS.frontier_cells(observed, occupied, fits).sum()))

    def test_the_snapshot_is_read_in_the_memory_bit_order(self) -> None:
        # One observed free cell at (3, 5, 2) of chunk (1, 0, 0): bit x + 16 y + 256 z of the chunk's words.
        li = 3 + 16 * 5 + 256 * 2
        words = ["0" * 16] * 64
        words[li // 64] = f"{1 << (li % 64):016x}"
        document = {"revision": 7, "grid": {"resolution_m": RES, "origin_m": [-30.0, -30.0, 0.0],
                                            "chunk_size_cells": 16, "shape_cells": [64, 64, 64]},
                    "selection": {"bounds_m": [-30.0, -30.0, 0.0, -22.0, -22.0, 8.0]},
                    "chunks": [{"index": [1, 0, 0], "observed_words_hex": words,
                                "occupied_words_hex": ["0" * 16] * 64}]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "raw_snapshot_3d_revision_7.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            revision, res, origin, observed, occupied = FRONTIERS.load_memory(path, None)
        self.assertEqual(7, revision)
        self.assertEqual(RES, res)
        self.assertEqual((32, 32, 32), observed.shape)
        self.assertEqual(1, int(observed.sum()))
        self.assertTrue(observed[16 + 3, 5, 2])
        self.assertEqual(0, int(occupied.sum()))

    def test_the_tool_is_recorded(self) -> None:
        readme = (REPOSITORY / "tools/README.md").read_text(encoding="utf-8")
        self.assertIn("| `frontiers.py` |", readme)


if __name__ == "__main__":
    unittest.main()
