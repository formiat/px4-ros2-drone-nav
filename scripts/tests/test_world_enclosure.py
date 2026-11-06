#!/usr/bin/env python3
"""Specification K63: a scenario's start and goals are proven inside the world. The measure by rays
(scripts/world_enclosure.py), the flight check's reading of a scenario that carries the proof, the proof the
repository's scenarios carry, and the proof tool's own parts (tools/scenario_inside.py, the host's)."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))

import world_enclosure  # noqa: E402


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


WRITER = load("survey_scenario_tests", REPOSITORY / "scripts/tests/test_survey_scenario.py")
PROVEN = ("finals_prize_round_world_07_point_to_point_scenario.json",
          "finals_prize_round_world_07_survey_scenario.json",
          "cave_circuit_practice_01_point_to_point_scenario.json",
          "urban_circuit_practice_01_point_to_point_scenario.json",
          "urban_circuit_practice_01_survey_scenario.json")


def shell_world():
    """A grid of 0.5 m cells: a closed room of 20 x 20 x 6 m, and beside it two slabs, one over the other, with the
    void between them (a tunnel overhead and one underfoot: the place both held-out scenarios stood in)."""
    occupied = np.zeros((120, 80, 40), dtype=bool)
    occupied[10:52, 10:52, 10:24] = True
    occupied[11:51, 11:51, 11:23] = False
    occupied[70:110, 10:52, 12] = True
    occupied[70:110, 10:52, 22] = True
    return occupied


class RayEnclosureTest(unittest.TestCase):
    def test_a_room_meets_every_ray_and_the_void_between_two_slabs_does_not(self) -> None:
        occupied = shell_world()
        room, void = world_enclosure.ray_enclosure(occupied, [(30, 30, 16), (90, 30, 16)], 80)
        self.assertEqual(1.0, room)
        self.assertLess(void, world_enclosure.ENCLOSED_SHARE)
        # A floor under the void's point and a ceiling over it: the test that would have passed it.
        self.assertTrue(occupied[90, 30, 12] and occupied[90, 30, 22])

    def test_a_room_with_a_door_open_to_the_void_is_still_inside(self) -> None:
        # The Urban world's staging area is open: its inside is joined to the grid's faces, and the rays hold.
        occupied = shell_world()
        occupied[10, 22:38, 11:21] = False  # a gate 8 m wide and 5 m high
        (share,) = world_enclosure.ray_enclosure(occupied, [(30, 30, 16)], 80)
        self.assertGreaterEqual(share, world_enclosure.ENCLOSED_SHARE)
        self.assertLess(share, 1.0)

    def test_the_directions_cover_the_sphere(self) -> None:
        directions = world_enclosure.ray_directions()
        self.assertEqual((world_enclosure.RAYS, 3), directions.shape)
        np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1.0, atol=1e-9)
        np.testing.assert_allclose(directions.mean(axis=0), 0.0, atol=0.02)


class FlightCheckTest(unittest.TestCase):
    def scenario(self, directory: Path, start, proof: bool) -> Path:
        (directory / "world.json").write_text(json.dumps({"map_to_sdf": {
            "sdf_x_from": "map_x", "sdf_y_from": "map_y", "sdf_x_offset_m": 1.0, "sdf_y_offset_m": 2.0,
            "sdf_z_offset_m": -3.0}}))
        document = {"canonical_world": "world.json", "vehicle": {"map_start_m": list(start)},
                    "mission_goal_sequence_m": [[14.0, 13.0, 11.0]]}
        if proof:
            document["inside"] = {"tool": "tools/scenario_inside.py"}
        path = directory / "scenario.json"
        path.write_text(json.dumps(document))
        return path

    def test_a_proven_scenario_is_measured_again_and_one_in_the_void_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            truth = directory / "truth.occupancy3d"
            WRITER.write_truth(truth, shell_world(), 0.5, (0.0, 0.0, 0.0))
            # The start's cell is its map point plus the frame's offset and the takeoff climb: (30, 30, 16) in the room.
            inside = self.scenario(directory, (14.25, 13.25, 9.25), True)
            errors: list[str] = []
            with contextlib.redirect_stdout(io.StringIO()) as out:
                world_enclosure.validate_scenario_inside(inside, truth, errors)
            self.assertEqual([], errors)
            self.assertIn("OK: the scenario's start and goals lie inside the world", out.getvalue())
            # The same scenario started between the slabs.
            void = self.scenario(directory, (44.25, 13.25, 9.25), True)
            world_enclosure.validate_scenario_inside(void, truth, errors)
            self.assertEqual(1, len(errors))
            self.assertIn("FAIL: the scenario's start lies inside the world", errors[0])
            # A scenario without the proof is not judged: the open sky has no inside.
            errors.clear()
            world_enclosure.validate_scenario_inside(self.scenario(directory, (44.25, 13.25, 9.25), False), truth, errors)
            self.assertEqual([], errors)

    def test_the_flight_check_and_the_scenario_tool_call_the_proof(self) -> None:
        check = (REPOSITORY / "scripts/validate_drone_nav_headless.py").read_text(encoding="utf-8")
        self.assertIn("validate_scenario_inside(Path(__file__).resolve().parents[1] / scenario_path,", check)
        tool = (REPOSITORY / "tools/heldout_scenario.py").read_text(encoding="utf-8")
        self.assertIn("scenario_inside.prove(args.scenario, args.grid, scenario_inside.default_world_sdf(args.scenario))",
                      tool)
        self.assertIn('document["inside"] = proof', tool)


class RepositoryScenariosTest(unittest.TestCase):
    def test_the_scenarios_flown_inside_a_world_carry_their_proof(self) -> None:
        for name in PROVEN:
            with self.subTest(scenario=name):
                proof = json.loads((REPOSITORY / "drone_city_nav/config" / name).read_text(encoding="utf-8"))["inside"]
                self.assertEqual("tools/scenario_inside.py", proof["tool"])
                self.assertRegex(proof["grid_sha256"], r"^[0-9a-f]{64}$")
                self.assertEqual((world_enclosure.RAYS, world_enclosure.REACH_M, world_enclosure.ENCLOSED_SHARE),
                                 (proof["rays"], proof["reach_m"], proof["enclosed_share"]))
                self.assertGreaterEqual(len(proof["points"]), 2)
                for point in proof["points"]:
                    self.assertGreaterEqual(point["rays_meeting_a_surface"], world_enclosure.ENCLOSED_SHARE)
                    self.assertEqual({"rays": True, "tiles": True, "shells": True, "floor": True}, point["passed"])
                # The world's own parts voted for the space the points are in.
                self.assertGreater(proof["world"]["anchors_in_the_tiles_component"],
                                   proof["world"]["anchors_free"] / 2)


class ProofToolTest(unittest.TestCase):
    def setUp(self) -> None:
        try:
            self.tool = load("scenario_inside", REPOSITORY / "tools/scenario_inside.py")
        except ImportError as error:
            # The tool is the host's; the dev container's NumPy 2 does not load the distribution's SciPy.
            self.skipTest(f"scipy does not load in this interpreter: {error}".splitlines()[0])

    def test_the_worlds_parts_vote_for_the_room_and_a_stray_origin_does_not_decide(self) -> None:
        sdf = "".join(
            f'<model name="include_{name}_base_link_collision_{index}">\n <static>true</static>\n <pose>{pose} 0 0 0</pose>'
            for index, (name, pose) in enumerate((("staging_area_start", "15 15 5.5"), ("tile_1_room", "20 20 5.5"),
                                                  ("tile_2_room", "10 20 5.5"), ("tile_3_cave", "45 15 6.5"),
                                                  ("tile_1_room_link_collision_ground", "20 20 5"))))
        parts = self.tool.world_parts(sdf)
        self.assertEqual({"staging_area", "tile_1", "tile_2", "tile_3"}, set(parts))
        occupied = shell_world()
        labels, outside = self.tool.free_components(occupied)
        bounds = type("Bounds", (), {"origin_x_m": 0.0, "origin_y_m": 0.0, "origin_z_m": 0.0, "resolution_m": 0.5})()
        component, votes, free = self.tool.tiles_component(labels, bounds, parts)
        self.assertEqual((3, 4), (votes, free))
        self.assertNotEqual(outside, component)
        self.assertEqual(component, labels[30, 30, 16])
        self.assertEqual(outside, labels[90, 30, 16])
        self.assertEqual(4, self.tool.straight(occupied, (90, 30, 16), -1, 16))
        self.assertIsNone(self.tool.straight(occupied, (60, 60, 30), 1, 5))


if __name__ == "__main__":
    unittest.main()
