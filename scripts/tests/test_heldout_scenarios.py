#!/usr/bin/env python3
"""Roadmap item 33: the held-out worlds, flown by the point-to-point mission
with nothing adapted to them; the scenarios drawn from the truth grids by
tools/heldout_scenario.py, each world's map frame placed over it by its
canonical world file."""

from __future__ import annotations

import importlib.util
import json
import math
import re
import tempfile
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
LOADER_PATH = REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py"
WORLDS = {
    "cave": ("cave_circuit_practice_01", "cave_circuit_practice_01_collisions",
             "cave_practice_01_r050.occupancy3d"),
    "finals": ("finals_prize_round_world_07", "final_event_07_collisions",
               "final_event_07_r050.occupancy3d"),
}
URBAN_CONSTANTS = ("REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING", "OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M",
                   "RAW_SNAPSHOT_BOUNDS_M", "SMOKE_ZONES", "URBAN_TRUTH_OCCUPANCY_3D")

SPEC = importlib.util.spec_from_file_location("point_to_point_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


def target_recipe(makefile: str, target: str) -> str:
    match = re.search(rf"^{re.escape(target)}: build\n(.*?)\n\n", makefile, re.S | re.M)
    assert match is not None, target
    return match.group(1)


class HeldOutScenariosTest(unittest.TestCase):
    def test_the_scenarios_are_drawn_from_the_truth_grids_in_the_route_band(self) -> None:
        for short, (world_id, gazebo_world_name, grid) in WORLDS.items():
            with self.subTest(world=world_id):
                path = REPOSITORY / f"drone_city_nav/config/{world_id}_point_to_point_scenario.json"
                scenario = SCENARIO_MODULE.load_point_to_point_scenario(path)
                self.assertEqual(scenario["gazebo_world_name"], gazebo_world_name)
                self.assertEqual(scenario["canonical_world_path"],
                                 (REPOSITORY / f"drone_city_nav/worlds/{world_id}.world3d.json").resolve())
                self.assertFalse(scenario["gazebo_axes_swapped"])
                # The map frame placed over the world: the spawn is the start plus the offset.
                transform = scenario["map_to_sdf"]
                for axis, key in enumerate(("sdf_x_offset_m", "sdf_y_offset_m", "sdf_z_offset_m")):
                    self.assertAlmostEqual(scenario["gazebo_spawn_m"][axis],
                                           scenario["map_start_m"][axis] + transform[key])
                document = json.loads(path.read_text(encoding="utf-8"))
                selection = document["selection"]
                self.assertEqual(selection["tool"], "tools/heldout_scenario.py")
                self.assertTrue(selection["grid"].endswith(grid))
                self.assertIsInstance(selection["seed"], int)
                self.assertGreaterEqual(selection["route_length_m"], 300.0)
                self.assertLessEqual(selection["route_length_m"], 600.0)
                self.assertLessEqual(selection["straight_distance_m"], selection["route_length_m"])
                self.assertGreaterEqual(selection["goal_over_floor_m"], 1.5)
                self.assertLessEqual(selection["goal_over_floor_m"], 3.0)
                self.assertLessEqual(selection["goal_under_ceiling_m"], 15.0)
                # The pad 0.5 m over the floor, base_link 0.23 m over the pad: the dark hall's start.
                (pad,) = scenario["launch_platforms"]
                self.assertEqual(pad["size_m"], (3.0, 3.0, 1.0))
                self.assertAlmostEqual(scenario["map_start_m"][2], pad["top_z_m"] + 0.23)
                self.assertAlmostEqual(scenario["initial_altitude_m"], scenario["map_start_m"][2] + 2.0)
                # The flight band is the memory's box of heights (urban_mvp.yaml, 0 to 40 m).
                self.assertEqual(scenario["minimum_target_z_m"], 1.0)
                self.assertEqual(scenario["maximum_target_z_m"], 39.0)
                goal = scenario["mission_goal_sequence_m"][-1]
                self.assertAlmostEqual(math.dist(goal, scenario["map_start_m"]),
                                       selection["straight_distance_m"], delta=0.1)

    def test_the_targets_set_nothing_of_the_urban_location(self) -> None:
        makefile = (REPOSITORY / "Makefile").read_text(encoding="utf-8")
        for short, (world_id, _, grid) in WORLDS.items():
            for mode in ("headless", "gui"):
                target = f"sim-{short}-point-to-point-{mode}"
                with self.subTest(target=target):
                    recipe = target_recipe(makefile, target)
                    self.assertIn(f"--environment {world_id} --runtime-map-mode no-static", recipe)
                    self.assertIn(f"$({short.upper()}_SCENARIO_PATH)", recipe)
                    for constant in URBAN_CONSTANTS:
                        self.assertNotIn(constant, recipe)
                    wrapper = REPOSITORY / f"scripts/sim_{short}_point_to_point_{mode}.sh"
                    self.assertIn(f"make {target}\n", wrapper.read_text(encoding="utf-8"))
                    lidar = REPOSITORY / f"scripts/sim_{short}_point_to_point_lidar_{mode}.sh"
                    self.assertIn(f"make sim-{short}-point-to-point-lidar-{mode}\n",
                                  lidar.read_text(encoding="utf-8"))
                    self.assertIn(f"$(LIDAR_SCENARIO) $(MAKE) --no-print-directory {target}\n", makefile)
            self.assertIn(f"{short.upper()}_TRUTH_OCCUPANCY_3D := external/environment-candidates/work/{grid}",
                          makefile)
        scenarios = (REPOSITORY / "docs/scenarios.md").read_text(encoding="utf-8")
        self.assertIn("sim_cave_point_to_point_headless.sh", scenarios)
        self.assertIn("sim_finals_point_to_point_headless.sh", scenarios)

    def test_the_true_pose_is_recorded_in_the_map_frame(self) -> None:
        # The mission check holds the goal (map) against the true pose: a
        # world whose map frame is placed over it needs the offset taken back.
        runtime = (REPOSITORY / "scripts/runtime_evidence_runtime.sh").read_text(encoding="utf-8")
        # With "=": the offset begins with a minus sign more often than not.
        self.assertIn('--sdf-to-map-offset="${sdf_to_map_offset}"', runtime)
        self.assertIn("a[1] - b[1], a[2] - b[2], a[3] - b[3]", runtime)
        capture = (REPOSITORY / "scripts/capture_gazebo_pose.py").read_text(encoding="utf-8")
        self.assertIn("pose.position.x + offset[0]", capture)

    def test_a_flight_that_states_no_route_volume_is_judged_without_a_snapshot(self) -> None:
        # The acceptance metrics follow every single-vehicle mission check
        # (the helper), and the memory snapshot stays the route volume's.
        helper = (REPOSITORY / "scripts/multi_vehicle_sim_runtime.sh").read_text(encoding="utf-8")
        volume = helper.index('if bool_is_true "${require_observed_3d_route_volume_crossing}"; then')
        acceptance = helper.index("validation_args+=(--require-persistent-3d-acceptance)")
        self.assertLess(volume, acceptance)
        self.assertIn('if [[ -n "${mission_check}" ]] && ! bool_is_true "${multi_vehicle_mission}" &&\n'
                      '    ! bool_is_true "${active_static_map}"; then\n'
                      '    validation_args+=(--require-persistent-3d-acceptance)', helper)
        import sys
        sys.path.insert(0, str(REPOSITORY / "scripts"))
        import headless_runtime_evidence as evidence
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"
            manifest.write_text(json.dumps({
                "schema": "drone_city_nav_runtime_manifest_v1",
                "repository": {"commit": "0" * 40, "dirty": False},
                "mission": {"type": "point_to_point"},
            }), encoding="utf-8")
            errors: list[str] = []
            evidence.validate_runtime_manifest(manifest, None, True, errors)
        self.assertFalse([error for error in errors if "raw snapshot" in error], errors)

    def test_the_preparer_finds_a_bundle_whose_world_sits_under_source(self) -> None:
        preparer = (REPOSITORY / "scripts/prepare_environment_simulation.py").read_text(encoding="utf-8")
        self.assertIn('(source_root / "source" / entrypoint).is_file()', preparer)


if __name__ == "__main__":
    unittest.main()
