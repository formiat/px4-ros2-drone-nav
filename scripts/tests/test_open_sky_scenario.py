#!/usr/bin/env python3
"""Roadmap item 39, stage 0: the open-sky world (the autopilot's baylands park as
a fixture of the environment pipeline) flown by the point-to-point mission low
over the ground through a sequence of goals drawn by tools/open_sky_scenario.py."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import unittest
from pathlib import Path

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]
LOADER_PATH = REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py"
SCENARIO_PATH = REPOSITORY / "drone_city_nav/config/open_sky_baylands_point_to_point_scenario.json"
WORLD_PATH = REPOSITORY / "drone_city_nav/worlds/open_sky_baylands.world3d.json"
FIXTURE = REPOSITORY / "environments/fixtures/open_sky_baylands"
URBAN_CONSTANTS = ("REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING", "OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M",
                   "RAW_SNAPSHOT_BOUNDS_M", "SMOKE_ZONES", "URBAN_TRUTH_OCCUPANCY_3D", "SIM_DARK_WORLD_SDF_PATH")

SPEC = importlib.util.spec_from_file_location("point_to_point_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class OpenSkyScenarioTest(unittest.TestCase):
    def test_the_mission_is_a_sequence_of_goals_low_over_the_ground(self) -> None:
        scenario = SCENARIO_MODULE.load_point_to_point_scenario(SCENARIO_PATH)
        self.assertEqual(scenario["gazebo_world_name"], "open_sky_baylands_collisions")
        self.assertEqual(scenario["canonical_world_path"], WORLD_PATH.resolve())
        transform = scenario["map_to_sdf"]
        for axis, key in enumerate(("sdf_x_offset_m", "sdf_y_offset_m", "sdf_z_offset_m")):
            self.assertAlmostEqual(scenario["gazebo_spawn_m"][axis], scenario["map_start_m"][axis] + transform[key])
        selection = json.loads(SCENARIO_PATH.read_text(encoding="utf-8"))["selection"]
        self.assertEqual(selection["tool"], "tools/open_sky_scenario.py")
        goals = scenario["mission_goal_sequence_m"]
        self.assertEqual(len(goals), 3)
        self.assertEqual(len(selection["legs_m"]), 3)
        for leg in selection["legs_m"]:
            self.assertGreaterEqual(leg, 500.0)
            self.assertLessEqual(leg, 700.0)
        self.assertGreaterEqual(selection["route_length_m"], 1500.0)
        self.assertLessEqual(selection["route_length_m"], 2100.0)
        self.assertEqual(selection["goal_over_ground_m"], 6.0)
        self.assertEqual(selection["flight_over_ground_m"], [4.0, 8.0])
        (pad,) = scenario["launch_platforms"]
        self.assertAlmostEqual(scenario["map_start_m"][2], pad["top_z_m"] + 0.23)
        points = [scenario["map_start_m"], *goals]
        for straight, (a, b) in zip(selection["straight_distances_m"], zip(points, points[1:])):
            self.assertAlmostEqual(math.dist(a, b), straight, delta=0.1)

    def test_the_fixture_is_the_manifests_and_the_targets_set_nothing_of_urban(self) -> None:
        manifest = yaml.safe_load((REPOSITORY / "environments/environment_manifest.yaml").read_text(encoding="utf-8"))
        (environment,) = [e for e in manifest["environments"] if e["id"] == "open_sky_baylands"]
        self.assertEqual(environment["distribution"], "repository")
        for record in environment["repository_files"]:
            path = REPOSITORY / record["path"]
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), record["sha256"], record["path"])
            self.assertEqual(path.stat().st_size, record["size_bytes"])
        world = (FIXTURE / "world.sdf").read_text(encoding="utf-8")
        self.assertIn("<spherical_coordinates>", world)
        self.assertNotIn("relative_to=", world)
        makefile = (REPOSITORY / "Makefile").read_text(encoding="utf-8")
        for mode in ("headless", "gui"):
            target = f"sim-open-sky-point-to-point-{mode}"
            self.assertIn(f"{target}: build", makefile)
            recipe = makefile[makefile.index(f"{target}: build"):]
            recipe = recipe[:recipe.index("\n\n")]
            self.assertIn("--environment open_sky_baylands --runtime-map-mode no-static", recipe)
            for constant in URBAN_CONSTANTS:
                self.assertNotIn(constant, recipe)
            # The sun lights the world: no flicker of the carried light (A10 is the dark location's).
            self.assertIn('LIGHT_FAULTS="$${LIGHT_FAULTS-none}"', recipe)
            self.assertNotIn("$(ORDINARY_FLIGHT)", recipe)
            wrapper = REPOSITORY / f"scripts/sim_open_sky_point_to_point_{mode}.sh"
            self.assertIn(f"make {target}\n", wrapper.read_text(encoding="utf-8"))
            lidar = REPOSITORY / f"scripts/sim_open_sky_point_to_point_lidar_{mode}.sh"
            self.assertIn(f"make sim-open-sky-point-to-point-lidar-{mode}\n", lidar.read_text(encoding="utf-8"))
            self.assertIn(f"$(LIDAR_SCENARIO) $(MAKE) --no-print-directory {target}\n", makefile)
        # The cameras fly the sky's world under its sun with its textures (the
        # gui world), the lidar the sensor world; never the dark one.
        headless = makefile[makefile.index("sim-open-sky-point-to-point-headless: build"):]
        headless = headless[:headless.index("\n\n")]
        self.assertIn("SIM_GUI_WORLD_SDF_PATH", headless)
        self.assertIn("SIM_SENSOR_WORLD_SDF_PATH", headless)
        self.assertNotIn("SIM_DARK_WORLD_SDF_PATH", headless)
        scenarios = (REPOSITORY / "docs/scenarios.md").read_text(encoding="utf-8")
        self.assertIn("sim_open_sky_point_to_point_headless.sh", scenarios)


if __name__ == "__main__":
    unittest.main()
