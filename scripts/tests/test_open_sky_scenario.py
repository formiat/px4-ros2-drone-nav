#!/usr/bin/env python3
"""Roadmap item 39, stage 0: the open-sky world (the autopilot's baylands park as
a fixture of the environment pipeline) flown by the point-to-point mission low
over the ground through a sequence of goals drawn by tools/open_sky_scenario.py."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import runpy
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

    def test_the_world_states_the_open_skys_profile_and_the_launch_applies_it(self) -> None:
        # Roadmap item 39 stage 5 (specification K75): the world says what kind of space it is; a navigating
        # lidar on it guarantees the open sky's obstacles (0.75 m and more) to its 35 m and keeps 17 m of the distance
        # field ahead; the targets' speed ceiling is the profile's.
        world = json.loads(WORLD_PATH.read_text(encoding="utf-8"))
        self.assertEqual(world["flight_profile"], "open_sky")
        self.assertIn('"flight_profile": "open_sky"',
                      (REPOSITORY / "tools/open_sky_scenario.py").read_text(encoding="utf-8"))
        launch = REPOSITORY / "drone_city_nav/launch"
        scenario_module = runpy.run_path(str(launch / "point_to_point_scenario.py"))
        load = scenario_module["load_point_to_point_scenario"]
        self.assertEqual(load(REPOSITORY / "drone_city_nav/config/open_sky_baylands_point_to_point_scenario.json")[
            "flight_profile"], "open_sky")
        self.assertEqual(load(REPOSITORY / "drone_city_nav/config/urban_circuit_practice_01_point_to_point_scenario.json")[
            "flight_profile"], "cave")
        profile = runpy.run_path(str(launch / "sensor_profile.py"))["OPEN_SKY_LIDAR_OBSERVABILITY"]
        self.assertAlmostEqual(profile["guaranteed_lidar_detection_range_m"], 35.0)
        self.assertAlmostEqual(profile["guaranteed_obstacle_size_m"], 0.75)
        self.assertLessEqual(profile["guaranteed_lidar_detection_range_m"],
                             profile["guaranteed_obstacle_size_m"] / math.tan(math.radians(1.0)))
        self.assertAlmostEqual(profile["latest_sensor_obstacle_maximum_age_ms"], 300.0)
        self.assertAlmostEqual(profile["no_static_3d_esdf_horizontal_recenter_margin_m"], 17.0)
        self.assertAlmostEqual(profile["persistent_planner_overflight_margin_m"], 5.0)
        # The open sky has no roof: the memory's box and the world's ceiling stand 16 m over the caves' box.
        box = runpy.run_path(str(launch / "sensor_profile.py"))["OPEN_SKY_MEMORY"]["grid_depth_m"]
        self.assertAlmostEqual(box, 56.0)
        self.assertEqual(runpy.run_path(str(launch / "sensor_profile.py"))["OPEN_SKY_MEMORY"]["integration_threads"], 4)
        self.assertAlmostEqual(
            runpy.run_path(str(launch / "sensor_profile.py"))["OPEN_SKY_MEMORY"][
                "obstacle_memory_3d_transport_rate_hz"], 5.0)
        dynamics = runpy.run_path(str(launch / "sensor_profile.py"))["OPEN_SKY_DYNAMICS"]
        self.assertAlmostEqual(dynamics["maximum_vertical_acceleration_mps2"], 1.5)
        self.assertAlmostEqual(dynamics["maximum_control_jerk_mps3"], 18.0)
        self.assertNotIn("guaranteed_vertical_stopping_deceleration_mps2", dynamics)
        self.assertAlmostEqual(world["navigation"]["maximum_target_z_m"], box - 1.0)
        self.assertIn("box_size[2] + 16.0 - 1.0", (REPOSITORY / "tools/open_sky_scenario.py").read_text(encoding="utf-8"))
        text = (launch / "city_nav.launch.py").read_text(encoding="utf-8")
        self.assertIn('if flight_profile == "open_sky":\n            production_mppi_parameters.append(OPEN_SKY_DYNAMICS)',
                      text)
        self.assertIn('shadow=navigation_sensors == "lidar" and flight_profile != "open_sky"', text)
        self.assertIn('if flight_profile == "open_sky":\n                production_mppi_parameters.append('
                      'OPEN_SKY_LIDAR_OBSERVABILITY)', text)
        self.assertIn('"guaranteed_obstacle_size_m": OPEN_SKY_LIDAR_OBSERVABILITY[', text)
        self.assertIn('if flight_profile == "open_sky":\n            obstacle_memory_parameters.append(OPEN_SKY_MEMORY)', text)
        memory = (REPOSITORY / "drone_city_nav/src/obstacle_memory_3d_node.cpp").read_text(encoding="utf-8")
        self.assertIn('declare_parameter<double>("guaranteed_obstacle_size_m", 0.0)', memory)
        self.assertIn("2.0 * placement_unit_m / std::tan(heading_uncertainty_rad)", memory)
        makefile = (REPOSITORY / "Makefile").read_text(encoding="utf-8")
        for name in ("sim-open-sky-point-to-point-headless: build", "sim-open-sky-point-to-point-gui: build"):
            target = makefile[makefile.index(name):]
            target = target[:target.index("./scripts/run_drone_nav_sim.sh")]
            self.assertIn('CRUISE_SPEED_MPS="$${CRUISE_SPEED_MPS:-15}"', target)
            self.assertIn('ABSOLUTE_SPEED_LIMIT_MPS="$${ABSOLUTE_SPEED_LIMIT_MPS:-15}"', target)
            self.assertIn('MAXIMUM_HORIZONTAL_ACCELERATION_MPS2="$${MAXIMUM_HORIZONTAL_ACCELERATION_MPS2:-5}"',
                          target)

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
