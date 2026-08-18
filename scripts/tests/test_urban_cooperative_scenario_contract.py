#!/usr/bin/env python3
"""Contracts for the Urban Circuit static cooperative scenario."""

from __future__ import annotations

import importlib.util
import json
import math
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIO_PATH = (
    REPOSITORY
    / "drone_city_nav"
    / "config"
    / "cooperative_traffic_urban_scenario.json"
)
LOADER_PATH = REPOSITORY / "drone_city_nav" / "launch" / "intercept_scenario.py"
MANIFEST_PATH = REPOSITORY / "environments" / "environment_manifest.yaml"
PREPARER_PATH = REPOSITORY / "scripts" / "prepare_environment_simulation.py"
VALIDATOR_PATH = REPOSITORY / "scripts" / "validate_static_cooperative_scenario.py"
RUNNER_PATH = REPOSITORY / "scripts" / "run_drone_nav_sim.sh"
CONTAINER_RUNNER_PATH = REPOSITORY / "scripts" / "container_run.sh"
GUI_WRAPPER_PATH = REPOSITORY / "scripts" / "sim_cooperative_traffic_urban_gui.sh"
MAKEFILE_PATH = REPOSITORY / "Makefile"

SPEC = importlib.util.spec_from_file_location("multi_vehicle_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class UrbanCooperativeScenarioContractTest(unittest.TestCase):
    def test_identity_world_transform_and_altitude_profile_are_explicit(self) -> None:
        scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(SCENARIO_PATH)

        self.assertEqual(
            "urban_circuit_practice_01_collisions",
            scenario["gazebo_world_name"],
        )
        self.assertEqual(
            scenario["navigation"],
            {
                "initial_altitude_m": 15.051717758,
                "minimum_target_z_m": 1.0,
                "maximum_target_z_m": 20.0,
            },
        )
        self.assertEqual(
            scenario["px4_to_map_matrix"],
            (0.0, 1.0, 1.0, 0.0),
        )
        for vehicle in scenario["vehicles"]:
            self.assertEqual(vehicle["map_start_m"], vehicle["gazebo_spawn_m"])

    def test_four_routes_cross_between_paired_passage_regions(self) -> None:
        scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(SCENARIO_PATH)
        starts = [vehicle["map_start_m"] for vehicle in scenario["vehicles"]]
        goals = {
            goal["id"]: goal["goal_m"] for goal in scenario["vehicle_goals"]
        }

        self.assertAlmostEqual(
            math.dist(starts[0][:2], starts[1][:2]), 2.0, places=3
        )
        self.assertAlmostEqual(
            math.dist(starts[2][:2], starts[3][:2]), 2.0, places=3
        )
        self.assertAlmostEqual(
            math.dist(goals["civilian_0"][:2], goals["civilian_1"][:2]),
            2.0,
            places=3,
        )
        self.assertAlmostEqual(
            math.dist(goals["civilian_2"][:2], goals["civilian_3"][:2]),
            2.0,
            places=3,
        )
        for vehicle in scenario["vehicles"]:
            self.assertGreater(
                math.dist(vehicle["map_start_m"][:2], goals[vehicle["id"]][:2]),
                20.0,
            )
        opposite_pairs = (
            (goals["civilian_0"], starts[2]),
            (goals["civilian_1"], starts[2]),
            (goals["civilian_2"], starts[0]),
            (goals["civilian_3"], starts[0]),
        )
        for goal, opposite_start in opposite_pairs:
            self.assertLessEqual(math.dist(goal[:2], opposite_start[:2]), 20.0)
        group_a_center = tuple(
            sum(start[axis] for start in starts[:2]) / 2.0 for axis in range(2)
        )
        group_b_center = tuple(
            sum(start[axis] for start in starts[2:]) / 2.0 for axis in range(2)
        )
        self.assertEqual(group_a_center, (0.749319792, 27.246976852))
        self.assertEqual(goals["civilian_2"], starts[0])
        self.assertEqual(goals["civilian_3"], starts[1])
        self.assertGreater(math.dist(group_a_center, group_b_center), 20.0)
        self.assertGreater(math.dist(group_a_center, group_b_center), 150.0)
        self.assertEqual(
            {start[2] for start in starts}, {1.8, 15.051717758}
        )
        self.assertEqual({goal[2] for goal in goals.values()}, {15.051717758})

        source = json.loads(SCENARIO_PATH.read_text(encoding="utf-8"))
        self.assertEqual(
            source["launch_platforms"],
            [
                {
                    "id": "region_a",
                    "vehicle_ids": ["civilian_0", "civilian_1"],
                    "size_m": [6.0, 6.0, 0.5],
                    "top_z_m": 14.751717758,
                },
                {
                    "id": "region_b",
                    "vehicle_ids": ["civilian_2", "civilian_3"],
                    "size_m": [6.0, 6.0, 0.5],
                    "top_z_m": 1.5,
                },
            ],
        )

    def test_runtime_uses_manifest_assets_and_generic_world_overrides(self) -> None:
        world = json.loads(
            (
                REPOSITORY
                / "drone_city_nav/worlds/urban_circuit_practice_01.world3d.json"
            ).read_text(encoding="utf-8")
        )
        preparer = PREPARER_PATH.read_text(encoding="utf-8")
        validator = VALIDATOR_PATH.read_text(encoding="utf-8")
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        container = CONTAINER_RUNNER_PATH.read_text(encoding="utf-8")
        gui_wrapper = GUI_WRAPPER_PATH.read_text(encoding="utf-8")
        makefile = MAKEFILE_PATH.read_text(encoding="utf-8")

        self.assertEqual(
            world["environment"],
            {"manifest_id": "urban_circuit_practice_01", "static_map_id": "r050"},
        )
        self.assertIn("environment_manifest.yaml", preparer)
        self.assertIn("world_collision.sdf", preparer)
        self.assertIn("world_gui.sdf", preparer)
        self.assertIn("validate_visual_resource_uris", preparer)
        self.assertIn("add_launch_platforms", preparer)
        self.assertIn("center_is_clear", validator)
        self.assertIn("spawn_has_support", validator)
        self.assertIn("shortest_planar_route_m", validator)
        self.assertIn("planar_segment_is_clear", validator)
        self.assertIn('choices=("direct", "connected")', validator)
        self.assertIn("SIM_WORLD_SDF_PATH", runner)
        self.assertIn("STATIC_OCCUPANCY_3D_PATH", runner)
        self.assertIn("SIM_WORLD_SDF_PATH", container)
        self.assertIn("sim-cooperative-traffic-urban-gui", gui_wrapper)
        self.assertIn("sim-cooperative-traffic-urban-gui:", makefile)
        self.assertIn("sim-cooperative-traffic-urban-headless:", makefile)
        self.assertIn('SIM_WORLD_SDF_PATH="$$SIM_COLLISION_WORLD_SDF_PATH"', makefile)
        self.assertIn('SIM_WORLD_SDF_PATH="$$SIM_GUI_WORLD_SDF_PATH"', makefile)
        self.assertNotIn("STATIC_CRUISE_SPEED_MPS", makefile)
        self.assertNotIn("STATIC_ABSOLUTE_SPEED_LIMIT_MPS", makefile)
        self.assertGreaterEqual(
            makefile.count("STATIC_GLOBAL_LATTICE_DEADLINE_MS=2000"), 2
        )
        self.assertGreaterEqual(
            makefile.count("STATIC_ROUTE_TRACKING_MARGIN_M=0.25"), 2
        )
        self.assertGreaterEqual(
            makefile.count("--static-route-tracking-margin-m 0.25"), 2
        )
        self.assertGreaterEqual(makefile.count("--route-contract connected"), 2)
        self.assertGreaterEqual(makefile.count("--minimum-route-length-m 20"), 2)
        self.assertEqual(
            makefile.count("scripts/validate_static_cooperative_scenario.py"), 2
        )
        self.assertEqual(
            makefile.count("--scenario drone_city_nav/config/cooperative_traffic_urban_scenario.json"),
            4,
        )
        self.assertIn("CRUISE_SPEED_MPS", container)
        self.assertIn("ABSOLUTE_SPEED_LIMIT_MPS", container)
        self.assertIn("MAXIMUM_HORIZONTAL_ACCELERATION_MPS2", container)
        self.assertIn("STATIC_ROUTE_TRACKING_MARGIN_M", container)

        manifest = MANIFEST_PATH.read_text(encoding="utf-8")
        self.assertIn("model: Urban Platform", manifest)
        self.assertIn("version: 3", manifest)

        launch = (
            REPOSITORY / "drone_city_nav/launch/multi_vehicle.launch.py"
        ).read_text(encoding="utf-8")
        for parameter in (
            "px4_to_map_m00",
            "px4_to_map_m01",
            "px4_to_map_m10",
            "px4_to_map_m11",
        ):
            self.assertGreaterEqual(launch.count(f'"{parameter}"'), 4)
        self.assertIn('"static_route_tracking_margin_m"', launch)


if __name__ == "__main__":
    unittest.main()
