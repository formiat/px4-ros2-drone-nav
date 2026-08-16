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
RUNNER_PATH = REPOSITORY / "scripts" / "run_drone_nav_sim.sh"
CONTAINER_RUNNER_PATH = REPOSITORY / "scripts" / "container_run.sh"
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
                "initial_altitude_m": 4.0,
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

    def test_four_routes_force_initial_separation_then_fan_out(self) -> None:
        scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(SCENARIO_PATH)
        starts = [vehicle["map_start_m"] for vehicle in scenario["vehicles"]]
        goals = {
            goal["id"]: goal["goal_m"] for goal in scenario["vehicle_goals"]
        }

        self.assertEqual(math.dist(starts[0][:2], starts[1][:2]), 2.0)
        self.assertEqual(math.dist(starts[2][:2], starts[3][:2]), 2.0)
        self.assertEqual(
            math.dist(goals["civilian_0"][:2], goals["civilian_1"][:2]), 8.0
        )
        self.assertEqual(
            math.dist(goals["civilian_2"][:2], goals["civilian_3"][:2]), 8.0
        )
        self.assertEqual({start[2] for start in starts}, {1.8})
        self.assertEqual({goal[2] for goal in goals.values()}, {4.0})

    def test_runtime_uses_manifest_assets_and_generic_world_overrides(self) -> None:
        world = json.loads(
            (
                REPOSITORY
                / "drone_city_nav/worlds/urban_circuit_practice_01.world3d.json"
            ).read_text(encoding="utf-8")
        )
        preparer = PREPARER_PATH.read_text(encoding="utf-8")
        runner = RUNNER_PATH.read_text(encoding="utf-8")
        container = CONTAINER_RUNNER_PATH.read_text(encoding="utf-8")
        makefile = MAKEFILE_PATH.read_text(encoding="utf-8")

        self.assertEqual(
            world["environment"],
            {"manifest_id": "urban_circuit_practice_01", "static_map_id": "r050"},
        )
        self.assertIn("environment_manifest.yaml", preparer)
        self.assertIn("SIM_WORLD_SDF_PATH", runner)
        self.assertIn("STATIC_OCCUPANCY_3D_PATH", runner)
        self.assertIn("SIM_WORLD_SDF_PATH", container)
        self.assertIn("sim-cooperative-traffic-urban-headless:", makefile)

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


if __name__ == "__main__":
    unittest.main()
