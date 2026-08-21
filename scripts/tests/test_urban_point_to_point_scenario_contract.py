#!/usr/bin/env python3
"""Contracts for the Urban Circuit no-static point-to-point scenario."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIO_PATH = (
    REPOSITORY
    / "drone_city_nav"
    / "config"
    / "urban_circuit_practice_01_point_to_point_scenario.json"
)
WORLD_PATH = REPOSITORY / "drone_city_nav/worlds/urban_circuit_practice_01.world3d.json"
LOADER_PATH = REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py"
MAKEFILE_PATH = REPOSITORY / "Makefile"
HEADLESS_WRAPPER_PATH = REPOSITORY / "scripts/sim_urban_point_to_point_headless.sh"
GUI_WRAPPER_PATH = REPOSITORY / "scripts/sim_urban_point_to_point_gui.sh"

SPEC = importlib.util.spec_from_file_location("point_to_point_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class UrbanPointToPointScenarioContractTest(unittest.TestCase):
    def test_scenario_uses_a_single_canonical_coordinate_contract(self) -> None:
        scenario = SCENARIO_MODULE.load_point_to_point_scenario(SCENARIO_PATH)

        self.assertEqual(
            scenario["canonical_world_path"], WORLD_PATH.resolve()
        )
        self.assertEqual(
            scenario["gazebo_world_name"], "urban_circuit_practice_01_collisions"
        )
        self.assertEqual(scenario["px4_model_target"], "gz_x500_lidar_3d")
        self.assertEqual(scenario["gazebo_model_name"], "x500_lidar_3d_0")
        self.assertEqual(scenario["map_start_m"], scenario["gazebo_spawn_m"])
        self.assertEqual(
            scenario["map_start_m"],
            (0.749319792, 27.246976852, 15.051717758),
        )
        self.assertEqual(
            scenario["mission_goal_sequence_m"],
            ((63.009487152, 23.856639862, 12.592997551),),
        )
        self.assertEqual(scenario["initial_altitude_m"], 15.051717758)
        self.assertEqual(len(scenario["launch_platforms"]), 1)
        final_waypoint = scenario["mission_goal_sequence_m"][-1]
        route_distance_m = (
            (final_waypoint[0] - scenario["map_start_m"][0]) ** 2
            + (final_waypoint[1] - scenario["map_start_m"][1]) ** 2
        ) ** 0.5
        self.assertGreater(route_distance_m, 50.0)
        self.assertLess(route_distance_m, 70.0)

    def test_no_static_runtime_has_3d_lidar_and_gui_entrypoints(self) -> None:
        makefile = MAKEFILE_PATH.read_text(encoding="utf-8")
        headless_wrapper = HEADLESS_WRAPPER_PATH.read_text(encoding="utf-8")
        gui_wrapper = GUI_WRAPPER_PATH.read_text(encoding="utf-8")

        self.assertIn("POINT_TO_POINT_SCENARIO_PATH", makefile)
        self.assertIn("sim-urban-point-to-point-headless:", makefile)
        self.assertIn("sim-urban-point-to-point-gui:", makefile)
        self.assertIn("SIM_COLLISION_WORLD_SDF_PATH", makefile)
        self.assertIn("SIM_GUI_WORLD_SDF_PATH", makefile)
        self.assertEqual(makefile.count("--runtime-map-mode no-static"), 4)
        self.assertEqual(makefile.count("ENABLE_STATIC_MAP=false LIDAR_PROFILE=3d"), 4)
        self.assertEqual(
            makefile.count("REQUIRE_INCREMENTAL_TOPOLOGY_EVIDENCE=true"), 2
        )
        self.assertNotIn("scripts/run_static_scenario_preflight.sh", makefile)
        self.assertIn("sim-urban-point-to-point-headless", headless_wrapper)
        self.assertIn("sim-urban-point-to-point-gui", gui_wrapper)


if __name__ == "__main__":
    unittest.main()
