#!/usr/bin/env python3
"""Contracts for the Manhattan Roadmap 8 low-altitude scenarios."""

from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
WORLD_PATH = REPOSITORY / "drone_city_nav/worlds/canonical_city.world3d.json"
PARAMS_PATH = REPOSITORY / "drone_city_nav/config/urban_mvp.yaml"
POINT_TO_POINT_PATH = (
    REPOSITORY
    / "drone_city_nav/config/manhattan_low_altitude_point_to_point_scenario.json"
)
COOPERATIVE_PATH = (
    REPOSITORY
    / "drone_city_nav/config/cooperative_traffic_manhattan_low_altitude_scenario.json"
)
DEFAULT_COOPERATIVE_PATH = (
    REPOSITORY / "drone_city_nav/config/cooperative_traffic_scenario.json"
)
POINT_TO_POINT_LOADER_PATH = (
    REPOSITORY / "drone_city_nav/launch/point_to_point_scenario.py"
)
MULTI_VEHICLE_LOADER_PATH = (
    REPOSITORY / "drone_city_nav/launch/intercept_scenario.py"
)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


POINT_TO_POINT_LOADER = _load_module(
    "point_to_point_scenario", POINT_TO_POINT_LOADER_PATH
)
MULTI_VEHICLE_LOADER = _load_module(
    "multi_vehicle_scenario", MULTI_VEHICLE_LOADER_PATH
)


class ManhattanLowAltitudeScenarioContractTest(unittest.TestCase):
    def test_all_openings_include_the_shared_flight_altitude(self) -> None:
        world = json.loads(WORLD_PATH.read_text(encoding="utf-8"))
        flight_altitude_m = 5.0
        self.assertEqual(len(world["passage_structures"]), 4)
        for structure in world["passage_structures"]:
            with self.subTest(structure=structure["id"]):
                center_m = float(structure["opening_center_z_m"])
                half_height_m = 0.5 * float(structure["height_m"])
                self.assertLessEqual(abs(center_m - flight_altitude_m), 5.0)
                self.assertLess(center_m - half_height_m, flight_altitude_m)
                self.assertGreater(center_m + half_height_m, flight_altitude_m)

    def test_point_to_point_profile_stays_at_opening_altitude(self) -> None:
        scenario = POINT_TO_POINT_LOADER.load_point_to_point_scenario(
            POINT_TO_POINT_PATH, lidar_profile="3d"
        )
        self.assertEqual(scenario["initial_altitude_m"], 5.0)
        self.assertEqual(scenario["mission_goal_sequence_m"], ((54.0, 378.0, 5.0),))
        self.assertEqual(scenario["gazebo_world_name"], "generated_city")
        self.assertEqual(scenario["px4_model_target"], "gz_x500_lidar_3d")
        # The Manhattan map_to_sdf already swaps the axes, so PX4 NED north and
        # east fall on map +X and +Y without a further swap.
        self.assertEqual(scenario["px4_to_map_matrix"], (1.0, 0.0, 0.0, 1.0))
        self.assertTrue(scenario["gazebo_axes_swapped"])

    def test_3d_mapping_activates_below_the_shared_flight_altitude(self) -> None:
        params = yaml.safe_load(PARAMS_PATH.read_text(encoding="utf-8"))
        activation_altitude_m = params["obstacle_memory_3d_node"]["ros__parameters"][
            "min_mapping_altitude_m"
        ]
        self.assertLess(activation_altitude_m, 5.0)
        self.assertGreaterEqual(activation_altitude_m, 1.0)

    def test_3d_mapping_resolution_supports_physical_openings(self) -> None:
        params = yaml.safe_load(PARAMS_PATH.read_text(encoding="utf-8"))
        mapping = params["obstacle_memory_3d_node"]["ros__parameters"]
        base_resolution_m = float(mapping["grid_resolution_m"])

        self.assertEqual(base_resolution_m, 0.25)
        self.assertLess(base_resolution_m, 0.5)

    def test_cooperative_profile_stays_at_opening_altitude(self) -> None:
        scenario = MULTI_VEHICLE_LOADER.load_multi_vehicle_scenario(
            COOPERATIVE_PATH, lidar_profile="3d"
        )
        self.assertEqual(scenario["navigation"]["initial_altitude_m"], 5.0)
        self.assertEqual(
            {goal["goal_m"][2] for goal in scenario["vehicle_goals"]}, {5.0}
        )
        self.assertTrue(
            all(vehicle["px4_model_target"] == "gz_x500_lidar_3d"
                for vehicle in scenario["vehicles"])
        )

    def test_default_cooperative_profile_is_unchanged(self) -> None:
        scenario = MULTI_VEHICLE_LOADER.load_multi_vehicle_scenario(
            DEFAULT_COOPERATIVE_PATH
        )
        self.assertEqual(scenario["navigation"]["initial_altitude_m"], 18.0)
        self.assertEqual(
            {goal["goal_m"][2] for goal in scenario["vehicle_goals"]}, {18.0}
        )


if __name__ == "__main__":
    unittest.main()
