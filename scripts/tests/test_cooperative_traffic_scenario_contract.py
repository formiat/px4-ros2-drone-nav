#!/usr/bin/env python3
"""Contracts for the finite cooperative civilian traffic scenario."""

from __future__ import annotations

import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIO_PATH = (
    REPOSITORY
    / "drone_city_nav"
    / "config"
    / "cooperative_traffic_scenario.json"
)
LOADER_PATH = REPOSITORY / "drone_city_nav" / "launch" / "intercept_scenario.py"
VEHICLE_DESTROYED_PATH = (
    REPOSITORY / "drone_city_nav" / "msg" / "VehicleDestroyed.msg"
)

SPEC = importlib.util.spec_from_file_location("multi_vehicle_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


class CooperativeTrafficScenarioContractTest(unittest.TestCase):
    def test_four_civilians_have_opposing_outer_edge_routes(self) -> None:
        scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(SCENARIO_PATH)

        self.assertEqual(scenario["mission_name"], "cooperative_traffic")
        self.assertEqual(
            scenario["civilian_ids"],
            ["civilian_0", "civilian_1", "civilian_2", "civilian_3"],
        )
        self.assertEqual(scenario["interceptor_ids"], [])
        self.assertEqual(scenario["evaders"], [])
        self.assertEqual(
            [vehicle["map_start_m"] for vehicle in scenario["vehicles"]],
            [
                (-4.0, 54.0, 0.3),
                (4.0, 54.0, 0.3),
                (-4.0, 378.0, 0.3),
                (4.0, 378.0, 0.3),
            ],
        )
        self.assertEqual(
            [vehicle["gazebo_spawn_m"] for vehicle in scenario["vehicles"]],
            [
                (-171.0, -139.0, 0.3),
                (-171.0, -131.0, 0.3),
                (153.0, -139.0, 0.3),
                (153.0, -131.0, 0.3),
            ],
        )
        self.assertEqual(
            [vehicle["yaw_rad"] for vehicle in scenario["vehicles"]],
            [0.0, 0.0, math.pi, math.pi],
        )
        goals = {goal["id"]: goal["goal_m"] for goal in scenario["vehicle_goals"]}
        self.assertEqual(
            goals,
            {
                "civilian_0": (-4.0, 378.0, 18.0),
                "civilian_1": (4.0, 378.0, 18.0),
                "civilian_2": (-4.0, 54.0, 18.0),
                "civilian_3": (4.0, 54.0, 18.0),
            },
        )
        for vehicle in scenario["vehicles"]:
            self.assertNotEqual(vehicle["map_start_m"][:2], goals[vehicle["id"]][:2])

    def test_routes_stay_in_a_channel_free_outer_corridor(self) -> None:
        scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(SCENARIO_PATH)
        world = json.loads(
            scenario["canonical_world_path"].read_text(encoding="utf-8")
        )

        route_x_coordinates = {
            vehicle["map_start_m"][0] for vehicle in scenario["vehicles"]
        } | {goal["goal_m"][0] for goal in scenario["vehicle_goals"]}
        first_building_west_face_m = (
            min(world["building_grid"]["x_centers_m"])
            - 0.5 * world["building_grid"]["size_m"][0]
        )
        nearest_channel_x_m = min(
            channel["intersection_center_m"][0] for channel in world["channels"]
        )

        self.assertEqual(route_x_coordinates, {-4.0, 4.0})
        self.assertLess(max(route_x_coordinates), first_building_west_face_m)
        self.assertGreaterEqual(
            nearest_channel_x_m - max(route_x_coordinates),
            50.0,
        )

        start_positions = {
            vehicle["id"]: vehicle["map_start_m"][:2]
            for vehicle in scenario["vehicles"]
        }
        goals = {goal["id"]: goal["goal_m"][:2] for goal in scenario["vehicle_goals"]}
        self.assertEqual(start_positions["civilian_0"], goals["civilian_2"])
        self.assertEqual(start_positions["civilian_1"], goals["civilian_3"])
        self.assertEqual(start_positions["civilian_2"], goals["civilian_0"])
        self.assertEqual(start_positions["civilian_3"], goals["civilian_1"])
        self.assertEqual(
            start_positions["civilian_1"][0] - start_positions["civilian_0"][0],
            8.0,
        )

    def test_cooperative_schema_rejects_mixed_roles(self) -> None:
        document = json.loads(SCENARIO_PATH.read_text(encoding="utf-8"))
        document["canonical_world"] = str(
            (SCENARIO_PATH.parent / document["canonical_world"]).resolve()
        )
        document["vehicles"][0]["role"] = "interceptor"
        with tempfile.TemporaryDirectory() as directory:
            malformed = Path(directory) / "scenario.json"
            malformed.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "only civilian"):
                SCENARIO_MODULE.load_multi_vehicle_scenario(malformed)

    def test_cooperative_schema_rejects_preassigned_altitude_layers(self) -> None:
        document = json.loads(SCENARIO_PATH.read_text(encoding="utf-8"))
        document["canonical_world"] = str(
            (SCENARIO_PATH.parent / document["canonical_world"]).resolve()
        )
        document["vehicles"][1]["goal_m"][2] = 24.0
        with tempfile.TemporaryDirectory() as directory:
            malformed = Path(directory) / "scenario.json"
            malformed.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "share start and cruise altitudes"):
                SCENARIO_MODULE.load_multi_vehicle_scenario(malformed)

    def test_vehicle_destruction_contract_has_typed_civilian_role(self) -> None:
        message = VEHICLE_DESTROYED_PATH.read_text(encoding="utf-8")
        self.assertIn("uint8 ROLE_CIVILIAN=3", message)


if __name__ == "__main__":
    unittest.main()
