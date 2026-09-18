#!/usr/bin/env python3
"""Contracts for the cooperative civilian traffic scenario schema."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIO_PATH = (
    REPOSITORY
    / "drone_city_nav"
    / "config"
    / "cooperative_traffic_urban_scenario.json"
)
LOADER_PATH = (
    REPOSITORY / "drone_city_nav" / "launch" / "multi_vehicle_scenario.py"
)
VEHICLE_DESTROYED_PATH = (
    REPOSITORY / "drone_city_nav" / "msg" / "VehicleDestroyed.msg"
)

SPEC = importlib.util.spec_from_file_location("multi_vehicle_scenario", LOADER_PATH)
assert SPEC is not None and SPEC.loader is not None
SCENARIO_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO_MODULE)


def _absolute_document() -> dict:
    document = json.loads(SCENARIO_PATH.read_text(encoding="utf-8"))
    document["canonical_world"] = str(
        (SCENARIO_PATH.parent / document["canonical_world"]).resolve()
    )
    return document


class CooperativeTrafficScenarioContractTest(unittest.TestCase):
    def test_scenario_declares_only_civilian_vehicles(self) -> None:
        scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(SCENARIO_PATH)

        self.assertEqual(scenario["mission_name"], "cooperative_traffic")
        self.assertEqual(
            scenario["civilian_ids"],
            ["civilian_0", "civilian_1", "civilian_2", "civilian_3"],
        )
        self.assertTrue(
            all(vehicle["role"] == "civilian" for vehicle in scenario["vehicles"])
        )
        self.assertTrue(
            all(
                vehicle["px4_model_target"].startswith("gz_x500_lidar_3d")
                for vehicle in scenario["vehicles"]
            )
        )

    def test_cooperative_schema_rejects_non_civilian_roles(self) -> None:
        document = _absolute_document()
        document["vehicles"][0]["role"] = "civilian_escort"
        with tempfile.TemporaryDirectory() as directory:
            malformed = Path(directory) / "scenario.json"
            malformed.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "invalid role"):
                SCENARIO_MODULE.load_multi_vehicle_scenario(malformed)

    def test_cooperative_schema_accepts_distinct_valid_goal_altitudes(self) -> None:
        document = _absolute_document()
        document["vehicles"][1]["goal_m"][2] = 18.0
        with tempfile.TemporaryDirectory() as directory:
            scenario_path = Path(directory) / "scenario.json"
            scenario_path.write_text(json.dumps(document), encoding="utf-8")
            scenario = SCENARIO_MODULE.load_multi_vehicle_scenario(scenario_path)
        goals = {goal["id"]: goal["goal_m"] for goal in scenario["vehicle_goals"]}
        self.assertEqual(goals["civilian_1"][2], 18.0)

    def test_cooperative_schema_rejects_goal_outside_flight_envelope(self) -> None:
        document = _absolute_document()
        document["vehicles"][1]["goal_m"][2] = 32.0
        with tempfile.TemporaryDirectory() as directory:
            malformed = Path(directory) / "scenario.json"
            malformed.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "outside the flight envelope"):
                SCENARIO_MODULE.load_multi_vehicle_scenario(malformed)

    def test_vehicle_destruction_contract_has_typed_civilian_role(self) -> None:
        message = VEHICLE_DESTROYED_PATH.read_text(encoding="utf-8")
        self.assertIn("uint8 ROLE_CIVILIAN=1", message)


if __name__ == "__main__":
    unittest.main()
