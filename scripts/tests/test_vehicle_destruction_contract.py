#!/usr/bin/env python3
"""Static contracts for typed vehicle destruction and force-disarm ownership."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
LAUNCH = PACKAGE / "launch" / "intercept.launch.py"
CONFIG = PACKAGE / "config" / "urban_mvp.yaml"
MESSAGE = PACKAGE / "msg" / "VehicleDestroyed.msg"
REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
OFFBOARD = SOURCE / "mppi_offboard_node.cpp"
COLLISION = SOURCE / "collision_crash_node.cpp"


class VehicleDestructionContractTest(unittest.TestCase):
    def test_message_has_only_typed_physical_death_causes(self) -> None:
        text = MESSAGE.read_text(encoding="utf-8")
        self.assertIn("CAUSE_PHYSICAL_COLLISION=1", text)
        self.assertIn("CAUSE_PROXIMITY_INTERCEPT=2", text)
        self.assertIn("uint8 vehicle_role", text)
        self.assertIn("uint8 death_cause", text)
        self.assertNotIn("mission_failure", text.lower())
        self.assertNotIn("goal", text.lower())

    def test_legacy_generic_termination_contract_is_absent(self) -> None:
        self.assertFalse((PACKAGE / "msg" / "VehicleTermination.msg").exists())
        self.assertFalse((PACKAGE / "msg" / "CrashState.msg").exists())
        searchable = [PACKAGE / "CMakeLists.txt", CONFIG, LAUNCH]
        searchable.extend(SOURCE.glob("*.cpp"))
        searchable.extend(INCLUDE.glob("*.hpp"))
        for path in searchable:
            with self.subTest(path=path.name):
                text = path.read_text(encoding="utf-8")
                self.assertNotIn("VehicleTermination", text)
                self.assertNotIn("CrashState", text)
                self.assertNotIn("vehicle_termination", text)
                self.assertNotIn("crash_state", text)

    def test_only_offboard_owns_force_disarm_command(self) -> None:
        owners = {
            path.name
            for path in SOURCE.glob("*.cpp")
            if "kPx4ForceDisarmMagicParam2" in path.read_text(encoding="utf-8")
        }
        self.assertEqual(owners, {"mppi_offboard_node.cpp"})
        offboard = OFFBOARD.read_text(encoding="utf-8")
        self.assertIn("destruction_disarm.force_disarm_requested", offboard)
        self.assertIn("validVehicleDeathCause", offboard)
        self.assertIn("expected_vehicle_role_", offboard)

    def test_producers_assign_distinct_structured_causes(self) -> None:
        referee = REFEREE.read_text(encoding="utf-8")
        collision = COLLISION.read_text(encoding="utf-8")
        self.assertIn("CAUSE_PROXIMITY_INTERCEPT", referee)
        self.assertIn("CAUSE_PHYSICAL_COLLISION", collision)
        self.assertNotIn("VehicleCommand", referee)

    def test_intercept_launch_wires_role_and_epoch_per_vehicle(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"vehicle_destroyed_topic": f"{prefix}/vehicle_destroyed"', text)
        self.assertGreaterEqual(text.count('"vehicle_role": 1 if primary else 2'), 2)
        self.assertGreaterEqual(text.count('"mission_epoch": 1'), 2)


if __name__ == "__main__":
    unittest.main()
