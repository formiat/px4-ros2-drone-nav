#!/usr/bin/env python3
"""Tests for mission-aware headless log validation."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


VALIDATOR_PATH = (
    Path(__file__).resolve().parents[1] / "validate_drone_nav_headless.py"
)
SPEC = importlib.util.spec_from_file_location("validate_drone_nav_headless", VALIDATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class SafetyRelevantRosLogTest(unittest.TestCase):
    def test_intercept_ignores_contact_after_terminal_result(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted\n"
            "PHYSICAL_COLLISION crashed=true\n"
        )
        relevant = VALIDATOR.safety_relevant_ros_log(log, "intercept")
        self.assertNotIn("crashed=true", relevant)

    def test_intercept_keeps_contact_before_terminal_result(self) -> None:
        log = (
            "PHYSICAL_COLLISION crashed=true\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted\n"
        )
        relevant = VALIDATOR.safety_relevant_ros_log(log, "intercept")
        self.assertIn("crashed=true", relevant)

    def test_point_to_point_keeps_complete_log(self) -> None:
        log = "MISSION_RESULT success=true\nPHYSICAL_COLLISION crashed=true\n"
        self.assertEqual(
            log, VALIDATOR.safety_relevant_ros_log(log, "point_to_point")
        )


class InterceptSettlementValidationTest(unittest.TestCase):
    def test_intercept_requires_two_confirmed_disarms_before_result(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true\n"
            "[vehicles.interceptor.mppi_offboard_node]: VEHICLE_TERMINATION "
            "force_disarm_confirmed=true detail='intercepted'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_TERMINATION "
            "force_disarm_confirmed=true detail='intercepted'\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_evader_goal_requires_hold_without_disarm(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=evader_reached_goal first_terminal_event=true\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "INTERCEPTOR_HOLD_CONFIRMED position_error_m=0.2 speed_mps=0.1\n"
            "MISSION_RESULT success=true mission=intercept "
            "outcome=evader_reached_goal\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_evader_goal_rejects_disarm_settlement(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=evader_reached_goal first_terminal_event=true\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "VEHICLE_TERMINATION force_disarm_confirmed=true detail='goal'\n"
            "MISSION_RESULT success=true mission=intercept "
            "outcome=evader_reached_goal\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertIn(
            "FAIL: evader goal settlement must not disarm either vehicle", errors
        )

    def test_late_capture_preserves_evader_goal_and_requires_both_disarms(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=evader_reached_goal first_terminal_event=true\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "INTERCEPT_LATE_CAPTURE outcome_preserved=evader_reached_goal\n"
            "INTERCEPTOR_HOLD_ABORTED reason=late_capture\n"
            "[vehicles.interceptor.mppi_offboard_node]: VEHICLE_TERMINATION "
            "force_disarm_confirmed=true detail='late_intercept_after_evader_goal'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_TERMINATION "
            "force_disarm_confirmed=true detail='late_intercept_after_evader_goal'\n"
            "MISSION_RESULT success=true mission=intercept "
            "outcome=evader_reached_goal\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
