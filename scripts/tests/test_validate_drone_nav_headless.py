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
    def test_intercept_requires_complete_radar_data_path(self) -> None:
        log = (
            "RADAR_DATA_BOUNDARY verified=true\n"
            "RADAR_SCAN published=true sequence=2 detections=1 "
            "source=ideal_truth_adapter\n"
            "RADAR_TRACK status=tracking measurement_count=2 velocity_valid=true\n"
            "INTERCEPT_GUIDANCE source=radar_track mode=analytic_intercept\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_radar_pipeline(log, errors)
        self.assertEqual(errors, [])

    def test_intercept_rejects_ground_truth_boundary_violation(self) -> None:
        log = "ground_truth_boundary_violation:/vehicles/interceptor/guidance\n"
        errors: list[str] = []
        VALIDATOR.validate_intercept_radar_pipeline(log, errors)
        self.assertIn(
            "FAIL: interceptor data path accessed evader ground truth", errors
        )

    def test_intercept_ignores_contact_after_terminal_result(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted\n"
            "VEHICLE_DESTROYED cause=physical_collision\n"
        )
        relevant = VALIDATOR.safety_relevant_ros_log(log, "intercept")
        self.assertNotIn("cause=physical_collision", relevant)

    def test_intercept_keeps_contact_before_terminal_result(self) -> None:
        log = (
            "VEHICLE_DESTROYED cause=physical_collision\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted\n"
        )
        relevant = VALIDATOR.safety_relevant_ros_log(log, "intercept")
        self.assertIn("cause=physical_collision", relevant)

    def test_point_to_point_keeps_complete_log(self) -> None:
        log = (
            "MISSION_RESULT success=true\n"
            "VEHICLE_DESTROYED cause=physical_collision\n"
        )
        self.assertEqual(
            log, VALIDATOR.safety_relevant_ros_log(log, "point_to_point")
        )


class InterceptSettlementValidationTest(unittest.TestCase):
    def test_intercept_requires_two_confirmed_disarms_in_completed_log(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true\n"
            "[vehicles.interceptor.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_intercept_accepts_confirmation_logged_during_shutdown(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true\n"
            "[vehicles.interceptor.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
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
            "VEHICLE_DESTROYED force_disarm_sent=true "
            "cause=proximity_intercept\n"
            "MISSION_RESULT success=true mission=intercept "
            "outcome=evader_reached_goal\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertIn(
            "FAIL: unreported late proximity intercept changed goal settlement",
            errors,
        )

    def test_late_capture_preserves_evader_goal_and_requires_both_disarms(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=evader_reached_goal first_terminal_event=true\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "INTERCEPT_LATE_CAPTURE outcome_preserved=evader_reached_goal\n"
            "INTERCEPTOR_HOLD_ABORTED reason=late_capture\n"
            "[vehicles.interceptor.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='late_intercept_after_evader_goal'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='late_intercept_after_evader_goal'\n"
            "MISSION_RESULT success=true mission=intercept "
            "outcome=evader_reached_goal\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_evader_physical_destruction_requires_disarm_and_interceptor_hold(
        self,
    ) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=evader "
            "cause=physical_collision mission_epoch=1 detail='gazebo_contact'\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=physical_collision "
            "mission_epoch=1 detail='gazebo_contact'\n"
            "INTERCEPTOR_HOLD_CONFIRMED position_error_m=0.2 speed_mps=0.1\n"
            "MISSION_RESULT success=false mission=intercept outcome=system_failure "
            "reason='physical_collision_evader' mission_epoch=1 "
            "disarm_requested=false\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_interceptor_physical_destruction_requires_its_disarm(self) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=interceptor "
            "cause=physical_collision mission_epoch=1 detail='gazebo_contact'\n"
            "[vehicles.interceptor.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=physical_collision "
            "mission_epoch=1 detail='gazebo_contact'\n"
            "MISSION_RESULT success=false mission=intercept outcome=system_failure "
            "reason='physical_collision_interceptor' mission_epoch=1 "
            "disarm_requested=false\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_evader_physical_destruction_rejects_missing_hold_confirmation(self) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=evader "
            "cause=physical_collision\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=physical_collision\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "MISSION_RESULT success=false mission=intercept outcome=system_failure "
            "reason='physical_collision_evader' mission_epoch=1 "
            "disarm_requested=false\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertIn(
            "FAIL: interceptor hold is confirmed after evader destruction",
            errors,
        )


if __name__ == "__main__":
    unittest.main()
