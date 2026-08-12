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
            "SIMULATION_TRUTH_ALIGNMENT ready=true failure_confirmed=false "
            "reason=aligned\n"
            "RADAR_SCAN published=true sequence=2 detections=1 "
            "source=gazebo_physical_truth\n"
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

    def test_cooperative_result_keeps_late_physical_failures(self) -> None:
        log = (
            "MISSION_RESULT success=true mission=cooperative_traffic "
            "outcome=all_goals_reached\n"
            "VEHICLE_DESTROYED cause=physical_collision\n"
        )
        relevant = VALIDATOR.safety_relevant_ros_log(log, "cooperative_traffic")
        self.assertIn("cause=physical_collision", relevant)


class CooperativeTrafficValidationTest(unittest.TestCase):
    def test_complete_physical_settlement_is_accepted(self) -> None:
        log = (
            "COOPERATIVE_GROUND_TRUTH_BOUNDARY verified=true vehicles=4\n"
            "SIMULATION_TRUTH_ALIGNMENT ready=true failure_confirmed=false "
            "reason=aligned\n"
            + "".join(
                f"COOPERATIVE_AGENT_READY vehicle_id='civilian_{index}'\n"
                for index in range(4)
            )
            + "COOPERATIVE_TRAFFIC_MISSION state=running vehicle_count=4 "
            "mission_epoch=1 startup_coordinate_contract_latched=true "
            "all_intents_ready=true\n"
            + "".join(
                f"COOPERATIVE_GOAL_HOLD_CONFIRMED vehicle_id='civilian_{index}'\n"
                for index in range(4)
            )
            + "MISSION_RESULT success=true mission=cooperative_traffic "
            "outcome=all_goals_reached vehicle_count=4 "
            "minimum_physical_separation_m=5.400 minimum_pair='civilian_0:civilian_1' "
            "desired_separation_m=5.000 desired_separation_violation_events=0 "
            "active_desired_violations=0 physical_collisions=0 "
            "building_collisions=0 mission_epoch=1\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_cooperative_traffic(log, 4, False, errors)

        self.assertEqual(errors, [])

    def test_any_vehicle_destruction_is_rejected(self) -> None:
        errors: list[str] = []
        VALIDATOR.validate_cooperative_traffic(
            "COOPERATIVE_VEHICLE_DESTROYED referee_observed=true", 4, False, errors
        )
        self.assertIn("FAIL: cooperative traffic contains a physical loss", errors)

    def test_no_static_requires_actual_peer_memory_filtering(self) -> None:
        errors: list[str] = []
        VALIDATOR.validate_cooperative_traffic(
            "COOPERATIVE_PEER_LIDAR_FILTER filtered_beams=2 matched_peers=1 "
            "known_peers=3 latest_safety_excluded=false",
            4,
            True,
            errors,
        )
        self.assertNotIn(
            "FAIL: cooperative peers are filtered only from persistent lidar memory",
            errors,
        )


class InterceptSettlementValidationTest(unittest.TestCase):
    def test_multi_intercept_requires_complete_2v2_settlement(self) -> None:
        log = (
            "INTERCEPT_MISSION state=running mission='multi_intercept' epoch=1 "
            "interceptor_count=2 target_count=2\n"
            "TARGET_ASSIGNMENT interceptor_id='interceptor_0' detection_id=1\n"
            "TARGET_ASSIGNMENT interceptor_id='interceptor_1' detection_id=2\n"
            "INTERCEPT_TARGET_OUTCOME target_id='evader_0' detection_id=1 "
            "outcome=intercepted first_target_terminal_event=true "
            "capturing_interceptor_id='interceptor_0'\n"
            "PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true "
            "interceptor_id='interceptor_0' target_id='evader_0' "
            "measured_swept_separation_m=4.8 current_separation_m=5.1 "
            "separation_threshold_m=5.0\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept\n"
            "[vehicles.evader_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept\n"
            "INTERCEPT_TARGET_OUTCOME target_id='evader_1' detection_id=2 "
            "outcome=reached_goal first_target_terminal_event=true "
            "capturing_interceptor_id='none'\n"
            "INTERCEPTOR_HOLD_CONFIRMED vehicle_id='interceptor_1'\n"
            "MISSION_RESULT success=true mission=multi_intercept outcome=mixed "
            "intercepted_targets=1 reached_goal_targets=1 destroyed_targets=0 "
            "target_count=2 mission_epoch=1\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_multi_intercept_settlement(log, errors)

        self.assertEqual(errors, [])

    def test_interceptor_physical_losses_require_typed_disarm_settlement(self) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=interceptor "
            "vehicle_id='interceptor_1' cause=physical_collision\n"
            "[vehicles.interceptor_1.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=physical_collision\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_physical_losses(log, errors)
        self.assertEqual(errors, [])

    def test_interceptor_physical_loss_without_disarm_is_rejected(self) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=interceptor "
            "vehicle_id='interceptor_2' cause=physical_collision\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_physical_losses(log, errors)
        self.assertIn(
            "FAIL: physical loss is disarm-confirmed for interceptor_2", errors
        )

    def test_evader_physical_loss_remains_a_validation_failure(self) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=evader "
            "vehicle_id='evader' cause=physical_collision\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_physical_losses(log, errors)
        self.assertIn("FAIL: evader physical crash was reported", errors)

    def test_interceptor_building_collision_is_a_validation_failure(self) -> None:
        log = (
            "[vehicles.interceptor_1.collision_crash_node]: VEHICLE_DESTROYED "
            "role=1 vehicle_id='interceptor_1' cause=physical_collision "
            "drone_collision='x500_interceptor_1::base_link::collision' "
            "obstacle_collision='building_014::link::collision'\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_building_collisions(log, errors)

        self.assertEqual(
            errors,
            [
                "FAIL: interceptor_1 collided with building obstacle "
                "'building_014::link::collision'"
            ],
        )

    def test_evader_building_collision_is_a_validation_failure(self) -> None:
        log = (
            "VEHICLE_DESTROYED role=2 vehicle_id='evader' "
            "cause=physical_collision drone_collision='evader' "
            "obstacle_collision='building_037::link::collision'\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_building_collisions(log, errors)

        self.assertEqual(len(errors), 1)
        self.assertIn("evader collided with building", errors[0])

    def test_proximity_intercept_is_not_a_building_collision(self) -> None:
        log = (
            "VEHICLE_DESTROYED role=1 vehicle_id='interceptor_0' "
            "cause=proximity_intercept obstacle_collision=''\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_building_collisions(log, errors)

        self.assertEqual(errors, [])

    def test_intercept_requires_two_confirmed_disarms_in_completed_log(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true "
            "live_interceptors=3\n"
            "PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true "
            "interceptor_id='interceptor_0' measured_swept_separation_m=4.8 "
            "current_separation_m=5.1 separation_threshold_m=5.0 "
            "interpolation_fraction=0.8 interceptor_position=(1,2,3) "
            "evader_position=(4,5,6)\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "INTERCEPTOR_HOLD requested=true vehicle_id='interceptor_1'\n"
            "INTERCEPTOR_HOLD requested=true vehicle_id='interceptor_2'\n"
            "INTERCEPTOR_HOLD_CONFIRMED vehicle_id='interceptor_1' "
            "position_error_m=0.2 speed_mps=0.1\n"
            "INTERCEPTOR_HOLD_CONFIRMED vehicle_id='interceptor_2' "
            "position_error_m=0.2 speed_mps=0.1\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted "
            "capturing_interceptor_id='interceptor_0'\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_intercept_accepts_confirmation_logged_during_shutdown(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true "
            "live_interceptors=1\n"
            "PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true "
            "interceptor_id='interceptor_0' measured_swept_separation_m=4.9 "
            "current_separation_m=4.9 separation_threshold_m=5.0 "
            "interpolation_fraction=1.0 interceptor_position=(1,2,3) "
            "evader_position=(4,5,6)\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted "
            "capturing_interceptor_id='interceptor_0'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_intercept_accepts_survivor_proximity_collision_after_capture(
        self,
    ) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true "
            "live_interceptors=3\n"
            "PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true "
            "interceptor_id='interceptor_1' measured_swept_separation_m=4.2 "
            "current_separation_m=4.2 separation_threshold_m=5.0 "
            "interpolation_fraction=1.0 interceptor_position=(1,2,3) "
            "evader_position=(4,5,6)\n"
            "INTERCEPTOR_HOLD requested=true vehicle_id='interceptor_0'\n"
            "INTERCEPTOR_HOLD requested=true vehicle_id='interceptor_2'\n"
            "[vehicles.interceptor_1.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "VEHICLE_DESTROYED referee_observed=true role=interceptor "
            "vehicle_id='interceptor_0' cause=proximity_collision\n"
            "VEHICLE_DESTROYED referee_observed=true role=interceptor "
            "vehicle_id='interceptor_2' cause=proximity_collision\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_collision\n"
            "[vehicles.interceptor_2.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_collision\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted "
            "capturing_interceptor_id='interceptor_1'\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_intercept_settlement(log, errors)

        self.assertEqual(errors, [])

    def test_intercept_rejects_unsettled_survivor_after_capture(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true "
            "live_interceptors=2\n"
            "PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true "
            "interceptor_id='interceptor_0' measured_swept_separation_m=4.9 "
            "current_separation_m=4.9 separation_threshold_m=5.0 "
            "interpolation_fraction=1.0 interceptor_position=(1,2,3) "
            "evader_position=(4,5,6)\n"
            "INTERCEPTOR_HOLD requested=true vehicle_id='interceptor_1'\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted "
            "capturing_interceptor_id='interceptor_0'\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_intercept_settlement(log, errors)

        self.assertIn(
            "FAIL: interceptor_1 confirms post-capture hold or proximity death",
            errors,
        )

    def test_intercept_without_physical_proximity_evidence_is_rejected(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=intercepted first_terminal_event=true "
            "live_interceptors=1\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=proximity_intercept "
            "mission_epoch=1 detail='intercepted'\n"
            "MISSION_RESULT success=true mission=intercept outcome=intercepted "
            "capturing_interceptor_id='interceptor_0'\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertIn(
            "FAIL: intercept has physical Gazebo proximity evidence", errors
        )

    def test_evader_goal_requires_hold_without_disarm(self) -> None:
        log = (
            "INTERCEPT_OUTCOME outcome=evader_reached_goal first_terminal_event=true\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "INTERCEPTOR_HOLD_CONFIRMED vehicle_id='interceptor_0' "
            "position_error_m=0.2 speed_mps=0.1\n"
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
            "INTERCEPTOR_HOLD_ABORTED vehicle_id='interceptor_0' "
            "reason=late_capture\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
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
            "vehicle_id='evader' "
            "cause=physical_collision mission_epoch=1 detail='gazebo_contact'\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=physical_collision "
            "mission_epoch=1 detail='gazebo_contact'\n"
            "INTERCEPTOR_HOLD_CONFIRMED vehicle_id='interceptor_0' "
            "position_error_m=0.2 speed_mps=0.1\n"
            "MISSION_RESULT success=false mission=intercept "
            "outcome=evader_crashed mission_epoch=1\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_no_interceptors_remaining_requires_a_confirmed_disarm(self) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=interceptor "
            "vehicle_id='interceptor_0' "
            "cause=physical_collision mission_epoch=1 detail='gazebo_contact'\n"
            "[vehicles.interceptor_0.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=interceptor cause=physical_collision "
            "mission_epoch=1 detail='gazebo_contact'\n"
            "MISSION_RESULT success=false mission=intercept "
            "outcome=no_interceptors_remaining mission_epoch=1\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertEqual(errors, [])

    def test_evader_physical_destruction_rejects_missing_hold_confirmation(self) -> None:
        log = (
            "VEHICLE_DESTROYED referee_observed=true role=evader "
            "vehicle_id='evader' "
            "cause=physical_collision\n"
            "[vehicles.evader.mppi_offboard_node]: VEHICLE_DESTROYED "
            "disarm_confirmed=true role=evader cause=physical_collision\n"
            "INTERCEPTOR_HOLD requested=true\n"
            "MISSION_RESULT success=false mission=intercept "
            "outcome=evader_crashed mission_epoch=1\n"
        )
        errors: list[str] = []
        VALIDATOR.validate_intercept_settlement(log, errors)
        self.assertIn(
            "FAIL: interceptor hold is confirmed after evader destruction",
            errors,
        )


if __name__ == "__main__":
    unittest.main()
