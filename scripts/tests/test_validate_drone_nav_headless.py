#!/usr/bin/env python3
"""Tests for mission-aware headless log validation."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


VALIDATOR_PATH = (
    Path(__file__).resolve().parents[1] / "validate_drone_nav_headless.py"
)
SPEC = importlib.util.spec_from_file_location("validate_drone_nav_headless", VALIDATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class ValidatorCliTest(unittest.TestCase):
    def test_main_runs_with_current_required_arguments(self) -> None:
        stderr = io.StringIO()
        argv = [
            str(VALIDATOR_PATH),
            "--ros-log",
            "ros.log",
            "--px4-log",
            "px4.log",
        ]

        with (
            mock.patch.object(sys, "argv", argv),
            mock.patch.object(VALIDATOR, "read_text", return_value=""),
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(stderr),
        ):
            result = VALIDATOR.main()

        self.assertEqual(result, 1)
        self.assertIn("FAIL: production MPPI is ready", stderr.getvalue())

    def test_a_measurement_outside_its_reference_is_a_note_and_never_a_failure(
        self,
    ) -> None:
        # The project's requirements are no crash, the mission completed and the
        # mean speed. Everything else the check measures is reported.
        stdout = io.StringIO()
        stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"
            manifest.write_text("{}", encoding="utf-8")
            argv = [
                str(VALIDATOR_PATH), "--ros-log", "ros.log", "--px4-log", "px4.log",
                "--runtime-manifest", str(manifest), "--expected-static", "false",
                "--lidar-profile", "3d", "--mission-check",
                "--require-observed-3d-route-volume-crossing",
                "--observed-3d-route-volume-bounds-m", "4,20,9,16,32,18",
                "--require-persistent-3d-acceptance",
            ]
            with (
                mock.patch.object(sys, "argv", argv),
                mock.patch.object(VALIDATOR, "read_text", return_value=""),
                contextlib.redirect_stdout(stdout),
                contextlib.redirect_stderr(stderr),
            ):
                VALIDATOR.main()

        notes = [line for line in stdout.getvalue().splitlines()
                 if line.startswith("NOTE:")]
        failures = stderr.getvalue()
        for measurement in ("persistent 3D acceptance", "sensor evidence age",
                            "setpoints and local position", "route validation"):
            with self.subTest(measurement=measurement):
                self.assertTrue(any(measurement in note for note in notes), notes)
                self.assertNotIn(measurement, failures)
        for requirement in ("mean flight speed", "mission monitor verifies"):
            with self.subTest(requirement=requirement):
                self.assertIn(requirement, failures)


class MappingPipelineValidationTest(unittest.TestCase):

    def test_execution_chain_rejects_safe_unpublished_ticks(self) -> None:
        errors: list[str] = []
        VALIDATOR.validate_execution_chain(
            "PRODUCTION_MPPI_TICK execution_mode=position_hold execution_reason=none "
            "execution_published=false finite_path_validation_status=valid "
            "raw_invalidation_active=false "
            "target_source=persistent_route_3d route_reaches_mission_goal=false\n",
            errors,
        )
        self.assertIn("FAIL: exact planner horizon is accepted and applied", errors)

    def test_execution_chain_requires_one_exact_identity_through_offboard(self) -> None:
        errors: list[str] = []
        VALIDATOR.validate_execution_chain(
            "PRODUCTION_MPPI_TICK execution_mode=planned execution_reason=none "
            "execution_published=true finite_path_validation_status=valid "
            "route_reaches_mission_goal=true target_source=persistent_route_3d "
            "raw_invalidation_active=false "
            "liveness_actual_route_progress_m=0.25 liveness_route_progress_used=true\n"
            "EXECUTION_HORIZON published=true producer=7 sequence=9 mode=planned\n"
            "EXECUTION_HORIZON accepted=true producer=7 sequence=9 mode=planned\n"
            "OFFBOARD_PLANNED_HORIZON_APPLIED producer=7 sequence=9\n",
            errors,
        )
        self.assertEqual(errors, [])
    def test_3d_pipeline_accepts_hit_miss_memory_and_selected_debug_clouds(
        self,
    ) -> None:
        log = (
            "LIDAR3D_CURRENT_SCAN accepted=true stamp_ns=123 sequence=8 "
            "source=4080 hits=1500 invalid=80 debug=true\n"
            "LIDAR3D_MEMORY accepted=true stamp_ns=123 source=4080 processed=4000 "
            "hits=1500 misses=2500 revision=8 debug=true\n"
            "ONLINE_OCCUPANCY3D_UPDATE revision=8 known=12000 "
            "snapshot=true delta=false debug_cloud=true\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_mapping_pipeline(log, "3d", True, True, errors)

        self.assertEqual(errors, [])

    def test_3d_pipeline_rejects_a_hit_only_log(self) -> None:
        errors: list[str] = []

        VALIDATOR.validate_mapping_pipeline(
            "LIDAR3D_MEMORY accepted=true stamp_ns=123 hits=4300 revision=8\n",
            "3d",
            True,
            False,
            errors,
        )

        self.assertIn(
            "FAIL: 3D obstacle memory receives timestamped hit/miss scans",
            errors,
        )

    def test_3d_pipeline_rejects_zero_miss_scans(self) -> None:
        errors: list[str] = []

        VALIDATOR.validate_mapping_pipeline(
            "LIDAR3D_MEMORY accepted=true stamp_ns=123 hits=4300 misses=0 revision=8\n",
            "3d",
            True,
            False,
            errors,
        )

        self.assertIn(
            "FAIL: 3D obstacle memory receives timestamped hit/miss scans",
            errors,
        )

    def test_2d_pipeline_preserves_legacy_contract(self) -> None:
        log = "First lidar scan\nRaw obstacle snapshot revision=3\n"
        errors: list[str] = []

        VALIDATOR.validate_mapping_pipeline(log, "2d", True, False, errors)

        self.assertEqual(errors, [])

    def test_observed_route_volume_requires_generic_route_and_physical_crossing(
        self,
    ) -> None:
        log = (
            "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
            "certified_pending=true validation=accepted route_generation=15\n"
            "PRODUCTION_MPPI_TICK tick=1 state_position=(54.0,120.0,5.8)\n"
            "PRODUCTION_MPPI_TICK tick=2 state_position=(54.0,124.0,5.8)\n"
            "PRODUCTION_MPPI_TICK tick=3 state_position=(54.0,160.0,5.7)\n"
            "PRODUCTION_MPPI_TICK tick=4 state_position=(54.0,200.0,5.7)\n"
            "PRODUCTION_MPPI_TICK tick=5 state_position=(54.0,204.0,5.7)\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_observed_3d_route_volume(
            log,
            (42.0, 123.0, 1.5, 66.0, 201.0, 8.5),
            errors,
        )

        self.assertEqual(errors, [])

    def test_observed_route_volume_rejects_a_roof_level_flyover(self) -> None:
        log = (
            "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
            "certified_pending=true validation=accepted route_generation=8\n"
            "PRODUCTION_MPPI_TICK tick=1 state_position=(54.0,120.0,26.4)\n"
            "PRODUCTION_MPPI_TICK tick=2 state_position=(54.0,160.0,26.7)\n"
            "PRODUCTION_MPPI_TICK tick=3 state_position=(54.0,204.0,27.0)\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_observed_3d_route_volume(
            log,
            (42.0, 123.0, 1.5, 66.0, 201.0, 8.5),
            errors,
        )

        self.assertIn(
            "FAIL: vehicle physically crosses the observed 3D route volume",
            errors,
        )

    def test_observed_route_volume_does_not_join_different_vehicles(self) -> None:
        log = (
            "[vehicles.civilian_0.production_mppi_node]: PRODUCTION_MPPI_TICK "
            "state_position=(40,160,5)\n"
            "[vehicles.civilian_0.production_mppi_node]: PRODUCTION_MPPI_TICK "
            "state_position=(43,160,5)\n"
            "[vehicles.civilian_0.production_mppi_node]: PRODUCTION_MPPI_TICK "
            "state_position=(50,160,5)\n"
            "[vehicles.civilian_1.production_mppi_node]: PRODUCTION_MPPI_TICK "
            "state_position=(55,160,5)\n"
            "[vehicles.civilian_1.production_mppi_node]: PRODUCTION_MPPI_TICK "
            "state_position=(65,160,5)\n"
            "[vehicles.civilian_1.production_mppi_node]: PRODUCTION_MPPI_TICK "
            "state_position=(70,160,5)\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_observed_3d_route_volume(
            log,
            (42.0, 147.0, 1.5, 66.0, 177.0, 8.5),
            errors,
        )

        self.assertIn(
            "FAIL: vehicle physically crosses the observed 3D route volume",
            errors,
        )

    def test_observed_route_volume_tolerates_a_side_excursion(self) -> None:
        # r600: the vehicle grazed the box's y-minimum halfway through its
        # crossing along x; one pass, not two.
        log = (
            "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
            "certified_pending=true validation=accepted route_generation=3\n"
            "PRODUCTION_MPPI_TICK tick=1 state_position=(3.9,21.0,10.5)\n"
            "PRODUCTION_MPPI_TICK tick=2 state_position=(10.0,24.0,10.5)\n"
            "PRODUCTION_MPPI_TICK tick=3 state_position=(14.7,19.8,12.2)\n"
            "PRODUCTION_MPPI_TICK tick=4 state_position=(15.4,20.4,11.7)\n"
            "PRODUCTION_MPPI_TICK tick=5 state_position=(16.4,26.6,10.1)\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_observed_3d_route_volume(
            log, (4.0, 20.0, 9.0, 16.0, 32.0, 18.0), errors
        )

        self.assertEqual(errors, [])

    def test_observed_route_volume_tolerates_leaving_through_the_floor(self) -> None:
        # r629: out through the floor 1.8 m short of the far face, then on
        # past it; the passage was flown.
        log = (
            "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
            "certified_pending=true validation=accepted route_generation=3\n"
            "PRODUCTION_MPPI_TICK tick=1 state_position=(3.6,21.2,9.2)\n"
            "PRODUCTION_MPPI_TICK tick=2 state_position=(9.0,23.0,9.5)\n"
            "PRODUCTION_MPPI_TICK tick=3 state_position=(12.0,24.0,9.3)\n"
            "PRODUCTION_MPPI_TICK tick=4 state_position=(14.2,24.7,8.9)\n"
            "PRODUCTION_MPPI_TICK tick=5 state_position=(17.5,25.0,8.7)\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_observed_3d_route_volume(
            log, (4.0, 20.0, 9.0, 16.0, 32.0, 18.0), errors
        )

        self.assertEqual(errors, [])

    def test_observed_route_volume_rejects_two_visits_from_opposite_sides(
        self,
    ) -> None:
        log = (
            "PRODUCTION_MPPI_ROUTE3D planner=persistent_dstar_lite "
            "certified_pending=true validation=accepted route_generation=3\n"
            "PRODUCTION_MPPI_TICK tick=1 state_position=(3.0,25.0,10.5)\n"
            "PRODUCTION_MPPI_TICK tick=2 state_position=(6.0,25.0,10.5)\n"
            "PRODUCTION_MPPI_TICK tick=3 state_position=(3.0,25.0,10.5)\n"
            "PRODUCTION_MPPI_TICK tick=4 state_position=(17.0,25.0,10.5)\n"
            "PRODUCTION_MPPI_TICK tick=5 state_position=(14.0,25.0,10.5)\n"
            "PRODUCTION_MPPI_TICK tick=6 state_position=(17.0,25.0,10.5)\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_observed_3d_route_volume(
            log, (4.0, 20.0, 9.0, 16.0, 32.0, 18.0), errors
        )

        self.assertIn(
            "FAIL: vehicle physically crosses the observed 3D route volume",
            errors,
        )

    def test_route_volume_parser_rejects_inverted_bounds(self) -> None:
        with self.assertRaises(VALIDATOR.argparse.ArgumentTypeError):
            VALIDATOR.parse_route_volume_bounds("42,123,8.5,66,201,1.5")

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

        VALIDATOR.validate_cooperative_traffic(log, 4, False, errors, "2d")

        self.assertEqual(errors, [])

    def test_any_vehicle_destruction_is_rejected(self) -> None:
        errors: list[str] = []
        VALIDATOR.validate_cooperative_traffic(
            "COOPERATIVE_VEHICLE_DESTROYED referee_observed=true",
            4,
            False,
            errors,
            "2d",
        )
        self.assertIn("FAIL: cooperative traffic contains a physical loss", errors)

    def test_no_static_accepts_active_peer_filter_without_incidental_lidar_hit(
        self,
    ) -> None:
        errors: list[str] = []
        VALIDATOR.validate_cooperative_traffic(
            "COOPERATIVE_PEER_LIDAR_FILTER filtered_beams=0 matched_peers=0 "
            "known_peers=3",
            4,
            True,
            errors,
            "2d",
        )
        self.assertNotIn(
            "FAIL: cooperative peer memory filtering is active",
            errors,
        )

    def test_no_static_requires_peer_filter_wiring(self) -> None:
        errors: list[str] = []
        VALIDATOR.validate_cooperative_traffic(
            "COOPERATIVE_PEER_LIDAR_FILTER filtered_beams=0 matched_peers=0 "
            "known_peers=0",
            4,
            True,
            errors,
            "2d",
        )
        self.assertIn(
            "FAIL: cooperative peer memory filtering is active",
            errors,
        )

    def test_no_static_3d_accepts_active_peer_filter_without_incidental_hit(
        self,
    ) -> None:
        errors: list[str] = []
        VALIDATOR.validate_cooperative_traffic(
            "COOPERATIVE_PEER_LIDAR_FILTER3D filtered_beams=0 known_peers=3 "
            "forgotten_voxels=0",
            4,
            True,
            errors,
            "3d",
        )
        self.assertNotIn(
            "FAIL: cooperative peer memory filtering is active",
            errors,
        )


class BuildingCollisionValidationTest(unittest.TestCase):
    def test_civilian_building_collision_is_a_validation_failure(self) -> None:
        log = (
            "[vehicles.civilian_1.collision_crash_node]: VEHICLE_DESTROYED "
            "role=1 vehicle_id='civilian_1' cause=physical_collision "
            "drone_collision='x500_civilian_1::base_link::collision' "
            "obstacle_collision='building_014::link::collision'\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_building_collisions(log, errors)

        self.assertEqual(
            errors,
            [
                "FAIL: civilian_1 collided with static world obstacle "
                "'building_014::link::collision'"
            ],
        )

    def test_default_vehicle_building_collision_is_a_validation_failure(self) -> None:
        log = (
            "VEHICLE_DESTROYED role=0 vehicle_id='' cause=physical_collision "
            "drone_collision='x500_lidar_3d_0::rotor_0::rotor_0_collision' "
            "obstacle_collision='building_031::link::collision'\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_building_collisions(log, errors)

        self.assertEqual(
            errors,
            [
                "FAIL: default_vehicle collided with static world obstacle "
                "'building_031::link::collision'"
            ],
        )

    def test_passage_structure_collision_is_a_validation_failure(self) -> None:
        log = (
            "VEHICLE_DESTROYED role=0 vehicle_id='drone_0' "
            "cause=physical_collision "
            "obstacle_collision='passage_structure_54_162_straight_north_upper::"
            "link::collision'\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_building_collisions(log, errors)

        self.assertEqual(
            errors,
            [
                "FAIL: drone_0 collided with static world obstacle "
                "'passage_structure_54_162_straight_north_upper::link::collision'"
            ],
        )

    def test_proximity_collision_is_not_a_building_collision(self) -> None:
        log = (
            "VEHICLE_DESTROYED role=1 vehicle_id='civilian_0' "
            "cause=proximity_collision obstacle_collision=''\n"
        )
        errors: list[str] = []

        VALIDATOR.validate_building_collisions(log, errors)

        self.assertEqual(errors, [])

if __name__ == "__main__":
    unittest.main()
