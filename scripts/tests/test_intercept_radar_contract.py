#!/usr/bin/env python3
"""Static contracts for the interceptor radar-only data boundary."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
LAUNCH = PACKAGE / "launch" / "intercept.launch.py"
RADAR_DETECTION = PACKAGE / "msg" / "RadarDetection.msg"
RADAR_SCAN = PACKAGE / "msg" / "RadarScan.msg"
RADAR_TRACK_MODE_COMMAND = PACKAGE / "msg" / "RadarTrackModeCommand.msg"
GUIDANCE = SOURCE / "interceptor_guidance_node.cpp"
TRACKER = SOURCE / "radar_target_tracker_node.cpp"
REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
OBSTACLE_MEMORY = SOURCE / "obstacle_memory_node.cpp"
PLANNING_TICK = SOURCE / "production_mppi_node_planning_tick.cpp"
NAVIGATION_OBJECTIVE = PACKAGE / "msg" / "NavigationObjective.msg"


class InterceptRadarContractTest(unittest.TestCase):
    def test_radar_wire_contract_contains_no_absolute_target_state(self) -> None:
        detection = RADAR_DETECTION.read_text(encoding="utf-8")
        self.assertEqual(
            detection.splitlines(),
            [
                "uint64 detection_id",
                "float64 range_m",
                "float64 azimuth_rad",
                "float64 elevation_rad",
                "float64 radial_velocity_mps",
            ],
        )
        scan = RADAR_SCAN.read_text(encoding="utf-8")
        self.assertNotIn("geometry_msgs", detection + scan)
        self.assertNotIn("absolute", detection + scan)
        self.assertNotIn("truth", detection + scan)
        self.assertNotIn("entity", detection + scan)

    def test_track_mode_command_contains_only_typed_visibility_policy(self) -> None:
        command = RADAR_TRACK_MODE_COMMAND.read_text(encoding="utf-8")
        self.assertIn("uint8 MODE_SEARCH=0", command)
        self.assertIn("uint8 MODE_TRACK=1", command)
        self.assertIn("uint8 REASON_OBSERVED_TARGET_VISIBLE=2", command)
        self.assertNotIn("geometry_msgs", command)
        self.assertNotIn("position", command)
        self.assertNotIn("range_m", command)

    def test_only_simulation_boundary_nodes_subscribe_to_evader_truth(self) -> None:
        subscribers = {
            path.name
            for path in SOURCE.glob("*.cpp")
            if '"/vehicles/evader/state"' in path.read_text(encoding="utf-8")
        }
        self.assertEqual(
            subscribers,
            {"intercept_mission_referee_node.cpp", "radar_simulator_node.cpp"},
        )

    def test_tracker_and_guidance_have_no_truth_bypass(self) -> None:
        for path in (TRACKER, GUIDANCE):
            with self.subTest(source=path.name):
                text = path.read_text(encoding="utf-8")
                self.assertNotIn("/vehicles/evader/state", text)
                self.assertNotIn("evader_state_topic", text)
                self.assertNotIn("target_state_topic", text)
                self.assertNotIn("use_ground_truth_target", text)

    def test_referee_cannot_publish_interceptor_navigation_objective(self) -> None:
        text = REFEREE.read_text(encoding="utf-8")
        self.assertNotIn("/vehicles/interceptor/navigation_objective", text)
        self.assertNotIn("publishInterceptorObjective", text)
        self.assertIn("RADAR_DATA_BOUNDARY verified=true", text)
        self.assertIn("ground_truth_boundary_violation", text)

    def test_survivor_hold_brakes_then_requires_stationary_horizon(self) -> None:
        objective = NAVIGATION_OBJECTIVE.read_text(encoding="utf-8")
        guidance = GUIDANCE.read_text(encoding="utf-8")
        planning = PLANNING_TICK.read_text(encoding="utf-8")
        referee = REFEREE.read_text(encoding="utf-8")

        self.assertIn("uint8 TERMINAL_POLICY_IMMEDIATE_HOLD=2", objective)
        self.assertIn("TERMINAL_POLICY_IMMEDIATE_HOLD", guidance)
        self.assertIn("objective->immediate_hold", planning)
        self.assertIn("mission_command_braking_hold", planning)
        self.assertIn("interceptor_execution_horizon_topics", referee)
        self.assertIn("stationary_position_hold", referee)
        self.assertIn("EXECUTION_MODE_POSITION_HOLD", referee)

    def test_launch_wires_three_radar_pipelines_without_truth_filter(self) -> None:
        text = LAUNCH.read_text(encoding="utf-8")
        for executable in (
            "intercept_mission_referee_node",
            "radar_simulator_node",
            "radar_target_tracker_node",
            "interceptor_guidance_node",
        ):
            with self.subTest(executable=executable):
                self.assertIn(f'executable="{executable}"', text)
        self.assertEqual(text.count('"/vehicles/evader/state"'), 2)
        self.assertIn('"tracked_agent_track_topic"', text)
        self.assertIn('"target_track_readiness_topics"', text)
        self.assertIn('"interceptor_world_readiness_topics"', text)
        self.assertIn('"evader_world_readiness_topic"', text)
        self.assertIn('"track_mode_command_topic"', text)
        self.assertIn('"radar_track_mode_command_topic"', text)
        self.assertNotIn("tracked_agent_state_topic", text)
        self.assertNotIn('executable="intercept_mission_node"', text)
        self.assertIn('"interceptor_0"', text)
        self.assertIn('"interceptor_1"', text)
        self.assertIn('"interceptor_2"', text)
        self.assertIn('"radar_simulator_node_fqns"', text)

    def test_lidar_filter_uses_only_the_derived_target_track(self) -> None:
        text = OBSTACLE_MEMORY.read_text(encoding="utf-8")
        self.assertIn('declare_parameter<std::string>("tracked_agent_track_topic"', text)
        self.assertNotIn("tracked_agent_state_topic", text)
        self.assertNotIn("VehicleNavigationState>::SharedPtr tracked_agent", text)


if __name__ == "__main__":
    unittest.main()
