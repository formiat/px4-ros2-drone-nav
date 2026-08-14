#!/usr/bin/env python3
"""Static contracts for the interceptor radar-only data boundary."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
LAUNCH = PACKAGE / "launch" / "multi_vehicle.launch.py"
MISSION_LAUNCH = PACKAGE / "launch" / "multi_vehicle_mission_launch.py"
TRACKING_LAUNCH = PACKAGE / "launch" / "intercept_tracking_launch.py"
RADAR_DETECTION = PACKAGE / "msg" / "RadarDetection.msg"
RADAR_SCAN = PACKAGE / "msg" / "RadarScan.msg"
RADAR_TRACK_MODE_COMMAND = PACKAGE / "msg" / "RadarTrackModeCommand.msg"
GUIDANCE = SOURCE / "interceptor_guidance_node.cpp"
TRACKER = SOURCE / "radar_target_tracker_node.cpp"
REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
COOPERATIVE_REFEREE = SOURCE / "cooperative_traffic_referee_node.cpp"
COOPERATIVE_AGENT = SOURCE / "cooperative_traffic_agent_node.cpp"
REFEREE_LIFECYCLE = SOURCE / "intercept_mission_referee_lifecycle.cpp"
REFEREE_SUPPORT = SOURCE / "intercept_referee_support.cpp"
GROUND_TRUTH_BOUNDARY = SOURCE / "intercept_ground_truth_boundary.cpp"
RADAR_SIMULATOR = SOURCE / "radar_simulator_node.cpp"
ASSIGNMENT_COORDINATOR = SOURCE / "target_assignment_coordinator_node.cpp"
TRUTH_ADAPTER = SOURCE / "simulation_truth_adapter_node.cpp"
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
        self.assertIn("uint64 target_track_id", command)
        self.assertNotIn("geometry_msgs", command)
        self.assertNotIn("position", command)
        self.assertNotIn("range_m", command)

    def test_only_simulation_boundary_nodes_subscribe_to_target_truth(self) -> None:
        subscribers = {
            path.name
            for path in SOURCE.glob("*.cpp")
            if "create_subscription<msg::SimulationTruthState>"
            in path.read_text(encoding="utf-8")
        }
        self.assertEqual(
            subscribers,
            {
                "cooperative_traffic_referee_node.cpp",
                "intercept_mission_referee_node.cpp",
                "radar_simulator_node.cpp",
            },
        )
        self.assertIn(
            "create_publisher<msg::SimulationTruthState>",
            TRUTH_ADAPTER.read_text(encoding="utf-8"),
        )
        cooperative_referee = COOPERATIVE_REFEREE.read_text(encoding="utf-8")
        boundary = GROUND_TRUTH_BOUNDARY.read_text(encoding="utf-8")
        self.assertIn("makeExclusiveGroundTruthBoundary", cooperative_referee)
        self.assertIn(".allowed_subscribers = {required_subscriber_fqn}", boundary)
        self.assertNotIn(
            "SimulationTruthState", COOPERATIVE_AGENT.read_text(encoding="utf-8")
        )

    def test_tracker_and_guidance_have_no_truth_bypass(self) -> None:
        for path in (TRACKER, GUIDANCE, ASSIGNMENT_COORDINATOR):
            with self.subTest(source=path.name):
                text = path.read_text(encoding="utf-8")
                self.assertNotIn("SimulationTruthState", text)
                self.assertNotIn("/vehicles/evader/state", text)
                self.assertNotIn("evader_state_topic", text)
                self.assertNotIn("target_state_topic", text)
                self.assertNotIn("use_ground_truth_target", text)

    def test_referee_cannot_publish_interceptor_navigation_objective(self) -> None:
        text = REFEREE.read_text(encoding="utf-8") + REFEREE_LIFECYCLE.read_text(
            encoding="utf-8"
        )
        self.assertNotIn("/vehicles/interceptor/navigation_objective", text)
        self.assertNotIn("publishInterceptorObjective", text)
        self.assertIn("RADAR_DATA_BOUNDARY verified=true", text)
        self.assertIn("ground_truth_boundary_violation", text)

    def test_visualization_observers_do_not_receive_physical_truth(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8") + MISSION_LAUNCH.read_text(
            encoding="utf-8"
        )
        boundary = GROUND_TRUTH_BOUNDARY.read_text(encoding="utf-8")
        self.assertIn('"target_navigation_observer_fqns"', launch)
        self.assertIn('"/intercept_spectator_node"', launch)
        self.assertIn('"/intercept_diagnostics_mux_node"', launch)
        target_contracts = boundary.split(
            "for (const TargetTruthEndpoint& endpoint", 1
        )[1].split("return std::make_unique", 1)[0]
        self.assertIn(
            "navigation_subscribers.insert(target_navigation_observer_fqns",
            target_contracts,
        )
        self.assertIn(
            ".allowed_subscribers = std::move(navigation_subscribers)",
            target_contracts,
        )
        self.assertIn(
            ".allowed_subscribers = target_truth_subscribers", target_contracts
        )

    def test_survivor_hold_avoids_obstacles_then_requires_stationary_horizon(
        self,
    ) -> None:
        objective = NAVIGATION_OBJECTIVE.read_text(encoding="utf-8")
        guidance = GUIDANCE.read_text(encoding="utf-8")
        planning = PLANNING_TICK.read_text(encoding="utf-8")
        referee = REFEREE.read_text(encoding="utf-8") + REFEREE_LIFECYCLE.read_text(
            encoding="utf-8"
        )

        self.assertIn("uint8 TERMINAL_POLICY_IMMEDIATE_HOLD=2", objective)
        self.assertIn("TERMINAL_POLICY_IMMEDIATE_HOLD", guidance)
        self.assertIn("objective->immediate_hold", planning)
        self.assertIn("kObstacleAwareHold", planning)
        self.assertIn("mission_command_obstacle_aware_hold", planning)
        execution = (
            SOURCE / "production_mppi_node_execution.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("braking || obstacle_aware_hold", execution)
        self.assertIn("!forced_braking_hold", execution)
        self.assertIn(
            "interceptor_execution_horizon_topics",
            REFEREE_SUPPORT.read_text(encoding="utf-8"),
        )
        self.assertIn("stationary_position_hold", referee)
        self.assertIn("EXECUTION_MODE_POSITION_HOLD", referee)

    def test_launch_wires_generic_radar_pipeline_without_truth_filter(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8") + MISSION_LAUNCH.read_text(
            encoding="utf-8"
        )
        tracking = TRACKING_LAUNCH.read_text(encoding="utf-8")
        text = launch + tracking
        self.assertIn('executable="intercept_mission_referee_node"', launch)
        self.assertIn('executable="radar_simulator_node"', tracking)
        self.assertIn('plugin="drone_city_nav::RadarTargetTrackerNode"', text)
        self.assertIn(
            'plugin="drone_city_nav::TargetAssignmentCoordinatorNode"', text
        )
        self.assertIn('plugin="drone_city_nav::InterceptorGuidanceNode"', text)
        self.assertIn('name="interceptor_tracking_container"', text)
        self.assertIn('{"use_intra_process_comms": True}', text)
        self.assertNotIn('executable="radar_target_tracker_node"', text)
        self.assertNotIn('executable="interceptor_guidance_node"', text)
        self.assertIn('"target_truth_state_topics"', text)
        self.assertIn('"target_detection_ids"', text)
        self.assertIn('"target_track_array_topic"', text)
        self.assertIn('"selected_target_track_topics"', text)
        self.assertIn('executable="simulation_truth_adapter_node"', (
            LAUNCH.with_name("intercept_truth_launch.py")
        ).read_text(encoding="utf-8"))
        self.assertIn('"tracked_agent_track_topic"', text)
        self.assertIn('"target_track_readiness_topics"', text)
        self.assertIn('"interceptor_destroyed_topics"', text)
        self.assertIn('"interceptor_world_readiness_topics"', text)
        self.assertIn('"target_world_readiness_topics"', text)
        self.assertIn('"track_mode_command_topic"', text)
        self.assertIn('"radar_track_mode_command_topic"', text)
        self.assertIn(
            'create_subscription<msg::RadarTrackModeCommand>',
            GUIDANCE.read_text(encoding="utf-8"),
        )
        self.assertNotIn("tracked_agent_state_topic", text)
        self.assertNotIn('executable="intercept_mission_node"', text)
        self.assertIn('scenario["interceptor_ids"]', text)
        self.assertIn('"radar_simulator_node_fqns"', text)
        self.assertIn('name="airborne_radar_simulator_node"', tracking)
        self.assertIn('"fixed_track_mode": True', tracking)
        self.assertIn('"physical_los_required": True', tracking)
        self.assertIn('"maximum_detection_range_m": settings[', tracking)
        self.assertIn('"target_detection_ids": list(', tracking)
        self.assertIn('"target_track_array_topic": f"{prefix}/avoidance_tracks"', tracking)
        self.assertIn('"avoidance_radar_simulator_node_fqns"', launch)
        self.assertIn('"avoidance_tracker_node_fqns"', launch)

    def test_attacker_tracking_pipeline_has_no_cooperative_or_truth_bypass(self) -> None:
        tracking = TRACKING_LAUNCH.read_text(encoding="utf-8")
        attacker_pipeline = tracking.split(
            "for evader_index, evader in enumerate", 1
        )[1].split("prefixes =", 1)[0]
        self.assertIn("/avoidance_radar/scan", attacker_pipeline)
        self.assertIn("/avoidance_tracks", attacker_pipeline)
        self.assertNotIn("CooperativeFlightIntent", attacker_pipeline)
        self.assertNotIn("cooperative/command", attacker_pipeline)
        self.assertNotIn("target_assignment", attacker_pipeline)
        self.assertNotIn("vehicle_role", attacker_pipeline)
        self.assertNotIn("detection_id\"]", attacker_pipeline)

    def test_truth_boundary_allows_only_sensor_simulators_and_referee(self) -> None:
        boundary = GROUND_TRUTH_BOUNDARY.read_text(encoding="utf-8")
        support = REFEREE_SUPPORT.read_text(encoding="utf-8")
        self.assertIn("avoidance_radar_simulator_fqns", boundary)
        self.assertIn("target_truth_subscribers.insert(", boundary)
        self.assertIn("avoidance_radar_simulator_node_fqns", support)

    def test_assignment_drops_destroyed_and_capturing_interceptors(self) -> None:
        coordinator = ASSIGNMENT_COORDINATOR.read_text(encoding="utf-8")
        guidance = GUIDANCE.read_text(encoding="utf-8")
        tracking_launch = TRACKING_LAUNCH.read_text(encoding="utf-8")
        assignment_contract = (
            PACKAGE / "msg" / "TargetAssignment.msg"
        ).read_text(encoding="utf-8")
        target_status = (
            PACKAGE / "msg" / "InterceptTargetStatus.msg"
        ).read_text(encoding="utf-8")
        self.assertIn("string capturing_interceptor_id", target_status)
        self.assertIn("bool active", assignment_contract)
        self.assertIn("create_subscription<msg::VehicleDestroyed>", coordinator)
        self.assertIn("deactivateInterceptor", coordinator)
        self.assertIn("!runtime.active", coordinator)
        self.assertIn("message.active = false", coordinator)
        self.assertIn("create_subscription<msg::TargetAssignment>", guidance)
        self.assertIn("clearTrackingAssignment", guidance)
        self.assertIn("publishAssignmentClearIfReady", guidance)
        self.assertIn('"target_assignment_topic":', tracking_launch)

    def test_lidar_filter_uses_only_the_derived_target_track(self) -> None:
        text = OBSTACLE_MEMORY.read_text(encoding="utf-8")
        self.assertIn('declare_parameter<std::string>("tracked_agent_track_topic"', text)
        self.assertNotIn("tracked_agent_state_topic", text)
        self.assertNotIn("VehicleNavigationState>::SharedPtr tracked_agent", text)


if __name__ == "__main__":
    unittest.main()
