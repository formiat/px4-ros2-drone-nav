#!/usr/bin/env python3
"""Static contracts for Stage-2 execution publication and revocation."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
EXECUTION_RUNTIME = SOURCE / "route_application"
MPPI_RUNTIME = SOURCE / "runtime"
ROS_RUNTIME = MPPI_RUNTIME / "ros"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
CONFIG = PACKAGE / "config" / "urban_mvp.yaml"
PLANNER_NODE = ROS_RUNTIME / "production_mppi_node.cpp"
PLANNER_CONFIG_HEADER = MPPI_RUNTIME / "production_mppi_config.hpp"
PLANNER_CONFIG_LOADER = ROS_RUNTIME / "production_mppi_config_ros.cpp"
EXECUTION = ROS_RUNTIME / "production_mppi_node_execution.cpp"
EXECUTION_ASSEMBLER = MPPI_RUNTIME / "execution_horizon_assembler_3d.cpp"
EXECUTION_PUBLICATION = ROS_RUNTIME / "production_mppi_node_execution_publication.cpp"
EXECUTION_HOLDS = ROS_RUNTIME / "production_mppi_node_execution_holds.cpp"
EXECUTION_RETENTION = ROS_RUNTIME / "production_mppi_node_execution_retention.cpp"
ROUTE_ACTIVATION = EXECUTION_RUNTIME / "route_activation_coordinator_3d.cpp"
ROUTE_ACTIVATION_PREPARATION = (
    EXECUTION_RUNTIME / "route_activation_preparation_3d.cpp"
)
ROUTE_EXECUTION = EXECUTION_RUNTIME / "production_mppi_route_execution.cpp"
INTERCEPT_REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
CAPTURE_GATE_HEADER = INCLUDE / "mission_waypoint_capture_gate.hpp"
CAPTURE_GATE_TEST = PACKAGE / "tests" / "mission_waypoint_capture_gate_test.cpp"


def read_execution_sources() -> str:
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            EXECUTION,
            EXECUTION_ASSEMBLER,
            EXECUTION_PUBLICATION,
            EXECUTION_HOLDS,
            EXECUTION_RETENTION,
        )
    )


class Stage2ExecutionTransportContractTest(unittest.TestCase):
    def test_route_consumers_preserve_source_ownership(self) -> None:
        observed_consumers = {
            "activation": (
                ROUTE_ACTIVATION.read_text(encoding="utf-8")
                + ROUTE_ACTIVATION_PREPARATION.read_text(encoding="utf-8")
            ),
            "execution": read_execution_sources(),
            "route_execution": ROUTE_EXECUTION.read_text(encoding="utf-8"),
        }
        for name, source in observed_consumers.items():
            with self.subTest(observed_consumer=name):
                self.assertIn("deriveRouteEvidence(", source)
                self.assertNotIn("VersionedObservedRawWorld3D::capture(", source)
                self.assertNotIn("VersionedObservedRawWorld3D::captureOwned(", source)

        static_consumers = {
            "activation": observed_consumers["activation"],
            "execution": observed_consumers["execution"],
        }
        for name, source in static_consumers.items():
            with self.subTest(static_consumer=name):
                self.assertIn("VersionedStaticWorld3D::captureOwned(", source)
                self.assertNotIn("VersionedStaticWorld3D::capture(", source)

    def test_vehicle_status_budget_covers_px4_two_hertz_cadence(self) -> None:
        planner_node = PLANNER_NODE.read_text(encoding="utf-8")
        planner_config_header = PLANNER_CONFIG_HEADER.read_text(encoding="utf-8")
        planner_config_loader = PLANNER_CONFIG_LOADER.read_text(encoding="utf-8")
        capture_header = CAPTURE_GATE_HEADER.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        capture_test = CAPTURE_GATE_TEST.read_text(encoding="utf-8")

        self.assertIn(
            'declare<double>("maximum_vehicle_status_age_ms", 1000.0)',
            planner_config_loader,
        )
        self.assertIn(
            "double maximum_vehicle_status_age_ms{1000.0};",
            planner_config_header,
        )
        self.assertEqual(
            capture_header.count("maximum_vehicle_status_age_s{1.0}"), 2
        )
        self.assertIn("maximum_vehicle_status_age_ms: 1000.0", config)
        self.assertNotIn("maximum_vehicle_status_age_ms: 200.0", config)
        self.assertIn("maximum_control_feedback_age_ms: 200.0", config)
        self.assertIn(
            "AcceptsNominalTwoHertzVehicleStatusAndRejectsAfterBudget",
            capture_test,
        )
        self.assertIn(
            "nominal.vehicle_status_receive_stamp_ns = 1'500'000'000",
            capture_test,
        )

    def test_intercept_horizon_transport_is_reliable(self) -> None:
        referee = INTERCEPT_REFEREE.read_text(encoding="utf-8")
        self.assertEqual(
            referee.count(
                "runtime.horizon_sub = "
                "create_subscription<msg::MppiTrajectoryHorizon>("
            ),
            2,
        )
        self.assertEqual(
            referee.count("topics.execution_horizon[index], control_feedback_qos"),
            2,
        )
        self.assertNotIn(
            "topics.execution_horizon[index], rclcpp::QoS{10}.best_effort()",
            referee,
        )

if __name__ == "__main__":
    unittest.main()
