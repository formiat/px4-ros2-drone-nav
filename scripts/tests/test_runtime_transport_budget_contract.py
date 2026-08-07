#!/usr/bin/env python3
"""Static contracts for bounded memory and planner diagnostic transport."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"


class RuntimeTransportBudgetContractTest(unittest.TestCase):
    def test_planner_consumes_lightweight_memory_status(self) -> None:
        message = (PACKAGE / "msg" / "ObstacleMemoryStatus.msg").read_text(
            encoding="utf-8"
        )
        planner = (PACKAGE / "src" / "production_mppi_node.cpp").read_text(
            encoding="utf-8"
        )
        header = (PACKAGE / "src" / "production_mppi_node.hpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("uint64 sequence", message)
        self.assertIn("uint64 occupied_cell_count", message)
        self.assertIn("create_subscription<msg::ObstacleMemoryStatus>", planner)
        self.assertNotIn("create_subscription<msg::ObstacleMemorySnapshot>", planner)
        self.assertIn("memory_status_sub_", header)
        self.assertNotIn("memory_snapshot_sub_", header)

    def test_full_memory_snapshot_is_debug_cadence_only(self) -> None:
        source = (PACKAGE / "src" / "obstacle_memory_transport.cpp").read_text(
            encoding="utf-8"
        )
        debug_block = source.split("if (publish_debug) {", maxsplit=1)[1].split(
            "msg::ObstacleMemoryStatus status", maxsplit=1
        )[0]

        self.assertIn("makeObstacleMemorySnapshotMessage", debug_block)
        self.assertIn("const bool publish_raw = !use_static_map_ || publish_debug", source)
        self.assertIn("status_pub_->publish(status)", source)

    def test_intercept_launch_wires_per_vehicle_status_topics(self) -> None:
        launch = (PACKAGE / "launch" / "intercept.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('f"{prefix}/obstacle_memory_status"', launch)
        planner_parameters = launch.split("planner_params =", maxsplit=1)[1].split(
            "offboard_params =", maxsplit=1
        )[0]
        self.assertIn('"obstacle_memory_status_topic": memory_status', planner_parameters)
        self.assertNotIn(
            '"obstacle_memory_snapshot_topic": memory_snapshot', planner_parameters
        )

    def test_json_diagnostics_are_rate_limited_and_error_buffered(self) -> None:
        planner = (PACKAGE / "src" / "production_mppi_node.cpp").read_text(
            encoding="utf-8"
        )
        diagnostics = (
            PACKAGE / "src" / "production_mppi_node_diagnostics.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn('"diagnostics_file_rate_hz"', planner)
        self.assertIn('"diagnostics_flush_period_s"', planner)
        self.assertIn("diagnostics_file_due", diagnostics)
        self.assertIn("diagnostics_error_ring_", diagnostics)
        for stage in (
            "gpu_warm_start_ms",
            "gpu_noise_generation_ms",
            "gpu_rollout_simulation_ms",
            "gpu_risk_reduction_ms",
            "gpu_weight_calculation_ms",
            "gpu_control_update_ms",
            "gpu_repair_validation_ms",
            "horizon_reconstruction_ms",
        ):
            self.assertIn(stage, diagnostics)
        self.assertIn("mppi_error_context.jsonl", planner)


if __name__ == "__main__":
    unittest.main()
