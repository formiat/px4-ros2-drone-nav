#!/usr/bin/env python3
"""Static architecture boundaries for lightweight planner transport wiring."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
PLANNER_SOURCES = (
    PACKAGE / "src" / "production_mppi_node.cpp",
    PACKAGE / "src" / "production_mppi_node_interfaces.cpp",
)


def _read_planner_sources() -> str:
    return "\n".join(path.read_text(encoding="utf-8") for path in PLANNER_SOURCES)


class RuntimeTransportBudgetContractTest(unittest.TestCase):
    def test_planner_consumes_lightweight_memory_status(self) -> None:
        message = (PACKAGE / "msg" / "ObstacleMemoryStatus.msg").read_text(
            encoding="utf-8"
        )
        planner = _read_planner_sources()
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
        self.assertIn("raw_base_revision_ = sequence_", source)
        self.assertIn("dirty_chunks_since_base_", source)
        self.assertIn("makeRawObstacleDelta", source)
        self.assertIn("status_pub_->publish(status)", source)

        planner = _read_planner_sources()
        self.assertIn("create_subscription<msg::RawObstacleDelta3D>", planner)
        self.assertNotIn("create_subscription<msg::RawObstacleDelta>", planner)

    def test_intercept_launch_wires_per_vehicle_status_topics(self) -> None:
        launch = (PACKAGE / "launch" / "multi_vehicle.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('f"{prefix}/obstacle_memory_status"', launch)
        self.assertIn('f"{prefix}/raw_obstacle_delta_3d"', launch)
        planner_parameters = launch.split("planner_params =", maxsplit=1)[1].split(
            "offboard_params =", maxsplit=1
        )[0]
        self.assertIn('"obstacle_memory_status_topic": memory_status', planner_parameters)
        self.assertIn('"raw_obstacle_delta_3d_topic": (', planner_parameters)
        self.assertNotIn('"raw_obstacle_delta_topic": raw_delta', planner_parameters)
        self.assertNotIn(
            '"obstacle_memory_snapshot_topic": memory_snapshot', planner_parameters
        )

if __name__ == "__main__":
    unittest.main()
