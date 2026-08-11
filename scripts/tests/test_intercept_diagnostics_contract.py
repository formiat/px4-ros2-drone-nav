#!/usr/bin/env python3
"""Static contracts for multi-interceptor RViz diagnostics selection."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
LAUNCH = PACKAGE / "launch" / "intercept.launch.py"
DIAGNOSTICS_LAUNCH = PACKAGE / "launch" / "intercept_diagnostics_launch.py"
MUX = PACKAGE / "src" / "intercept_diagnostics_mux_node.cpp"
OBSTACLE_MEMORY = PACKAGE / "src" / "obstacle_memory_node.cpp"
LIDAR_HEADER = PACKAGE / "src" / "lidar_debug_node.hpp"
LIDAR_CALLBACKS = PACKAGE / "src" / "lidar_debug_node_callbacks.cpp"
LIDAR_LIFECYCLE = PACKAGE / "src" / "lidar_debug_node_lifecycle.cpp"
RVIZ_CONFIGS = (
    PACKAGE / "rviz" / "city_nav_debug.rviz",
    PACKAGE / "rviz" / "city_nav_debug_top_down.rviz",
)


class InterceptDiagnosticsContractTest(unittest.TestCase):
    def test_launch_keeps_all_vehicle_sources_namespaced(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        diagnostics_launch = DIAGNOSTICS_LAUNCH.read_text(encoding="utf-8")
        self.assertIn('path_topic = f"{prefix}/mppi/path"', launch)
        self.assertIn('marker_topic = f"{prefix}/mppi/markers"', launch)
        self.assertNotIn('"/drone_city_nav/mppi/path" if primary', launch)
        self.assertIn(
            'plugin="drone_city_nav::InterceptDiagnosticsMuxNode"',
            diagnostics_launch,
        )
        self.assertIn('name="intercept_diagnostics_container"', diagnostics_launch)
        self.assertIn('"vehicle_ids": vehicle_ids', diagnostics_launch)
        self.assertIn('"vehicle_roles": vehicle_roles', diagnostics_launch)
        self.assertIn('"initial_vehicle_id": initial_vehicle_id', diagnostics_launch)
        self.assertIn('"reselection_policy": reselection_policy', diagnostics_launch)
        self.assertIn(
            "[f\"/vehicles/{role}\" for role in role_names]", launch
        )

    def test_lidar_debug_is_selector_gated_for_every_vehicle(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        diagnostics_launch = DIAGNOSTICS_LAUNCH.read_text(encoding="utf-8")
        header = LIDAR_HEADER.read_text(encoding="utf-8")
        callbacks = LIDAR_CALLBACKS.read_text(encoding="utf-8")
        lifecycle = LIDAR_LIFECYCLE.read_text(encoding="utf-8")
        self.assertIn("if lidar_debug_enabled:", launch)
        self.assertIn(
            'plugin="drone_city_nav::LidarDebugNode"', diagnostics_launch
        )
        self.assertIn('"spectator_vehicle_id": role', launch)
        self.assertIn('namespace=f"vehicles/{role}"', launch)
        self.assertIn("diagnosticsSelected", header)
        self.assertIn("if (!diagnosticsSelected())", callbacks)
        self.assertIn("LIDAR_DEBUG_SPECTATOR", lifecycle)

    def test_static_persistent_memory_is_selector_gated(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        obstacle_memory = OBSTACLE_MEMORY.read_text(encoding="utf-8")
        self.assertIn("role_persistent_memory_enabled", launch)
        self.assertIn(
            "role_persistent_memory_enabled = obstacle_memory_enabled", launch
        )
        self.assertIn('persistent_memory_spectator_vehicle_id = (', launch)
        self.assertIn('"persistent_memory_spectator_vehicle_id": (', launch)
        self.assertIn("SpectatorDiagnosticsSelection", obstacle_memory)
        self.assertIn("OBSTACLE_MEMORY_SPECTATOR", obstacle_memory)
        self.assertIn("!persistent_memory_selection_.selected()", obstacle_memory)

    def test_static_lidar_artifacts_are_bounded_per_interceptor(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('"max_snapshots": (', launch)
        self.assertRegex(launch, r'"max_snapshots": \(\s+1\s+if use_static_map')

    def test_mux_clears_then_republishes_selected_diagnostics(self) -> None:
        source = MUX.read_text(encoding="utf-8")
        self.assertIn("msg::SpectatorTarget", source)
        self.assertIn("clearSelectedDiagnostics();", source)
        self.assertIn("publishCachedDiagnostics(index);", source)
        self.assertIn("visualization_msgs::msg::Marker::DELETEALL", source)
        self.assertIn("INTERCEPT_DIAGNOSTICS_SELECTION", source)
        for topic in (
            "/drone_city_nav/mppi/path",
            "/drone_city_nav/mppi/markers",
            "/drone_city_nav/raw_memory_obstacle_points_3d",
            "/drone_city_nav/lidar_debug_points",
            "/drone_city_nav/interceptor_directions",
        ):
            with self.subTest(topic=topic):
                self.assertIn(topic, source)

    def test_rviz_shows_all_routes_and_only_selected_full_diagnostics(self) -> None:
        for config_path in RVIZ_CONFIGS:
            config = config_path.read_text(encoding="utf-8")
            with self.subTest(config=config_path.name):
                for index in range(3):
                    self.assertIn(f"Name: Interceptor {index} Route", config)
                    self.assertIn(
                        f"Value: /vehicles/interceptor_{index}/mppi/path", config
                    )
                    self.assertRegex(
                        config,
                        rf"Enabled: false\n\s+Name: Interceptor {index} "
                        r"Memory \(Optional\)",
                    )
                self.assertIn("Name: Interceptor Directions", config)
                self.assertIn(
                    "Value: /drone_city_nav/interceptor_directions", config
                )
                self.assertIn("Name: Selected MPPI Candidate Horizon", config)
                self.assertIn("Value: /drone_city_nav/mppi/markers", config)


if __name__ == "__main__":
    unittest.main()
