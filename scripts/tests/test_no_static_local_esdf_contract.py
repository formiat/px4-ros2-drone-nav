#!/usr/bin/env python3
"""Source contracts for bounded no-static ESDF updates and latest-raw safety."""

from __future__ import annotations

import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE = REPO_ROOT / "drone_city_nav"


class NoStaticLocalEsdfContractTest(unittest.TestCase):
    def test_local_esdf_budget_is_explicit_and_has_recenter_hysteresis(self) -> None:
        config = yaml.safe_load((PACKAGE / "config/urban_mvp.yaml").read_text())
        parameters = config["production_mppi_node"]["ros__parameters"]

        self.assertGreater(parameters["no_static_3d_esdf_update_rate_hz"], 0.0)
        self.assertGreater(parameters["no_static_3d_esdf_horizontal_half_extent_m"], 0.0)
        self.assertGreater(parameters["no_static_3d_esdf_vertical_half_extent_m"], 0.0)
        self.assertGreaterEqual(
            parameters["no_static_3d_esdf_horizontal_recenter_margin_m"], 0.0
        )
        self.assertGreaterEqual(
            parameters["no_static_3d_esdf_vertical_recenter_margin_m"], 0.0
        )
        self.assertLess(
            parameters["no_static_3d_esdf_horizontal_recenter_margin_m"],
            parameters["no_static_3d_esdf_horizontal_half_extent_m"],
        )
        self.assertLess(
            parameters["no_static_3d_esdf_vertical_recenter_margin_m"],
            parameters["no_static_3d_esdf_vertical_half_extent_m"],
        )

    def test_no_static_build_selects_local_3d_window_before_distance_update(self) -> None:
        source = (PACKAGE / "src/production_mppi_node_observed_esdf.cpp").read_text()

        local_window = source.index("selectLocalObservedEsdfBounds")
        distance_field = source.index("updateObservedEsdf3D", local_window)
        self.assertLess(local_window, distance_field)
        self.assertIn("localObservedEsdfNeedsRecenter", source)
        self.assertIn("source_occupied_fingerprint", source)

    def test_execution_validates_latest_reconstructed_raw_world_independently(
        self,
    ) -> None:
        raw_input = (PACKAGE / "src/production_mppi_node_raw_input.cpp").read_text()
        execution = "\n".join(
            (PACKAGE / "src" / name).read_text()
            for name in (
                "production_mppi_node_execution.cpp",
                "production_mppi_node_execution_publication.cpp",
                "production_mppi_node_execution_retention.cpp",
            )
        )

        self.assertIn("latest_raw_world_3d_.store", raw_input)
        self.assertIn("latest_raw_world_3d_.load", execution)
        self.assertIn("committed_world_current", execution)
        self.assertIn("execution_owner", execution)
        self.assertNotIn("ProductionMppiRawWorld2D", raw_input)
        self.assertNotIn("rawOccupancyGridViewFromRos", execution)

    def test_latest_lidar_safety_age_covers_pose_alignment_wait(self) -> None:
        config = yaml.safe_load((PACKAGE / "config/urban_mvp.yaml").read_text())
        memory = config["obstacle_memory_3d_node"]["ros__parameters"]
        planner = config["production_mppi_node"]["ros__parameters"]

        self.assertGreaterEqual(
            planner["latest_lidar_obstacle_maximum_age_ms"],
            1000.0 * memory["lidar_scan_alignment_maximum_wait_s"],
        )

    def test_hard_planning_footprint_is_the_physical_hull(self) -> None:
        source = (PACKAGE / "src/production_mppi_node.cpp").read_text()

        self.assertNotIn("+ static_route_tracking_margin_m", source)
        self.assertIn(
            "lattice_config_.physical_footprint_radius_m = "
            "physical_footprint_config_.radius_m;",
            source,
        )
        self.assertIn(
            "lattice_3d_config_.physical_footprint_radius_m = "
            "physical_footprint_config_.radius_m;",
            source,
        )
        self.assertIn(".physical_footprint = physical_footprint_config_", source)


if __name__ == "__main__":
    unittest.main()
