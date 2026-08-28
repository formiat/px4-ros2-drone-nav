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
        config = yaml.safe_load((PACKAGE / "config/urban_mvp.yaml").read_text())
        parameters = config["production_mppi_node"]["ros__parameters"]

        self.assertNotIn("static_route_tracking_margin_m", parameters)
        self.assertGreater(parameters["tracking_error_tube_response_time_s"], 0.0)
        self.assertNotIn("lattice_config_", source)
        self.assertNotIn("lattice_3d_config_", source)
        self.assertIn(
            "persistent_planner_config_.physical_footprint = "
            "physical_footprint_config_;",
            source,
        )
        self.assertIn(".physical_footprint = physical_footprint_config_", source)

    def test_tracking_uncertainty_caps_speed_from_raw_occupied_evidence(self) -> None:
        tube = (PACKAGE / "src/tracking_error_tube_3d.cpp").read_text()
        compiler = (PACKAGE / "src/route_compiler_3d.cpp").read_text()
        materialization = (
            PACKAGE / "src/production_mppi_route_materialization.cpp"
        ).read_text()
        world_binding = (
            PACKAGE / "src/production_mppi_node_route_compilation.cpp"
        ).read_text()
        parameterization = (
            PACKAGE / "src/route_time_parameterization.cpp"
        ).read_text()

        self.assertIn("ObservedSpaceValidationPolicy::kAllowUnknown", tube)
        self.assertIn("inflatedFootprint(physical_footprint", tube)
        self.assertIn("trackingErrorTubeRadiusM(config, maximum_speed_mps)", tube)
        self.assertIn("makeTrackingErrorTubeProfile3D", compiler)
        self.assertIn(".tracking_world = trackingErrorTubeWorld3D(world)", materialization)
        self.assertIn(
            "observed_raw_world_owner->occupiedContentFingerprint()", world_binding
        )
        self.assertNotIn(
            ".occupied_content_fingerprint = world.source_occupied_fingerprint",
            world_binding,
        )
        self.assertIn("tracking_speed_limits_mps[index]", parameterization)


if __name__ == "__main__":
    unittest.main()
