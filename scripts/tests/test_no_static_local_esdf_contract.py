#!/usr/bin/env python3
"""Source contracts for bounded no-static ESDF updates and latest-raw safety."""

from __future__ import annotations

import math
import unittest
import xml.etree.ElementTree as ET
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

    def test_sensor_braking_limit_is_bound_to_3d_lidar_and_freshness(self) -> None:
        config = yaml.safe_load((PACKAGE / "config/urban_mvp.yaml").read_text())
        memory = config["obstacle_memory_3d_node"]["ros__parameters"]
        planner = config["production_mppi_node"]["ros__parameters"]
        model = ET.parse(PACKAGE / "models/lidar_3d_v1/model.sdf").getroot()
        sensor = next(
            element
            for element in model.iter("sensor")
            if element.attrib.get("name") == "lidar_3d_v1"
        )
        physical_range_m = float(sensor.findtext("ray/range/max", "nan"))
        vertical = sensor.find("ray/scan/vertical")
        self.assertIsNotNone(vertical)
        vertical_samples = int(vertical.findtext("samples", "0"))
        vertical_min = float(vertical.findtext("min_angle", "nan"))
        vertical_max = float(vertical.findtext("max_angle", "nan"))
        guaranteed_range_m = planner["guaranteed_lidar_detection_range_m"]
        physical_margin_m = planner["sensor_braking_physical_margin_m"]
        source = (PACKAGE / "src/production_mppi_node.cpp").read_text()
        cpu_dynamics = (PACKAGE / "src/mppi/mppi_reference.cpp").read_text()
        cuda_dynamics = (PACKAGE / "src/mppi/mppi_engine_kernels.cuh").read_text()

        self.assertGreater(guaranteed_range_m, physical_margin_m)
        self.assertLessEqual(guaranteed_range_m, physical_range_m)
        self.assertLessEqual(guaranteed_range_m, memory["max_lidar_range_m"])
        self.assertEqual(vertical_samples, memory["lidar_3d_vertical_samples"])
        self.assertAlmostEqual(vertical_min, memory["lidar_3d_vertical_min_angle_rad"])
        self.assertAlmostEqual(vertical_max, memory["lidar_3d_vertical_max_angle_rad"])
        self.assertAlmostEqual(vertical_min, -0.5 * math.pi, places=12)
        self.assertAlmostEqual(vertical_max, 0.5 * math.pi, places=12)
        self.assertNotIn("observation_distance_m", planner)
        self.assertNotIn("observation_margin_m", planner)
        self.assertIn(
            ".maximum_evidence_age_s = "
            "latest_lidar_obstacle_maximum_age_ms_ * 1.0e-3",
            source,
        )
        self.assertIn("std::hypot(", source)
        self.assertIn("maximum_horizontal_acceleration_mps2", source)
        self.assertIn("maximum_vertical_acceleration_mps2", source)
        self.assertIn(
            ".maximum_control_jerk_mps3 = maximum_control_jerk_mps3", source
        )
        self.assertIn("sensorBrakingMaximumSpeedMps", source)
        self.assertIn("maximum_translational_speed_mps", source)
        self.assertIn("clampTranslational(", cpu_dynamics)
        self.assertIn("maximum_translational_speed_mps", cpu_dynamics)
        self.assertIn("clampTranslational(", cuda_dynamics)
        self.assertIn("maximum_translational_speed_mps", cuda_dynamics)

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
        activation = (PACKAGE / "src/production_mppi_route_activation.cpp").read_text()
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
        self.assertIn(
            "activation_raw_owner->occupiedContentFingerprint()", activation
        )
        self.assertIn(".tracking_world = tracking_world", activation)
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
