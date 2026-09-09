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
SOURCE = PACKAGE / "src"
ROS_RUNTIME = SOURCE / "runtime" / "ros"


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

    def test_execution_uses_only_the_3d_raw_world_contract(self) -> None:
        raw_input = (ROS_RUNTIME / "production_mppi_node_raw_input.cpp").read_text()
        execution = "\n".join(
            path.read_text()
            for path in (
                ROS_RUNTIME / "production_mppi_node_execution.cpp",
                ROS_RUNTIME / "production_mppi_node_execution_publication.cpp",
                ROS_RUNTIME / "production_mppi_node_execution_retention.cpp",
                SOURCE / "execution_supervisor_3d_retention.cpp",
            )
        )

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
        source = (ROS_RUNTIME / "production_mppi_config_ros.cpp").read_text()
        cpu_dynamics = (PACKAGE / "src/motion_dynamics_3d.cpp").read_text()
        cuda_dynamics = (PACKAGE / "src/mppi/mppi_engine_kernels.cuh").read_text()

        self.assertGreater(guaranteed_range_m, physical_margin_m)
        self.assertLessEqual(guaranteed_range_m, physical_range_m)
        self.assertLessEqual(guaranteed_range_m, memory["max_lidar_range_m"])
        self.assertEqual(vertical_samples, memory["lidar_3d_vertical_samples"])
        self.assertAlmostEqual(vertical_min, memory["lidar_3d_vertical_min_angle_rad"])
        self.assertAlmostEqual(vertical_max, memory["lidar_3d_vertical_max_angle_rad"])
        self.assertAlmostEqual(vertical_min, -0.5 * math.pi, places=12)
        self.assertAlmostEqual(vertical_max, 0.5 * math.pi, places=12)
        # Sensing geometry behind the guaranteed range: at that range adjacent
        # scan rows must land no farther apart than the vertical body band plus
        # one occupancy voxel, and adjacent columns no farther apart than the
        # body diameter plus one voxel, so a surface crossing the vehicle's path
        # is painted inside the swept footprint by measured returns alone.
        horizontal = sensor.find("ray/scan/horizontal")
        self.assertIsNotNone(horizontal)
        horizontal_samples = int(horizontal.findtext("samples", "0"))
        horizontal_min = float(horizontal.findtext("min_angle", "nan"))
        horizontal_max = float(horizontal.findtext("max_angle", "nan"))
        self.assertGreater(vertical_samples, 1)
        self.assertGreater(horizontal_samples, 1)
        vertical_spacing_rad = (vertical_max - vertical_min) / (vertical_samples - 1)
        horizontal_spacing_rad = (horizontal_max - horizontal_min) / (
            horizontal_samples - 1
        )
        voxel_m = memory["grid_resolution_m"]
        body_band_m = (
            planner["physical_footprint_lower_extent_m"]
            + planner["physical_footprint_upper_extent_m"]
        )
        body_diameter_m = 2.0 * planner["physical_footprint_radius_m"]
        self.assertLessEqual(
            guaranteed_range_m * math.tan(vertical_spacing_rad), body_band_m + voxel_m
        )
        self.assertLessEqual(
            guaranteed_range_m * math.tan(horizontal_spacing_rad),
            body_diameter_m + voxel_m,
        )
        self.assertNotIn("observation_distance_m", planner)
        self.assertNotIn("observation_margin_m", planner)
        self.assertIn(
            "config_.execution.latest_lidar_obstacle_maximum_age_ms * 1.0e-3",
            source,
        )
        # The contract carries each axis's acceleration separately and assesses
        # the stop along the direction of motion; a vector sum of both axes
        # paired with the weakest deceleration used to bound every direction.
        self.assertIn(
            ".maximum_horizontal_acceleration_mps2 = maximum_horizontal_acceleration_mps2",
            source,
        )
        self.assertIn(
            ".maximum_vertical_acceleration_mps2 = maximum_vertical_acceleration_mps2",
            source,
        )
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
        config_source = (ROS_RUNTIME / "production_mppi_config_ros.cpp").read_text()
        runtime_source = (ROS_RUNTIME / "production_mppi_node_interfaces.cpp").read_text()
        config = yaml.safe_load((PACKAGE / "config/urban_mvp.yaml").read_text())
        parameters = config["production_mppi_node"]["ros__parameters"]

        self.assertNotIn("static_route_tracking_margin_m", parameters)
        self.assertGreater(parameters["tracking_error_tube_response_time_s"], 0.0)
        self.assertNotIn("lattice_config_", config_source)
        self.assertNotIn("lattice_3d_config_", config_source)
        # The hard body is the physical hull, stood upright by every validator,
        # planner and execution alike; never a tracking margin. The tilt the
        # airframe reaches is the tube's lean law.
        self.assertIn(
            "control.tracking_error_tube.maximum_body_tilt_rad = maximumBodyTiltRad(",
            config_source,
        )
        self.assertIn(
            "planning.persistent_planner.physical_footprint = "
            "world.physical_footprint;",
            config_source,
        )
        # A departure answers to the hull, not to the envelope that contains
        # the hull at every tilt: the vehicle leaves at hover and upright.
        self.assertIn(
            "planning.persistent_planner.departure_footprint =\n"
            "      physicalBodyFootprint(world.physical_footprint);",
            config_source,
        )
        self.assertIn(
            ".physical_footprint = config_.world.physical_footprint",
            runtime_source,
        )

if __name__ == "__main__":
    unittest.main()
