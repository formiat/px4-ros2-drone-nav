#!/usr/bin/env python3
"""Static checks for offboard telemetry log fields."""

from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OFFBOARD_NODE = REPO_ROOT / "drone_city_nav/src/px4_offboard_node.cpp"
TRAJECTORY_PLANNER = REPO_ROOT / "drone_city_nav/src/trajectory_planner.cpp"


class OffboardTelemetryContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.offboard_text = OFFBOARD_NODE.read_text(encoding="utf-8")
        cls.trajectory_planner_text = TRAJECTORY_PLANNER.read_text(encoding="utf-8")

    def test_telemetry_logs_attitude_at_runtime_rate(self) -> None:
        self.assertIn("telemetry_log_period_s", self.offboard_text)
        self.assertIn("Drone telemetry:", self.offboard_text)
        self.assertIn("attitude[valid=%s age_s=%.2f", self.offboard_text)
        self.assertIn("roll=%.3frad pitch=%.3frad yaw=%.3frad", self.offboard_text)
        self.assertIn("roll_deg=%.1f pitch_deg=%.1f yaw_deg=%.1f", self.offboard_text)
        self.assertIn("tilt_deg=%.1f", self.offboard_text)

    def test_telemetry_logs_path_command_and_obstacle_diagnostics(self) -> None:
        self.assertIn("path_id_topic", self.offboard_text)
        self.assertIn("create_subscription<std_msgs::msg::UInt64>", self.offboard_text)
        self.assertIn("Drone path diagnostics:", self.offboard_text)
        self.assertIn("path_id[local_update=%", self.offboard_text)
        self.assertIn("cross_track=%.2f", self.offboard_text)
        self.assertIn("heading_error=%.3f", self.offboard_text)
        self.assertIn("Drone command diagnostics:", self.offboard_text)
        self.assertIn("command[target_delta=%.2f", self.offboard_text)
        self.assertIn("Drone velocity command diagnostics:", self.offboard_text)
        self.assertIn("velocity_setpoint=(%.2f, %.2f, %.2f)", self.offboard_text)
        self.assertIn("speed_limit_reason=%s", self.offboard_text)
        self.assertIn("limiting_constraint[type=%s", self.offboard_text)
        self.assertIn("trajectory[valid=%s", self.offboard_text)
        self.assertIn("arc_radius=%.2f", self.offboard_text)
        self.assertIn("braking_distance=%.2f", self.offboard_text)
        self.assertIn("Drone obstacle diagnostics:", self.offboard_text)
        self.assertIn("nearest_obstacle[valid=%s", self.offboard_text)
        self.assertIn("bearing_body_deg=%.1f", self.offboard_text)

    def test_velocity_mode_uses_final_trajectory_planner(self) -> None:
        self.assertIn("rebuildFinalTrajectory(", self.offboard_text)
        self.assertIn("planTrajectory(", self.offboard_text)
        self.assertIn("final_trajectory_samples_", self.offboard_text)
        self.assertIn("trajectory_speed_profile_", self.offboard_text)
        self.assertIn("planVelocitySetpoint(trajectory_, trajectory_speed_profile_", self.offboard_text)
        self.assertIn("velocityCruiseReady()", self.offboard_text)
        self.assertIn("finalTrajectoryReady()", self.offboard_text)
        self.assertNotIn("cruise_velocity_control_enabled_", self.offboard_text)

    def test_final_trajectory_rebuilds_after_grid_lifecycle_events(self) -> None:
        self.assertIn("finalTrajectoryGridRebuildReason(", self.offboard_text)
        self.assertIn("trajectoryGridRebuildReason(", self.offboard_text)
        self.assertIn("buildCorridor(", self.offboard_text)
        self.assertIn("TrajectoryPlannerStatus::kMissingGrid", self.trajectory_planner_text)
        self.assertIn("trajectoryPlannerStatusName", self.trajectory_planner_text)
        self.assertIn('"prohibited_intersection"', self.trajectory_planner_text)
        self.assertIn('"corridor_bounds_changed"', self.trajectory_planner_text)
        self.assertIn("Final trajectory grid-triggered rebuild", self.offboard_text)

    def test_telemetry_writes_jsonl_flight_blackbox(self) -> None:
        self.assertIn("flight_blackbox_enabled", self.offboard_text)
        self.assertIn("flight_blackbox_path", self.offboard_text)
        self.assertIn("offboard_blackbox.jsonl", self.offboard_text)
        self.assertIn("writeFlightBlackbox", self.offboard_text)
        self.assertIn("cross_track_error_m", self.offboard_text)
        self.assertIn("bearing_body_rad", self.offboard_text)
        self.assertIn("final_goal_hold_active", self.offboard_text)
        self.assertIn("velocity_command", self.offboard_text)
        self.assertIn("setpoint_speed_mps", self.offboard_text)
        self.assertIn("raw_speed_limit_mps", self.offboard_text)
        self.assertIn("limiting_constraint_type", self.offboard_text)
        self.assertIn("limiting_constraint_distance_m", self.offboard_text)
        self.assertIn("limiting_constraint_speed_mps", self.offboard_text)
        self.assertIn("limiting_allowed_speed_now_mps", self.offboard_text)
        self.assertIn("limiting_curve_radius_m", self.offboard_text)
        self.assertIn("final_stop_distance_m", self.offboard_text)
        self.assertIn("final_stop_braking_distance_m", self.offboard_text)
        self.assertIn("cross_track_correction_mps", self.offboard_text)
        self.assertIn("trajectory_valid", self.offboard_text)
        self.assertIn("trajectory_total_length_m", self.offboard_text)
        self.assertIn("trajectory_line_segments", self.offboard_text)
        self.assertIn("trajectory_arc_segments", self.offboard_text)
        self.assertIn("trajectory_s_m", self.offboard_text)
        self.assertIn("trajectory_segment_type", self.offboard_text)
        self.assertIn("trajectory_curvature_1pm", self.offboard_text)
        self.assertIn("trajectory_arc_radius_m", self.offboard_text)
        self.assertIn("speed_profile_limit_mps", self.offboard_text)
        self.assertIn("speed_profile_reason", self.offboard_text)
        self.assertIn("speed_profile_distance_to_constraint_m", self.offboard_text)
        self.assertIn("final_trajectory_samples", self.offboard_text)
        self.assertIn("trajectory_planner_status", self.offboard_text)
        self.assertIn("corridor_width_min_m", self.offboard_text)
        self.assertIn("racing_line_cost_final", self.offboard_text)
        self.assertIn("speed_profile_limited_by_curvature_count", self.offboard_text)
        self.assertNotIn("trajectory_fallback_reason", self.offboard_text)
        self.assertNotIn("baseline_rounded_corners", self.offboard_text)
        self.assertIn("rough_route_debug_turn_angle_rad", self.offboard_text)
        self.assertIn("rough_route_debug_segment_type", self.offboard_text)
        self.assertNotIn('\\"turn_angle_rad\\"', self.offboard_text)
        self.assertNotIn('\\"turn_valid\\"', self.offboard_text)
        self.assertNotIn('\\"turn_distance_m\\"', self.offboard_text)

    def test_offboard_node_subscribes_to_px4_attitude(self) -> None:
        self.assertIn("#include <px4_msgs/msg/vehicle_attitude.hpp>", self.offboard_text)
        self.assertIn('"px4_vehicle_attitude_topic"', self.offboard_text)
        self.assertIn("create_subscription<px4_msgs::msg::VehicleAttitude>", self.offboard_text)
        self.assertIn("quaternionToEuler(msg.q)", self.offboard_text)

    def test_runtime_configs_expose_attitude_topic(self) -> None:
        for relative_path in ("drone_city_nav/config/urban_mvp.yaml",):
            with self.subTest(relative_path=relative_path):
                text = (REPO_ROOT / relative_path).read_text(encoding="utf-8")
                self.assertIn(
                    "px4_vehicle_attitude_topic: /fmu/out/vehicle_attitude", text
                )
                self.assertIn("path_id_topic: /drone_city_nav/path_id", text)
                self.assertIn("flight_blackbox_enabled: true", text)
                self.assertIn("flight_blackbox_path: log/offboard_blackbox.jsonl", text)
                self.assertIn("cruise_speed_mps: 22.0", text)
                self.assertIn("min_turn_speed_mps: 1.5", text)
                self.assertIn("max_accel_mps2: 7.0", text)
                self.assertIn("max_decel_mps2: 20.0", text)
                self.assertIn("speed_profile_decel_mps2: 3.0", text)
                self.assertIn("turn_preview_distance_m: 90.0", text)
                self.assertIn("speed_profile_sample_step_m: 0.5", text)
                self.assertNotIn("racing_trajectory_enabled", text)
                self.assertNotIn("trajectory_baseline_rounding_", text)
                self.assertIn("corridor_max_radius_m: 40.0", text)
                self.assertIn("corridor_sample_step_m: 1.0", text)
                self.assertIn("corridor_safety_margin_m: 0.5", text)
                self.assertIn("corridor_rebuild_width_threshold_m: 0.5", text)
                self.assertIn("racing_line_max_iterations: 80", text)
                self.assertIn("racing_line_weight_curvature: 25.0", text)
                self.assertIn(
                    "final_trajectory_debug_topic: /drone_city_nav/final_trajectory_path",
                    text,
                )
                self.assertIn("final_trajectory_debug_sample_step_m: 1.0", text)
                self.assertIn("telemetry_log_period_s: 0.1", text)


if __name__ == "__main__":
    unittest.main()
