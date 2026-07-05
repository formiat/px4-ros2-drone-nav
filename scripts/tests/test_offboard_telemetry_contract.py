#!/usr/bin/env python3
"""Static checks for offboard telemetry log fields."""

from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OFFBOARD_SOURCES = [
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node.cpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node.hpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node_control.cpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node_inputs.cpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node_lifecycle.cpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node_telemetry.cpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node_trajectory.cpp",
    REPO_ROOT / "drone_city_nav/src/offboard_blackbox.cpp",
    REPO_ROOT / "drone_city_nav/src/offboard_debug_markers.cpp",
    REPO_ROOT / "drone_city_nav/src/offboard_trajectory_state.cpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_node_config.cpp",
    REPO_ROOT / "drone_city_nav/src/px4_offboard_setpoint_io.cpp",
]
PLANNER_SOURCES = [
    REPO_ROOT / "drone_city_nav/src/planner_node.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_node.hpp",
    REPO_ROOT / "drone_city_nav/src/planner_node_inputs.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_node_lifecycle.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_node_refinement.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_node_publish.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_node_runtime.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_node_trajectory_publication.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_diagnostics_format.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_path_publication.cpp",
    REPO_ROOT / "drone_city_nav/src/planner_runtime_state.cpp",
]
TRAJECTORY_PLANNER = REPO_ROOT / "drone_city_nav/src/trajectory_planner.cpp"
TRAJECTORY_DIAGNOSTICS_IO_SOURCES = [
    REPO_ROOT / "drone_city_nav/src/trajectory_diagnostics_io.cpp",
    REPO_ROOT / "drone_city_nav/src/trajectory_diagnostics_io_csv.cpp",
    REPO_ROOT / "drone_city_nav/src/trajectory_diagnostics_io_json_fields.cpp",
    REPO_ROOT / "drone_city_nav/src/trajectory_diagnostics_io_json_summary.cpp",
    REPO_ROOT / "drone_city_nav/src/trajectory_diagnostics_io_parser.cpp",
]
FINAL_TRAJECTORY_DEBUG_IO = (
    REPO_ROOT / "drone_city_nav/src/final_trajectory_debug_io.cpp"
)


def read_all(paths: list[Path]) -> str:
    return "\n".join(path.read_text(encoding="utf-8") for path in paths)


class OffboardTelemetryContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.offboard_text = read_all(OFFBOARD_SOURCES)
        cls.planner_text = read_all(PLANNER_SOURCES)
        cls.trajectory_planner_text = TRAJECTORY_PLANNER.read_text(encoding="utf-8")
        cls.trajectory_diagnostics_io_text = read_all(TRAJECTORY_DIAGNOSTICS_IO_SOURCES)
        cls.final_trajectory_debug_io_text = FINAL_TRAJECTORY_DEBUG_IO.read_text(
            encoding="utf-8"
        )

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
        self.assertIn("desired_velocity=(%.2f, %.2f)", self.offboard_text)
        self.assertIn("velocity_tracking_error=%.2f", self.offboard_text)
        self.assertIn("velocity_setpoint_jerk=%.2f", self.offboard_text)
        self.assertIn("lateral_control[feedback=", self.offboard_text)
        self.assertIn("curvature_ff=", self.offboard_text)
        self.assertIn("curvature_angle=%.1fdeg", self.offboard_text)
        self.assertNotIn("curvature_anticipation=(%.2f, %.2f)", self.offboard_text)
        self.assertNotIn("accel_feedforward=(%.2f, %.2f)", self.offboard_text)
        self.assertNotIn("accel_feedforward_jerk=%.2f", self.offboard_text)
        self.assertIn("speed_limit_reason=%s", self.offboard_text)
        self.assertIn("profile_speed_limit=%.2f", self.offboard_text)
        self.assertIn("lookahead_distance=%.2f", self.offboard_text)
        self.assertIn("lookahead_speed_limit=%.2f", self.offboard_text)
        self.assertIn("speed_after_lookahead=%.2f", self.offboard_text)
        self.assertIn("lookahead_constraint[type=%s", self.offboard_text)
        self.assertNotIn("cross_track_speed_factor=%.2f", self.offboard_text)
        self.assertNotIn("cross_track_limited_speed=%.2f", self.offboard_text)
        self.assertIn("final_command_speed=%.2f", self.offboard_text)
        self.assertIn("smoother[reset_reason=%s", self.offboard_text)
        self.assertIn("limiting_constraint[type=%s", self.offboard_text)
        self.assertIn("trajectory[valid=%s", self.offboard_text)
        self.assertIn("arc_radius=%.2f", self.offboard_text)
        self.assertIn("braking_distance=%.2f", self.offboard_text)
        self.assertIn("tracking_prediction[horizon=%.2fs", self.offboard_text)
        self.assertIn("current_cross=%.2f predicted_cross=%.2f", self.offboard_text)
        self.assertIn("response_delay_distance=%.2f", self.offboard_text)
        self.assertIn("control_tangent[smoothed=%s mode=%s", self.offboard_text)
        self.assertIn("Drone obstacle diagnostics:", self.offboard_text)
        self.assertIn("nearest_prohibited_cell[valid=%s", self.offboard_text)
        self.assertIn("prohibited_grid_clearance=%.2f", self.offboard_text)
        self.assertNotIn("local_clearance=%.2f", self.offboard_text)
        self.assertNotIn("nearest_obstacle[valid=%s", self.offboard_text)
        self.assertIn("bearing_body_deg=%.1f", self.offboard_text)

    def test_velocity_mode_consumes_planner_final_trajectory(self) -> None:
        self.assertNotIn("rebuildFinalTrajectory(", self.offboard_text)
        self.assertNotIn("planBaselineTrajectory(", self.offboard_text)
        self.assertNotIn("planOptimizedTrajectory(", self.offboard_text)
        self.assertIn("planOptimizedTrajectory(", self.trajectory_planner_text)
        self.assertIn("rough A* route will not be published", self.planner_text)
        self.assertIn("applyReceivedFinalTrajectoryPath(", self.offboard_text)
        self.assertIn("trajectoryPointSamplesFromPoints(", self.offboard_text)
        self.assertIn("lineTrajectoryFromSamples(", self.offboard_text)
        self.assertIn("buildTrajectorySpeedProfile(", self.offboard_text)
        self.assertIn("final_trajectory_samples_", self.offboard_text)
        self.assertIn("trajectory_speed_profile_", self.offboard_text)
        self.assertIn("planVelocitySetpoint(", self.offboard_text)
        self.assertIn(
            "final_trajectory_samples_, trajectory_speed_profile_", self.offboard_text
        )
        self.assertIn("velocityCruiseReady()", self.offboard_text)
        self.assertIn("finalTrajectoryReady()", self.offboard_text)
        self.assertNotIn("cruise_velocity_control_enabled_", self.offboard_text)

    def test_velocity_mode_does_not_command_px4_acceleration(self) -> None:
        self.assertIn("msg.acceleration = false;", self.offboard_text)
        self.assertIn(
            "msg.acceleration = std::array<float, 3>{nan, nan, nan};",
            self.offboard_text,
        )

    def test_final_trajectory_does_not_rebuild_on_grid_churn(self) -> None:
        self.assertNotIn("finalTrajectoryGridRebuildReason(", self.offboard_text)
        self.assertNotIn("trajectoryGridRebuildReason(", self.offboard_text)
        self.assertNotIn("Final trajectory rebuild", self.offboard_text)
        self.assertNotIn("grid-triggered rebuild", self.offboard_text)
        self.assertIn("buildCorridor(", self.trajectory_planner_text)
        self.assertIn("TrajectoryPlannerStatus::kMissingGrid", self.trajectory_planner_text)
        self.assertIn("trajectoryPlannerStatusName", self.trajectory_planner_text)
        self.assertNotIn('"prohibited_intersection"', self.trajectory_planner_text)
        self.assertNotIn('"corridor_bounds_changed"', self.trajectory_planner_text)

    def test_telemetry_writes_jsonl_flight_blackbox(self) -> None:
        self.assertIn("flight_blackbox_enabled", self.offboard_text)
        self.assertIn("flight_blackbox_path", self.offboard_text)
        self.assertIn("offboard_blackbox.jsonl", self.offboard_text)
        self.assertIn("writeFlightBlackbox", self.offboard_text)
        self.assertIn("cross_track_error_m", self.offboard_text)
        self.assertIn("prohibited_grid_clearance_m", self.offboard_text)
        self.assertIn("nearest_prohibited_cell_valid", self.offboard_text)
        self.assertIn("nearest_prohibited_grid_clearance_m", self.offboard_text)
        self.assertIn("nearest_prohibited_cell_bearing_body_rad", self.offboard_text)
        self.assertIn("nearest_prohibited_cell_x", self.offboard_text)
        self.assertNotIn("local_clearance_m", self.offboard_text)
        self.assertNotIn("nearest_clearance_m", self.offboard_text)
        self.assertIn("final_goal_hold_active", self.offboard_text)
        self.assertIn("velocity_command", self.offboard_text)
        self.assertIn("setpoint_speed_mps", self.offboard_text)
        self.assertIn("final_command_speed_mps", self.offboard_text)
        self.assertIn("smoother_reset_reason", self.offboard_text)
        self.assertIn("path_update_reset_count", self.offboard_text)
        self.assertIn("desired_setpoint_x", self.offboard_text)
        self.assertIn("desired_setpoint_y", self.offboard_text)
        self.assertIn("desired_setpoint_speed_mps", self.offboard_text)
        self.assertIn("cross_track_feedback_mps", self.offboard_text)
        self.assertIn("cross_track_derivative_damping_mps", self.offboard_text)
        self.assertIn("curvature_feedforward_mps", self.offboard_text)
        self.assertIn("raw_lateral_control_mps", self.offboard_text)
        self.assertIn("lateral_control_mps", self.offboard_text)
        self.assertNotIn("lateral_control_delta_mps", self.offboard_text)
        self.assertIn("velocity_setpoint_accel_norm_mps2", self.offboard_text)
        self.assertIn("velocity_setpoint_jerk_mps3", self.offboard_text)
        self.assertIn("curvature_feedforward_angle_rad", self.offboard_text)
        self.assertNotIn("curvature_anticipation_mps", self.offboard_text)
        self.assertNotIn("raw_curvature_anticipation_mps", self.offboard_text)
        self.assertNotIn("curvature_anticipation_delta_mps", self.offboard_text)
        self.assertNotIn("curvature_anticipation_angle_rad", self.offboard_text)
        self.assertNotIn('"setpoint_accel_norm_mps2"', self.offboard_text)
        self.assertNotIn('"raw_setpoint_accel_norm_mps2"', self.offboard_text)
        self.assertNotIn('"setpoint_accel_delta_mps2"', self.offboard_text)
        self.assertNotIn('"setpoint_accel_jerk_mps3"', self.offboard_text)
        self.assertNotIn('"curvature_feedforward_accel_mps2"', self.offboard_text)
        self.assertIn("raw_speed_limit_mps", self.offboard_text)
        self.assertIn("profile_speed_limit_mps", self.offboard_text)
        self.assertIn("lookahead_distance_m", self.offboard_text)
        self.assertIn("lookahead_speed_limit_mps", self.offboard_text)
        self.assertIn("speed_after_lookahead_mps", self.offboard_text)
        self.assertIn("lookahead_limiting_constraint_type", self.offboard_text)
        self.assertIn("lookahead_limiting_constraint_distance_m", self.offboard_text)
        self.assertNotIn("cross_track_speed_factor", self.offboard_text)
        self.assertNotIn("cross_track_limited_speed_mps", self.offboard_text)
        self.assertIn("limiting_constraint_type", self.offboard_text)
        self.assertIn("limiting_constraint_distance_m", self.offboard_text)
        self.assertIn("limiting_constraint_speed_mps", self.offboard_text)
        self.assertIn("limiting_allowed_speed_now_mps", self.offboard_text)
        self.assertIn("limiting_curve_radius_m", self.offboard_text)
        self.assertIn("final_stop_distance_m", self.offboard_text)
        self.assertIn("final_stop_braking_distance_m", self.offboard_text)
        self.assertNotIn("cross_track_correction_mps", self.offboard_text)
        self.assertNotIn("cross_track_correction_x", self.offboard_text)
        self.assertNotIn("cross_track_correction_y", self.offboard_text)
        self.assertNotIn("raw_cross_track_correction_mps", self.offboard_text)
        self.assertNotIn("cross_track_correction_delta_mps", self.offboard_text)
        self.assertIn("cross_track_lateral_velocity_mps", self.offboard_text)
        self.assertIn("current_velocity_tangent_mps", self.offboard_text)
        self.assertIn("current_velocity_normal_mps", self.offboard_text)
        self.assertIn("desired_velocity_tangent_mps", self.offboard_text)
        self.assertIn("desired_velocity_normal_mps", self.offboard_text)
        self.assertIn("setpoint_velocity_tangent_mps", self.offboard_text)
        self.assertIn("setpoint_velocity_normal_mps", self.offboard_text)
        self.assertIn("tracking_prediction_horizon_s", self.offboard_text)
        self.assertIn("tracking_prediction_distance_m", self.offboard_text)
        self.assertIn("control_projection_smoothing_mode", self.offboard_text)
        self.assertIn("tracking_predicted_x", self.offboard_text)
        self.assertIn("tracking_predicted_y", self.offboard_text)
        self.assertIn("current_projection_x", self.offboard_text)
        self.assertIn("current_projection_y", self.offboard_text)
        self.assertIn("predicted_projection_x", self.offboard_text)
        self.assertIn("predicted_projection_y", self.offboard_text)
        self.assertIn("current_cross_track_error_m", self.offboard_text)
        self.assertIn("predicted_cross_track_error_m", self.offboard_text)
        self.assertIn("response_delay_distance_m", self.offboard_text)
        self.assertIn("trajectory_valid", self.offboard_text)
        self.assertIn("trajectory_total_length_m", self.offboard_text)
        self.assertIn("trajectory_line_segments", self.offboard_text)
        self.assertIn("trajectory_arc_segments", self.offboard_text)
        self.assertIn("trajectory_s_m", self.offboard_text)
        self.assertIn("trajectory_segment_type", self.offboard_text)
        self.assertIn("trajectory_curvature_1pm", self.offboard_text)
        self.assertIn("trajectory_arc_radius_m", self.offboard_text)
        self.assertIn("speed_profile_limit_mps", self.offboard_text)
        self.assertIn("speed_profile_lookahead_distance_m", self.offboard_text)
        self.assertIn("speed_profile_lookahead_limit_mps", self.offboard_text)
        self.assertIn("speed_profile_reason", self.offboard_text)
        self.assertIn("speed_profile_distance_to_constraint_m", self.offboard_text)
        self.assertIn("final_trajectory_samples", self.offboard_text)
        self.assertIn("trajectory_shape_segment_len_min_m", self.offboard_text)
        self.assertIn("trajectory_shape_max_heading_delta_rad", self.offboard_text)
        self.assertIn("trajectory_shape_max_curvature_jump_1pm", self.offboard_text)
        self.assertIn("trajectory_shape_max_offset_delta_m", self.offboard_text)
        self.assertIn("trajectory_planner_status", self.offboard_text)
        self.assertIn("corridor_width_min_m", self.offboard_text)
        self.assertIn("corridor_width_max_m", self.offboard_text)
        self.assertIn("corridor_lateral_limited_samples", self.offboard_text)
        self.assertIn("corridor_center_recovered_samples", self.offboard_text)
        self.assertIn("corridor_center_unrecoverable_samples", self.offboard_text)
        self.assertIn("corridor_center_recovery_max_m", self.offboard_text)
        self.assertIn("corridor_lateral_reduction_max_m", self.offboard_text)
        self.assertIn("trajectory_optimizer_cost_final", self.offboard_text)
        self.assertIn("turnSmoothingDiagnosticsJsonFields", self.offboard_text)
        self.assertIn(
            "turn_smoothing_smoothed_corners", self.trajectory_diagnostics_io_text
        )
        self.assertIn(
            "turn_smoothing_heading_delta_after_rad",
            self.trajectory_diagnostics_io_text,
        )
        self.assertIn(
            "turn_smoothing_max_outer_shift_m", self.trajectory_diagnostics_io_text
        )
        self.assertIn(
            "turn_smoothing_accepted_relaxed_angle_deg",
            self.trajectory_diagnostics_io_text,
        )
        self.assertIn("speed_profile_limited_by_curvature_count", self.offboard_text)
        self.assertNotIn("trajectory_fallback_reason", self.offboard_text)
        self.assertNotIn("baseline_rounded_corners", self.offboard_text)
        self.assertIn("rough_route_debug_turn_angle_rad", self.offboard_text)
        self.assertIn("rough_route_debug_segment_type", self.offboard_text)
        self.assertNotIn('\\"turn_angle_rad\\"', self.offboard_text)
        self.assertNotIn('\\"turn_valid\\"', self.offboard_text)
        self.assertNotIn('\\"turn_distance_m\\"', self.offboard_text)

    def test_final_trajectory_samples_csv_dump_is_written(self) -> None:
        self.assertIn(
            "drone_city_nav::writeFinalTrajectorySamplesCsv", self.offboard_text
        )
        self.assertIn('diagnosticDumpDirectory("final_trajectory_samples")', self.offboard_text)
        self.assertIn("latest.csv", self.offboard_text)
        self.assertIn(
            "finalTrajectorySamplesCsvHeader()",
            self.final_trajectory_debug_io_text,
        )
        self.assertIn(
            "sample_index,s_m,x,y",
            self.trajectory_diagnostics_io_text,
        )
        self.assertIn("lateral_offset_m", self.trajectory_diagnostics_io_text)
        self.assertIn(
            "speed_profiled_limit_mps",
            self.trajectory_diagnostics_io_text,
        )

    def test_corridor_samples_are_planner_owned(self) -> None:
        self.assertIn("result.corridor_samples = corridor.samples", self.trajectory_planner_text)
        self.assertIn("smoothTrajectoryTurns(", self.trajectory_planner_text)
        self.assertNotIn("corridor_debug_samples_", self.offboard_text)
        self.assertNotIn("writeCorridorSamplesCsv", self.offboard_text)
        self.assertNotIn('diagnosticDumpDirectory("corridor_samples")', self.offboard_text)
        self.assertNotIn("trajectory_optimizer_corridor_", self.offboard_text)

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
                self.assertIn(
                    "trajectory_diagnostics_topic: "
                    "/drone_city_nav/trajectory_diagnostics",
                    text,
                )
                self.assertIn("flight_blackbox_enabled: true", text)
                self.assertIn("flight_blackbox_path: log/offboard_blackbox.jsonl", text)
                self.assertIn("cruise_speed_mps: 22.0", text)
                self.assertIn("min_turn_speed_mps: 1.5", text)
                self.assertIn("speed_profile_accel_mps2: 7.0", text)
                self.assertIn("setpoint_forward_accel_mps2: 7.0", text)
                self.assertIn("setpoint_forward_decel_mps2: 20.0", text)
                self.assertIn("speed_profile_decel_mps2: 2.0", text)
                self.assertIn("speed_profile_lookahead_time_s: 1.0", text)
                self.assertIn("speed_profile_lookahead_min_m: 5.0", text)
                self.assertIn("speed_profile_lookahead_max_m: 35.0", text)
                self.assertIn("diagnostic_turn_preview_distance_m: 90.0", text)
                self.assertIn("speed_profile_sample_step_m: 0.5", text)
                self.assertIn(
                    "terminal_position_capture_max_entry_speed_mps: 3.0", text
                )
                self.assertIn("terminal_stuck_speed_mps: 0.5", text)
                self.assertIn("cross_track_derivative_gain: 0.5", text)
                self.assertIn("tracking_prediction_horizon_s: 0.35", text)
                self.assertIn("max_lateral_control_angle_deg: 55.0", text)
                self.assertNotIn("max_lateral_control_rate_mps2:", text)
                self.assertNotIn("max_cross_track_correction_angle_deg:", text)
                self.assertNotIn("max_cross_track_correction_rate_mps2:", text)
                self.assertNotIn("cross_track_speed_guard_start_m:", text)
                self.assertNotIn("cross_track_speed_guard_full_m:", text)
                self.assertNotIn("cross_track_speed_guard_min_factor:", text)
                self.assertIn("curvature_feedforward_time_s: 0.25", text)
                self.assertIn("max_curvature_feedforward_angle_deg: 30.0", text)
                self.assertNotIn("curvature_velocity_anticipation_time_s:", text)
                self.assertNotIn(
                    "max_curvature_velocity_anticipation_angle_deg:", text
                )
                self.assertNotIn(
                    "max_curvature_velocity_anticipation_rate_mps2:", text
                )
                self.assertIn("max_velocity_jerk_mps3: 12.0", text)
                self.assertNotIn("max_feedforward_accel_mps2:", text)
                self.assertNotIn("max_feedforward_jerk_mps3:", text)
                self.assertNotIn("acceleration_feedforward_scale:", text)
                self.assertNotIn("trajectory_optimizer_trajectory_enabled", text)
                self.assertNotIn("trajectory_baseline_rounding_", text)
                self.assertIn("corridor_max_radius_m: 40.0", text)
                self.assertIn("corridor_sample_step_m: 1.0", text)
                self.assertIn("corridor_lateral_limit_margin_m: 1.0", text)
                self.assertIn("corridor_center_recovery_max_m: 3.0", text)
                self.assertNotIn("corridor_rebuild_width_threshold_m", text)
                self.assertIn("trajectory_optimizer_max_iterations: 80", text)
                self.assertIn("trajectory_optimizer_weight_curvature: 300.0", text)
                self.assertIn("trajectory_optimizer_weight_offset_change: 0.5", text)
                self.assertIn("trajectory_optimizer_weight_offset_second_change: 6.5", text)
                self.assertIn("turn_smoothing_trigger_heading_delta_deg: 37.0", text)
                self.assertIn("turn_smoothing_trigger_min_radius_m: 16.0", text)
                self.assertIn("turn_smoothing_entry_distance_m: 45.0", text)
                self.assertIn("turn_smoothing_exit_distance_m: 45.0", text)
                self.assertIn("turn_smoothing_outer_bias_ratio: 0.45", text)
                self.assertIn("turn_smoothing_max_passes: 8", text)
                self.assertIn(
                    "final_trajectory_debug_topic: /drone_city_nav/final_trajectory_path",
                    text,
                )
                self.assertIn("final_trajectory_debug_sample_step_m: 1.0", text)
                self.assertNotIn("executable_trajectory_max_step_m:", text)
                self.assertNotIn("trajectory_result_stale_cross_track_m:", text)
                self.assertIn("telemetry_log_period_s: 0.1", text)


if __name__ == "__main__":
    unittest.main()
