#include "drone_city_nav/px4_offboard_node_config.hpp"

#include <rclcpp/rclcpp.hpp>

#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t secondsToNanoseconds(const double seconds) {
  return static_cast<std::int64_t>(seconds * 1.0e9);
}

[[nodiscard]] double radiansFromDegrees(const double degrees) noexcept {
  return degrees * std::numbers::pi / 180.0;
}

} // namespace

[[nodiscard]] double boundedFiniteDouble(const double value, const double fallback,
                                         const double min_value,
                                         const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] std::uint8_t boundedUint8(const std::int64_t value) {
  return static_cast<std::uint8_t>(std::clamp<std::int64_t>(value, 0, 255));
}

[[nodiscard]] std::uint16_t boundedUint16(const std::int64_t value) {
  return static_cast<std::uint16_t>(std::clamp<std::int64_t>(value, 0, 65535));
}

void sanitizePx4OffboardNodeConfig(Px4OffboardNodeConfig& config) {
  config.min_navigation_altitude_m = std::clamp(config.min_navigation_altitude_m, 0.0,
                                                std::abs(config.cruise_altitude_m));
  config.takeoff_hover_s = std::clamp(config.takeoff_hover_s, 0.0, 30.0);
  config.acceptance_radius_m =
      boundedFiniteDouble(config.acceptance_radius_m, 1.5, 0.0, 100.0);
  config.turn_preview_distance_m =
      boundedFiniteDouble(config.turn_preview_distance_m, 32.0, 0.0, 500.0);
  config.command_resend_period_s =
      boundedFiniteDouble(config.command_resend_period_s, 2.0, 0.05, 60.0);
  config.trajectory_update_max_start_cross_track_m = boundedFiniteDouble(
      config.trajectory_update_max_start_cross_track_m, 8.0, 0.0, 1000.0);
  config.velocity_follower.min_turn_speed_mps =
      std::clamp(config.velocity_follower.min_turn_speed_mps, 0.0,
                 config.velocity_follower.cruise_speed_mps);
  config.velocity_follower.speed_profile_lookahead_max_m =
      std::max(config.velocity_follower.speed_profile_lookahead_max_m,
               config.velocity_follower.speed_profile_lookahead_min_m);
  config.velocity_follower.curvature_feedforward_deadband_angle_rad =
      boundedFiniteDouble(
          config.velocity_follower.curvature_feedforward_deadband_angle_rad,
          2.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi / 2.0);
  config.velocity_follower.curvature_feedforward_full_angle_rad = std::max(
      config.velocity_follower.curvature_feedforward_deadband_angle_rad,
      boundedFiniteDouble(config.velocity_follower.curvature_feedforward_full_angle_rad,
                          8.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi / 2.0));
  config.velocity_follower.speed_aware_derivative_damping_min_speed_mps =
      boundedFiniteDouble(
          config.velocity_follower.speed_aware_derivative_damping_min_speed_mps, 8.0,
          0.0, 1000.0);
  config.velocity_follower.speed_aware_derivative_damping_full_speed_mps = std::max(
      config.velocity_follower.speed_aware_derivative_damping_min_speed_mps,
      boundedFiniteDouble(
          config.velocity_follower.speed_aware_derivative_damping_full_speed_mps, 20.0,
          0.0, 1000.0));
  config.velocity_follower.speed_aware_derivative_damping_max_factor =
      boundedFiniteDouble(
          config.velocity_follower.speed_aware_derivative_damping_max_factor, 1.5, 1.0,
          100.0);
  config.velocity_follower.lateral_smoothing_min_speed_mps = boundedFiniteDouble(
      config.velocity_follower.lateral_smoothing_min_speed_mps, 8.0, 0.0, 1000.0);
  config.velocity_follower.lateral_smoothing_full_speed_mps = std::max(
      config.velocity_follower.lateral_smoothing_min_speed_mps,
      boundedFiniteDouble(config.velocity_follower.lateral_smoothing_full_speed_mps,
                          20.0, 0.0, 1000.0));
  config.velocity_follower.lateral_smoothing_max_factor = boundedFiniteDouble(
      config.velocity_follower.lateral_smoothing_max_factor, 1.0, 1.0, 100.0);
  config.velocity_follower.control_tangent_smoothing_back_m = boundedFiniteDouble(
      config.velocity_follower.control_tangent_smoothing_back_m, 8.0, 0.0, 1000.0);
  config.velocity_follower.control_tangent_smoothing_forward_m = boundedFiniteDouble(
      config.velocity_follower.control_tangent_smoothing_forward_m, 18.0, 0.0, 1000.0);
  config.velocity_follower.control_tangent_smoothing_max_heading_span_rad =
      boundedFiniteDouble(
          config.velocity_follower.control_tangent_smoothing_max_heading_span_rad,
          12.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  config.velocity_follower.control_tangent_smoothing_max_abs_curvature_1pm =
      boundedFiniteDouble(
          config.velocity_follower.control_tangent_smoothing_max_abs_curvature_1pm,
          0.015, 0.0, 1000.0);
  config.velocity_follower.final_acceptance_radius_m = config.acceptance_radius_m;
}

[[nodiscard]] Px4OffboardNodeConfig loadPx4OffboardNodeConfig(rclcpp::Node& node) {
  Px4OffboardNodeConfig config{};
  config.cruise_altitude_m =
      node.declare_parameter<double>("cruise_altitude_m", config.cruise_altitude_m);
  config.min_navigation_altitude_m = node.declare_parameter<double>(
      "min_navigation_altitude_m", config.min_navigation_altitude_m);
  config.takeoff_hover_s =
      node.declare_parameter<double>("takeoff_hover_s", config.takeoff_hover_s);
  config.acceptance_radius_m =
      node.declare_parameter<double>("acceptance_radius_m", config.acceptance_radius_m);
  config.turn_preview_distance_m = node.declare_parameter<double>(
      "turn_preview_distance_m", config.turn_preview_distance_m);
  config.max_clearance_grid_staleness_ns = secondsToNanoseconds(std::clamp<double>(
      node.declare_parameter<double>("max_clearance_grid_staleness_s", 1.5), 0.0,
      3600.0));
  config.max_pose_staleness_ns = secondsToNanoseconds(boundedFiniteDouble(
      node.declare_parameter<double>("max_pose_staleness_s", 1.0), 1.0, 0.0, 3600.0));

  config.velocity_follower.cruise_speed_mps =
      std::clamp(node.declare_parameter<double>("cruise_speed_mps", 12.0), 0.0, 100.0);
  config.velocity_follower.min_turn_speed_mps =
      std::clamp(node.declare_parameter<double>("min_turn_speed_mps", 2.0), 0.0,
                 config.velocity_follower.cruise_speed_mps);
  config.velocity_follower.max_accel_mps2 =
      std::clamp(node.declare_parameter<double>("max_accel_mps2", 3.0), 0.0, 100.0);
  config.velocity_follower.max_decel_mps2 =
      std::clamp(node.declare_parameter<double>("max_decel_mps2", 4.0), 0.0, 100.0);
  config.velocity_follower.max_lateral_accel_mps2 = std::clamp(
      node.declare_parameter<double>("max_lateral_accel_mps2", 3.0), 0.0, 100.0);
  config.velocity_follower.speed_profile_decel_mps2 = std::clamp(
      node.declare_parameter<double>("speed_profile_decel_mps2", 2.0), 0.0, 100.0);
  config.velocity_follower.speed_profile_sample_step_m = std::clamp(
      node.declare_parameter<double>("speed_profile_sample_step_m", 1.0), 0.1, 10.0);
  config.velocity_follower.speed_profile_lookahead_time_s = std::clamp(
      node.declare_parameter<double>("speed_profile_lookahead_time_s", 1.0), 0.0, 30.0);
  config.velocity_follower.speed_profile_lookahead_min_m = std::clamp(
      node.declare_parameter<double>("speed_profile_lookahead_min_m", 5.0), 0.0, 500.0);
  const double requested_speed_profile_lookahead_max_m =
      std::clamp(node.declare_parameter<double>("speed_profile_lookahead_max_m", 35.0),
                 0.0, 5000.0);
  config.velocity_follower.speed_profile_lookahead_max_m =
      std::max(requested_speed_profile_lookahead_max_m,
               config.velocity_follower.speed_profile_lookahead_min_m);
  config.velocity_follower.cross_track_gain =
      std::clamp(node.declare_parameter<double>("cross_track_gain", 0.5), 0.0, 10.0);
  config.velocity_follower.cross_track_derivative_gain = std::clamp(
      node.declare_parameter<double>("cross_track_derivative_gain", 0.5), 0.0, 10.0);
  config.velocity_follower.tracking_prediction_horizon_s = std::clamp(
      node.declare_parameter<double>("tracking_prediction_horizon_s", 0.45), 0.0, 2.0);
  config.velocity_follower.max_lateral_control_angle_rad =
      std::clamp(radiansFromDegrees(node.declare_parameter<double>(
                     "max_lateral_control_angle_deg", 55.0)),
                 0.0, std::numbers::pi / 2.0);
  config.velocity_follower.max_lateral_control_rate_mps2 = std::clamp(
      node.declare_parameter<double>("max_lateral_control_rate_mps2", 5.0), 0.0, 100.0);
  config.velocity_follower.velocity_lateral_response_accel_mps2 = std::clamp(
      node.declare_parameter<double>("velocity_lateral_response_accel_mps2", 8.0), 0.0,
      100.0);
  config.velocity_follower.curvature_feedforward_time_s = std::clamp(
      node.declare_parameter<double>("curvature_feedforward_time_s", 0.25), 0.0, 10.0);
  config.velocity_follower.curvature_feedforward_deadband_angle_rad =
      std::clamp(radiansFromDegrees(node.declare_parameter<double>(
                     "curvature_feedforward_deadband_angle_deg", 2.0)),
                 0.0, std::numbers::pi / 2.0);
  config.velocity_follower.curvature_feedforward_full_angle_rad =
      std::max(config.velocity_follower.curvature_feedforward_deadband_angle_rad,
               std::clamp(radiansFromDegrees(node.declare_parameter<double>(
                              "curvature_feedforward_full_angle_deg", 8.0)),
                          0.0, std::numbers::pi / 2.0));
  config.velocity_follower.max_curvature_feedforward_angle_rad =
      std::clamp(radiansFromDegrees(node.declare_parameter<double>(
                     "max_curvature_feedforward_angle_deg", 30.0)),
                 0.0, std::numbers::pi / 2.0);
  config.velocity_follower.max_velocity_jerk_mps3 = std::clamp(
      node.declare_parameter<double>("max_velocity_jerk_mps3", 12.0), 0.0, 1000.0);
  config.velocity_follower.max_lateral_velocity_jerk_mps3 =
      std::clamp(node.declare_parameter<double>("max_lateral_velocity_jerk_mps3", 22.0),
                 0.0, 1000.0);
  config.velocity_follower.lateral_smoothing_min_speed_mps =
      std::clamp(node.declare_parameter<double>("lateral_smoothing_min_speed_mps", 8.0),
                 0.0, 1000.0);
  config.velocity_follower.lateral_smoothing_full_speed_mps =
      std::max(config.velocity_follower.lateral_smoothing_min_speed_mps,
               std::clamp(node.declare_parameter<double>(
                              "lateral_smoothing_full_speed_mps", 20.0),
                          0.0, 1000.0));
  config.velocity_follower.lateral_smoothing_max_factor = std::clamp(
      node.declare_parameter<double>("lateral_smoothing_max_factor", 1.0), 1.0, 100.0);
  config.velocity_follower.speed_aware_derivative_damping_min_speed_mps =
      std::clamp(node.declare_parameter<double>(
                     "speed_aware_derivative_damping_min_speed_mps", 8.0),
                 0.0, 1000.0);
  config.velocity_follower.speed_aware_derivative_damping_full_speed_mps =
      std::max(config.velocity_follower.speed_aware_derivative_damping_min_speed_mps,
               std::clamp(node.declare_parameter<double>(
                              "speed_aware_derivative_damping_full_speed_mps", 20.0),
                          0.0, 1000.0));
  config.velocity_follower.speed_aware_derivative_damping_max_factor = std::clamp(
      node.declare_parameter<double>("speed_aware_derivative_damping_max_factor", 1.5),
      1.0, 100.0);
  config.velocity_follower.control_tangent_smoothing_back_m = std::clamp(
      node.declare_parameter<double>("control_tangent_smoothing_back_m", 8.0), 0.0,
      1000.0);
  config.velocity_follower.control_tangent_smoothing_forward_m = std::clamp(
      node.declare_parameter<double>("control_tangent_smoothing_forward_m", 18.0), 0.0,
      1000.0);
  config.velocity_follower.control_tangent_smoothing_max_heading_span_rad =
      std::clamp(radiansFromDegrees(node.declare_parameter<double>(
                     "control_tangent_smoothing_max_heading_span_deg", 12.0)),
                 0.0, std::numbers::pi);
  config.velocity_follower.control_tangent_smoothing_max_abs_curvature_1pm =
      std::clamp(node.declare_parameter<double>(
                     "control_tangent_smoothing_max_abs_curvature_1pm", 0.015),
                 0.0, 1000.0);
  config.velocity_follower.adaptive_lateral_response_scale_m = std::clamp(
      node.declare_parameter<double>("adaptive_lateral_response_scale_m", 3.0), 0.1,
      1000.0);
  config.velocity_follower.adaptive_lateral_response_max_factor = std::clamp(
      node.declare_parameter<double>("adaptive_lateral_response_max_factor", 1.2), 1.0,
      100.0);
  config.velocity_follower.final_hold_max_speed_mps = std::clamp(
      node.declare_parameter<double>("final_hold_max_speed_mps", 0.8), 0.0, 100.0);
  config.trajectory_update_max_start_cross_track_m = std::clamp(
      node.declare_parameter<double>("trajectory_update_max_start_cross_track_m",
                                     config.trajectory_update_max_start_cross_track_m),
      0.0, 1000.0);

  config.topics.final_trajectory_debug = node.declare_parameter<std::string>(
      "final_trajectory_debug_topic", config.topics.final_trajectory_debug);
  config.final_trajectory_debug_sample_step_m = std::clamp(
      node.declare_parameter<double>("final_trajectory_debug_sample_step_m",
                                     config.final_trajectory_debug_sample_step_m),
      0.1, 20.0);
  config.topics.offboard_debug_marker = node.declare_parameter<std::string>(
      "offboard_debug_marker_topic", config.topics.offboard_debug_marker);
  config.altitude_hold_kp =
      std::clamp(node.declare_parameter<double>("altitude_hold_kp", 0.5), 0.0, 10.0);
  config.max_vertical_speed_mps = std::clamp(
      node.declare_parameter<double>("max_vertical_speed_mps", 2.0), 0.0, 20.0);
  config.telemetry_log_period_ns = secondsToNanoseconds(std::clamp(
      node.declare_parameter<double>("telemetry_log_period_s", 0.5), 0.1, 60.0));
  config.flight_blackbox_enabled = node.declare_parameter<bool>(
      "flight_blackbox_enabled", config.flight_blackbox_enabled);
  config.flight_blackbox_path = node.declare_parameter<std::string>(
      "flight_blackbox_path", config.flight_blackbox_path);
  config.warmup_setpoints = static_cast<int>(std::clamp<std::int64_t>(
      node.declare_parameter<std::int64_t>("warmup_setpoints", 20), 1, 100000));
  config.auto_arm = node.declare_parameter<bool>("auto_arm", config.auto_arm);
  config.auto_offboard =
      node.declare_parameter<bool>("auto_offboard", config.auto_offboard);
  const double requested_command_resend_period_s = node.declare_parameter<double>(
      "command_resend_period_s", config.command_resend_period_s);
  config.command_resend_period_s =
      boundedFiniteDouble(requested_command_resend_period_s, 2.0, 0.05, 60.0);
  if (!std::isfinite(requested_command_resend_period_s) ||
      requested_command_resend_period_s != config.command_resend_period_s) {
    RCLCPP_WARN(node.get_logger(),
                "Sanitized command_resend_period_s: requested=%.3f final=%.3f "
                "allowed_range=[0.050, 60.000]",
                requested_command_resend_period_s, config.command_resend_period_s);
  }
  config.px4_local_origin =
      Point2{node.declare_parameter<double>("px4_local_origin_x_m", 0.0),
             node.declare_parameter<double>("px4_local_origin_y_m", 0.0)};
  config.mission_goal = Point2{node.declare_parameter<double>("goal_x_m", 85.0),
                               node.declare_parameter<double>("goal_y_m", 0.0)};
  config.hold_x_m = node.declare_parameter<double>("hold_x_m", 0.0);
  config.hold_y_m = node.declare_parameter<double>("hold_y_m", 0.0);
  config.target_system =
      boundedUint8(node.declare_parameter<std::int64_t>("target_system", 1));
  config.target_component =
      boundedUint8(node.declare_parameter<std::int64_t>("target_component", 1));
  config.source_system =
      boundedUint8(node.declare_parameter<std::int64_t>("source_system", 1));
  config.source_component =
      boundedUint16(node.declare_parameter<std::int64_t>("source_component", 1));

  config.topics.path =
      node.declare_parameter<std::string>("path_topic", config.topics.path);
  config.topics.path_id =
      node.declare_parameter<std::string>("path_id_topic", config.topics.path_id);
  config.topics.trajectory_diagnostics = node.declare_parameter<std::string>(
      "trajectory_diagnostics_topic", config.topics.trajectory_diagnostics);
  config.topics.px4_local_position = node.declare_parameter<std::string>(
      "px4_local_position_topic", config.topics.px4_local_position);
  config.topics.px4_vehicle_attitude = node.declare_parameter<std::string>(
      "px4_vehicle_attitude_topic", config.topics.px4_vehicle_attitude);
  config.topics.px4_vehicle_status = node.declare_parameter<std::string>(
      "px4_vehicle_status_topic", config.topics.px4_vehicle_status);
  config.topics.emergency_stop = node.declare_parameter<std::string>(
      "emergency_stop_topic", config.topics.emergency_stop);
  config.topics.prohibited_grid = node.declare_parameter<std::string>(
      "prohibited_grid_topic", config.topics.prohibited_grid);
  config.topics.offboard_control_mode = node.declare_parameter<std::string>(
      "offboard_control_mode_topic", config.topics.offboard_control_mode);
  config.topics.trajectory_setpoint = node.declare_parameter<std::string>(
      "trajectory_setpoint_topic", config.topics.trajectory_setpoint);
  config.topics.vehicle_command = node.declare_parameter<std::string>(
      "vehicle_command_topic", config.topics.vehicle_command);

  sanitizePx4OffboardNodeConfig(config);
  return config;
}

} // namespace drone_city_nav
