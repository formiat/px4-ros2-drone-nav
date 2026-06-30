#include "drone_city_nav/offboard_velocity_follower.hpp"

#include "drone_city_nav/velocity_command_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

struct ControlTangentSmoothingDiagnostics {
  bool applied{false};
  Point2 raw_tangent{};
  double heading_span_rad{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_curvature_1pm{std::numeric_limits<double>::quiet_NaN()};
  double window_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double window_end_s_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double normalizedAngle(double angle_rad) noexcept {
  angle_rad = std::fmod(angle_rad, kTwoPi);
  if (angle_rad < 0.0) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

[[nodiscard]] double headingSpanRad(std::vector<double> angles_rad) {
  if (angles_rad.size() < 2U) {
    return 0.0;
  }
  for (double& angle_rad : angles_rad) {
    angle_rad = normalizedAngle(angle_rad);
  }
  std::ranges::sort(angles_rad);

  double max_gap = 0.0;
  for (std::size_t i = 0U; i + 1U < angles_rad.size(); ++i) {
    max_gap = std::max(max_gap, angles_rad[i + 1U] - angles_rad[i]);
  }
  max_gap = std::max(max_gap, angles_rad.front() + kTwoPi - angles_rad.back());
  return kTwoPi - max_gap;
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double sanitizedCruiseSpeed(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
}

[[nodiscard]] double
effectiveSpeedProfileDecelMps2(const VelocityFollowerConfig& config) {
  const double fallback = sanitizedPositive(config.max_decel_mps2, 4.0, 1.0e-6, 100.0);
  return sanitizedPositive(config.speed_profile_decel_mps2, fallback, 1.0e-6, 100.0);
}

[[nodiscard]] double finalHoldMaxSpeedMps(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.final_hold_max_speed_mps, 0.8, 0.0, 100.0);
}

[[nodiscard]] double terminalCaptureRadiusM(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.terminal_capture_radius_m, 8.0, 0.0, 1000.0);
}

[[nodiscard]] double terminalCaptureGain1ps(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.terminal_capture_gain_1ps, 1.0, 0.0, 100.0);
}

[[nodiscard]] double terminalCaptureMaxSpeedMps(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.terminal_capture_max_speed_mps, 4.0, 0.0, 100.0);
}

[[nodiscard]] double
trackingPredictionHorizonS(const VelocityFollowerConfig& config) noexcept {
  return sanitizedPositive(config.tracking_prediction_horizon_s, 0.45, 0.0, 2.0);
}

[[nodiscard]] Point2 predictedTrackingPosition(const Point2 current_position,
                                               const Point2 current_velocity,
                                               const bool current_velocity_valid,
                                               const double horizon_s) noexcept {
  if (!(horizon_s > 0.0) || !current_velocity_valid || !finite2D(current_velocity)) {
    return current_position;
  }
  return current_position + current_velocity * horizon_s;
}

[[nodiscard]] double currentSpeedMps(const Point2 current_velocity,
                                     const bool current_velocity_valid,
                                     const VelocityFollowerState& previous_state) {
  if (current_velocity_valid && finite2D(current_velocity)) {
    return norm(current_velocity);
  }
  if (previous_state.previous_velocity_setpoint_valid &&
      finite2D(previous_state.previous_velocity_setpoint)) {
    return norm(previous_state.previous_velocity_setpoint);
  }
  return 0.0;
}

[[nodiscard]] double
previousCommandSpeedMps(const VelocityFollowerState& previous_state,
                        const double current_speed_mps) {
  if (previous_state.previous_scalar_speed_command_valid &&
      std::isfinite(previous_state.previous_scalar_speed_command_mps)) {
    return std::max(0.0, previous_state.previous_scalar_speed_command_mps);
  }
  return current_speed_mps;
}

[[nodiscard]] std::optional<TrajectoryPointSample>
sampleAtS(const std::span<const TrajectoryPointSample> samples, const double s_m) {
  if (!trajectorySamplesAreUsable(samples)) {
    return std::nullopt;
  }
  const double clamped_s = std::clamp(std::isfinite(s_m) ? s_m : 0.0,
                                      samples.front().s_m, samples.back().s_m);
  if (clamped_s <= samples.front().s_m) {
    return samples.front();
  }
  if (clamped_s >= samples.back().s_m) {
    return samples.back();
  }

  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i];
    const TrajectoryPointSample& end = samples[i + 1U];
    if (clamped_s < start.s_m || clamped_s > end.s_m) {
      continue;
    }
    const double station_delta_m = end.s_m - start.s_m;
    if (!(station_delta_m > kTinyDistanceM)) {
      continue;
    }
    const double t = std::clamp((clamped_s - start.s_m) / station_delta_m, 0.0, 1.0);
    TrajectoryPointSample sample{};
    sample.s_m = clamped_s;
    sample.point = start.point * (1.0 - t) + end.point * t;
    sample.tangent = normalized(start.tangent * (1.0 - t) + end.tangent * t);
    if (!(norm(sample.tangent) > kTinyDistanceM)) {
      sample.tangent = normalized(end.point - start.point);
    }
    sample.curvature_1pm = start.curvature_1pm * (1.0 - t) + end.curvature_1pm * t;
    return sample;
  }

  return std::nullopt;
}

[[nodiscard]] ControlTangentSmoothingDiagnostics
smoothControlTangentIfStraightish(const std::span<const TrajectoryPointSample> samples,
                                  TrajectoryProjection& control_projection,
                                  const VelocityFollowerConfig& config) {
  ControlTangentSmoothingDiagnostics diagnostics{};
  diagnostics.raw_tangent = control_projection.tangent;
  if (!control_projection.valid ||
      !(norm(control_projection.tangent) > kTinyDistanceM) ||
      !trajectorySamplesAreUsable(samples)) {
    return diagnostics;
  }

  const double back_m =
      sanitizedPositive(config.control_tangent_smoothing_back_m, 8.0, 0.0, 1000.0);
  const double forward_m =
      sanitizedPositive(config.control_tangent_smoothing_forward_m, 18.0, 0.0, 1000.0);
  if (!(back_m + forward_m > kTinyDistanceM)) {
    return diagnostics;
  }

  diagnostics.window_start_s_m =
      std::max(samples.front().s_m, control_projection.s_m - back_m);
  diagnostics.window_end_s_m =
      std::min(samples.back().s_m, control_projection.s_m + forward_m);
  if (!(diagnostics.window_end_s_m - diagnostics.window_start_s_m > kTinyDistanceM)) {
    return diagnostics;
  }

  const std::optional<TrajectoryPointSample> start_sample =
      sampleAtS(samples, diagnostics.window_start_s_m);
  const std::optional<TrajectoryPointSample> end_sample =
      sampleAtS(samples, diagnostics.window_end_s_m);
  if (!start_sample.has_value() || !end_sample.has_value()) {
    return diagnostics;
  }

  std::vector<double> headings_rad;
  headings_rad.reserve(samples.size() + 2U);
  auto add_sample = [&](const TrajectoryPointSample& sample) {
    if (norm(sample.tangent) > kTinyDistanceM && std::isfinite(sample.curvature_1pm)) {
      headings_rad.push_back(std::atan2(sample.tangent.y, sample.tangent.x));
      diagnostics.max_abs_curvature_1pm =
          std::max(std::isfinite(diagnostics.max_abs_curvature_1pm)
                       ? diagnostics.max_abs_curvature_1pm
                       : 0.0,
                   std::abs(sample.curvature_1pm));
    }
  };
  add_sample(*start_sample);
  for (const TrajectoryPointSample& sample : samples) {
    if (sample.s_m > diagnostics.window_start_s_m &&
        sample.s_m < diagnostics.window_end_s_m) {
      add_sample(sample);
    }
  }
  add_sample(*end_sample);
  diagnostics.heading_span_rad = headingSpanRad(std::move(headings_rad));

  const double max_heading_span_rad =
      sanitizedPositive(config.control_tangent_smoothing_max_heading_span_rad,
                        12.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  const double max_abs_curvature = sanitizedPositive(
      config.control_tangent_smoothing_max_abs_curvature_1pm, 0.015, 0.0, 1000.0);
  if (!(diagnostics.heading_span_rad <= max_heading_span_rad) ||
      !(diagnostics.max_abs_curvature_1pm <= max_abs_curvature)) {
    return diagnostics;
  }

  const Point2 smoothed_tangent = normalized(end_sample->point - start_sample->point);
  if (!(norm(smoothed_tangent) > kTinyDistanceM) ||
      dot(smoothed_tangent, control_projection.tangent) <= 0.0) {
    return diagnostics;
  }

  control_projection.tangent = smoothed_tangent;
  diagnostics.applied = true;
  return diagnostics;
}

} // namespace

const char* velocitySetpointReasonName(const VelocitySetpointReason reason) noexcept {
  switch (reason) {
    case VelocitySetpointReason::kInvalidPath:
      return "invalid_path";
    case VelocitySetpointReason::kHold:
      return "hold";
    case VelocitySetpointReason::kStraight:
      return "straight";
    case VelocitySetpointReason::kTrajectorySpeedProfile:
      return "trajectory_profile";
    case VelocitySetpointReason::kFinalApproach:
      return "final_approach";
    case VelocitySetpointReason::kTerminalCapture:
      return "terminal_capture";
  }
  return "unknown";
}

namespace {

void populateVelocityBasisErrors(VelocitySetpointPlan& plan) {
  if (std::isfinite(plan.desired_velocity_tangent_mps) &&
      std::isfinite(plan.setpoint_velocity_tangent_mps)) {
    plan.desired_to_setpoint_tangent_error_mps =
        plan.desired_velocity_tangent_mps - plan.setpoint_velocity_tangent_mps;
  }
  if (std::isfinite(plan.desired_velocity_normal_mps) &&
      std::isfinite(plan.setpoint_velocity_normal_mps)) {
    plan.desired_to_setpoint_normal_error_mps =
        plan.desired_velocity_normal_mps - plan.setpoint_velocity_normal_mps;
  }
  if (std::isfinite(plan.setpoint_velocity_tangent_mps) &&
      std::isfinite(plan.current_velocity_tangent_mps)) {
    plan.setpoint_to_actual_tangent_error_mps =
        plan.setpoint_velocity_tangent_mps - plan.current_velocity_tangent_mps;
  }
  if (std::isfinite(plan.setpoint_velocity_normal_mps) &&
      std::isfinite(plan.current_velocity_normal_mps)) {
    plan.setpoint_to_actual_normal_error_mps =
        plan.setpoint_velocity_normal_mps - plan.current_velocity_normal_mps;
  }
  if (std::isfinite(plan.desired_velocity_tangent_mps) &&
      std::isfinite(plan.current_velocity_tangent_mps)) {
    plan.desired_to_actual_tangent_error_mps =
        plan.desired_velocity_tangent_mps - plan.current_velocity_tangent_mps;
  }
  if (std::isfinite(plan.desired_velocity_normal_mps) &&
      std::isfinite(plan.current_velocity_normal_mps)) {
    plan.desired_to_actual_normal_error_mps =
        plan.desired_velocity_normal_mps - plan.current_velocity_normal_mps;
  }
}

VelocitySetpointPlan planVelocitySetpointFromProjection(
    const TrajectoryProjection& control_projection,
    const TrajectoryProjection& current_projection, const Point2 predicted_position,
    const double prediction_horizon_s, const Point2 final_point,
    const double trajectory_length_m, const TrajectorySegmentKind segment_kind,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config,
    const ControlTangentSmoothingDiagnostics& tangent_smoothing) {
  VelocitySetpointPlan plan{};
  if (!finite2D(current_position) || !finite2D(predicted_position) ||
      !finite2D(final_point) || !control_projection.valid ||
      !current_projection.valid ||
      !(norm(control_projection.tangent) > kTinyDistanceM) ||
      !std::isfinite(trajectory_length_m) || !speed_profile.valid ||
      speed_profile.samples.empty()) {
    return plan;
  }

  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const double final_acceptance =
      sanitizedPositive(config.final_acceptance_radius_m, 1.0, 0.0, 100.0);
  const double current_speed =
      currentSpeedMps(current_velocity, current_velocity_valid, previous_state);
  const double bounded_prediction_horizon = std::max(0.0, prediction_horizon_s);
  const double prediction_distance = distance(current_position, predicted_position);
  const double response_delay_distance = current_speed * bounded_prediction_horizon;
  const double terminal_goal_distance = distance(current_position, final_point);
  const double remaining_trajectory_distance =
      std::max(0.0, trajectory_length_m - std::max(0.0, control_projection.s_m));
  const double terminal_capture_radius = terminalCaptureRadiusM(config);
  const double terminal_hold_max_speed = finalHoldMaxSpeedMps(config);
  const bool terminal_hold_distance_met = terminal_goal_distance <= final_acceptance;
  const bool terminal_hold_speed_met = current_speed <= terminal_hold_max_speed;
  const bool terminal_capture_goal_distance_triggered =
      terminal_goal_distance <= terminal_capture_radius;
  const bool terminal_capture_remaining_distance_triggered =
      remaining_trajectory_distance <= terminal_capture_radius;
  if (terminal_hold_distance_met && terminal_hold_speed_met) {
    plan.valid = true;
    plan.final_goal_reached = true;
    plan.reason = VelocitySetpointReason::kHold;
    plan.projection = final_point;
    plan.current_projection = current_projection.point;
    plan.predicted_position = predicted_position;
    plan.predicted_projection = control_projection.point;
    plan.prediction_horizon_s = bounded_prediction_horizon;
    plan.prediction_distance_m = prediction_distance;
    plan.response_delay_distance_m = response_delay_distance;
    plan.terminal_goal_distance_m = terminal_goal_distance;
    plan.terminal_remaining_trajectory_distance_m = remaining_trajectory_distance;
    plan.terminal_acceptance_radius_m = final_acceptance;
    plan.terminal_hold_max_speed_mps = terminal_hold_max_speed;
    plan.terminal_hold_distance_met = terminal_hold_distance_met;
    plan.terminal_hold_speed_met = terminal_hold_speed_met;
    plan.terminal_capture_goal_distance_triggered =
        terminal_capture_goal_distance_triggered;
    plan.terminal_capture_remaining_distance_triggered =
        terminal_capture_remaining_distance_triggered;
    plan.current_cross_track_error_m = std::sqrt(current_projection.distance_sq);
    plan.predicted_cross_track_error_m = std::sqrt(control_projection.distance_sq);
    plan.trajectory_cross_track_error_m = plan.predicted_cross_track_error_m;
    plan.trajectory_s_m = control_projection.s_m;
    plan.trajectory_segment_index = control_projection.segment_index;
    plan.trajectory_segment_kind = segment_kind;
    plan.trajectory_curvature_1pm = control_projection.curvature_1pm;
    plan.trajectory_projection = control_projection;
    plan.control_tangent_raw = tangent_smoothing.raw_tangent;
    plan.control_tangent_smoothed = tangent_smoothing.applied;
    plan.control_tangent_smoothing_heading_span_rad =
        tangent_smoothing.heading_span_rad;
    plan.control_tangent_smoothing_max_abs_curvature_1pm =
        tangent_smoothing.max_abs_curvature_1pm;
    plan.control_tangent_smoothing_window_start_s_m =
        tangent_smoothing.window_start_s_m;
    plan.control_tangent_smoothing_window_end_s_m = tangent_smoothing.window_end_s_m;
    return plan;
  }

  if (terminal_capture_goal_distance_triggered ||
      terminal_capture_remaining_distance_triggered) {
    const Point2 goal_delta = final_point - current_position;
    Point2 terminal_tangent = normalized(goal_delta);
    if (!(norm(terminal_tangent) > kTinyDistanceM)) {
      terminal_tangent = control_projection.tangent;
    }
    const double capture_gain_speed_limit =
        terminalCaptureGain1ps(config) * terminal_goal_distance;
    const double capture_max_speed = terminalCaptureMaxSpeedMps(config);
    const double capture_speed_limit =
        terminal_goal_distance <= final_acceptance
            ? 0.0
            : std::min(capture_max_speed, capture_gain_speed_limit);
    const Point2 desired_velocity = terminal_tangent * capture_speed_limit;
    const VelocitySmootherPlan smoothed = smoothVelocityCommand(
        VelocitySmootherInput{
            .desired_velocity_xy = desired_velocity,
            .path_tangent = terminal_tangent,
            .previous_velocity_setpoint = previous_state.previous_velocity_setpoint,
            .previous_velocity_acceleration_setpoint =
                previous_state.previous_velocity_acceleration_setpoint,
            .previous_velocity_setpoint_valid =
                previous_state.previous_velocity_setpoint_valid,
            .previous_velocity_acceleration_setpoint_valid =
                previous_state.previous_velocity_acceleration_setpoint_valid,
            .dt_s = dt,
            .lateral_response_factor = 1.0},
        config);
    if (!smoothed.valid) {
      return plan;
    }

    const Point2 left_normal{-terminal_tangent.y, terminal_tangent.x};
    plan.valid = true;
    plan.reason = VelocitySetpointReason::kTerminalCapture;
    plan.terminal_capture_active = true;
    plan.terminal_goal_distance_m = terminal_goal_distance;
    plan.terminal_remaining_trajectory_distance_m = remaining_trajectory_distance;
    plan.terminal_acceptance_radius_m = final_acceptance;
    plan.terminal_hold_max_speed_mps = terminal_hold_max_speed;
    plan.terminal_hold_distance_met = terminal_hold_distance_met;
    plan.terminal_hold_speed_met = terminal_hold_speed_met;
    plan.terminal_capture_goal_distance_triggered =
        terminal_capture_goal_distance_triggered;
    plan.terminal_capture_remaining_distance_triggered =
        terminal_capture_remaining_distance_triggered;
    plan.terminal_capture_gain_speed_limit_mps = capture_gain_speed_limit;
    plan.terminal_capture_max_speed_mps = capture_max_speed;
    plan.terminal_capture_speed_limit_mps = capture_speed_limit;
    plan.velocity_xy = smoothed.velocity_xy;
    plan.desired_velocity_xy = desired_velocity;
    plan.speed_mps = norm(plan.velocity_xy);
    plan.desired_speed_mps = norm(plan.desired_velocity_xy);
    plan.final_command_speed_mps = plan.speed_mps;
    plan.velocity_setpoint_acceleration_xy = smoothed.velocity_setpoint_acceleration_xy;
    plan.velocity_setpoint_acceleration_mps2 =
        smoothed.velocity_setpoint_acceleration_mps2;
    plan.velocity_setpoint_jerk_mps3 = smoothed.velocity_setpoint_jerk_mps3;
    plan.path_frame_lateral_smoothing_applied =
        smoothed.path_frame_lateral_smoothing_applied;
    plan.lateral_smoothing_factor = smoothed.lateral_smoothing_factor;
    plan.smoother_lateral_response_accel_mps2 =
        smoothed.smoother_lateral_response_accel_mps2;
    plan.path_tangent = terminal_tangent;
    plan.control_tangent_raw = tangent_smoothing.raw_tangent;
    plan.control_tangent_smoothed = tangent_smoothing.applied;
    plan.control_tangent_smoothing_heading_span_rad =
        tangent_smoothing.heading_span_rad;
    plan.control_tangent_smoothing_max_abs_curvature_1pm =
        tangent_smoothing.max_abs_curvature_1pm;
    plan.control_tangent_smoothing_window_start_s_m =
        tangent_smoothing.window_start_s_m;
    plan.control_tangent_smoothing_window_end_s_m = tangent_smoothing.window_end_s_m;
    plan.projection = final_point;
    plan.current_projection = current_projection.point;
    plan.predicted_position = predicted_position;
    plan.predicted_projection = control_projection.point;
    plan.raw_speed_limit_mps = capture_speed_limit;
    plan.profile_speed_limit_mps = capture_speed_limit;
    plan.speed_after_lookahead_mps = capture_speed_limit;
    plan.cross_track_limited_speed_mps = capture_speed_limit;
    plan.accel_limited_speed_mps = plan.speed_mps;
    plan.velocity_delta_mps = smoothed.velocity_delta_mps;
    plan.desired_velocity_delta_mps = smoothed.desired_velocity_delta_mps;
    plan.velocity_tracking_error_mps =
        current_velocity_valid && finite2D(current_velocity)
            ? norm(plan.velocity_xy - current_velocity)
            : std::numeric_limits<double>::quiet_NaN();
    plan.current_velocity_tangent_mps =
        current_velocity_valid && finite2D(current_velocity)
            ? dot(current_velocity, terminal_tangent)
            : std::numeric_limits<double>::quiet_NaN();
    plan.current_velocity_normal_mps =
        current_velocity_valid && finite2D(current_velocity)
            ? dot(current_velocity, left_normal)
            : std::numeric_limits<double>::quiet_NaN();
    plan.desired_velocity_tangent_mps = dot(plan.desired_velocity_xy, terminal_tangent);
    plan.desired_velocity_normal_mps = dot(plan.desired_velocity_xy, left_normal);
    plan.setpoint_velocity_tangent_mps = dot(plan.velocity_xy, terminal_tangent);
    plan.setpoint_velocity_normal_mps = dot(plan.velocity_xy, left_normal);
    populateVelocityBasisErrors(plan);
    plan.trajectory_cross_track_error_m = std::sqrt(control_projection.distance_sq);
    plan.current_cross_track_error_m = std::sqrt(current_projection.distance_sq);
    plan.predicted_cross_track_error_m = std::sqrt(control_projection.distance_sq);
    plan.prediction_horizon_s = bounded_prediction_horizon;
    plan.prediction_distance_m = prediction_distance;
    plan.response_delay_distance_m = response_delay_distance;
    plan.limiting_constraint_type = SpeedConstraintType::kGoal;
    plan.limiting_constraint_distance_m = terminal_goal_distance;
    plan.limiting_constraint_speed_mps = 0.0;
    plan.limiting_allowed_speed_now_mps = capture_speed_limit;
    plan.trajectory_s_m = control_projection.s_m;
    plan.trajectory_segment_index = control_projection.segment_index;
    plan.trajectory_segment_kind = segment_kind;
    plan.trajectory_curvature_1pm = control_projection.curvature_1pm;
    plan.trajectory_arc_radius_m =
        std::abs(plan.trajectory_curvature_1pm) > kTinyDistanceM
            ? 1.0 / std::abs(plan.trajectory_curvature_1pm)
            : std::numeric_limits<double>::quiet_NaN();
    plan.trajectory_projection = control_projection;
    plan.final_stop.valid = true;
    plan.final_stop.distance_to_stop_m = terminal_goal_distance;
    plan.final_stop.braking_distance_m =
        response_delay_distance +
        current_speed * current_speed / (2.0 * effectiveSpeedProfileDecelMps2(config));
    plan.final_stop.raw_speed_limit_mps = capture_speed_limit;
    return plan;
  }

  const ScalarSpeedPlan scalar_speed = planScalarSpeed(
      speed_profile,
      ScalarSpeedQuery{.trajectory_s_m = control_projection.s_m,
                       .previous_command_speed_mps =
                           previousCommandSpeedMps(previous_state, current_speed),
                       .current_speed_mps = current_speed,
                       .dt_s = dt},
      config);
  if (!scalar_speed.valid) {
    return plan;
  }

  const VelocityCommandPlan command = planVelocityCommand(
      VelocityCommandQuery{
          .projection = control_projection,
          .current_position = predicted_position,
          .current_velocity = current_velocity,
          .current_velocity_valid = current_velocity_valid,
          .scalar_speed_mps = scalar_speed.final_scalar_speed_mps,
          .dt_s = dt,
          .previous_lateral_control_velocity =
              previous_state.previous_lateral_control_velocity,
          .previous_lateral_control_velocity_valid =
              previous_state.previous_lateral_control_velocity_valid,
          .current_cross_track_error_m = std::sqrt(current_projection.distance_sq),
          .predicted_cross_track_error_m = std::sqrt(control_projection.distance_sq)},
      config);
  if (!command.valid) {
    return plan;
  }

  const VelocitySmootherPlan smoothed = smoothVelocityCommand(
      VelocitySmootherInput{
          .desired_velocity_xy = command.desired_velocity_xy,
          .path_tangent = control_projection.tangent,
          .previous_velocity_setpoint = previous_state.previous_velocity_setpoint,
          .previous_velocity_acceleration_setpoint =
              previous_state.previous_velocity_acceleration_setpoint,
          .previous_velocity_setpoint_valid =
              previous_state.previous_velocity_setpoint_valid,
          .previous_velocity_acceleration_setpoint_valid =
              previous_state.previous_velocity_acceleration_setpoint_valid,
          .dt_s = dt,
          .lateral_response_factor = command.adaptive_lateral_response_factor},
      config);
  if (!smoothed.valid) {
    return plan;
  }

  const double cruise_speed = sanitizedCruiseSpeed(config);
  plan.valid = true;
  plan.reason = VelocitySetpointReason::kStraight;
  if (scalar_speed.constraint_type == SpeedConstraintType::kArc &&
      scalar_speed.cross_track_limited_speed_mps + 1.0e-6 < cruise_speed) {
    plan.reason = VelocitySetpointReason::kTrajectorySpeedProfile;
  } else if (scalar_speed.constraint_type == SpeedConstraintType::kGoal &&
             scalar_speed.cross_track_limited_speed_mps + 1.0e-6 < cruise_speed) {
    plan.reason = VelocitySetpointReason::kFinalApproach;
  }

  plan.velocity_xy = smoothed.velocity_xy;
  plan.desired_velocity_xy = command.desired_velocity_xy;
  plan.speed_mps = norm(plan.velocity_xy);
  plan.final_command_speed_mps = plan.speed_mps;
  plan.desired_speed_mps = norm(plan.desired_velocity_xy);
  plan.velocity_setpoint_acceleration_xy = smoothed.velocity_setpoint_acceleration_xy;
  plan.velocity_setpoint_acceleration_mps2 =
      smoothed.velocity_setpoint_acceleration_mps2;
  plan.velocity_setpoint_jerk_mps3 = smoothed.velocity_setpoint_jerk_mps3;
  plan.path_frame_lateral_smoothing_applied =
      smoothed.path_frame_lateral_smoothing_applied;
  plan.lateral_smoothing_factor = smoothed.lateral_smoothing_factor;
  plan.smoother_lateral_response_accel_mps2 =
      smoothed.smoother_lateral_response_accel_mps2;
  plan.path_tangent = control_projection.tangent;
  plan.control_tangent_raw = tangent_smoothing.raw_tangent;
  plan.control_tangent_smoothed = tangent_smoothing.applied;
  plan.control_tangent_smoothing_heading_span_rad = tangent_smoothing.heading_span_rad;
  plan.control_tangent_smoothing_max_abs_curvature_1pm =
      tangent_smoothing.max_abs_curvature_1pm;
  plan.control_tangent_smoothing_window_start_s_m = tangent_smoothing.window_start_s_m;
  plan.control_tangent_smoothing_window_end_s_m = tangent_smoothing.window_end_s_m;
  plan.projection = control_projection.point;
  plan.current_projection = current_projection.point;
  plan.predicted_position = predicted_position;
  plan.predicted_projection = control_projection.point;
  plan.cross_track_feedback_velocity = command.cross_track_feedback_velocity;
  plan.cross_track_derivative_damping_velocity =
      command.cross_track_derivative_damping_velocity;
  plan.curvature_feedforward_velocity = command.curvature_feedforward_velocity;
  plan.raw_lateral_control_velocity = command.raw_lateral_control_velocity;
  plan.lateral_control_velocity = command.lateral_control_velocity;
  plan.raw_speed_limit_mps = scalar_speed.cross_track_limited_speed_mps;
  plan.profile_speed_limit_mps = scalar_speed.profile_speed_limit_mps;
  plan.speed_lookahead_distance_m = scalar_speed.lookahead_distance_m;
  plan.lookahead_speed_limit_mps = scalar_speed.lookahead_speed_limit_mps;
  plan.lookahead_limiting_constraint_type = scalar_speed.lookahead_constraint_type;
  plan.lookahead_limiting_constraint_index = scalar_speed.lookahead_constraint_index;
  plan.lookahead_limiting_constraint_distance_m =
      scalar_speed.lookahead_constraint_distance_m;
  plan.speed_after_lookahead_mps = scalar_speed.speed_after_lookahead_mps;
  plan.cross_track_speed_factor = scalar_speed.cross_track_speed_factor;
  plan.cross_track_limited_speed_mps = scalar_speed.cross_track_limited_speed_mps;
  plan.accel_limited_speed_mps = scalar_speed.accel_limited_speed_mps;
  plan.velocity_delta_mps = smoothed.velocity_delta_mps;
  plan.desired_velocity_delta_mps = smoothed.desired_velocity_delta_mps;
  plan.velocity_tracking_error_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? norm(plan.velocity_xy - current_velocity)
          : std::numeric_limits<double>::quiet_NaN();
  const Point2 left_normal{-control_projection.tangent.y, control_projection.tangent.x};
  plan.current_velocity_tangent_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? dot(current_velocity, control_projection.tangent)
          : std::numeric_limits<double>::quiet_NaN();
  plan.current_velocity_normal_mps =
      current_velocity_valid && finite2D(current_velocity)
          ? dot(current_velocity, left_normal)
          : std::numeric_limits<double>::quiet_NaN();
  plan.desired_velocity_tangent_mps = command.desired_velocity_tangent_mps;
  plan.desired_velocity_normal_mps = command.desired_velocity_normal_mps;
  plan.setpoint_velocity_tangent_mps =
      dot(plan.velocity_xy, control_projection.tangent);
  plan.setpoint_velocity_normal_mps = dot(plan.velocity_xy, left_normal);
  plan.cross_track_feedback_mps = command.cross_track_feedback_mps;
  plan.cross_track_derivative_damping_mps = command.cross_track_derivative_damping_mps;
  plan.cross_track_derivative_damping_factor =
      command.cross_track_derivative_damping_factor;
  plan.cross_track_derivative_gain_effective =
      command.cross_track_derivative_gain_effective;
  plan.cross_track_lateral_velocity_mps = command.cross_track_lateral_velocity_mps;
  plan.curvature_feedforward_mps = command.curvature_feedforward_mps;
  plan.curvature_feedforward_angle_rad = command.curvature_feedforward_angle_rad;
  plan.curvature_feedforward_raw_angle_rad =
      command.curvature_feedforward_raw_angle_rad;
  plan.curvature_feedforward_scale = command.curvature_feedforward_scale;
  plan.raw_lateral_control_mps = command.raw_lateral_control_mps;
  plan.lateral_control_mps = command.lateral_control_mps;
  plan.lateral_control_delta_mps = command.lateral_control_delta_mps;
  plan.adaptive_lateral_response_factor = command.adaptive_lateral_response_factor;
  plan.terminal_goal_distance_m = terminal_goal_distance;
  plan.terminal_remaining_trajectory_distance_m = remaining_trajectory_distance;
  plan.terminal_acceptance_radius_m = final_acceptance;
  plan.terminal_hold_max_speed_mps = terminal_hold_max_speed;
  plan.terminal_hold_distance_met = terminal_hold_distance_met;
  plan.terminal_hold_speed_met = terminal_hold_speed_met;
  plan.terminal_capture_goal_distance_triggered =
      terminal_capture_goal_distance_triggered;
  plan.terminal_capture_remaining_distance_triggered =
      terminal_capture_remaining_distance_triggered;
  populateVelocityBasisErrors(plan);
  plan.trajectory_cross_track_error_m = std::sqrt(control_projection.distance_sq);
  plan.current_cross_track_error_m = std::sqrt(current_projection.distance_sq);
  plan.predicted_cross_track_error_m = std::sqrt(control_projection.distance_sq);
  plan.prediction_horizon_s = bounded_prediction_horizon;
  plan.prediction_distance_m = prediction_distance;
  plan.response_delay_distance_m = response_delay_distance;
  plan.limiting_constraint_type = scalar_speed.constraint_type;
  plan.limiting_constraint_index = scalar_speed.constraint_index;
  plan.limiting_constraint_distance_m = scalar_speed.limiting_constraint_distance_m;
  plan.limiting_constraint_speed_mps = scalar_speed.limiting_constraint_speed_mps;
  plan.limiting_allowed_speed_now_mps = scalar_speed.limiting_allowed_speed_now_mps;
  plan.limiting_curve_radius_m = scalar_speed.limiting_curve_radius_m;
  plan.trajectory_s_m = control_projection.s_m;
  plan.trajectory_segment_index = control_projection.segment_index;
  plan.trajectory_segment_kind = segment_kind;
  plan.trajectory_curvature_1pm = control_projection.curvature_1pm;
  plan.trajectory_arc_radius_m =
      std::abs(plan.trajectory_curvature_1pm) > kTinyDistanceM
          ? 1.0 / std::abs(plan.trajectory_curvature_1pm)
          : std::numeric_limits<double>::quiet_NaN();
  plan.trajectory_projection = control_projection;

  plan.final_stop.valid = true;
  plan.final_stop.distance_to_stop_m =
      std::max(0.0, trajectory_length_m - std::max(0.0, control_projection.s_m));
  plan.final_stop.braking_distance_m =
      response_delay_distance +
      current_speed * current_speed / (2.0 * effectiveSpeedProfileDecelMps2(config));
  plan.final_stop.raw_speed_limit_mps = scalar_speed.limiting_allowed_speed_now_mps;

  return plan;
}

} // namespace

VelocitySetpointPlan planVelocitySetpoint(
    const std::span<const TrajectorySegment> trajectory,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config) {
  if (!trajectoryIsUsable(trajectory)) {
    return VelocitySetpointPlan{};
  }
  const std::optional<TrajectoryProjection> current_projection =
      projectOnTrajectory(trajectory, current_position);
  if (!current_projection.has_value()) {
    return VelocitySetpointPlan{};
  }
  const double prediction_horizon = trackingPredictionHorizonS(config);
  const Point2 predicted_position = predictedTrackingPosition(
      current_position, current_velocity, current_velocity_valid, prediction_horizon);
  const std::optional<TrajectoryProjection> control_projection =
      projectOnTrajectory(trajectory, predicted_position);
  if (!control_projection.has_value()) {
    return VelocitySetpointPlan{};
  }
  const TrajectorySegment& segment =
      trajectory[std::min(control_projection->segment_index, trajectory.size() - 1U)];
  ControlTangentSmoothingDiagnostics tangent_smoothing{};
  tangent_smoothing.raw_tangent = control_projection->tangent;
  return planVelocitySetpointFromProjection(
      *control_projection, *current_projection, predicted_position, prediction_horizon,
      trajectory.back().end, trajectoryLengthM(trajectory), segment.kind, speed_profile,
      current_position, current_velocity, current_velocity_valid, dt_s, previous_state,
      config, tangent_smoothing);
}

VelocitySetpointPlan planVelocitySetpoint(
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config) {
  if (!trajectorySamplesAreUsable(trajectory_samples)) {
    return VelocitySetpointPlan{};
  }
  const std::optional<TrajectoryProjection> current_projection =
      projectOnTrajectorySamples(trajectory_samples, current_position);
  if (!current_projection.has_value()) {
    return VelocitySetpointPlan{};
  }
  const double prediction_horizon = trackingPredictionHorizonS(config);
  const Point2 predicted_position = predictedTrackingPosition(
      current_position, current_velocity, current_velocity_valid, prediction_horizon);
  const std::optional<TrajectoryProjection> control_projection =
      projectOnTrajectorySamples(trajectory_samples, predicted_position);
  if (!control_projection.has_value()) {
    return VelocitySetpointPlan{};
  }
  TrajectoryProjection command_projection = *control_projection;
  const ControlTangentSmoothingDiagnostics tangent_smoothing =
      smoothControlTangentIfStraightish(trajectory_samples, command_projection, config);
  const TrajectorySegmentKind segment_kind =
      std::abs(control_projection->curvature_1pm) > kTinyDistanceM
          ? TrajectorySegmentKind::kArc
          : TrajectorySegmentKind::kLine;
  return planVelocitySetpointFromProjection(
      command_projection, *current_projection, predicted_position, prediction_horizon,
      trajectory_samples.back().point, trajectory_samples.back().s_m, segment_kind,
      speed_profile, current_position, current_velocity, current_velocity_valid, dt_s,
      previous_state, config, tangent_smoothing);
}

} // namespace drone_city_nav
