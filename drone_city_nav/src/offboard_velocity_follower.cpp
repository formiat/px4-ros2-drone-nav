#include "drone_city_nav/offboard_velocity_follower.hpp"

#include "drone_city_nav/control_projection_smoothing.hpp"
#include "drone_city_nav/velocity_command_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

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
  return sanitizedPositive(config.speed_profile_decel_mps2, 2.0, 1.0e-6, 100.0);
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
  return sanitizedPositive(config.terminal_capture_max_speed_mps, 8.0, 0.0, 100.0);
}

[[nodiscard]] double terminalCaptureDecelMps2(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.terminal_capture_decel_mps2, 4.0, 1.0e-6, 100.0);
}

[[nodiscard]] double
terminalCaptureBrakingMarginM(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.terminal_capture_braking_margin_m, 2.0, 0.0, 1000.0);
}

[[nodiscard]] double brakingSpeedLimitMps(const double distance_m,
                                          const double acceptance_radius_m,
                                          const double decel_mps2) noexcept {
  return std::sqrt(2.0 * decel_mps2 * std::max(0.0, distance_m - acceptance_radius_m));
}

[[nodiscard]] double
monotonicTerminalCaptureSpeedLimitMps(const double raw_speed_limit_mps,
                                      const VelocityFollowerState& previous_state) {
  if (!previous_state.previous_terminal_capture_active ||
      !previous_state.previous_terminal_capture_speed_limit_valid ||
      !std::isfinite(previous_state.previous_terminal_capture_speed_limit_mps)) {
    return raw_speed_limit_mps;
  }
  return std::min(raw_speed_limit_mps,
                  previous_state.previous_terminal_capture_speed_limit_mps);
}

[[nodiscard]] double terminalPostPlaneRecaptureMaxSpeedMps(
    const double terminal_hold_max_speed_mps,
    const double terminal_capture_max_speed_mps) noexcept {
  const double recapture_speed_mps =
      std::max(1.5, 2.0 * std::max(0.0, terminal_hold_max_speed_mps));
  return std::min(terminal_capture_max_speed_mps, recapture_speed_mps);
}

[[nodiscard]] double
trackingPredictionHorizonS(const VelocityFollowerConfig& config) noexcept {
  return sanitizedPositive(config.tracking_prediction_horizon_s, 0.35, 0.0, 2.0);
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

struct VerticalTrackabilitySpeedCap {
  bool active{false};
  double speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double constraint_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double altitude_error_m{std::numeric_limits<double>::quiet_NaN()};
};

struct VerticalHardWindow {
  bool valid{false};
  double start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double end_s_m{std::numeric_limits<double>::quiet_NaN()};
  double safe_min_z_m{std::numeric_limits<double>::quiet_NaN()};
  double safe_max_z_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] std::size_t
firstSampleIndexAtOrAfterS(const std::span<const TrajectoryPointSample> samples,
                           const double s_m) noexcept {
  if (samples.empty()) {
    return 0U;
  }
  const double clamped_s =
      std::clamp(std::isfinite(s_m) ? s_m : 0.0, 0.0, samples.back().s_m);
  const auto it =
      std::lower_bound(samples.begin(), samples.end(), clamped_s,
                       [](const TrajectoryPointSample& sample, const double station_m) {
                         return sample.s_m < station_m;
                       });
  if (it == samples.end()) {
    return samples.size() - 1U;
  }
  return static_cast<std::size_t>(std::distance(samples.begin(), it));
}

[[nodiscard]] double
verticalProfileWindowEndS(const std::span<const TrajectoryPointSample> samples,
                          const std::size_t start_index,
                          const std::string& passage_id) noexcept {
  if (samples.empty() || passage_id.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  for (std::size_t i = start_index; i < samples.size(); ++i) {
    if (!samples[i].vertical_profile_passage_id.empty() &&
        samples[i].vertical_profile_passage_id != passage_id) {
      return samples[i].s_m;
    }
    if (samples[i].vertical_profile_passage_id.empty() && i > start_index) {
      return samples[i].s_m;
    }
  }
  return samples.back().s_m;
}

[[nodiscard]] bool
hardWindowSampleUsable(const TrajectoryPointSample& sample) noexcept {
  return sample.vertical_hard_window_active &&
         std::isfinite(sample.vertical_safe_min_z_m) &&
         std::isfinite(sample.vertical_safe_max_z_m) &&
         sample.vertical_safe_max_z_m >= sample.vertical_safe_min_z_m;
}

[[nodiscard]] bool sameHardWindow(const TrajectoryPointSample& lhs,
                                  const TrajectoryPointSample& rhs) noexcept {
  if (!hardWindowSampleUsable(lhs) || !hardWindowSampleUsable(rhs)) {
    return false;
  }
  if (!lhs.vertical_profile_passage_id.empty() ||
      !rhs.vertical_profile_passage_id.empty()) {
    return lhs.vertical_profile_passage_id == rhs.vertical_profile_passage_id;
  }
  return std::abs(lhs.vertical_safe_min_z_m - rhs.vertical_safe_min_z_m) <= 1.0e-6 &&
         std::abs(lhs.vertical_safe_max_z_m - rhs.vertical_safe_max_z_m) <= 1.0e-6;
}

[[nodiscard]] VerticalHardWindow
upcomingVerticalHardWindow(const std::span<const TrajectoryPointSample> samples,
                           const double trajectory_s_m) noexcept {
  VerticalHardWindow window{};
  if (samples.empty()) {
    return window;
  }

  std::size_t search_index = firstSampleIndexAtOrAfterS(samples, trajectory_s_m);
  if (search_index > 0U && hardWindowSampleUsable(samples[search_index - 1U]) &&
      samples[search_index - 1U].s_m <= trajectory_s_m + kTinyDistanceM) {
    --search_index;
  }

  while (search_index < samples.size() &&
         !hardWindowSampleUsable(samples[search_index])) {
    ++search_index;
  }
  if (search_index >= samples.size()) {
    return window;
  }

  std::size_t begin_index = search_index;
  while (begin_index > 0U &&
         sameHardWindow(samples[begin_index - 1U], samples[search_index])) {
    --begin_index;
  }
  std::size_t end_index = search_index;
  while (end_index + 1U < samples.size() &&
         sameHardWindow(samples[end_index + 1U], samples[search_index])) {
    ++end_index;
  }

  double safe_min_z_m = samples[begin_index].vertical_safe_min_z_m;
  double safe_max_z_m = samples[begin_index].vertical_safe_max_z_m;
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    safe_min_z_m = std::max(safe_min_z_m, samples[i].vertical_safe_min_z_m);
    safe_max_z_m = std::min(safe_max_z_m, samples[i].vertical_safe_max_z_m);
  }
  if (!(safe_max_z_m >= safe_min_z_m)) {
    return window;
  }

  window.valid = true;
  window.start_s_m = samples[begin_index].s_m;
  window.end_s_m = end_index + 1U < samples.size() ? samples[end_index + 1U].s_m
                                                   : samples[end_index].s_m;
  window.safe_min_z_m = safe_min_z_m;
  window.safe_max_z_m = safe_max_z_m;
  return window;
}

[[nodiscard]] double altitudeErrorToSafeWindow(const double altitude_m,
                                               const VerticalHardWindow& window,
                                               const double tolerance_m) noexcept {
  if (!window.valid || !std::isfinite(altitude_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (altitude_m < window.safe_min_z_m - tolerance_m) {
    return window.safe_min_z_m - altitude_m;
  }
  if (altitude_m > window.safe_max_z_m + tolerance_m) {
    return window.safe_max_z_m - altitude_m;
  }
  return 0.0;
}

[[nodiscard]] VerticalTrackabilitySpeedCap computeVerticalTrackabilitySpeedCap(
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const double trajectory_s_m, const double current_altitude_m,
    const bool altitude_valid, const VelocityFollowerConfig& config) {
  VerticalTrackabilitySpeedCap cap{};
  if (!altitude_valid || !std::isfinite(current_altitude_m) ||
      !trajectorySamplesAreUsable(trajectory_samples)) {
    return cap;
  }

  const TrajectoryVerticalTarget target =
      trajectoryVerticalTargetAtS(trajectory_samples, trajectory_s_m);
  if (!target.valid || !std::isfinite(target.z_m)) {
    return cap;
  }

  const double tolerance = sanitizedPositive(
      config.vertical_trackability_altitude_tolerance_m, 0.4, 0.0, 100.0);
  const VerticalHardWindow hard_window =
      upcomingVerticalHardWindow(trajectory_samples, target.s_m);
  const double altitude_error =
      hard_window.valid
          ? altitudeErrorToSafeWindow(current_altitude_m, hard_window, tolerance)
          : target.z_m - current_altitude_m;
  if (!std::isfinite(altitude_error)) {
    return cap;
  }
  const double required_error = std::abs(altitude_error) - tolerance;
  if (!(required_error > 0.0)) {
    return cap;
  }

  double constraint_distance_m = std::numeric_limits<double>::quiet_NaN();
  if (hard_window.valid) {
    constraint_distance_m = target.s_m < hard_window.start_s_m
                                ? std::max(0.0, hard_window.start_s_m - target.s_m)
                                : std::max(0.0, hard_window.end_s_m - target.s_m);
  } else {
    if (target.vertical_profile_passage_id.empty()) {
      return cap;
    }
    const std::size_t start_index =
        firstSampleIndexAtOrAfterS(trajectory_samples, target.s_m);
    const double window_end_s = verticalProfileWindowEndS(
        trajectory_samples, start_index, target.vertical_profile_passage_id);
    if (!std::isfinite(window_end_s)) {
      return cap;
    }
    constraint_distance_m = std::max(0.0, window_end_s - target.s_m);
  }

  const double vertical_speed_mps = sanitizedPositive(
      config.vertical_trackability_max_vertical_speed_mps,
      std::max(1.0, config.vertical_profile_max_vertical_speed_mps), 1.0e-6, 100.0);
  const double response_time_s =
      sanitizedPositive(config.vertical_trackability_response_time_s, 0.4, 0.0, 10.0);
  const double time_needed_s = required_error / vertical_speed_mps + response_time_s;
  if (!(time_needed_s > 1.0e-6)) {
    return cap;
  }

  const double cruise_speed_mps = sanitizedCruiseSpeed(config);
  const double min_speed_mps = std::min(
      cruise_speed_mps,
      sanitizedPositive(config.vertical_trackability_min_speed_mps, 1.0, 0.0, 100.0));
  const double speed_limit_mps = std::clamp(constraint_distance_m / time_needed_s,
                                            min_speed_mps, cruise_speed_mps);
  if (speed_limit_mps + 1.0e-9 >= cruise_speed_mps) {
    return cap;
  }

  cap.active = true;
  cap.speed_limit_mps = speed_limit_mps;
  cap.constraint_distance_m = constraint_distance_m;
  cap.altitude_error_m = altitude_error;
  return cap;
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
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectoryProjection& control_projection,
    const TrajectoryProjection& current_projection, const Point2 predicted_position,
    const double prediction_horizon_s, const Point2 final_point,
    const Point2 final_tangent, const double trajectory_length_m,
    const TrajectorySegmentKind segment_kind,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid,
    const double current_altitude_m, const bool altitude_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config,
    const ControlProjectionSmoothingDiagnostics& projection_smoothing) {
  VelocitySetpointPlan plan{};
  if (!finite2D(current_position) || !finite2D(predicted_position) ||
      !finite2D(final_point) || !control_projection.valid ||
      !current_projection.valid ||
      !(norm(control_projection.tangent) > kTinyDistanceM) ||
      !(norm(final_tangent) > kTinyDistanceM) || !std::isfinite(trajectory_length_m) ||
      !speed_profile.valid || speed_profile.samples.empty()) {
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
  const double terminal_signed_along_track_distance =
      dot(final_point - current_position, final_tangent);
  const double remaining_trajectory_distance =
      std::max(0.0, trajectory_length_m - std::max(0.0, control_projection.s_m));
  const double terminal_capture_radius = terminalCaptureRadiusM(config);
  const double terminal_capture_decel = terminalCaptureDecelMps2(config);
  const double terminal_capture_braking_margin = terminalCaptureBrakingMarginM(config);
  const double terminal_capture_braking_distance =
      current_speed * current_speed / (2.0 * terminal_capture_decel);
  const double terminal_capture_activation_distance =
      std::max(terminal_capture_radius,
               terminal_capture_braking_distance + terminal_capture_braking_margin);
  const double terminal_hold_max_speed = finalHoldMaxSpeedMps(config);
  const bool terminal_hold_distance_met = terminal_goal_distance <= final_acceptance;
  const bool terminal_hold_speed_met = current_speed <= terminal_hold_max_speed;
  const bool past_final_plane = terminal_signed_along_track_distance <= 0.0;
  const bool terminal_capture_goal_distance_triggered =
      terminal_goal_distance <= terminal_capture_activation_distance;
  const bool terminal_capture_remaining_distance_triggered =
      remaining_trajectory_distance <= terminal_capture_activation_distance;
  const bool terminal_capture_after_final_plane =
      past_final_plane &&
      terminal_goal_distance <= terminal_capture_activation_distance;
  const bool terminal_capture_triggered =
      terminal_capture_remaining_distance_triggered ||
      terminal_capture_after_final_plane;
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
    plan.terminal_signed_along_track_distance_m = terminal_signed_along_track_distance;
    plan.terminal_remaining_trajectory_distance_m = remaining_trajectory_distance;
    plan.terminal_acceptance_radius_m = final_acceptance;
    plan.terminal_hold_max_speed_mps = terminal_hold_max_speed;
    plan.terminal_hold_distance_met = terminal_hold_distance_met;
    plan.terminal_hold_speed_met = terminal_hold_speed_met;
    plan.terminal_capture_goal_distance_triggered =
        terminal_capture_goal_distance_triggered;
    plan.terminal_capture_remaining_distance_triggered =
        terminal_capture_remaining_distance_triggered;
    plan.terminal_capture_decel_mps2 = terminal_capture_decel;
    plan.terminal_capture_braking_margin_m = terminal_capture_braking_margin;
    plan.terminal_capture_braking_distance_m = terminal_capture_braking_distance;
    plan.terminal_capture_activation_distance_m = terminal_capture_activation_distance;
    plan.current_cross_track_error_m = std::sqrt(current_projection.distance_sq);
    plan.predicted_cross_track_error_m = std::sqrt(control_projection.distance_sq);
    plan.trajectory_cross_track_error_m = plan.predicted_cross_track_error_m;
    plan.trajectory_s_m = control_projection.s_m;
    plan.trajectory_segment_index = control_projection.segment_index;
    plan.trajectory_segment_kind = segment_kind;
    plan.trajectory_curvature_1pm = control_projection.curvature_1pm;
    plan.trajectory_projection = control_projection;
    plan.control_tangent_raw = projection_smoothing.raw_tangent;
    plan.control_tangent_smoothed = projection_smoothing.applied;
    plan.control_projection_smoothing_mode = projection_smoothing.mode;
    plan.control_tangent_smoothing_heading_span_rad =
        projection_smoothing.heading_span_rad;
    plan.control_tangent_smoothing_max_abs_curvature_1pm =
        projection_smoothing.max_abs_curvature_1pm;
    plan.control_tangent_smoothing_window_start_s_m =
        projection_smoothing.window_start_s_m;
    plan.control_tangent_smoothing_window_end_s_m = projection_smoothing.window_end_s_m;
    return plan;
  }

  if (terminal_capture_triggered) {
    const Point2 goal_delta = final_point - current_position;
    const bool brake_after_final_plane = past_final_plane &&
                                         terminal_hold_distance_met &&
                                         current_speed > terminal_hold_max_speed;
    const double terminal_capture_distance =
        past_final_plane ? terminal_goal_distance : remaining_trajectory_distance;
    Point2 terminal_tangent = final_tangent;
    if ((!past_final_plane ||
         (!brake_after_final_plane && terminal_goal_distance > final_acceptance)) &&
        norm(goal_delta) > kTinyDistanceM) {
      terminal_tangent = normalized(goal_delta);
    }
    const double capture_gain_speed_limit =
        terminalCaptureGain1ps(config) * terminal_capture_distance;
    const double capture_max_speed = terminalCaptureMaxSpeedMps(config);
    const double capture_braking_speed_limit = brakingSpeedLimitMps(
        terminal_capture_distance, final_acceptance, terminal_capture_decel);
    const double post_plane_recapture_speed_limit =
        past_final_plane && !brake_after_final_plane
            ? terminalPostPlaneRecaptureMaxSpeedMps(terminal_hold_max_speed,
                                                    capture_max_speed)
            : capture_max_speed;
    const double raw_capture_speed_limit =
        brake_after_final_plane
            ? 0.0
            : std::min({post_plane_recapture_speed_limit, capture_gain_speed_limit,
                        capture_braking_speed_limit});
    const double capture_speed_limit =
        past_final_plane ? raw_capture_speed_limit
                         : monotonicTerminalCaptureSpeedLimitMps(
                               raw_capture_speed_limit, previous_state);
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
            .dt_s = dt},
        config);
    if (!smoothed.valid) {
      return plan;
    }

    const Point2 left_normal{-terminal_tangent.y, terminal_tangent.x};
    plan.valid = true;
    plan.reason = VelocitySetpointReason::kTerminalCapture;
    plan.terminal_capture_active = true;
    plan.terminal_goal_distance_m = terminal_goal_distance;
    plan.terminal_signed_along_track_distance_m = terminal_signed_along_track_distance;
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
    plan.terminal_capture_decel_mps2 = terminal_capture_decel;
    plan.terminal_capture_braking_margin_m = terminal_capture_braking_margin;
    plan.terminal_capture_braking_distance_m = terminal_capture_braking_distance;
    plan.terminal_capture_activation_distance_m = terminal_capture_activation_distance;
    plan.terminal_capture_braking_speed_limit_mps = capture_braking_speed_limit;
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
    plan.smoother_lateral_response_accel_mps2 =
        smoothed.smoother_lateral_response_accel_mps2;
    plan.path_tangent = terminal_tangent;
    plan.control_tangent_raw = projection_smoothing.raw_tangent;
    plan.control_tangent_smoothed = projection_smoothing.applied;
    plan.control_projection_smoothing_mode = projection_smoothing.mode;
    plan.control_tangent_smoothing_heading_span_rad =
        projection_smoothing.heading_span_rad;
    plan.control_tangent_smoothing_max_abs_curvature_1pm =
        projection_smoothing.max_abs_curvature_1pm;
    plan.control_tangent_smoothing_window_start_s_m =
        projection_smoothing.window_start_s_m;
    plan.control_tangent_smoothing_window_end_s_m = projection_smoothing.window_end_s_m;
    plan.projection = final_point;
    plan.current_projection = current_projection.point;
    plan.predicted_position = predicted_position;
    plan.predicted_projection = control_projection.point;
    plan.raw_speed_limit_mps = capture_speed_limit;
    plan.profile_speed_limit_mps = capture_speed_limit;
    plan.speed_after_lookahead_mps = capture_speed_limit;
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
    plan.limiting_constraint_distance_m = terminal_capture_distance;
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
    plan.final_stop.distance_to_stop_m = terminal_capture_distance;
    plan.final_stop.braking_distance_m =
        response_delay_distance + terminal_capture_braking_distance;
    plan.final_stop.raw_speed_limit_mps = capture_speed_limit;
    return plan;
  }

  const VerticalTrackabilitySpeedCap vertical_trackability =
      computeVerticalTrackabilitySpeedCap(trajectory_samples, control_projection.s_m,
                                          current_altitude_m, altitude_valid, config);
  const ScalarSpeedPlan scalar_speed = planScalarSpeed(
      speed_profile,
      ScalarSpeedQuery{.trajectory_s_m = control_projection.s_m,
                       .previous_command_speed_mps =
                           previousCommandSpeedMps(previous_state, current_speed),
                       .current_speed_mps = current_speed,
                       .dt_s = dt,
                       .vertical_trackability_speed_cap_active =
                           vertical_trackability.active,
                       .vertical_trackability_speed_limit_mps =
                           vertical_trackability.speed_limit_mps,
                       .vertical_trackability_constraint_distance_m =
                           vertical_trackability.constraint_distance_m,
                       .vertical_trackability_altitude_error_m =
                           vertical_trackability.altitude_error_m},
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
          .curvature_feedforward_context_scale =
              projection_smoothing.curvature_feedforward_context_scale},
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
          .dt_s = dt},
      config);
  if (!smoothed.valid) {
    return plan;
  }

  const double cruise_speed = sanitizedCruiseSpeed(config);
  plan.valid = true;
  plan.reason = VelocitySetpointReason::kStraight;
  if ((scalar_speed.constraint_type == SpeedConstraintType::kArc ||
       scalar_speed.constraint_type == SpeedConstraintType::kVerticalProfile ||
       scalar_speed.constraint_type == SpeedConstraintType::kVerticalTrackability) &&
      scalar_speed.speed_after_lookahead_mps + 1.0e-6 < cruise_speed) {
    plan.reason = VelocitySetpointReason::kTrajectorySpeedProfile;
  } else if (scalar_speed.constraint_type == SpeedConstraintType::kGoal &&
             scalar_speed.speed_after_lookahead_mps + 1.0e-6 < cruise_speed) {
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
  plan.smoother_lateral_response_accel_mps2 =
      smoothed.smoother_lateral_response_accel_mps2;
  plan.path_tangent = control_projection.tangent;
  plan.control_tangent_raw = projection_smoothing.raw_tangent;
  plan.control_tangent_smoothed = projection_smoothing.applied;
  plan.control_projection_smoothing_mode = projection_smoothing.mode;
  plan.control_tangent_smoothing_heading_span_rad =
      projection_smoothing.heading_span_rad;
  plan.control_tangent_smoothing_max_abs_curvature_1pm =
      projection_smoothing.max_abs_curvature_1pm;
  plan.control_tangent_smoothing_window_start_s_m =
      projection_smoothing.window_start_s_m;
  plan.control_tangent_smoothing_window_end_s_m = projection_smoothing.window_end_s_m;
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
  plan.raw_speed_limit_mps = scalar_speed.speed_after_lookahead_mps;
  plan.profile_speed_limit_mps = scalar_speed.profile_speed_limit_mps;
  plan.speed_lookahead_distance_m = scalar_speed.lookahead_distance_m;
  plan.lookahead_speed_limit_mps = scalar_speed.lookahead_speed_limit_mps;
  plan.lookahead_limiting_constraint_type = scalar_speed.lookahead_constraint_type;
  plan.lookahead_limiting_constraint_index = scalar_speed.lookahead_constraint_index;
  plan.lookahead_limiting_constraint_distance_m =
      scalar_speed.lookahead_constraint_distance_m;
  plan.speed_after_lookahead_mps = scalar_speed.speed_after_lookahead_mps;
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
  plan.cross_track_p_gain_factor = command.cross_track_p_gain_factor;
  plan.cross_track_derivative_damping_mps = command.cross_track_derivative_damping_mps;
  plan.cross_track_d_gain_factor = command.cross_track_d_gain_factor;
  plan.cross_track_d_gain_effective = command.cross_track_d_gain_effective;
  plan.cross_track_lateral_velocity_mps = command.cross_track_lateral_velocity_mps;
  plan.curvature_feedforward_mps = command.curvature_feedforward_mps;
  plan.curvature_feedforward_angle_rad = command.curvature_feedforward_angle_rad;
  plan.curvature_feedforward_raw_angle_rad =
      command.curvature_feedforward_raw_angle_rad;
  plan.curvature_feedforward_scale = command.curvature_feedforward_scale;
  plan.curvature_feedforward_context_scale =
      command.curvature_feedforward_context_scale;
  plan.raw_lateral_control_mps = command.raw_lateral_control_mps;
  plan.lateral_control_mps = command.lateral_control_mps;
  plan.terminal_goal_distance_m = terminal_goal_distance;
  plan.terminal_signed_along_track_distance_m = terminal_signed_along_track_distance;
  plan.terminal_remaining_trajectory_distance_m = remaining_trajectory_distance;
  plan.terminal_acceptance_radius_m = final_acceptance;
  plan.terminal_hold_max_speed_mps = terminal_hold_max_speed;
  plan.terminal_hold_distance_met = terminal_hold_distance_met;
  plan.terminal_hold_speed_met = terminal_hold_speed_met;
  plan.terminal_capture_goal_distance_triggered =
      terminal_capture_goal_distance_triggered;
  plan.terminal_capture_remaining_distance_triggered =
      terminal_capture_remaining_distance_triggered;
  plan.terminal_capture_decel_mps2 = terminal_capture_decel;
  plan.terminal_capture_braking_margin_m = terminal_capture_braking_margin;
  plan.terminal_capture_braking_distance_m = terminal_capture_braking_distance;
  plan.terminal_capture_activation_distance_m = terminal_capture_activation_distance;
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
  plan.vertical_trackability_speed_cap_active =
      scalar_speed.vertical_trackability_speed_cap_active;
  plan.vertical_trackability_speed_limit_mps =
      scalar_speed.vertical_trackability_speed_limit_mps;
  plan.vertical_trackability_constraint_distance_m =
      scalar_speed.vertical_trackability_constraint_distance_m;
  plan.vertical_trackability_altitude_error_m =
      scalar_speed.vertical_trackability_altitude_error_m;
  plan.trajectory_s_m = control_projection.s_m;
  plan.trajectory_segment_index = control_projection.segment_index;
  plan.trajectory_curvature_1pm = control_projection.curvature_1pm;
  plan.trajectory_segment_kind =
      std::abs(plan.trajectory_curvature_1pm) > kTinyDistanceM
          ? TrajectorySegmentKind::kArc
          : TrajectorySegmentKind::kLine;
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
  ControlProjectionSmoothingDiagnostics projection_smoothing{};
  projection_smoothing.raw_tangent = control_projection->tangent;
  const double trajectory_length_m = trajectoryLengthM(trajectory);
  const Point2 final_tangent = trajectoryTangentAtS(trajectory, trajectory_length_m);
  const std::span<const TrajectoryPointSample> trajectory_samples{};
  return planVelocitySetpointFromProjection(
      trajectory_samples, *control_projection, *current_projection, predicted_position,
      prediction_horizon, trajectory.back().end, final_tangent, trajectory_length_m,
      segment.kind, speed_profile, current_position, current_velocity,
      current_velocity_valid, std::numeric_limits<double>::quiet_NaN(), false, dt_s,
      previous_state, config, projection_smoothing);
}

VelocitySetpointPlan planVelocitySetpoint(
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid, const double dt_s,
    const VelocityFollowerState& previous_state, const VelocityFollowerConfig& config) {
  return planVelocitySetpoint(trajectory_samples, speed_profile, current_position,
                              current_velocity, current_velocity_valid,
                              std::numeric_limits<double>::quiet_NaN(), false, dt_s,
                              previous_state, config);
}

VelocitySetpointPlan planVelocitySetpoint(
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const TrajectorySpeedProfile& speed_profile, const Point2 current_position,
    const Point2 current_velocity, const bool current_velocity_valid,
    const double current_altitude_m, const bool altitude_valid, const double dt_s,
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
  const SmoothedControlProjection smoothed_projection =
      smoothControlProjection(trajectory_samples, *control_projection, config);
  const TrajectorySegmentKind segment_kind =
      std::abs(smoothed_projection.projection.curvature_1pm) > kTinyDistanceM
          ? TrajectorySegmentKind::kArc
          : TrajectorySegmentKind::kLine;
  Point2 final_tangent = normalized(trajectory_samples.back().tangent);
  if (!(norm(final_tangent) > kTinyDistanceM) && trajectory_samples.size() >= 2U) {
    final_tangent =
        normalized(trajectory_samples.back().point -
                   trajectory_samples[trajectory_samples.size() - 2U].point);
  }
  return planVelocitySetpointFromProjection(
      trajectory_samples, smoothed_projection.projection, *current_projection,
      predicted_position, prediction_horizon, trajectory_samples.back().point,
      final_tangent, trajectory_samples.back().s_m, segment_kind, speed_profile,
      current_position, current_velocity, current_velocity_valid, current_altitude_m,
      altitude_valid, dt_s, previous_state, config, smoothed_projection.diagnostics);
}

} // namespace drone_city_nav
