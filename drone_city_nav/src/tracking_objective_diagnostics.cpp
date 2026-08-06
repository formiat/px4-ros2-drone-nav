#include "tracking_objective_diagnostics.hpp"

#include "drone_city_nav/mppi_debug_markers.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "production_mppi_node.hpp"

namespace drone_city_nav::detail {
namespace {

[[nodiscard]] const char* radarCadenceReasonName(const std::uint8_t reason) noexcept {
  switch (reason) {
    case msg::RadarTrackModeCommand::REASON_NO_TRACKING_OBJECTIVE:
      return "no_tracking_objective";
    case msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_OCCLUDED:
      return "observed_target_occluded";
    case msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_VISIBLE:
      return "observed_target_visible";
    case msg::RadarTrackModeCommand::REASON_WORLD_UNAVAILABLE:
      return "world_unavailable";
    default:
      return "unknown";
  }
}

} // namespace

TrackingPursuitDiagnostics
trackingPursuitDiagnostics(const ProductionNavigationObjective* navigation_objective,
                           const mppi::MppiTickInput& input,
                           const ProductionMppiExecutionPublication& execution) {
  TrackingPursuitDiagnostics diagnostics;
  diagnostics.actual_speed_mps =
      std::hypot(input.initial_state.vx, input.initial_state.vy);
  const mppi::State& commanded_state =
      execution.horizon.size() > 1U ? execution.horizon[1U] : input.initial_state;
  diagnostics.commanded_speed_mps = std::hypot(commanded_state.vx, commanded_state.vy);
  if (navigation_objective == nullptr || !navigation_objective->tracking.has_value()) {
    return diagnostics;
  }

  const ProductionTrackingObjective& tracking = navigation_objective->tracking.value();
  diagnostics.selected_prediction_fraction = tracking.resolved_fraction;
  diagnostics.radar_cadence_reason = tracking.radar_cadence_reason;
  diagnostics.observed_target_visible = tracking.observed_target_visible;
  diagnostics.predicted_intercept_path_clear = tracking.predicted_intercept_path_clear;
  diagnostics.direct_interception_active = tracking.direct_interception_active;
  const double observation_age_s =
      static_cast<double>(std::max<std::int64_t>(
          0, input.planning_stamp_ns - tracking.observation_stamp_ns)) *
      1.0e-9;
  diagnostics.radar_age_ms = observation_age_s * 1.0e3;
  const Vec3 relative_position{
      tracking.observed_position.x + tracking.observed_velocity.x * observation_age_s -
          input.initial_state.x,
      tracking.observed_position.y + tracking.observed_velocity.y * observation_age_s -
          input.initial_state.y,
      tracking.observed_position.z + tracking.observed_velocity.z * observation_age_s -
          input.initial_state.z,
  };
  const Vec3 relative_velocity{
      tracking.observed_velocity.x - input.initial_state.vx,
      tracking.observed_velocity.y - input.initial_state.vy,
      tracking.observed_velocity.z - input.initial_state.vz,
  };
  diagnostics.target_separation_m = std::hypot(
      std::hypot(relative_position.x, relative_position.y), relative_position.z);
  if (diagnostics.target_separation_m > 1.0e-6) {
    diagnostics.closing_speed_mps = -(relative_position.x * relative_velocity.x +
                                      relative_position.y * relative_velocity.y +
                                      relative_position.z * relative_velocity.z) /
                                    diagnostics.target_separation_m;
  }
  return diagnostics;
}

std::string trackingPursuitInfoFields(const TrackingPursuitDiagnostics& diagnostics,
                                      const MppiSpeedPolicyResult& speed_policy,
                                      const mppi::MppiTickResult& result) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3)
         << " commanded_speed_mps=" << diagnostics.commanded_speed_mps
         << " actual_speed_mps=" << diagnostics.actual_speed_mps
         << " active_speed_limiter="
         << mppiSpeedLimiterName(speed_policy.active_limiter)
         << " terminal_goal_limit_enabled="
         << (speed_policy.terminal_goal_limit_enabled ? "true" : "false")
         << " tracking_separation_m=" << diagnostics.target_separation_m
         << " closing_speed_mps=" << diagnostics.closing_speed_mps
         << " radar_age_ms=" << diagnostics.radar_age_ms << " observed_target_visible="
         << (diagnostics.observed_target_visible ? "true" : "false")
         << " predicted_intercept_path_clear="
         << (diagnostics.predicted_intercept_path_clear ? "true" : "false")
         << " direct_interception="
         << (diagnostics.direct_interception_active ? "true" : "false")
         << " selected_prediction_fraction=" << diagnostics.selected_prediction_fraction
         << " radar_cadence_reason="
         << radarCadenceReasonName(diagnostics.radar_cadence_reason)
         << " minimum_target_separation_m=" << result.minimum_target_separation_m
         << " predicted_capture_time_s=" << result.predicted_capture_time_s;
  return output.str();
}

std::string trackingPursuitJsonFields(const TrackingPursuitDiagnostics& diagnostics,
                                      const MppiSpeedPolicyResult& speed_policy,
                                      const mppi::MppiTickResult& result) {
  std::ostringstream output;
  output << ",\"commanded_speed_mps\":" << diagnostics.commanded_speed_mps
         << ",\"actual_speed_mps\":" << diagnostics.actual_speed_mps
         << ",\"active_speed_limiter\":\""
         << mppiSpeedLimiterName(speed_policy.active_limiter) << '"'
         << ",\"terminal_goal_limit_enabled\":"
         << (speed_policy.terminal_goal_limit_enabled ? "true" : "false")
         << ",\"tracking_separation_m\":" << diagnostics.target_separation_m
         << ",\"closing_speed_mps\":" << diagnostics.closing_speed_mps
         << ",\"radar_age_ms\":" << diagnostics.radar_age_ms
         << ",\"observed_target_visible\":"
         << (diagnostics.observed_target_visible ? "true" : "false")
         << ",\"predicted_intercept_path_clear\":"
         << (diagnostics.predicted_intercept_path_clear ? "true" : "false")
         << ",\"direct_interception_active\":"
         << (diagnostics.direct_interception_active ? "true" : "false")
         << ",\"selected_prediction_fraction\":"
         << diagnostics.selected_prediction_fraction << ",\"radar_cadence_reason\":\""
         << radarCadenceReasonName(diagnostics.radar_cadence_reason) << '"'
         << ",\"minimum_target_separation_m\":" << result.minimum_target_separation_m
         << ",\"predicted_capture_time_s\":" << result.predicted_capture_time_s;
  return output.str();
}

std::string
trackingObjectiveJsonFields(const ProductionNavigationObjective* navigation_objective,
                            const Point3& resolved_position,
                            const std::int64_t now_ns) {
  const ProductionTrackingObjective* objective =
      navigation_objective != nullptr && navigation_objective->tracking.has_value()
          ? &navigation_objective->tracking.value()
          : nullptr;
  const ProductionTrackingObjective empty_objective;
  const ProductionTrackingObjective& data =
      objective != nullptr ? *objective : empty_objective;
  const bool active = objective != nullptr;
  std::ostringstream output;
  output << ",\"tracking_objective_active\":" << (active ? "true" : "false")
         << ",\"tracking_guidance_mode\":\""
         << (active ? interceptGuidanceModeName(data.guidance_mode) : "none") << '"'
         << ",\"tracking_resolution\":\""
         << (active ? trackingObjectiveResolutionStatusName(data.resolution_status)
                    : "none")
         << '"' << ",\"tracking_prediction_horizon_s\":" << data.prediction_horizon_s
         << ",\"tracking_observation_stamp_ns\":" << data.observation_stamp_ns
         << ",\"tracking_observation_age_ms\":"
         << (active ? static_cast<double>(std::max<std::int64_t>(
                          0, now_ns - data.observation_stamp_ns)) /
                          1.0e6
                    : 0.0)
         << ",\"tracking_resolved_fraction\":" << data.resolved_fraction
         << ",\"tracking_observed_x_m\":" << data.observed_position.x
         << ",\"tracking_observed_y_m\":" << data.observed_position.y
         << ",\"tracking_observed_z_m\":" << data.observed_position.z
         << ",\"tracking_current_target_x_m\":" << data.current_target_position.x
         << ",\"tracking_current_target_y_m\":" << data.current_target_position.y
         << ",\"tracking_current_target_z_m\":" << data.current_target_position.z
         << ",\"tracking_velocity_x_mps\":" << data.observed_velocity.x
         << ",\"tracking_velocity_y_mps\":" << data.observed_velocity.y
         << ",\"tracking_velocity_z_mps\":" << data.observed_velocity.z
         << ",\"tracking_predicted_x_m\":" << data.unconstrained_predicted_position.x
         << ",\"tracking_predicted_y_m\":" << data.unconstrained_predicted_position.y
         << ",\"tracking_predicted_z_m\":" << data.unconstrained_predicted_position.z
         << ",\"tracking_vertical_prediction_clipped\":"
         << (data.vertical_prediction_clipped ? "true" : "false")
         << ",\"tracking_resolved_x_m\":" << resolved_position.x
         << ",\"tracking_resolved_y_m\":" << resolved_position.y
         << ",\"tracking_resolved_z_m\":" << resolved_position.z
         << ",\"tracking_direct_target_status\":\""
         << directTrackingTargetStatusName(data.direct_target_status) << '"'
         << ",\"tracking_observed_target_visible\":"
         << (data.observed_target_visible ? "true" : "false")
         << ",\"tracking_predicted_intercept_path_clear\":"
         << (data.predicted_intercept_path_clear ? "true" : "false")
         << ",\"tracking_direct_interception_active\":"
         << (data.direct_interception_active ? "true" : "false")
         << ",\"tracking_direct_los\":"
         << (data.direct_interception_active ? "true" : "false")
         << ",\"tracking_radar_cadence_reason\":\""
         << radarCadenceReasonName(data.radar_cadence_reason) << '"'
         << ",\"tracking_los_generation\":" << data.line_of_sight_generation;
  return output.str();
}

void populateTrackingObjectiveMarkers(
    const ProductionNavigationObjective* navigation_objective,
    MppiDebugMarkerInput& marker_input) {
  if (navigation_objective == nullptr || !navigation_objective->tracking.has_value()) {
    return;
  }
  const ProductionTrackingObjective& tracking = navigation_objective->tracking.value();
  marker_input.tracking_objective_active = true;
  marker_input.observed_tracking_target = tracking.current_target_position;
  marker_input.predicted_tracking_target = tracking.unconstrained_predicted_position;
  marker_input.resolved_tracking_target = navigation_objective->goal;
}

} // namespace drone_city_nav::detail
