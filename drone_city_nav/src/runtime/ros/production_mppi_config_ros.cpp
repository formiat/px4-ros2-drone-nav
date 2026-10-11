#include "production_mppi_config_ros.hpp"

#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/sensor_braking_contract_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_config_ros_loader.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t durationNanoseconds(const double seconds,
                                               const char* const parameter_name,
                                               const bool allow_zero = false) {
  const long double nanoseconds = static_cast<long double>(seconds) * 1'000'000'000.0L;
  const long double first_unrepresentable_rounding_input =
      static_cast<long double>(std::numeric_limits<std::int64_t>::max()) + 0.5L;
  if (!std::isfinite(seconds) || (allow_zero ? seconds < 0.0 : !(seconds > 0.0)) ||
      nanoseconds >= first_unrepresentable_rounding_input) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " must be finite, non-negative, and representable"};
  }
  const std::int64_t duration_ns = static_cast<std::int64_t>(std::llround(nanoseconds));
  if (!allow_zero && duration_ns <= 0) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " rounds to a non-positive duration"};
  }
  return duration_ns;
}

[[nodiscard]] std::int64_t checkedDurationSum(const std::int64_t first_ns,
                                              const std::int64_t second_ns,
                                              const std::int64_t third_ns,
                                              const char* const name) {
  if (first_ns < 0 || second_ns < 0 || third_ns < 0 ||
      first_ns > std::numeric_limits<std::int64_t>::max() - second_ns ||
      first_ns + second_ns > std::numeric_limits<std::int64_t>::max() - third_ns) {
    throw std::invalid_argument{std::string{name} + " is not representable"};
  }
  return first_ns + second_ns + third_ns;
}

} // namespace

namespace production_mppi_config_ros_detail {

void ProductionMppiConfigLoader::finalize() {
  ProductionMppiConfig::World& world = config_.world;
  ProductionMppiConfig::Planning& planning = config_.planning;
  ProductionMppiConfig::Execution& execution = config_.execution;
  ProductionMppiConfig::Control& control = config_.control;
  ProductionMppiConfig::Diagnostics& diagnostics = config_.diagnostics;

  if (!std::isfinite(planning.tick_rate_hz) || !(planning.tick_rate_hz > 0.0)) {
    throw std::invalid_argument{"tick_rate_hz must be finite and positive"};
  }
  const double planning_tick_period_s = 1.0 / planning.tick_rate_hz;
  if (!std::isfinite(planning.planning_tick_phase_offset_s) ||
      planning.planning_tick_phase_offset_s < 0.0 ||
      planning.planning_tick_phase_offset_s >= planning_tick_period_s) {
    throw std::invalid_argument{
        "planning tick phase offset must be in [0, one planning tick period)"};
  }
  // The planner ranks a tight metre by the time execution will actually spend
  // on it: the tracking-error tube law caps the speed there, and the same
  // tube configuration tells the planner how much.
  planning.persistent_planner.tracking_error_tube = control.tracking_error_tube;
  execution.stationary_hold_validity_ns = durationNanoseconds(
      execution.stationary_hold_validity_s, "stationary_hold_validity_s");
  execution.horizon_acknowledgement_grace_ns =
      durationNanoseconds(execution.horizon_acknowledgement_grace_ms * 1.0e-3,
                          "execution_horizon_acknowledgement_grace_ms", true);
  execution.mission_waypoint_capture_gate = MissionWaypointCaptureGateConfig{
      .goal_radius_m = planning.mission_goal_capture.capture_radius_m,
      .target_match_tolerance_m =
          execution.mission_waypoint_capture_gate.target_match_tolerance_m,
      .stop_speed_mps = planning.mission_waypoint_sequence.stop_speed_mps,
      .stop_hold_s = planning.mission_waypoint_sequence.stop_hold_s,
      .maximum_pose_age_s = execution.maximum_pose_age_ms * 1.0e-3,
      .maximum_vehicle_status_age_s = execution.maximum_vehicle_status_age_ms * 1.0e-3,
      .maximum_feedback_age_s = execution.maximum_control_feedback_age_ms * 1.0e-3,
  };
  const std::int64_t goal_capture_hold_ns =
      durationNanoseconds(planning.mission_waypoint_sequence.stop_hold_s,
                          "mission waypoint stop hold", true);
  const std::int64_t goal_capture_feedback_margin_ns =
      durationNanoseconds(execution.maximum_control_feedback_age_ms * 1.0e-3,
                          "mission goal capture feedback margin");
  const std::int64_t goal_capture_tick_margin_ns = durationNanoseconds(
      2.0 / planning.tick_rate_hz, "mission goal capture tick margin");
  execution.mission_goal_capture_hold_validity_ns = checkedDurationSum(
      goal_capture_hold_ns, goal_capture_feedback_margin_ns,
      goal_capture_tick_margin_ns, "mission goal capture hold lease");

  // The lean law: the airframe tilts with the acceleration it commands, so
  // the tube measures its clearance from the hull enveloped over every tilt
  // the dynamics reach and floors the speed where the leaning hull no longer
  // fits. The upright hull stays the hard rule of every validator: a taller
  // hard body turned every covered street into a trap once mapped, while the
  // hull passes them at the speed the lean law leaves it.
  control.tracking_error_tube.maximum_body_tilt_rad = maximumBodyTiltRad(
      static_cast<double>(control.mppi.dynamics.maximum_horizontal_acceleration_mps2),
      static_cast<double>(control.mppi.dynamics.maximum_vertical_acceleration_mps2));
  planning.cooperative_passage_volume.flight_envelope = world.flight_envelope;
  planning.cooperative_passage_volume.footprint = SweptFootprintConfig{
      .radius_m = world.physical_footprint.radius_m,
      .lower_extent_m = world.physical_footprint.lower_extent_m,
      .upper_extent_m = world.physical_footprint.upper_extent_m,
      .body_radius_m = world.physical_footprint.body_radius_m,
      .body_lower_extent_m = world.physical_footprint.body_lower_extent_m,
      .body_upper_extent_m = world.physical_footprint.body_upper_extent_m,
      .perimeter_samples = world.physical_footprint.perimeter_samples,
      .radial_rings = world.physical_footprint.radial_rings,
      .axial_samples = world.physical_footprint.axial_samples,
      .sweep_step_m = world.physical_footprint.sweep_step_m,
  };
  planning.cooperative_passage_route.footprint =
      planning.cooperative_passage_volume.footprint;

  planning.static_route_extension.maximum_horizontal_acceleration_mps2 =
      control.mppi.dynamics.maximum_horizontal_acceleration_mps2;
  planning.static_route_extension.maximum_vertical_acceleration_mps2 =
      control.mppi.dynamics.maximum_vertical_acceleration_mps2;
  planning.static_route_extension.maximum_control_jerk_mps3 =
      control.mppi.dynamics.maximum_control_jerk_mps3;
  planning.static_route_extension.stopping_capability =
      control.speed_policy.stopping_capability;
  planning.persistent_planner.time_model = FlightTimeModel3D{
      .maximum_horizontal_speed_mps =
          std::min(control.speed_policy.cruise_speed_mps,
                   control.speed_policy.absolute_speed_limit_mps),
      .maximum_vertical_speed_mps =
          static_cast<double>(control.mppi.dynamics.maximum_vertical_speed_mps),
      .maximum_translational_speed_mps =
          static_cast<double>(control.mppi.dynamics.maximum_translational_speed_mps),
      .translational_speed_limit_by_vertical_share =
          control.mppi.dynamics.translational_speed_limit_by_vertical_share,
      .maximum_horizontal_acceleration_mps2 = static_cast<double>(
          control.mppi.dynamics.maximum_horizontal_acceleration_mps2),
      .maximum_vertical_acceleration_mps2 =
          static_cast<double>(control.mppi.dynamics.maximum_vertical_acceleration_mps2),
      .maximum_control_jerk_mps3 =
          static_cast<double>(control.mppi.dynamics.maximum_control_jerk_mps3),
      .maximum_yaw_acceleration_radps2 =
          static_cast<double>(control.mppi.dynamics.maximum_yaw_acceleration_radps2),
      .maximum_yaw_rate_radps =
          static_cast<double>(control.mppi.dynamics.maximum_yaw_rate_radps),
  };
  planning.persistent_planner.minimum_continuous_turn_alignment =
      planning.future_route_connector.minimum_continuous_turn_alignment;
  planning.persistent_planner.physical_footprint = world.physical_footprint;
  planning.persistent_planner.flight_envelope = world.flight_envelope;

  diagnostics.rviz_period_ns =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics.rviz_rate_hz));
  diagnostics.info_period_ns =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics.info_rate_hz));
  diagnostics.file_period_ns =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics.file_rate_hz));

  execution.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      world.flight_envelope, control.mppi.dynamics, control.mppi.altitude_envelope,
      world.physical_footprint, execution.latest_sensor_obstacle_maximum_age_ms,
      execution.maximum_pose_prediction_age_ms,
      execution.maximum_control_feedback_age_ms,
      planning.optional_constraints.route_cross_track_constraints_enabled,
      planning.optional_constraints.latest_sensor_freshness_required,
      planning.optional_constraints.route_tracking_tube_constraints_enabled,
      control.speed_policy.minimum_target_lookahead_m);
  if (execution.validation_policy == nullptr) {
    throw std::invalid_argument{"invalid immutable execution validation policy"};
  }
}

} // namespace production_mppi_config_ros_detail

ProductionMppiConfig declareProductionMppiConfig(rclcpp::Node& node) {
  return production_mppi_config_ros_detail::ProductionMppiConfigLoader{node}.load();
}

} // namespace drone_city_nav
