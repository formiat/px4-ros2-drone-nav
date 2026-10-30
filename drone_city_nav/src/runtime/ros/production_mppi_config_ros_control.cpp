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
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_config_ros.hpp"
#include "production_mppi_config_ros_loader.hpp"

namespace drone_city_nav::production_mppi_config_ros_detail {

void ProductionMppiConfigLoader::declareControl() {
  ProductionMppiConfig::Control& control = config_.control;
  mppi::BenchmarkConfig& mppi = control.mppi;
  mppi.altitude_envelope = mppi::AltitudeEnvelopeConfig{
      .minimum_z_m =
          static_cast<float>(config_.world.flight_envelope.minimum_target_z_m),
      .maximum_z_m =
          static_cast<float>(config_.world.flight_envelope.maximum_target_z_m),
  };
  mppi.rollouts = declarePositiveSize("rollouts", 8192);
  // Roadmap item 28: the engine, cuda (the flight's), cpu (the build without
  // CUDA) or vulkan (way B).
  const std::string engine_name = declare<std::string>("mppi_engine", "cuda");
  const std::optional<mppi::MppiEngineKind> engine =
      mppi::parseMppiEngineKind(engine_name);
  if (!engine.has_value()) {
    throw std::invalid_argument{"mppi_engine must be cuda, cpu or vulkan, got '" +
                                engine_name + "'"};
  }
  control.engine = *engine;
  control.rollout_budget.full_rollouts = mppi.rollouts;
  control.rollout_budget.open_static_rollouts =
      declarePositiveSize("open_static_rollouts", 6144);
  control.rollout_budget.minimum_reduced_clearance_m =
      static_cast<float>(declare<double>("adaptive_rollout_minimum_clearance_m", 8.0));
  control.rollout_budget.maximum_world_age_ms =
      declare<double>("adaptive_rollout_maximum_world_age_ms", 250.0);
  const double dt_s = declare<double>("dt_s", 0.05);
  if (!std::isfinite(dt_s) || !(dt_s > 0.0)) {
    throw std::invalid_argument{"dt_s must be finite and positive"};
  }
  mppi.dynamics.dt_s = static_cast<float>(dt_s);
  const double static_horizon_duration_s =
      declare<double>("static_horizon_duration_s", 6.0);
  const double no_static_horizon_duration_s =
      declare<double>("no_static_horizon_duration_s", 4.0);
  const double active_horizon_duration_s = config_.world.use_static_map
                                               ? static_horizon_duration_s
                                               : no_static_horizon_duration_s;
  if (!std::isfinite(active_horizon_duration_s) || !(active_horizon_duration_s > 0.0)) {
    throw std::invalid_argument{"active horizon duration must be finite and positive"};
  }
  const double static_stale_esdf_execution_window_s = declare<double>(
      "static_stale_esdf_execution_window_s", static_horizon_duration_s);
  const double no_static_stale_esdf_execution_window_s = declare<double>(
      "no_static_stale_esdf_execution_window_s", no_static_horizon_duration_s);
  config_.execution.stale_esdf_execution_window_ms =
      1000.0 * (config_.world.use_static_map ? static_stale_esdf_execution_window_s
                                             : no_static_stale_esdf_execution_window_s);
  config_.execution.maximum_assembly_ms =
      declare<double>("execution_maximum_assembly_ms", 12.0);
  const double horizon_steps =
      std::ceil(active_horizon_duration_s / static_cast<double>(mppi.dynamics.dt_s));
  if (!std::isfinite(horizon_steps) || horizon_steps < 1.0 ||
      horizon_steps > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    throw std::invalid_argument{"MPPI horizon step count is not representable"};
  }
  mppi.steps = static_cast<std::size_t>(horizon_steps);

  control.speed_policy.horizon_duration_s = active_horizon_duration_s;
  control.speed_policy.cruise_speed_mps = declare<double>("cruise_speed_mps", 5.0);
  control.speed_policy.absolute_speed_limit_mps =
      declare<double>("absolute_speed_limit_mps", 10.0);
  const double maximum_horizontal_acceleration_mps2 =
      declare<double>("maximum_horizontal_acceleration_mps2", 4.0);
  const double maximum_vertical_acceleration_mps2 =
      declare<double>("maximum_vertical_acceleration_mps2", 4.0);
  const double maximum_control_jerk_mps3 =
      declare<double>("maximum_control_jerk_mps3", 12.0);
  // The lateral acceleration a turn may demand is a separate capability from
  // the longitudinal one: the airframe banks for it and PX4's tilt limit
  // admits more than the acceleration the speed profile is built from. The
  // curvature limiter and the route time profile read this one value.
  control.speed_policy.maximum_lateral_acceleration_mps2 =
      declare<double>("maximum_lateral_acceleration_mps2", 6.0);
  // The reference climbs no faster than the airframe can follow it, so a
  // limit that lifts cannot snap the reference back up.
  control.speed_policy.reference_speed_rise_mps2 = maximum_horizontal_acceleration_mps2;
  control.speed_policy.reference_tracking_lag_s =
      declare<double>("reference_tracking_lag_s", 0.0);
  control.speed_policy.stopping_capability
      .maximum_commanded_horizontal_deceleration_mps2 =
      maximum_horizontal_acceleration_mps2;
  control.speed_policy.stopping_capability.guaranteed_horizontal_deceleration_mps2 =
      maximum_horizontal_acceleration_mps2;
  control.speed_policy.stopping_capability.guaranteed_vertical_deceleration_mps2 =
      declare<double>("guaranteed_vertical_stopping_deceleration_mps2", 2.0);
  control.speed_policy.stopping_capability.reaction_latency_s =
      declare<double>("speed_reaction_latency_s", 0.10);
  control.speed_policy.sensor_braking_contract = SensorBrakingContract3D{
      .guaranteed_detection_range_m =
          declare<double>("guaranteed_lidar_detection_range_m", 30.0),
      .maximum_evidence_age_s =
          config_.execution.latest_sensor_obstacle_maximum_age_ms * 1.0e-3,
      .physical_margin_m = declare<double>("sensor_braking_physical_margin_m", 3.0),
      .maximum_horizontal_acceleration_mps2 = maximum_horizontal_acceleration_mps2,
      .maximum_vertical_acceleration_mps2 = maximum_vertical_acceleration_mps2,
      .maximum_control_jerk_mps3 = maximum_control_jerk_mps3,
      // Where the obstacle sensors look; the defaults are a lidar that sees
      // the whole sphere.
      .forward_vertical_half_angle_rad =
          declare<double>("forward_detection_vertical_half_angle_deg", 90.0) *
          std::numbers::pi / 180.0,
      .forward_horizontal_half_angle_rad =
          declare<double>("forward_detection_horizontal_half_angle_deg", 180.0) *
          std::numbers::pi / 180.0,
      .vertical_detection_range_m = declare<double>("vertical_detection_range_m", 0.0),
      .vertical_cone_half_angle_rad =
          declare<double>("vertical_detection_cone_half_angle_deg", 0.0) *
          std::numbers::pi / 180.0,
      .unobserved_speed_mps = declare<double>("unobserved_motion_speed_mps", 1.0),
      .vertical_physical_margin_m =
          declare<double>("vertical_sensor_braking_physical_margin_m", 0.0),
  };
  control.speed_policy.sensor_expected_beam_count =
      static_cast<std::size_t>(std::max<std::int64_t>(
          0, declare<std::int64_t>("sensor_expected_beam_count", 0)));
  mppi.gaze_follows_motion = declare<bool>("gaze_follows_motion", false);
  mppi.gaze_survey_yaw_rate_radps =
      static_cast<float>(declare<double>("gaze_survey_yaw_rate_radps", 0.0));
  mppi.gaze_minimum_horizontal_share = static_cast<float>(std::sin(
      control.speed_policy.sensor_braking_contract.vertical_cone_half_angle_rad));
  // Braking completes at the goal capture's stationary tolerance: a wider
  // margin leaves the vehicle drifting to rest outside the tolerance with no
  // reference speed left to close the gap.
  // Half the stationary hold tolerance: the limiter must still allow motion
  // at the capture boundary, or the vehicle settles just outside it.
  control.speed_policy.goal_margin_m = declare<double>(
      "goal_braking_margin_m", 0.5 * kStationaryExecutionHoldPositionToleranceM);
  control.speed_policy.curvature_preview_distance_m =
      declare<double>("curvature_preview_distance_m", 60.0);
  control.speed_policy.curvature_measurement_window_m =
      declare<double>("route_curvature_measurement_window_m", 5.0);
  control.speed_policy.minimum_target_lookahead_m =
      declare<double>("minimum_target_lookahead_m", 30.0);
  control.speed_policy.maximum_target_lookahead_m =
      declare<double>("maximum_target_lookahead_m", 100.0);
  mppi.stopping_capability = control.speed_policy.stopping_capability;
  mppi.dynamics.maximum_horizontal_speed_mps =
      static_cast<float>(control.speed_policy.absolute_speed_limit_mps);
  // Every rollout stays under the speed the contract admits along its own
  // direction, read from a table by the direction's vertical share; the
  // scalar bound is the largest entry. One bound for every
  // direction used to be the contract's worst case, a pure climb or descent:
  // with the 1.4 m/s^2 vertical law that is 4.62 m/s while level flight is
  // admitted at 5.65, and the urban flights r312 to r319 flew their level
  // legs at a p90 of 4.3 to 4.6 m/s against 4.8 to 5.0 before, a tenth of
  // their mean speed.
  TranslationalSpeedLimitByVerticalShare3D& speed_limit_table =
      mppi.dynamics.translational_speed_limit_by_vertical_share;
  float level_speed_limit_mps{0.0F};
  for (std::size_t sample = 0U;
       sample < TranslationalSpeedLimitByVerticalShare3D::kSamples; ++sample) {
    const double vertical_share =
        static_cast<double>(sample) /
        static_cast<double>(TranslationalSpeedLimitByVerticalShare3D::kSamples - 1U);
    const double limit_mps = sensorBrakingMaximumSpeedMps(
        control.speed_policy.sensor_braking_contract,
        control.speed_policy.stopping_capability,
        control.speed_policy.absolute_speed_limit_mps,
        Vec3{std::sqrt(std::max(0.0, 1.0 - vertical_share * vertical_share)), 0.0,
             vertical_share});
    speed_limit_table.limit_mps[sample] =
        std::nextafter(static_cast<float>(limit_mps), 0.0F);
    level_speed_limit_mps =
        std::max(level_speed_limit_mps, speed_limit_table.limit_mps[sample]);
  }
  mppi.dynamics.maximum_translational_speed_mps = level_speed_limit_mps;
  mppi.dynamics.maximum_horizontal_acceleration_mps2 =
      static_cast<float>(maximum_horizontal_acceleration_mps2);
  config_.execution.finite_horizon =
      mppi::makeFiniteHorizonConfig(control.speed_policy.stopping_capability);
  mppi.dynamics.maximum_control_jerk_mps3 =
      static_cast<float>(maximum_control_jerk_mps3);
  // The yaw the gaze may command; the time model's stationary turns and the
  // angular-derivative contract read the same two values below.
  mppi.dynamics.maximum_yaw_rate_radps =
      static_cast<float>(declare<double>("maximum_yaw_rate_radps", 1.5));
  mppi.dynamics.maximum_yaw_acceleration_radps2 =
      static_cast<float>(declare<double>("maximum_yaw_acceleration_radps2", 2.0));
  mppi.dynamics.maximum_vertical_acceleration_mps2 =
      static_cast<float>(maximum_vertical_acceleration_mps2);
  mppi.altitude_envelope.guaranteed_vertical_deceleration_mps2 = static_cast<float>(
      control.speed_policy.stopping_capability.guaranteed_vertical_deceleration_mps2);
  mppi.altitude_envelope.reaction_latency_s =
      static_cast<float>(control.speed_policy.stopping_capability.reaction_latency_s);
  mppi.dynamics.maximum_vertical_speed_mps =
      static_cast<float>(declare<double>("maximum_vertical_speed_mps", 5.0));

  const double angular_minimum_interval_ms =
      declare<double>("navigation_angular_derivative_minimum_interval_ms", 1.0);
  const double angular_maximum_interval_ms =
      declare<double>("navigation_angular_derivative_maximum_interval_ms", 100.0);
  control.navigation_angular_derivative = NavigationAngularDerivativeConfig{
      .minimum_interval_s = angular_minimum_interval_ms * 1.0e-3,
      .maximum_interval_s = angular_maximum_interval_ms * 1.0e-3,
      .maximum_yaw_rate_radps =
          static_cast<double>(mppi.dynamics.maximum_yaw_rate_radps),
      .maximum_yaw_acceleration_radps2 =
          static_cast<double>(mppi.dynamics.maximum_yaw_acceleration_radps2),
  };
  control.constrained_route.maximum_vertical_acceleration_mps2 =
      mppi.dynamics.maximum_vertical_acceleration_mps2;
  control.constrained_route.maximum_vertical_speed_mps =
      mppi.dynamics.maximum_vertical_speed_mps;
  control.constrained_route.alignment_distance_buffer_m =
      declare<double>("constrained_route_alignment_distance_buffer_m", 5.0);
  control.constrained_route.stationary_hold_distance_m =
      declare<double>("constrained_route_stationary_hold_distance_m", 2.0);
  control.constrained_route.vertical_capture_margin_m =
      declare<double>("constrained_route_vertical_capture_margin_m", 0.5);
  control.constrained_route.vertical_capture_speed_mps =
      declare<double>("constrained_route_vertical_capture_speed_mps", 0.75);
  control.tracking_error_tube.response_time_s =
      declare<double>("tracking_error_tube_response_time_s", 0.15);
  control.tracking_error_tube.minimum_progress_speed_mps =
      declare<double>("tracking_error_tube_minimum_progress_speed_mps", 1.0);
  // The live clearance speed limit enforces the same tube law as the route
  // certification, so it shares the tube configuration.
  control.speed_policy.clearance_response_time_s =
      control.tracking_error_tube.response_time_s;
  control.speed_policy.clearance_minimum_progress_speed_mps =
      control.tracking_error_tube.minimum_progress_speed_mps;

  mppi.footprint = mppi::FootprintConfig{
      .radius_m = static_cast<float>(config_.world.physical_footprint.radius_m),
      .lower_extent_m =
          static_cast<float>(config_.world.physical_footprint.lower_extent_m),
      .upper_extent_m =
          static_cast<float>(config_.world.physical_footprint.upper_extent_m),
      .body_radius_m =
          static_cast<float>(config_.world.physical_footprint.body_radius_m),
      .body_lower_extent_m =
          static_cast<float>(config_.world.physical_footprint.body_lower_extent_m),
      .body_upper_extent_m =
          static_cast<float>(config_.world.physical_footprint.body_upper_extent_m),
      .perimeter_samples = static_cast<std::uint32_t>(
          config_.world.physical_footprint.perimeter_samples),
      .radial_rings =
          static_cast<std::uint32_t>(config_.world.physical_footprint.radial_rings),
      .axial_samples =
          static_cast<std::uint32_t>(config_.world.physical_footprint.axial_samples),
  };
  mppi.footprint.clearance_broad_phase_enabled =
      declare<bool>("mppi_footprint_clearance_broad_phase_enabled", true);
  mppi.costs.temperature = static_cast<float>(declare<double>("mppi_temperature", 8.0));
  mppi.costs.target_effective_sample_fraction = static_cast<float>(
      declare<double>("mppi_target_effective_sample_fraction", 0.07));
  mppi.costs.maximum_temperature_growth =
      static_cast<float>(declare<double>("mppi_maximum_temperature_growth", 1000.0));
  mppi.costs.body_collision_gate_enabled =
      declare<bool>("mppi_body_collision_gate_enabled", false);
  mppi.costs.route_directed_candidate_cost_tolerance = static_cast<float>(
      declare<double>("route_directed_candidate_cost_tolerance", 0.5));
  mppi.costs.route_directed_candidate_switch_ticks =
      static_cast<std::uint32_t>(std::max<std::int64_t>(
          1, declare<std::int64_t>("route_directed_candidate_switch_ticks", 3)));
  mppi.costs.head_progress_horizon_s =
      static_cast<float>(declare<double>("head_progress_horizon_s", 0.4));
  mppi.costs.head_progress_weight =
      static_cast<float>(declare<double>("head_progress_weight", 8.0));
  mppi.costs.guide_deviation_weight =
      static_cast<float>(declare<double>("guide_deviation_weight", 1.0));
  mppi.costs.altitude_tracking_weight =
      static_cast<float>(declare<double>("altitude_tracking_weight", 4.0));
  mppi.costs.route_progress_integral_weight =
      static_cast<float>(declare<double>("route_progress_integral_weight", 2.0));
  const float clearance_preference_weight =
      static_cast<float>(declare<double>("clearance_preference_weight", 2.0));
  const float obstacle_approach_weight =
      static_cast<float>(declare<double>("obstacle_approach_weight", 40.0));
  if (config_.planning.optional_constraints.clearance_costs_enabled) {
    mppi.costs.clearance_preference_weight = clearance_preference_weight;
    mppi.costs.obstacle_approach_weight = obstacle_approach_weight;
  } else {
    mppi.costs.clearance_preference_weight = 0.0F;
    mppi.costs.obstacle_approach_weight = 0.0F;
  }
  mppi.horizon_sampling.full_rate_duration_s =
      static_cast<float>(declare<double>("far_horizon_full_rate_duration_s", 2.0));
  mppi.horizon_sampling.far_cost_stride =
      static_cast<std::uint32_t>(declare<std::int64_t>("far_horizon_cost_stride", 2));
  mppi.costs.speed_tracking_weight =
      static_cast<float>(declare<double>("speed_tracking_weight", 1.0));
  mppi.costs.overspeed_weight =
      static_cast<float>(declare<double>("overspeed_weight", 200.0));
  mppi.risk.critical_distance_m =
      static_cast<float>(declare<double>("critical_distance_m", 1.0));
  mppi.risk.preferred_distance_m =
      static_cast<float>(declare<double>("preferred_distance_m", 6.0));
  // One source for each law. The tube law the rollout cost charges beside the
  // motion is the route certification's tube; the stopping law it charges
  // ahead of the motion is the speed policy's stopping capability. Neither is
  // a separate tunable: two copies of one law drift apart, and the reference
  // speed then admits what the optimiser prices as too fast.
  mppi.risk.tube_response_time_s =
      static_cast<float>(control.tracking_error_tube.response_time_s);
  mppi.risk.stopping_response_time_s =
      static_cast<float>(control.speed_policy.stopping_capability.reaction_latency_s);
  mppi.risk.stopping_deceleration_mps2 = static_cast<float>(
      control.speed_policy.stopping_capability.guaranteed_horizontal_deceleration_mps2);
  const std::int64_t seed = declare<std::int64_t>("seed", 42);
  if (seed < 0) {
    throw std::invalid_argument{"seed must be non-negative"};
  }
  mppi.seed = static_cast<std::uint64_t>(seed);
  mppi.early_exit_on_altitude_envelope_violation = true;

  mppi.cooperative.desired_minimum_separation_m = static_cast<float>(
      declare<double>("cooperative_desired_minimum_separation_m", 5.0));
  mppi.cooperative.candidate_acceleration_fraction = static_cast<float>(
      declare<double>("cooperative_candidate_acceleration_fraction", 0.75));
  mppi.cooperative.candidate_duration_s =
      static_cast<float>(declare<double>("cooperative_candidate_duration_s", 1.5));
  mppi.costs.peer_separation_weight =
      static_cast<float>(declare<double>("cooperative_peer_separation_weight", 80.0));
  mppi.costs.cooperative_maneuver_preference_weight = static_cast<float>(
      declare<double>("cooperative_maneuver_preference_weight", 1.5));
}

} // namespace drone_city_nav::production_mppi_config_ros_detail
