#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"
#include "drone_city_nav/ros_conversions.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <span>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {

void ProductionMppiNode::planningTick() {
  if (!engine_) {
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const ProductionTrackingObjective* tracking_objective =
      objective && objective->tracking.has_value() ? &objective->tracking.value()
                                                   : nullptr;
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const bool terminal_hold_enabled = !objective || !objective->continuous_tracking;
  const bool direct_tracking_interception =
      objective && objective->continuous_tracking && tracking_objective != nullptr &&
      tracking_objective->direct_interception_active;
  const std::uint64_t line_of_sight_generation =
      tracking_objective != nullptr ? tracking_objective->line_of_sight_generation : 0U;
  const std::uint64_t effective_guide_generation =
      direct_tracking_interception
          ? (std::uint64_t{1} << 63U) | line_of_sight_generation
          : 0U;
  const StaticRouteObjective current_route_objective =
      objective ? makeStaticRouteObjective(*objective) : StaticRouteObjective{};
  const std::uint64_t required_route_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  const std::uint64_t required_route_sample =
      objective && objective->mission_epoch == required_route_epoch
          ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
          : 0U;
  const auto snapshot_started = std::chrono::steady_clock::now();
  ProductionMppiNavigation navigation;
  ProductionMppiPredictionError prediction;
  ProductionMppiAppliedControl applied_control;
  std::uint64_t memory_sequence{0U};
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    prediction = latest_prediction_error_;
    applied_control = applied_control_;
    memory_sequence = memory_sequence_;
  }
  std::optional<ProductionMppiPreparedEsdf> esdf;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    esdf = prepared_esdf_;
  }
  const std::shared_ptr<const msg::RawObstacleSnapshot> latest_raw_snapshot =
      latest_raw_snapshot_.load(std::memory_order_acquire);
  const auto raw_revision = [&](const std::uint64_t esdf_revision) {
    return !use_static_map_ && latest_raw_snapshot
               ? latest_raw_snapshot->obstacle_snapshot_revision
               : esdf_revision;
  };
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const double pose_age_ms =
      static_cast<double>(now_ns - navigation.receive_stamp_ns) / 1.0e6;
  double esdf_age_ms = std::numeric_limits<double>::infinity();
  if (esdf.has_value()) {
    esdf_age_ms = use_static_map_
                      ? 0.0
                      : static_cast<double>(now_ns - esdf->ready_stamp_ns) / 1.0e6;
  }
  const double control_feedback_age_ms =
      applied_control.valid
          ? static_cast<double>(now_ns - applied_control.receive_stamp_ns) / 1.0e6
          : std::numeric_limits<double>::infinity();
  if (!navigation.valid || pose_age_ms < 0.0) {
    return;
  }
  bool pose_predicted = false;
  if (pose_age_ms > maximum_pose_age_ms_) {
    const NavigationStatePredictionResult predicted =
        predictNavigationState(navigation.state, pose_age_ms / 1000.0,
                               maximum_pose_prediction_age_ms_ / 1000.0);
    if (!predicted.valid) {
      return;
    }
    navigation.state = predicted.state;
    pose_predicted = predicted.predicted;
  }
  if (!esdf.has_value() || esdf_age_ms < 0.0 ||
      esdf_age_ms > maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_) {
    const ProductionMppiPreparedEsdf stale_esdf =
        esdf.value_or(ProductionMppiPreparedEsdf{});
    mppi::MppiTickInput input{
        .initial_state = navigation.state,
        .target = navigation.state,
        .pose_revision = navigation.revision,
        .obstacle_revision = raw_revision(stale_esdf.revision),
        .planning_stamp_ns = now_ns,
        .previous_applied_control = std::nullopt,
        .nominal_reseed_generation = 0U,
        .reference_speed_mps = 0.0F,
        .moving_target = std::nullopt,
        .route = std::nullopt,
        .active_rollouts = std::nullopt,
    };
    const MppiHorizonSafetyResult fallback =
        buildMppiBrakingFallback(input.initial_state, safety_config_);
    mppi::MppiTickResult result;
    result.horizon = fallback.fallback_horizon;
    result.controls = fallback.fallback_controls;
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.known_solid_collision = false;
    result.esdf_revision = stale_esdf.revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
    const ProductionMppiPlanningState planning_state =
        ProductionMppiPlanningState::kUnavailableWorldBrakingHold;
    ++tick_sequence_;
    recordTickStatistics(result, planning_state, false);
    ProductionMppiExecutionPublication execution =
        publishExecutionHorizon(input, result, stale_esdf, planning_state, now_ns);
    std::optional<ProductionMppiRvizSnapshot> rviz;
    if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
      rviz = ProductionMppiRvizSnapshot{
          .candidate_horizon = result.horizon,
          .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                           : std::vector<mppi::State>{},
          .execution_horizon = execution.horizon,
          .route = stale_esdf.mppi_route,
          .channel_edges = stale_esdf.channel_edges,
          .selected_channel_ids = stale_esdf.selected_channel_ids,
      };
      last_rviz_stamp_ns_ = now_ns;
    }
    ProductionMppiPreparedEsdf diagnostic_esdf = stale_esdf;
    diagnostic_esdf.distances_m.reset();
    diagnostic_esdf.mppi_route.reset();
    enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot{
        .input = input,
        .result = result,
        .esdf = std::move(diagnostic_esdf),
        .stability = {},
        .prediction = prediction,
        .liveness = {},
        .speed_policy = {},
        .guide_progress = {},
        .goal_capture = {},
        .execution = std::move(execution),
        .planning_state = planning_state,
        .rviz = std::move(rviz),
        .objective = objective,
        .target_source = "unavailable_world_braking",
        .tick_sequence = tick_sequence_,
        .memory_sequence = memory_sequence,
        .pose_age_ms = pose_age_ms,
        .esdf_age_ms = esdf_age_ms,
        .control_feedback_age_ms = control_feedback_age_ms,
        .snapshot_ms = result.timings.host_total_ms,
        .stability_ms = 0.0,
        .liveness_reseed_requested = false,
        .pose_predicted = pose_predicted,
        .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
    });
    previous_result_ = std::move(result);
    return;
  }
  if (!engine_->ready()) {
    return;
  }
  const bool route_objective_matches = staticRouteObjectiveMatches(
      esdf->route_objective, current_route_objective, required_route_sample,
      std::numeric_limits<double>::infinity());
  if (!direct_tracking_interception && objective && objective->continuous_tracking &&
      !route_objective_matches) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TRACKING_ROUTE_HANDOFF status=waiting_for_current_route "
        "current_epoch=%" PRIu64 " current_sample=%" PRIu64 " required_sample=%" PRIu64
        " route_epoch=%" PRIu64 " route_sample=%" PRIu64,
        objective->mission_epoch, objective->sample_sequence, required_route_sample,
        esdf->route_objective.mission_epoch, esdf->route_objective.sample_sequence);
  }
  bool route_usable = !direct_tracking_interception && route_objective_matches;
  bool route_cross_track_rejected = false;
  const std::span<const Point2> guide =
      route_usable && esdf->route_2d_projection
          ? std::span<const Point2>{*esdf->route_2d_projection}
          : std::span<const Point2>{};
  if (tracked_route_generation_ != esdf->global_guide_generation) {
    tracked_route_generation_ = esdf->global_guide_generation;
    tracked_route_station_m_ = 0.0;
  }
  GlobalGuideProjection measured_route_projection;
  if (route_usable && esdf->route_3d) {
    const RouteProjection3D measured_route_3d = projectOntoRoute3D(
        *esdf->route_3d,
        Point3{navigation.state.x, navigation.state.y, navigation.state.z},
        tracked_route_station_m_);
    measured_route_projection = GlobalGuideProjection{
        .valid = measured_route_3d.valid,
        .station_m = measured_route_3d.station_m,
        .total_length_m =
            esdf->route_3d->empty() ? 0.0 : esdf->route_3d->back().station_m,
        .remaining_m = measured_route_3d.remaining_m,
        .cross_track_m = measured_route_3d.distance_m,
        .point = Point2{measured_route_3d.point.x, measured_route_3d.point.y},
    };
  } else if (route_usable && esdf->route_2d_projection) {
    measured_route_projection = projectOntoGlobalGuide(
        *esdf->route_2d_projection, Point2{navigation.state.x, navigation.state.y},
        tracked_route_station_m_);
  }
  if (measured_route_projection.valid) {
    tracked_route_station_m_ =
        std::max(tracked_route_station_m_, measured_route_projection.station_m);
  }
  if (measured_route_projection.valid &&
      measured_route_projection.cross_track_m >
          active_guide_config_.maximum_cross_track_m) {
    route_usable = false;
    route_cross_track_rejected = true;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TRACKING_ROUTE_HANDOFF status=rejected_cross_track "
        "route_generation=%" PRIu64 " cross_track_m=%.2f maximum_m=%.2f",
        esdf->global_guide_generation, measured_route_projection.cross_track_m,
        active_guide_config_.maximum_cross_track_m);
    requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged,
                        esdf->global_guide_generation);
    measured_route_projection = {};
  }
  GlobalGuideProjection route_projection = measured_route_projection;
  if (route_projection.valid) {
    route_projection.station_m = tracked_route_station_m_;
    route_projection.remaining_m =
        std::max(0.0, route_projection.station_m + route_projection.remaining_m -
                          tracked_route_station_m_);
  }
  if (use_static_map_ && objective && objective->continuous_tracking) {
    maybeRequestStaticTrackingWorldRefresh(*esdf, navigation, *objective, now_ns);
  }
  if (use_static_map_ && route_usable && route_projection.valid) {
    maybeRequestStaticRouteExtension(*esdf, navigation, route_projection, now_ns);
  }
  const std::span<const RouteSample3D> route_3d =
      route_usable && esdf->route_3d ? std::span<const RouteSample3D>{*esdf->route_3d}
                                     : std::span<const RouteSample3D>{};
  const std::span<const ConstrainedRouteSpan> constrained_spans =
      route_usable && esdf->constrained_spans
          ? std::span<const ConstrainedRouteSpan>{*esdf->constrained_spans}
          : std::span<const ConstrainedRouteSpan>{};
  const ConstrainedRouteObservation route_constraint = observeConstrainedRoute(
      route_3d, constrained_spans, esdf->global_guide_generation,
      route_projection.station_m,
      Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
      route_envelope_config_, lattice_3d_config_.planning_goal_distance_m);
  const ConstrainedRouteControl route_control = constrained_route_coordinator_.update(
      route_constraint, speed_policy_config_.cruise_speed_mps,
      constrained_route_control_config_);
  const MissionGoalCaptureResult goal_capture =
      mission_goal_capture_latch_ && terminal_hold_enabled
          ? mission_goal_capture_latch_->update(MissionGoalCaptureObservation{
                .mission_goal = mission_goal,
                .state = navigation.state,
                .terminal_route_available = esdf->global_guide_reaches_mission_goal,
            })
          : MissionGoalCaptureResult{};
  MppiSpeedPolicyResult speed_policy = evaluateMppiSpeedPolicy(
      speed_policy_config_,
      MppiSpeedPolicyInput{
          .state = navigation.state,
          .mission_goal = mission_goal,
          .guide = guide,
          .route_constraint_speed_limit_mps =
              route_control.active
                  ? std::optional<double>{route_control.speed_limit_mps}
                  : std::nullopt,
          .terminal_goal_limit_enabled = terminal_hold_enabled,
      });
  std::string target_source;
  double target_station_m = 0.0;
  mppi::State target;
  if (direct_tracking_interception) {
    target = mppi::State{
        .x = static_cast<float>(mission_goal.x),
        .y = static_cast<float>(mission_goal.y),
        .z = static_cast<float>(mission_goal.z),
        .yaw = navigation.state.yaw,
    };
    target_source = tracking_objective != nullptr &&
                            tracking_objective->predicted_intercept_path_clear
                        ? "tracking_direct_full_prediction"
                        : "tracking_direct_shortened_prediction";
  } else if (!route_usable && objective && objective->continuous_tracking) {
    target = navigation.state;
    target_source = route_cross_track_rejected ? "tracking_route_cross_track_rejected"
                                               : "tracking_route_objective_mismatch";
  } else {
    target =
        selectTarget(*esdf, tracked_route_station_m_, speed_policy.target_lookahead_m,
                     target_source, target_station_m);
  }
  if (route_control.active) {
    target.z = static_cast<float>(route_control.reference_z_m);
    if (route_control.hold_xy) {
      target_source = "channel_vertical_alignment_hold";
    } else if (route_control.vertical_ready) {
      target_source = "channel_traversal";
    } else {
      target_source = "channel_vertical_alignment";
    }
    if (route_control.hold_xy) {
      target.x = navigation.state.x;
      target.y = navigation.state.y;
      speed_policy.reference_speed_mps = 0.0;
      speed_policy.target_lookahead_m = 0.0;
    }
  }
  ProductionMppiPlanningState planning_state = ProductionMppiPlanningState::kPlanned;
  if (objective && objective->immediate_hold) {
    planning_state = ProductionMppiPlanningState::kNoGuideBrakingHold;
    target = navigation.state;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_command_braking_hold";
  } else if (goal_capture.latched) {
    planning_state = ProductionMppiPlanningState::kMissionGoalPositionHold;
    target = mppi::State{
        .x = static_cast<float>(mission_goal.x),
        .y = static_cast<float>(mission_goal.y),
        .z = static_cast<float>(mission_goal.z),
        .yaw = navigation.state.yaw,
    };
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_goal_position_hold";
  } else if (target_source == "mission_goal_direct" ||
             target_source == "tracking_route_objective_mismatch" ||
             target_source == "tracking_route_cross_track_rejected") {
    planning_state = ProductionMppiPlanningState::kNoGuideBrakingHold;
    target = navigation.state;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = target_source == "mission_goal_direct"
                        ? "no_guide_braking_hold"
                        : target_source + "_braking_hold";
  }
  const bool control_feedback_fresh =
      applied_control.valid && control_feedback_age_ms >= 0.0 &&
      control_feedback_age_ms <= maximum_control_feedback_age_ms_;
  MppiLivenessResult liveness;
  if (liveness_supervisor_ && !direct_tracking_interception) {
    liveness = liveness_supervisor_->evaluate(MppiLivenessObservation{
        .stamp_ns = now_ns,
        .actual_state = navigation.state,
        .controller_active = control_feedback_fresh &&
                             planning_state == ProductionMppiPlanningState::kPlanned &&
                             !route_control.hold_xy && route_projection.valid,
        .emergency_braking =
            control_feedback_fresh && applied_control.emergency_braking,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .predicted_terminal_progress_m =
            previous_result_.has_value() ? previous_result_->terminal_progress_m : 0.0,
        .route_generation = esdf->global_guide_generation,
        .route_station_m = route_projection.station_m,
        .route_station_valid = route_projection.valid,
    });
  }
  GlobalGuideProgressUpdate guide_progress;
  if (guide_progress_tracker_ && !direct_tracking_interception) {
    const GlobalGuideProjection& projection = route_projection;
    guide_progress = guide_progress_tracker_->evaluate(GlobalGuideProgressObservation{
        .stamp_ns = now_ns,
        .guide_generation =
            projection.valid && planning_state == ProductionMppiPlanningState::kPlanned
                ? esdf->global_guide_generation
                : 0U,
        .station_m = projection.station_m,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .controller_active = control_feedback_fresh && !route_control.hold_xy,
        .emergency_braking =
            control_feedback_fresh && applied_control.emergency_braking,
    });
    if (guide_progress.stalled) {
      requestGuideRelease(guide_progress.persistent_safety_rejection
                              ? GlobalGuideReleaseReason::kPersistentSafetyRejection
                              : GlobalGuideReleaseReason::kStalled,
                          esdf->global_guide_generation);
    }
  }
  const MppiNominalReseedUpdate nominal_reseed =
      nominal_reseed_tracker_.update(MppiNominalReseedObservation{
          .guide_generation = direct_tracking_interception
                                  ? effective_guide_generation
                                  : esdf->global_guide_generation,
          .local_liveness_generation = liveness.reseed_generation,
          .guide_liveness_generation = guide_progress.local_reseed_generation,
          .safety_rejection_generation = guide_progress.persistent_safety_rejection
                                             ? guide_progress.stall_generation
                                             : 0U,
      });
  if (risk_escalation_) {
    const bool stable_progress = previous_result_.has_value() &&
                                 previous_result_->eligible_risk_contract.available &&
                                 guide_progress.progress_m > 1.0e-3;
    maximum_eligible_risk_tier_ =
        risk_escalation_
            ->update(MppiRiskEscalationObservation{
                .reseed_generation =
                    liveness.reseed_generation + guide_progress.local_reseed_generation,
                .no_eligible_recovery_generation =
                    nominal_reseed.no_eligible_recovery_generation,
                .stable_progress = stable_progress,
            })
            .maximum_eligible_tier;
  }
  mppi::RiskTier route_required_risk_tier = mppi::RiskTier::kPreferred;
  if (route_usable && esdf->mppi_route && route_projection.valid) {
    const double horizon_distance_m = std::max(
        target_station_m - route_projection.station_m,
        speed_policy.reference_speed_mps * static_cast<double>(mppi_config_.steps) *
            static_cast<double>(mppi_config_.dynamics.dt_s));
    route_required_risk_tier = mppi::maximumRequiredRiskTier(
        *esdf->mppi_route, static_cast<float>(route_projection.station_m),
        static_cast<float>(route_projection.station_m +
                           std::max(0.0, horizon_distance_m)));
    maximum_eligible_risk_tier_ = static_cast<mppi::RiskTier>(
        std::max(static_cast<std::uint8_t>(maximum_eligible_risk_tier_),
                 static_cast<std::uint8_t>(route_required_risk_tier)));
  }
  std::optional<mppi::MovingTargetReference> moving_target;
  if (objective && objective->continuous_tracking && objective->tracking.has_value()) {
    const ProductionTrackingObjective& tracking = objective->tracking.value();
    const double observation_age_s = static_cast<double>(std::max<std::int64_t>(
                                         0, now_ns - tracking.observation_stamp_ns)) *
                                     1.0e-9;
    const TargetVerticalPrediction vertical_prediction = predictTargetVerticalMotion(
        tracking.observed_position.z, tracking.observed_velocity.z, observation_age_s,
        mppi_config_.dynamics.maximum_vertical_acceleration_mps2,
        flight_envelope_config_);
    const float minimum_z =
        static_cast<float>(flight_envelope_config_.minimum_target_z_m);
    const float maximum_z = std::nextafter(
        static_cast<float>(flight_envelope_config_.maximum_target_z_m), minimum_z);
    if (vertical_prediction.valid && std::isfinite(minimum_z) &&
        std::isfinite(maximum_z) && maximum_z > minimum_z) {
      const float bounded_vertical_z = mppi::clampMovingTargetAltitude(
          static_cast<float>(vertical_prediction.z_m), minimum_z, maximum_z);
      moving_target = mppi::MovingTargetReference{
          .state =
              mppi::State{
                  .x = static_cast<float>(tracking.observed_position.x +
                                          tracking.observed_velocity.x *
                                              observation_age_s),
                  .y = static_cast<float>(tracking.observed_position.y +
                                          tracking.observed_velocity.y *
                                              observation_age_s),
                  .z = bounded_vertical_z,
                  .vx = static_cast<float>(tracking.observed_velocity.x),
                  .vy = static_cast<float>(tracking.observed_velocity.y),
                  .vz = static_cast<float>(vertical_prediction.velocity_mps),
              },
          .capture_radius_m = static_cast<float>(tracking_capture_radius_m_),
          .vertical_deceleration_mps2 =
              mppi_config_.dynamics.maximum_vertical_acceleration_mps2,
          .minimum_z_m = minimum_z,
          .maximum_z_m = maximum_z,
          .bounded_vertical_motion = true,
      };
    }
  }
  mppi::MppiTickInput input{
      .initial_state = navigation.state,
      .target = target,
      .pose_revision = navigation.revision,
      .obstacle_revision = raw_revision(esdf->revision),
      .planning_stamp_ns = now_ns,
      .previous_applied_control =
          control_feedback_fresh ? std::optional<mppi::Control>{applied_control.control}
                                 : std::nullopt,
      .nominal_reseed_generation = nominal_reseed.generation,
      .reference_speed_mps = speed_policy.enabled
                                 ? static_cast<float>(speed_policy.reference_speed_mps)
                                 : -1.0F,
      .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
      .moving_target = moving_target,
      .route =
          route_usable && esdf->mppi_route && route_projection.valid &&
                  !route_control.hold_xy
              ? std::optional<mppi::RouteReference>{mppi::RouteReference{
                    .points = esdf->mppi_route,
                    .generation = esdf->global_guide_generation,
                    .initial_station_m = static_cast<float>(route_projection.station_m),
                }}
              : std::nullopt,
      .active_rollouts = direct_tracking_interception
                             ? std::optional<std::size_t>{direct_tracking_rollouts_}
                             : std::nullopt,
  };
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  mppi::MppiTickResult result;
  MppiEligibleRolloutUpdate no_eligible_recovery{
      .no_eligible_recovery_generation = nominal_reseed.no_eligible_recovery_generation,
      .phase = nominal_reseed.no_eligible_phase,
  };
  if (planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold) {
    const MppiHorizonSafetyResult fallback =
        buildMppiBrakingFallback(input.initial_state, safety_config_);
    result.horizon = fallback.fallback_horizon;
    result.controls = fallback.fallback_controls;
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.esdf_revision = esdf->revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
  } else if (planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold) {
    result.horizon = {target, target};
    result.controls = {mppi::Control{}};
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.known_solid_collision = false;
    result.esdf_revision = esdf->revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
  } else {
    try {
      result = engine_->plan(input);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "PRODUCTION_MPPI_TICK failed: %s", error.what());
      return;
    }
    no_eligible_recovery = nominal_reseed_tracker_.observeEligibleRolloutResult(
        result.eligible_risk_contract.available, result.nominal_reseeded);
    if (no_eligible_recovery.guide_replan_requested && !direct_tracking_interception) {
      requestGuideRelease(GlobalGuideReleaseReason::kNoEligibleRollouts,
                          esdf->global_guide_generation);
    }
  }
  ++tick_sequence_;
  recordTickStatistics(result, planning_state,
                       liveness.reseed_requested ||
                           guide_progress.local_reseed_requested);
  ProductionMppiExecutionPublication execution =
      publishExecutionHorizon(input, result, *esdf, planning_state, now_ns);

  const auto stability_started = std::chrono::steady_clock::now();
  const ProductionMppiStability stability = compareWithPrevious(result);
  const double stability_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - stability_started)
                                  .count();
  std::optional<ProductionMppiRvizSnapshot> rviz;
  if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
    std::shared_ptr<const std::vector<mppi::RouteSample3D>> rviz_route =
        esdf->mppi_route;
    if (direct_tracking_interception) {
      const std::vector<Point3> direct_points{
          Point3{navigation.state.x, navigation.state.y, navigation.state.z},
          mission_goal,
      };
      rviz_route = makeMppiRoute3D(
          sampleRoute3D(
              direct_points,
              std::max(0.5, distance3D(direct_points.front(), direct_points.back())),
              speed_policy.reference_speed_mps),
          {}, speed_policy.reference_speed_mps, speed_policy.reference_speed_mps);
    }
    rviz = ProductionMppiRvizSnapshot{
        .candidate_horizon = result.horizon,
        .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                         : std::vector<mppi::State>{},
        .execution_horizon = execution.horizon,
        .route = std::move(rviz_route),
        .channel_edges = esdf->channel_edges,
        .selected_channel_ids = esdf->selected_channel_ids,
    };
    last_rviz_stamp_ns_ = now_ns;
  }

  mppi::MppiTickResult diagnostic_result;
  diagnostic_result.eligible_risk_contract = result.eligible_risk_contract;
  diagnostic_result.post_update_classification = result.post_update_classification;
  diagnostic_result.post_update_repair = result.post_update_repair;
  diagnostic_result.post_update_backtrack_ratio = result.post_update_backtrack_ratio;
  diagnostic_result.selected_tier = result.selected_tier;
  diagnostic_result.raw_collision = result.raw_collision;
  diagnostic_result.known_solid_collision = result.known_solid_collision;
  diagnostic_result.critical_exposure_m = result.critical_exposure_m;
  diagnostic_result.planning_exposure_m = result.planning_exposure_m;
  diagnostic_result.minimum_esdf_distance_m = result.minimum_esdf_distance_m;
  diagnostic_result.head_progress_m = result.head_progress_m;
  diagnostic_result.terminal_progress_m = result.terminal_progress_m;
  diagnostic_result.minimum_target_separation_m = result.minimum_target_separation_m;
  diagnostic_result.predicted_capture_time_s = result.predicted_capture_time_s;
  diagnostic_result.maximum_acceleration_mps2 = result.maximum_acceleration_mps2;
  diagnostic_result.maximum_jerk_mps3 = result.maximum_jerk_mps3;
  diagnostic_result.first_control_delta = result.first_control_delta;
  diagnostic_result.warm_start_shift_s = result.warm_start_shift_s;
  diagnostic_result.nominal_reseeded = result.nominal_reseeded;
  diagnostic_result.esdf_revision = result.esdf_revision;
  diagnostic_result.active_rollouts = result.active_rollouts;
  diagnostic_result.gpu_batch_size = result.gpu_batch_size;
  diagnostic_result.timings = result.timings;
  ProductionMppiPreparedEsdf diagnostic_esdf = *esdf;
  diagnostic_esdf.distances_m.reset();
  if (!rviz.has_value()) {
    diagnostic_esdf.mppi_route.reset();
  }
  enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot{
      .input = input,
      .result = std::move(diagnostic_result),
      .esdf = std::move(diagnostic_esdf),
      .stability = stability,
      .prediction = prediction,
      .liveness = liveness,
      .speed_policy = speed_policy,
      .guide_progress = guide_progress,
      .no_eligible_recovery = no_eligible_recovery,
      .goal_capture = goal_capture,
      .execution = std::move(execution),
      .planning_state = planning_state,
      .rviz = std::move(rviz),
      .objective = objective,
      .target_source = target_source,
      .tick_sequence = tick_sequence_,
      .memory_sequence = memory_sequence,
      .pose_age_ms = pose_age_ms,
      .esdf_age_ms = esdf_age_ms,
      .control_feedback_age_ms = control_feedback_age_ms,
      .route_station_m = route_projection.station_m,
      .route_remaining_m = route_projection.remaining_m,
      .snapshot_ms = snapshot_ms,
      .stability_ms = stability_ms,
      .route_projection_valid = route_projection.valid,
      .liveness_reseed_requested = liveness.reseed_requested,
      .pose_predicted = pose_predicted,
      .route_required_risk_tier = route_required_risk_tier,
      .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
  });
  {
    const std::scoped_lock lock{input_mutex_};
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
}

} // namespace drone_city_nav
