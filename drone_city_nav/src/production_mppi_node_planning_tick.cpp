#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"

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
  const bool observed_3d_world =
      !use_static_map_ &&
      no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D;
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
  std::optional<ProductionMppiCooperativeCommand> cooperative_command;
  ProductionMppiNonCooperativeTracks noncooperative_tracks;
  LatestObservation latest_observation;
  std::uint64_t memory_sequence{0U};
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    prediction = latest_prediction_error_;
    applied_control = applied_control_;
    cooperative_command = cooperative_command_;
    noncooperative_tracks = noncooperative_tracks_;
    latest_observation = latest_observation_tracker_.latest();
    memory_sequence = latest_observation.sequence;
  }
  std::optional<ProductionMppiPreparedEsdf> esdf;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    esdf = prepared_esdf_;
  }
  const std::shared_ptr<const ProductionMppiRawWorld2D> latest_raw_world =
      latest_raw_world_.load(std::memory_order_acquire);
  const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world_3d =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  const auto raw_revision = [&](const std::uint64_t esdf_revision) {
    if (use_static_map_) {
      return esdf_revision;
    }
    if (no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D) {
      return latest_raw_world_3d ? latest_raw_world_3d->version.revision
                                 : esdf_revision;
    }
    return latest_raw_world ? latest_raw_world->version.revision : esdf_revision;
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
  double observation_age_ms = std::numeric_limits<double>::infinity();
  if (use_static_map_) {
    observation_age_ms = 0.0;
  } else if (esdf.has_value() && latest_observation.available() &&
             latest_observation.producer_instance_id == esdf->producer_instance_id) {
    observation_age_ms = latest_observation.ageMs(now_ns);
  } else if (esdf.has_value() && !latest_observation.available()) {
    std::int64_t raw_ready_stamp_ns{0};
    if (observed_3d_world && latest_raw_world_3d &&
        latest_raw_world_3d->version.producer_instance_id ==
            esdf->producer_instance_id) {
      raw_ready_stamp_ns = latest_raw_world_3d->ready_stamp_ns;
    } else if (!observed_3d_world && latest_raw_world &&
               latest_raw_world->version.producer_instance_id ==
                   esdf->producer_instance_id) {
      raw_ready_stamp_ns = latest_raw_world->ready_stamp_ns;
    }
    if (raw_ready_stamp_ns > 0 && now_ns >= raw_ready_stamp_ns) {
      observation_age_ms = static_cast<double>(now_ns - raw_ready_stamp_ns) * 1.0e-6;
    }
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
  if (!esdf.has_value() || observation_age_ms < 0.0 ||
      observation_age_ms > maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PRODUCTION_MPPI_UNAVAILABLE_WORLD action=wait_for_world "
                         "observation_age_ms=%.1f esdf_content_age_ms=%.1f "
                         "maximum_execution_age_ms=%.1f",
                         observation_age_ms, esdf_age_ms,
                         maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_);
    return;
  }
  if (!esdf->local_world_generation.coherent()) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PRODUCTION_MPPI_UNAVAILABLE_WORLD action=wait_for_coherent_generation "
        "local_world_generation=%" PRIu64,
        esdf->local_world_generation.generation);
    return;
  }
  if (!engine_->ready()) {
    return;
  }
  maybeObserveIncrementalTopology3D(*esdf, navigation, now_ns);
  const bool uses_3d_route =
      use_static_map_ ||
      no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D;
  if (uses_3d_route && esdf->global_guide_generation == 0U) {
    requestGuideRelease(GlobalGuideReleaseReason::kNoActiveGuide, 0U);
  }
  ProductionRouteExecutionSelection3D route_execution;
  bool route_usable{false};
  RouteExecutionStatus3D route_execution_status{RouteExecutionStatus3D::kNoActiveRoute};
  GlobalGuideProjection measured_route_projection;
  Point3 route_hold_position{
      navigation.state.x,
      navigation.state.y,
      clampToFlightEnvelope(navigation.state.z, flight_envelope_config_)
          .value_or(flight_envelope_config_.minimum_target_z_m),
  };
  if (uses_3d_route) {
    route_execution = resolveRouteExecution3D(
        *esdf, objective.get(), navigation, latest_raw_world_3d, required_route_sample,
        direct_tracking_interception, observed_3d_world);
    route_usable = route_execution.route_usable;
    route_execution_status = route_execution.status;
    measured_route_projection = route_execution.projection;
    route_hold_position = route_execution.hold_position;
  } else {
    const bool route_objective_matches = staticRouteObjectiveMatches(
        esdf->route_objective, current_route_objective, required_route_sample,
        std::numeric_limits<double>::infinity());
    route_usable = !direct_tracking_interception && route_objective_matches;
    if (!direct_tracking_interception && objective && objective->continuous_tracking &&
        !route_objective_matches) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ROUTE_HANDOFF status=waiting_for_current_route continuous_tracking=true "
          "current_epoch=%" PRIu64 " current_sample=%" PRIu64
          " required_sample=%" PRIu64 " route_epoch=%" PRIu64 " route_sample=%" PRIu64,
          objective->mission_epoch, objective->sample_sequence, required_route_sample,
          esdf->route_objective.mission_epoch, esdf->route_objective.sample_sequence);
    }
    if (tracked_route_generation_ != esdf->global_guide_generation) {
      tracked_route_generation_ = esdf->global_guide_generation;
      tracked_route_station_m_ = 0.0;
    }
    if (route_usable && esdf->route_2d_projection) {
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
      route_execution_status = RouteExecutionStatus3D::kExcessiveCrossTrack;
      requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged,
                          esdf->global_guide_generation);
      measured_route_projection = {};
    } else if (route_usable && !measured_route_projection.valid) {
      route_usable = false;
      route_execution_status = RouteExecutionStatus3D::kInvalidProjection;
      requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged,
                          esdf->global_guide_generation);
    } else if (route_usable) {
      route_execution_status = RouteExecutionStatus3D::kUsable;
    } else if (!route_objective_matches) {
      route_execution_status = RouteExecutionStatus3D::kObjectiveMismatch;
    }
  }
  const ProductionActivatedRoute3D* const activated_route =
      uses_3d_route ? route_execution.route.get() : nullptr;
  const ProductionRouteGeometry3D* const route_geometry =
      activated_route != nullptr ? &activated_route->geometry : nullptr;
  const std::uint64_t route_generation = activated_route != nullptr
                                             ? activated_route->identity.generation
                                             : esdf->global_guide_generation;
  const bool route_reaches_mission_goal =
      activated_route != nullptr
          ? activated_route->identity.proposal.reaches_mission_goal
          : esdf->global_guide_reaches_mission_goal;
  const bool route_reaches_intent_target =
      activated_route != nullptr
          ? activated_route->identity.proposal.evidence.reaches_intent_target
          : esdf->route_segment_evidence.reaches_intent_target;
  const RouteIntent3D& route_intent = activated_route != nullptr
                                          ? activated_route->identity.proposal.intent
                                          : esdf->route_intent;
  const StaticRouteObjective& active_route_objective =
      activated_route != nullptr ? activated_route->identity.proposal.objective
                                 : esdf->route_objective;
  const RouteEndpointSemantics3D route_endpoint_semantics = routeEndpointSemantics3D(
      route_intent, route_reaches_intent_target, route_reaches_mission_goal);
  const std::uint64_t route_continuity_id = routeContinuityId3D(
      route_intent,
      RouteContinuityLineage3D{
          .mission_epoch = active_route_objective.mission_epoch,
          .assignment_generation = active_route_objective.assignment_generation,
          .target_detection_id = active_route_objective.target_detection_id,
          .target_track_id = active_route_objective.target_track_id,
      });
  const std::uint64_t route_geometry_revision =
      activated_route != nullptr ? activated_route->identity.proposal.route_fingerprint
                                 : esdf->route_fingerprint;
  const Lattice3DRoutePurpose route_purpose = route_geometry != nullptr
                                                  ? route_geometry->route_purpose
                                                  : esdf->lattice_3d_route_purpose;
  const std::shared_ptr<const std::vector<RouteSample3D>> execution_route =
      route_geometry != nullptr ? route_geometry->route : esdf->route_3d;
  const std::shared_ptr<const std::vector<mppi::RouteSample3D>> execution_mppi_route =
      route_geometry != nullptr ? route_geometry->mppi_route : esdf->mppi_route;
  const std::shared_ptr<const std::vector<Point2>> execution_route_2d_projection =
      route_geometry != nullptr ? route_geometry->route_2d_projection
                                : esdf->route_2d_projection;
  const std::shared_ptr<const std::vector<ConstrainedRouteSpan>>
      execution_constrained_spans =
          route_geometry != nullptr ? route_geometry->constrained_spans
                                    : esdf->constrained_spans;
  const std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      execution_cooperative_passage_assignments =
          route_geometry != nullptr ? route_geometry->cooperative_passage_assignments
                                    : esdf->cooperative_passage_assignments;
  const std::shared_ptr<const std::vector<PassageTraversalId>>
      execution_selected_passage_traversal_ids =
          route_geometry != nullptr ? route_geometry->selected_passage_traversal_ids
                                    : esdf->selected_passage_traversal_ids;
  const std::span<const Point2> guide =
      route_usable && execution_route_2d_projection
          ? std::span<const Point2>{*execution_route_2d_projection}
          : std::span<const Point2>{};
  const bool route_execution_blocked =
      !direct_tracking_interception && objective && !route_usable;
  const double route_station_m =
      uses_3d_route ? route_execution.station_m : tracked_route_station_m_;
  GlobalGuideProjection route_projection = measured_route_projection;
  if (route_projection.valid) {
    route_projection.station_m = route_station_m;
    route_projection.remaining_m =
        std::max(0.0, route_projection.total_length_m - route_station_m);
  }
  if (use_static_map_ && objective && objective->continuous_tracking) {
    maybeRequestStaticTrackingWorldRefresh(*esdf, navigation, *objective, now_ns);
  }
  if ((use_static_map_ || observed_3d_world) && route_usable &&
      route_projection.valid) {
    maybeRequestStaticRouteExtension(*esdf, navigation, route_projection, now_ns);
  }
  const std::span<const RouteSample3D> route_3d =
      route_usable && execution_route ? std::span<const RouteSample3D>{*execution_route}
                                      : std::span<const RouteSample3D>{};
  const std::span<const ConstrainedRouteSpan> constrained_spans =
      route_usable && execution_constrained_spans
          ? std::span<const ConstrainedRouteSpan>{*execution_constrained_spans}
          : std::span<const ConstrainedRouteSpan>{};
  const ConstrainedRouteObservation route_constraint = observeConstrainedRoute(
      route_3d, constrained_spans, route_generation, route_projection.station_m,
      Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
      route_envelope_config_, lattice_3d_config_.planning_goal_distance_m);
  const Point3 actual_position{navigation.state.x, navigation.state.y,
                               navigation.state.z};
  for (const PassageTraversalEvidenceEvent& event :
       passage_traversal_evidence_tracker_.update(route_constraint, actual_position,
                                                  now_ns)) {
    RCLCPP_INFO(get_logger(),
                "PASSAGE_TRAVERSAL_EVENT vehicle_id='%s' sequence=%" PRIu64
                " status=%s reason=%s passage='%s' route_generation=%" PRIu64
                " span_index=%zu observations=%zu duration_s=%.3f station_m=%.2f "
                "span_station_m=(%.2f,%.2f) position=(%.2f,%.2f,%.2f) "
                "maximum_cross_track_m=%.3f maximum_vertical_error_m=%.3f "
                "vertical_window_preserved=%s",
                vehicle_id_.c_str(), event.sequence,
                passageTraversalEvidenceStatusName(event.status).data(),
                passageTraversalEvidenceReasonName(event.reason).data(),
                event.passage_traversal_id.c_str(), event.route_generation,
                event.span_index, event.traversal_observation_count, event.duration_s,
                event.station_m, event.begin_station_m, event.end_station_m,
                event.actual_position.x, event.actual_position.y,
                event.actual_position.z, event.maximum_cross_track_error_m,
                event.maximum_absolute_vertical_error_m,
                event.vertical_window_preserved ? "true" : "false");
  }
  std::vector<PassageGeometryObservation> passage_geometry_observations;
  const PassageTraversalEdge* nearest_passage_entry = nullptr;
  RouteProjection3D nearest_passage_projection;
  double nearest_passage_entry_distance_m = std::numeric_limits<double>::infinity();
  if (esdf->passage_traversals) {
    passage_geometry_observations.reserve(esdf->passage_traversals->size());
    for (const PassageTraversalEdge& passage : *esdf->passage_traversals) {
      const RouteProjection3D projection =
          projectOntoRoute3D(passage.centerline, actual_position);
      const double entry_distance_m = distance3D(actual_position, passage.entry);
      if (entry_distance_m < nearest_passage_entry_distance_m) {
        nearest_passage_entry = &passage;
        nearest_passage_projection = projection;
        nearest_passage_entry_distance_m = entry_distance_m;
      }
      const double traversal_length_m =
          passage.centerline.empty() ? 0.0 : passage.centerline.back().station_m;
      passage_geometry_observations.push_back(PassageGeometryObservation{
          .passage_traversal_id = passage.id,
          .within_corridor =
              projection.valid && projection.distance_m <= passage.minimum_clearance_m,
          .station_m = projection.station_m,
          .traversal_length_m = traversal_length_m,
          .cross_track_error_m = projection.distance_m,
      });
    }
  }
  if (nearest_passage_entry != nullptr && nearest_passage_entry_distance_m < 8.0) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PASSAGE_GEOMETRY_PROXIMITY vehicle_id='%s' passage='%s' "
                         "entry_distance_m=%.2f projection_station_m=%.2f "
                         "projection_cross_track_m=%.2f minimum_clearance_m=%.2f "
                         "within_corridor=%s",
                         vehicle_id_.c_str(), nearest_passage_entry->id.c_str(),
                         nearest_passage_entry_distance_m,
                         nearest_passage_projection.station_m,
                         nearest_passage_projection.distance_m,
                         nearest_passage_entry->minimum_clearance_m,
                         nearest_passage_projection.valid &&
                                 nearest_passage_projection.distance_m <=
                                     nearest_passage_entry->minimum_clearance_m
                             ? "true"
                             : "false");
  }
  for (const PassageGeometryEvidenceEvent& event :
       passage_geometry_evidence_tracker_.update(passage_geometry_observations,
                                                 actual_position, now_ns,
                                                 PassageGeometryEvidenceConfig{})) {
    RCLCPP_INFO(get_logger(),
                "PASSAGE_GEOMETRY_EVENT vehicle_id='%s' sequence=%" PRIu64
                " status=%s reason=%s passage='%s' observations=%zu "
                "duration_s=%.3f station_m=%.2f traversal_length_m=%.2f "
                "maximum_station_m=%.2f position=(%.2f,%.2f,%.2f) "
                "maximum_cross_track_m=%.3f",
                vehicle_id_.c_str(), event.sequence,
                passageTraversalEvidenceStatusName(event.status).data(),
                passageTraversalEvidenceReasonName(event.reason).data(),
                event.passage_traversal_id.c_str(), event.observation_count,
                event.duration_s, event.station_m, event.traversal_length_m,
                event.maximum_station_m, event.actual_position.x,
                event.actual_position.y, event.actual_position.z,
                event.maximum_cross_track_error_m);
  }
  const ConstrainedRouteControl route_control = constrained_route_coordinator_.update(
      route_constraint, speed_policy_config_.cruise_speed_mps,
      constrained_route_control_config_);
  const MissionGoalCaptureResult goal_capture =
      mission_goal_capture_latch_ && terminal_hold_enabled
          ? mission_goal_capture_latch_->update(MissionGoalCaptureObservation{
                .mission_goal = mission_goal,
                .state = navigation.state,
                .terminal_route_available = route_reaches_mission_goal,
            })
          : MissionGoalCaptureResult{};
  const MissionWaypointUpdate waypoint_update =
      updateMissionWaypoint(objective, navigation, goal_capture, now_ns);
  const bool temporary_frontier_is_terminal =
      route_usable && route_projection.valid && !route_reaches_mission_goal;
  MppiSpeedPolicyResult speed_policy = evaluateMppiSpeedPolicy(
      speed_policy_config_,
      MppiSpeedPolicyInput{
          .state = navigation.state,
          .mission_goal = mission_goal,
          .guide = guide,
          .route_endpoint_remaining_m =
              route_usable && route_projection.valid && !route_reaches_mission_goal
                  ? std::optional<double>{route_projection.remaining_m}
                  : std::nullopt,
          .route_constraint_speed_limit_mps =
              route_control.active
                  ? std::optional<double>{route_control.speed_limit_mps}
                  : std::nullopt,
          .terminal_goal_limit_enabled = terminal_hold_enabled,
      });
  const std::span<const CooperativePassageAssignment> passage_assignments =
      execution_cooperative_passage_assignments
          ? std::span<
                const CooperativePassageAssignment>{*execution_cooperative_passage_assignments}
          : std::span<const CooperativePassageAssignment>{};
  const ProductionMppiCooperativeUpdate cooperative =
      prepareCooperativeTick(passage_assignments, route_constraint, cooperative_command,
                             now_ns, speed_policy.reference_speed_mps);
  const ProductionMppiNonCooperativeUpdate noncooperative =
      prepareNonCooperativeTick(navigation.state, noncooperative_tracks, now_ns);
  const bool noncooperative_cost_influence_active =
      noncooperative.enabled &&
      noncooperative.avoidance.influence.cost_influence_active;
  const bool noncooperative_evasive_maneuver_active =
      noncooperative.enabled &&
      noncooperative.avoidance.influence.evasive_maneuver_active;
  if (cooperative.yield.active) {
    speed_policy.reference_speed_mps =
        std::min(speed_policy.reference_speed_mps, cooperative.yield.maximum_speed_mps);
    speed_policy.target_lookahead_m = std::min(
        speed_policy.target_lookahead_m,
        std::max(0.0, cooperative.yield.hold_station_m - route_projection.station_m));
  }
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
  } else if (route_execution_blocked) {
    target = mppi::State{
        .x = static_cast<float>(route_hold_position.x),
        .y = static_cast<float>(route_hold_position.y),
        .z = static_cast<float>(route_hold_position.z),
        .yaw = navigation.state.yaw,
    };
    target_source = objective->continuous_tracking ? "tracking_no_executable_route_"
                                                   : "no_executable_route_";
    target_source += routeExecutionStatus3DName(route_execution_status);
  } else {
    const std::span<const mppi::RouteSample3D> mppi_route =
        execution_mppi_route
            ? std::span<const mppi::RouteSample3D>{*execution_mppi_route}
            : std::span<const mppi::RouteSample3D>{};
    target =
        selectTarget(route_3d, mppi_route, route_station_m,
                     speed_policy.target_lookahead_m, target_source, target_station_m);
  }
  if (route_control.active) {
    target.z = static_cast<float>(route_control.reference_z_m);
    if (route_control.hold_xy) {
      target_source = "passage_vertical_alignment_hold";
    } else if (route_control.vertical_ready) {
      target_source = "passage_traversal";
    } else {
      target_source = "passage_vertical_alignment";
    }
    if (route_control.hold_xy) {
      target.x = navigation.state.x;
      target.y = navigation.state.y;
      speed_policy.reference_speed_mps = 0.0;
      speed_policy.target_lookahead_m = 0.0;
    }
  }
  if (cooperative.yield.active && route_usable && route_projection.valid &&
      execution_route && !execution_route->empty() && !route_control.hold_xy) {
    const RouteSample3D hold_sample =
        sampleRoute3DAtStation(*execution_route, cooperative.yield.hold_station_m);
    target.x = static_cast<float>(hold_sample.position.x);
    target.y = static_cast<float>(hold_sample.position.y);
    target.z = static_cast<float>(hold_sample.position.z);
    target_station_m = hold_sample.station_m;
    target_source = cooperative.yield.hold_at_entry
                        ? "cooperative_passage_yield_hold"
                        : "cooperative_passage_yield_deceleration";
  }
  ProductionMppiPlanningState planning_state = ProductionMppiPlanningState::kPlanned;
  if (objective && objective->immediate_hold) {
    planning_state = ProductionMppiPlanningState::kMissionCommandPositionHold;
    target = mppi::State{
        .x = static_cast<float>(mission_goal.x),
        .y = static_cast<float>(mission_goal.y),
        .z = static_cast<float>(mission_goal.z),
        .yaw = navigation.state.yaw,
    };
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_command_position_hold";
  } else if (goal_capture.latched && !waypoint_update.advanced) {
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
  } else if (route_execution_blocked) {
    planning_state = ProductionMppiPlanningState::kNoExecutableRouteHold;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
  } else if (cooperative.yield.active && cooperative.yield.hold_at_entry &&
             !cooperative.mppi.avoidance_active && !route_control.hold_xy) {
    planning_state = ProductionMppiPlanningState::kCooperativePassageYieldHold;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "cooperative_passage_yield_hold";
  }
  const bool control_feedback_fresh =
      applied_control.valid && control_feedback_age_ms >= 0.0 &&
      control_feedback_age_ms <= maximum_control_feedback_age_ms_;
  std::optional<mppi::Control> previous_applied_control;
  ProductionMppiPreviousControlSource previous_control_source =
      ProductionMppiPreviousControlSource::kEngineFallback;
  if (control_feedback_fresh) {
    previous_applied_control = applied_control.control;
    previous_control_source = ProductionMppiPreviousControlSource::kOffboardFeedback;
  } else if (navigation.measured_acceleration_valid && !pose_predicted) {
    previous_applied_control = navigation.measured_equivalent_control;
    previous_control_source =
        ProductionMppiPreviousControlSource::kMeasuredAcceleration;
  }
  MppiLivenessResult liveness;
  if (liveness_supervisor_ && !direct_tracking_interception) {
    liveness = liveness_supervisor_->evaluate(MppiLivenessObservation{
        .stamp_ns = now_ns,
        .actual_state = navigation.state,
        .controller_active = control_feedback_fresh &&
                             planning_state == ProductionMppiPlanningState::kPlanned &&
                             !route_control.hold_xy && route_projection.valid,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .predicted_terminal_progress_m =
            previous_result_.has_value() ? previous_result_->terminal_progress_m : 0.0,
        .route_generation = route_generation,
        .route_station_m = route_projection.station_m,
        .route_station_valid = route_projection.valid,
    });
  }
  GlobalGuideProgressUpdate guide_progress;
  if (guide_progress_tracker_ && !direct_tracking_interception &&
      route_purpose != Lattice3DRoutePurpose::kLaunchDeparture &&
      route_purpose != Lattice3DRoutePurpose::kObservationFrontier) {
    const GlobalGuideProjection& projection = route_projection;
    guide_progress = guide_progress_tracker_->evaluate(GlobalGuideProgressObservation{
        .stamp_ns = now_ns,
        .guide_generation =
            projection.valid && planning_state == ProductionMppiPlanningState::kPlanned
                ? route_generation
                : 0U,
        .station_m = projection.station_m,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .controller_active = control_feedback_fresh &&
                             planning_state == ProductionMppiPlanningState::kPlanned &&
                             !route_control.hold_xy,
    });
    if (guide_progress.stalled) {
      requestGuideRelease(GlobalGuideReleaseReason::kStalled, route_generation);
    }
  }
  mppi::RiskTier route_required_risk_tier = mppi::RiskTier::kPreferred;
  if (route_usable && execution_mppi_route && route_projection.valid) {
    const double horizon_distance_m = std::max(
        target_station_m - route_projection.station_m,
        speed_policy.reference_speed_mps * static_cast<double>(mppi_config_.steps) *
            static_cast<double>(mppi_config_.dynamics.dt_s));
    route_required_risk_tier = mppi::maximumRequiredRiskTier(
        *execution_mppi_route, static_cast<float>(route_projection.station_m),
        static_cast<float>(route_projection.station_m +
                           std::max(0.0, horizon_distance_m)));
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
  DirectTrackingManeuverUpdate direct_tracking_maneuver;
  if (direct_tracking_interception && moving_target.has_value()) {
    direct_tracking_maneuver =
        direct_tracking_maneuver_lifecycle_.update(DirectTrackingManeuverObservation{
            .interceptor_position =
                Point3{navigation.state.x, navigation.state.y, navigation.state.z},
            .interceptor_velocity =
                Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
            .target_position = Point3{moving_target->state.x, moving_target->state.y,
                                      moving_target->state.z},
            .target_velocity = Vec3{moving_target->state.vx, moving_target->state.vy,
                                    moving_target->state.vz},
            .stamp_ns = now_ns,
            .line_of_sight_generation = line_of_sight_generation,
            .active = planning_state == ProductionMppiPlanningState::kPlanned,
        });
  } else {
    direct_tracking_maneuver = direct_tracking_maneuver_lifecycle_.update({});
  }
  const MppiNominalReseedUpdate nominal_reseed =
      nominal_reseed_tracker_.update(MppiNominalReseedObservation{
          .guide_generation = direct_tracking_interception ? effective_guide_generation
                                                           : route_generation,
          .local_liveness_generation = liveness.reseed_generation,
          .guide_liveness_generation = guide_progress.local_reseed_generation,
          .direct_tracking_maneuver_generation =
              direct_tracking_maneuver.reseed_generation,
      });
  const EsdfQueryResult current_clearance =
      queryConservativeEsdf3D(esdf->grid, *esdf->distances_m, navigation.state.x,
                              navigation.state.y, navigation.state.z);
  const double tracking_age_ms =
      tracking_objective != nullptr && tracking_objective->observation_stamp_ns > 0
          ? static_cast<double>(std::max<std::int64_t>(
                0, now_ns - tracking_objective->observation_stamp_ns)) /
                1.0e6
          : std::numeric_limits<double>::infinity();
  const MppiRolloutBudgetDecision rollout_budget = selectMppiRolloutBudget(
      rollout_budget_config_,
      MppiRolloutBudgetObservation{
          .static_world = use_static_map_,
          .guide_available =
              direct_tracking_interception || (route_usable && route_projection.valid),
          .direct_tracking = direct_tracking_interception,
          .clearance_valid = current_clearance.status == EsdfQueryStatus::kValid,
          .clearance_m = current_clearance.clearance_m,
          .world_age_ms = observation_age_ms,
          .tracking_age_ms = tracking_age_ms,
          .required_risk_tier = direct_tracking_interception
                                    ? mppi::RiskTier::kPreferred
                                    : route_required_risk_tier,
      });
  mppi::DeterministicCandidateKind deterministic_candidate =
      mppi::DeterministicCandidateKind::kDisabled;
  if (direct_tracking_interception) {
    deterministic_candidate =
        mppi::DeterministicCandidateKind::kTargetDirectedReacquisition;
  } else if (planning_state == ProductionMppiPlanningState::kPlanned && route_usable &&
             route_projection.valid && !route_control.hold_xy) {
    deterministic_candidate = mppi::DeterministicCandidateKind::kRouteDirectedCruise;
  }
  mppi::MppiTickInput input{
      .initial_state = navigation.state,
      .target = target,
      .pose_revision = navigation.revision,
      .obstacle_revision = raw_revision(esdf->revision),
      .expected_esdf_revision = esdf->local_world_generation.gpu_esdf_revision,
      .planning_stamp_ns = now_ns,
      .previous_applied_control = previous_applied_control,
      .nominal_reseed_generation = nominal_reseed.generation,
      .reference_speed_mps = speed_policy.enabled
                                 ? static_cast<float>(speed_policy.reference_speed_mps)
                                 : -1.0F,
      .moving_target = moving_target,
      .route =
          planning_state == ProductionMppiPlanningState::kPlanned && route_usable &&
                  execution_mppi_route && route_projection.valid &&
                  !route_control.hold_xy
              ? std::optional<mppi::RouteReference>{mppi::RouteReference{
                    .points = execution_mppi_route,
                    .generation = route_generation,
                    .initial_station_m = static_cast<float>(route_projection.station_m),
                }}
              : std::nullopt,
      .dynamic_aircraft = noncooperative_cost_influence_active
                              ? noncooperative.avoidance.trajectories
                              : cooperative.mppi.dynamic_aircraft,
      .dynamic_aircraft_cost_policy =
          noncooperative_cost_influence_active
              ? std::optional<mppi::DynamicAircraftCostPolicy>{noncooperative.avoidance
                                                                   .cost_policy}
              : std::nullopt,
      .cooperative_maneuver = cooperative.mppi.maneuver,
      .cooperative_acquisition = cooperative.mppi.acquisition,
      .noncooperative_acquisition = noncooperative_evasive_maneuver_active
                                        ? noncooperative.avoidance.acquisition
                                        : std::nullopt,
      .active_rollouts = rollout_budget.active_rollouts,
      .deterministic_candidate = deterministic_candidate,
      .cooperative_avoidance_active = cooperative.mppi.avoidance_active,
      .noncooperative_avoidance_active = noncooperative_evasive_maneuver_active,
  };
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  mppi::MppiTickResult result;
  MppiEligibleRolloutUpdate no_eligible_recovery{
      .no_eligible_recovery_generation = nominal_reseed.no_eligible_recovery_generation,
      .phase = nominal_reseed.no_eligible_phase,
  };
  if (planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold ||
      planning_state == ProductionMppiPlanningState::kNoExecutableRouteHold) {
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
    if (result.route_directed_candidate_injected &&
        !result.route_directed_candidate_raw_safe && !direct_tracking_interception) {
      // A route-directed seed is only one controller candidate. Its rejection does
      // not invalidate the certified route geometry while the remaining MPPI
      // rollouts can still provide an executable control result.
      bool event_applied{false};
      const RouteLifecycleEvent3D event{
          .kind = RouteLifecycleEventKind3D::kControlCandidateRejected,
          .generation = route_generation,
      };
      {
        const std::scoped_lock lock{route_supervisor_mutex_};
        event_applied = route_supervisor_.applyEvent(event);
      }
      static_cast<void>(execution_arbiter_.observe(event));
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ROUTE_EXECUTION status=seed_not_executable route_generation=%" PRIu64
          " cross_track_m=%.2f alternative_rollout_available=%s "
          "event_applied=%s action=reject_control_candidate",
          route_generation, route_projection.cross_track_m,
          result.feasibility_contract.available ? "true" : "false",
          event_applied ? "true" : "false");
    }
    no_eligible_recovery = nominal_reseed_tracker_.observeEligibleRolloutResult(
        result.feasibility_contract.available, result.nominal_reseeded);
    if (no_eligible_recovery.guide_replan_requested && !direct_tracking_interception) {
      requestGuideRelease(GlobalGuideReleaseReason::kNoEligibleRollouts,
                          route_generation);
    }
  }
  if (result.cooperative_acquisition_reseeded) {
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_SEPARATION_ACQUISITION_RESEED available=%s "
                "positive_progress=%s backward_fallback=%s candidate_index=%zu "
                "head_progress_m=%.3f terminal_progress_m=%.3f separation_gain_m=%.3f "
                "route_generation=%" PRIu64,
                result.cooperative_acquisition_available ? "true" : "false",
                result.cooperative_acquisition_positive_progress ? "true" : "false",
                result.cooperative_acquisition_backward_fallback ? "true" : "false",
                result.cooperative_acquisition_candidate_index,
                static_cast<double>(result.cooperative_acquisition_head_progress_m),
                static_cast<double>(result.cooperative_acquisition_terminal_progress_m),
                static_cast<double>(result.cooperative_acquisition_separation_gain_m),
                route_generation);
  }
  if (result.cooperative_release_reseeded) {
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_SEPARATION_RELEASE_RESEED route_generation=%" PRIu64,
                route_generation);
  }
  if (result.noncooperative_acquisition_reseeded) {
    RCLCPP_INFO(
        get_logger(),
        "NONCOOPERATIVE_SEPARATION_ACQUISITION_RESEED available=%s "
        "candidate_index=%zu maneuver=%s minimum_separation_m=%.3f "
        "separation_gain_m=%.3f head_progress_m=%.3f terminal_progress_m=%.3f "
        "lifecycle_generation=%" PRIu64,
        result.noncooperative_acquisition_available ? "true" : "false",
        result.noncooperative_acquisition_candidate_index,
        mppi::nonCooperativeManeuverName(result.noncooperative_acquisition_maneuver),
        static_cast<double>(result.noncooperative_acquisition_minimum_separation_m),
        static_cast<double>(result.noncooperative_acquisition_separation_gain_m),
        static_cast<double>(result.noncooperative_acquisition_head_progress_m),
        static_cast<double>(result.noncooperative_acquisition_terminal_progress_m),
        noncooperative.avoidance.lifecycle_generation);
  }
  if (result.noncooperative_release_reseeded) {
    RCLCPP_INFO(get_logger(),
                "NONCOOPERATIVE_SEPARATION_RELEASE_RESEED "
                "lifecycle_generation=%" PRIu64,
                noncooperative.avoidance.lifecycle_generation);
  }
  ++tick_sequence_;
  ProductionMppiExecutionPublication execution =
      publishExecutionHorizon(input, result, *esdf, planning_state, now_ns);
  const bool raw_invalidation_active =
      route_execution_status == RouteExecutionStatus3D::kRawCollision;
  const bool finite_braking_tail_active =
      execution.retained_previous_finite_path && execution.terminal_rest_state;
  const RollingRouteTelemetryObservation3D rolling_route{
      .route_generation = route_generation,
      .continuity_id = route_continuity_id,
      .geometry_revision = route_geometry_revision,
      .endpoint_semantics = effectiveRouteEndpointSemantics3D(
          route_endpoint_semantics, raw_invalidation_active,
          finite_braking_tail_active),
      .route_remaining_m = route_projection.valid
                               ? route_projection.remaining_m
                               : std::numeric_limits<double>::infinity(),
      .speed_mps = routeSpeed3D(
          Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz}),
      .resident_route_available = uses_3d_route && !direct_tracking_interception &&
                                  esdf->activated_route_3d != nullptr,
      .execution_owner_available = !uses_3d_route || direct_tracking_interception ||
                                   route_execution.execution_owner_available,
      .endpoint_limiter_active =
          speed_policy.active_limiter == MppiSpeedLimiter::kRouteEndpoint,
      .raw_invalidation_active = raw_invalidation_active,
      .finite_braking_tail_active = finite_braking_tail_active,
      .nominal_reseeded = result.nominal_reseeded,
  };
  recordTickStatistics(result, planning_state, execution,
                       liveness.reseed_requested ||
                           guide_progress.local_reseed_requested,
                       rolling_route);

  const auto stability_started = std::chrono::steady_clock::now();
  const ProductionMppiStability stability = compareWithPrevious(result);
  const double stability_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - stability_started)
                                  .count();
  std::optional<ProductionMppiRvizSnapshot> rviz;
  if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
    std::shared_ptr<const std::vector<mppi::RouteSample3D>> rviz_route =
        route_usable ? execution_mppi_route : nullptr;
    if (direct_tracking_interception) {
      const std::vector<Point3> direct_points{
          Point3{navigation.state.x, navigation.state.y, navigation.state.z},
          mission_goal,
      };
      rviz_route =
          makeMppiRoute3D(sampleRoute3D(direct_points,
                                        std::max(0.5, distance3D(direct_points.front(),
                                                                 direct_points.back())),
                                        speed_policy.reference_speed_mps),
                          {}, speed_policy.reference_speed_mps,
                          speed_policy.reference_speed_mps, speed_policy_config_);
    }
    rviz = ProductionMppiRvizSnapshot{
        .candidate_horizon = result.horizon,
        .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                         : std::vector<mppi::State>{},
        .execution_horizon = execution.horizon,
        .route = std::move(rviz_route),
        .passage_traversals = esdf->passage_traversals,
        .selected_passage_traversal_ids = execution_selected_passage_traversal_ids,
    };
    last_rviz_stamp_ns_ = now_ns;
  }

  {
    const std::scoped_lock lock{input_mutex_};
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
  mppi::MppiTickResult diagnostic_result = std::move(result);
  diagnostic_result.horizon.clear();
  diagnostic_result.controls.clear();
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
      .direct_tracking_maneuver = direct_tracking_maneuver,
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
      .observation_age_ms = observation_age_ms,
      .control_feedback_age_ms = control_feedback_age_ms,
      .route_station_m = route_projection.station_m,
      .route_remaining_m = route_projection.remaining_m,
      .snapshot_ms = snapshot_ms,
      .stability_ms = stability_ms,
      .rolling_route = rolling_route,
      .route_projection_valid = route_projection.valid,
      .temporary_frontier_is_terminal = temporary_frontier_is_terminal,
      .liveness_reseed_requested = liveness.reseed_requested,
      .pose_predicted = pose_predicted,
      .previous_control_source = previous_control_source,
      .rollout_budget = rollout_budget,
      .cooperative = cooperative,
      .noncooperative = noncooperative,
      .route_required_risk_tier = route_required_risk_tier,
  });
}

} // namespace drone_city_nav
