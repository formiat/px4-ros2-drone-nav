#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/execution_stop_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>

#include "production_mppi_node_execution_internal.hpp"
#include "production_mppi_node_planning_tick_rearm.hpp"
#include "raw_world_ingress_ros_3d.hpp"

namespace drone_city_nav {

ProductionMppiExecutionPublication
ProductionMppiNode::publishStopExecution(const ProductionMppiExecutionCycle& tick_cycle,
                                         const ProductionMppiExecutionReason reason,
                                         bool* const braking_path_blocked) {
  ProductionMppiExecutionPublication publication;
  // The stop derives its own world and validates against its own policy, so
  // it needs nothing of the cycle's route-bound validation world: a cycle
  // whose route has just been retired has none, and that is exactly the cycle
  // a moving vehicle needs the stop in. Guarded on that world, no stop was
  // attempted there, and the vehicle went to the offboard's blind brake: in
  // the urban flights r448 to r455 zero to two stops were published a flight
  // against three to eleven blind brakes above 2 m/s.
  if (tick_cycle.controller.result == nullptr ||
      tick_cycle.controller.resultRef().controls.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STOP_EXECUTION published=false status=controller_result_"
                         "unavailable");
    return publication;
  }
  const auto evidence_lock = evidence_boundary_.evidenceWithLatestSensor();
  // A stop certified against the world and refused by its commit only because
  // what the tick captured aged while it ran -- the input (a tick of the open
  // sky took 208 to 224 ms to reach its stop, r3846, against the input's
  // 200 ms), the control evidence it carries (the measured acceleration's
  // authority lost on the newest sample, r3845) or the objective (a return's
  // trail moves its objective every few metres) -- is prepared once more from
  // the newest input and objective, certified again and committed through the
  // same checks (roadmap item 43, specification K86). Refused, the moving
  // vehicle went to the offboard's local hold, which brakes with no world
  // behind it: 2673 times in the counted flights kept on 2026-11-17.
  std::optional<ProductionMppiExecutionCycle> refreshed;
  const auto refresh = [&]() -> bool {
    ProductionMppiNavigation navigation;
    std::shared_ptr<const CommittedExecutionAuthority3D> authority;
    {
      const auto input_lock = evidence_boundary_.input();
      navigation = navigation_;
      navigation.valid = navigation.valid && !navigation_revision_exhausted_ &&
                         !navigation_frame_reset_unresolved_;
      authority = execution_supervisor_.authority();
    }
    if (!navigation.valid || execution_input_capture_sequence_ ==
                                 std::numeric_limits<std::uint64_t>::max()) {
      return false;
    }
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    ProductionMppiExecutionInputPreparation input =
        prepareExecutionInputForPlanningTick(
            navigation, authority, ++execution_input_capture_sequence_, now_ns,
            config_.execution.maximum_control_feedback_age_ms,
            /*pose_predicted=*/false, /*stationary_capture_rearm=*/false);
    if (input.execution_input == nullptr &&
        navigation.linear_acceleration_authoritative &&
        execution_input_capture_sequence_ !=
            std::numeric_limits<std::uint64_t>::max()) {
      // The resident feedback makes no input when its stamps stand a clock
      // step apart: the offboard's clock a simulation step ahead of this
      // node's when the feedback arrived, its source stamp after its receipt
      // (r3901: 4 ms, a stop at 11.4 m/s left unprepared and the vehicle to
      // the local hold). The measured acceleration is the tick's other
      // evidence of the previous control, and a stop needs its linear part
      // only (K86): the feedback is set aside by an age of nought.
      navigation.measured_acceleration_valid = true;
      input = prepareExecutionInputForPlanningTick(
          navigation, authority, ++execution_input_capture_sequence_, now_ns,
          /*maximum_control_feedback_age_ms=*/0.0, /*pose_predicted=*/false,
          /*stationary_capture_rearm=*/false);
    }
    if (input.execution_input == nullptr) {
      return false;
    }
    refreshed = tick_cycle;
    refreshed->evidence.execution_input = input.execution_input;
    refreshed->evidence.exact_initial_state = input.execution_input->state();
    refreshed->evidence.exact_previous_control =
        input.execution_input->previousControl();
    refreshed->evidence.objective = navigationObjective();
    refreshed->controller.now_ns = now_ns;
    return true;
  };
  for (int attempt = 0;; ++attempt) {
    const ProductionMppiExecutionCycle& cycle =
        refreshed.has_value() ? *refreshed : tick_cycle;
    // A route-directed cycle derives no validation world of its own: the route
    // owns one, and a stop is exactly the answer to a route whose world can no
    // longer be trusted. Derive the newest observed evidence here, the way a
    // hold derives it.
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world =
        cycle.evidence.rearm_observed_world;
    std::shared_ptr<const VersionedStaticWorld3D> static_world =
        cycle.evidence.rearm_static_world;
    const WorldSnapshot3D& world = cycle.controller.worldRef();
    if (observed_raw_world == nullptr && static_world == nullptr) {
      if (config_.world.use_static_map && world.static_occupancy != nullptr) {
        static_world = VersionedStaticWorld3D::captureOwned(
            navigationWorldCertificate3D(world), world.static_occupancy);
      } else if (const RawWorldIngressSnapshot3D world_input =
                     raw_world_ingress_->snapshot();
                 world_input.latest_raw_world != nullptr) {
        const Point3 body_position{cycle.evidence.exact_initial_state.x,
                                   cycle.evidence.exact_initial_state.y,
                                   cycle.evidence.exact_initial_state.z};
        observed_raw_world = world_input.latest_raw_world->deriveRouteEvidence(
            proprioceptiveContactSeed3D(
                body_position, cycle.evidence.exact_previous_control,
                config_.world.physical_footprint,
                std::addressof(world_input.latest_raw_world->occupancy()))
                .value_or(proprioceptiveContactSeed3D(
                    body_position, cycle.evidence.exact_previous_control,
                    config_.world.physical_footprint, 0.0)),
            world.launch_support_contact);
      }
    }
    const ExecutionStopPreparation3D prepared =
        execution_supervisor_.prepareStop(ExecutionStopRequest3D{
            .cycle_source_plan = cycle.route.execution.source_snapshot,
            .execution_input = cycle.evidence.execution_input,
            .latest_sensor_evidence = cycle.evidence.latest_sensor_evidence,
            .observed_raw_world = observed_raw_world,
            .static_world = static_world,
            .validation_policy = cycle.evidence.selected_policy != nullptr
                                     ? cycle.evidence.selected_policy
                                     : config_.execution.validation_policy,
            .exact_initial_state = cycle.evidence.exact_initial_state,
            .exact_previous_control = cycle.evidence.exact_previous_control,
            .finite_horizon_config = config_.execution.finite_horizon,
            .minimum_control_count = cycle.controller.resultRef().controls.size(),
            .now_ns = cycle.controller.now_ns,
        });
    const StopExecution3D* const stop = prepared.stopExecution();
    if (braking_path_blocked != nullptr) {
      *braking_path_blocked = prepared.certification.status ==
                                  StopCertificationStatus3D::kPathValidationRejected &&
                              finiteExecutionPathOccupiedEvidenceVerdict3D(
                                  prepared.certification.path_validation_status);
    }
    if (!prepared.prepared() || stop == nullptr || stop->horizon == nullptr) {
      // A resident stop that still holds the vehicle is the answer, not a
      // failure: it is already the trajectory this call would have produced,
      // and it continues. Reported as nothing published, it fell through to a
      // revocation that a stop plan, unlike a resident route, lets through: the
      // braking vehicle was handed to the offboard's blind local hold a third
      // of a second into its stop, and one recorded flight met the wall the
      // stop had been braking for at the hold's own speed.
      if (prepared.status == ExecutionStopStatus3D::kResidentStopCurrent) {
        return residentOwnerContinuation(reason, cycle.controller.now_ns, {},
                                         /*retire_route_on_expiry=*/false);
      }
      if (prepared.status != ExecutionStopStatus3D::kAtRest) {
        // With the identities of everything the stop was prepared from, so
        // that a refusal can be built again in a test from the line alone
        // (naipi11's, their b8592414).
        const auto& input = cycle.evidence.execution_input;
        const auto& sensor = cycle.evidence.latest_sensor_evidence;
        const auto& policy = cycle.evidence.selected_policy != nullptr
                                 ? cycle.evidence.selected_policy
                                 : config_.execution.validation_policy;
        const auto& state = cycle.evidence.exact_initial_state;
        const auto& control = cycle.evidence.exact_previous_control;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "STOP_EXECUTION published=false status=%s transition=%.*s "
            "detail=%.*s certification=%.*s certification_dynamics=%s "
            "certification_path=%s clearance_reduction=%.2f failure_segment=%zu "
            "speed_mps=%.2f replacement_failure=%s "
            "cycle_ns=%" PRId64 " input_present=%d input_sequence=%" PRIu64
            " pose_revision=%" PRIu64 " pose_source_us=%" PRIu64
            " pose_receive_ns=%" PRId64 " control_source=%u control_producer=%" PRIu64
            " control_sequence=%" PRIu64 " control_source_ns=%" PRId64
            " control_receive_ns=%" PRId64
            " state=(%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g) "
            "previous_control=(%.9g,%.9g,%.9g,%.9g) minimum_controls=%zu "
            "raw_producer=%" PRIu64 " raw_base=%" PRIu64 " raw_revision=%" PRIu64
            " sensor_producer=%" PRIu64 " sensor_sequence=%" PRIu64
            " sensor_acquisition_ns=%" PRId64 " sensor_receive_ns=%" PRId64
            " policy_fingerprint=%" PRIu64,
            executionStopStatus3DName(prepared.status),
            static_cast<int>(
                executionRouteTransitionStatus3DName(prepared.transition_status)
                    .size()),
            executionRouteTransitionStatus3DName(prepared.transition_status).data(),
            static_cast<int>(
                executionRouteTransitionDetail3DName(prepared.transition_detail)
                    .size()),
            executionRouteTransitionDetail3DName(prepared.transition_detail).data(),
            static_cast<int>(
                stopCertificationStatus3DName(prepared.certification.status).size()),
            stopCertificationStatus3DName(prepared.certification.status).data(),
            motionDynamicsConsistency3DName(
                prepared.certification.dynamics_consistency),
            finiteExecutionPathStatus3DName(
                prepared.certification.path_validation_status),
            prepared.certification.clearance_reduction,
            prepared.certification.path_validation_failure_segment_index,
            prepared.initial_speed_mps, productionMppiExecutionReasonName(reason),
            cycle.controller.now_ns, input != nullptr ? 1 : 0,
            input != nullptr ? input->captureSequence() : std::uint64_t{0},
            input != nullptr ? input->poseRevision() : std::uint64_t{0},
            input != nullptr ? input->poseSourceTimestampUs() : std::uint64_t{0},
            input != nullptr ? input->poseReceiveStampNs() : std::int64_t{0},
            input != nullptr ? static_cast<unsigned>(input->previousControlSource())
                             : 0U,
            input != nullptr ? input->previousControlSourceProducerInstanceId()
                             : std::uint64_t{0},
            input != nullptr ? input->previousControlSourceSequence()
                             : std::uint64_t{0},
            input != nullptr ? input->previousControlSourceStampNs() : std::int64_t{0},
            input != nullptr ? input->previousControlReceiveStampNs() : std::int64_t{0},
            state.x, state.y, state.z, state.vx, state.vy, state.vz, state.yaw,
            state.yaw_rate, control.ax, control.ay, control.az, control.yaw_accel,
            cycle.controller.resultRef().controls.size(),
            observed_raw_world != nullptr
                ? observed_raw_world->version().producer_instance_id
                : std::uint64_t{0},
            observed_raw_world != nullptr
                ? observed_raw_world->version().base_snapshot_revision
                : std::uint64_t{0},
            observed_raw_world != nullptr ? observed_raw_world->version().revision
                                          : std::uint64_t{0},
            sensor != nullptr ? sensor->producerInstanceId() : std::uint64_t{0},
            sensor != nullptr ? sensor->sequence() : std::uint64_t{0},
            sensor != nullptr ? sensor->acquisitionStampNs() : std::int64_t{0},
            sensor != nullptr ? sensor->receiveStampNs() : std::int64_t{0},
            policy != nullptr ? policy->contentFingerprint() : std::uint64_t{0});
      }
      return publication;
    }

    msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
        cycle, stop->valid_until_ns, ProductionMppiExecutionMode::kPlanned,
        ProductionMppiExecutionReason::kNone);
    horizon.pose_revision = cycle.evidence.execution_input->poseRevision();
    // A stop follows no route, so it constrains none of the route metadata a
    // route-directed horizon carries.
    horizon.route_constrained = false;
    if (stop->observed_raw_world != nullptr) {
      horizon.obstacle_revision = stop->observed_raw_world->version().revision;
    }
    if (!production_mppi_execution_detail::appendFiniteExecutionPoints(
            horizon, stop->horizon->states, stop->horizon->controls,
            cycle.evidence.exact_previous_control, stop->control_interval_ns,
            config_.control.mppi.dynamics)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "STOP_EXECUTION published=false status=horizon_encoding_"
                           "rejected speed_mps=%.2f",
                           prepared.initial_speed_mps);
      return publication;
    }
    const std::shared_ptr<const ExecutionPlan3D> expected = prepared.expectedPlan();
    // The evidence boundary is already held for the whole preparation, so the
    // stop commits through the same boundary the way a hold does instead of
    // re-entering it through the snapshot commit.
    if (expected == nullptr ||
        commitAndPublishExecutionHorizon(
            cycle, horizon,
            ExecutionHorizonLeaseCandidate3D{
                .kind = ExecutionHorizonCommitKind3D::kTransition,
                .expected_authority = prepared.expected_authority,
                .expected_plan = expected,
                .certification_plan = expected,
                .progress_preparation = nullptr,
                .transition = prepared.transition,
                .expected_pending = nullptr,
                .owner = {},
                .expected_horizon_producer_instance_id = 0U,
                .execution_input = cycle.evidence.execution_input,
                .stationary_capture_rearm_intent = false,
            }) != ProductionMppiHorizonCommitStatus::kPublished) {
      const HorizonCommitRefusal& refusal = latest_commit_refusal_;
      const bool aged =
          refusal.status == ExecutionHorizonCommitStatus3D::kExecutionInputNotFresh ||
          refusal.status ==
              ExecutionHorizonCommitStatus3D::kControlEvidenceNotCurrent ||
          refusal.status == ExecutionHorizonCommitStatus3D::kObjectiveChanged;
      if (attempt == 0 && expected != nullptr && aged && refresh()) {
        continue;
      }
      // The stop was prepared and certified against the world: only the commit's
      // checks refused it, and what they measured is printed with it (roadmap
      // item 43), with the running count the throttle would otherwise hide.
      ++stop_commit_refusals_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STOP_EXECUTION published=false status=publication_commit_rejected "
          "speed_mps=%.2f stage=%.*s control_clause=%s revalidation_clause=%s "
          "refused_view=%s input_pose_age_ms=%.1f "
          "input_control_age_ms=%.1f maximum_pose_age_ms=%.1f "
          "maximum_control_age_ms=%.1f feedback_source_age_ms=%.1f "
          "feedback_receive_age_ms=%.1f maximum_feedback_age_ms=%.1f "
          "objective_current=%s navigation_valid=%s yaw_rate_authoritative=%s "
          "frame_reset_unresolved=%s refreshed=%s refused_total=%" PRIu64,
          prepared.initial_speed_mps, static_cast<int>(refusal.stage.size()),
          refusal.stage.data(),
          refusal.control_clause != nullptr ? refusal.control_clause : "none",
          refusal.revalidation_clause != nullptr ? refusal.revalidation_clause : "none",
          refusal.revalidation_view != nullptr ? refusal.revalidation_view : "none",
          refusal.input_pose_age_ms, refusal.input_control_age_ms,
          refusal.maximum_pose_age_ms, refusal.maximum_control_age_ms,
          refusal.feedback_source_age_ms, refusal.feedback_receive_age_ms,
          config_.execution.maximum_control_feedback_age_ms,
          refusal.objective_current ? "true" : "false",
          refusal.navigation_valid ? "true" : "false",
          refusal.navigation_yaw_rate_authoritative ? "true" : "false",
          refusal.navigation_frame_reset_unresolved ? "true" : "false",
          refreshed.has_value() ? "true" : "false", stop_commit_refusals_);
      return publication;
    }

    publication.horizon = stop->horizon->states;
    publication.mode = ProductionMppiExecutionMode::kPlanned;
    publication.reason = reason;
    publication.planned_control_count = stop->horizon->controls.size();
    publication.nominal_prefix_control_count =
        stop->horizon->nominal_prefix_control_count;
    publication.arrival_control_count = stop->horizon->arrival_control_count;
    publication.first_control = stop->horizon->controls.front();
    publication.first_control_available = true;
    publication.latest_sensor_obstacle_sequence =
        cycle.evidence.latest_sensor_obstacle_sequence;
    publication.latest_sensor_obstacle_hit_count =
        cycle.evidence.latest_sensor_obstacle_points.size();
    publication.latest_sensor_obstacle_age_ms =
        cycle.evidence.latest_sensor_obstacle_age_ms;
    publication.latest_sensor_obstacle_fresh =
        cycle.evidence.latest_sensor_obstacle_fresh;
    publication.latest_sensor_obstacle_receive_time_fallback =
        cycle.evidence.latest_sensor_obstacle_receive_time_fallback;
    publication.terminal_rest_state = true;
    publication.published = true;
    RCLCPP_WARN(get_logger(),
                "STOP_EXECUTION published=true trajectory_revision=%" PRIu64
                " snapshot_version=%" PRIu64 " speed_mps=%.2f stop_distance_m=%.2f "
                "controls=%zu rest=(%.2f,%.2f,%.2f) clearance_reduction=%.2f "
                "replacement_failure=%s refreshed=%s",
                stop->trajectory_revision, prepared.preparedPlan()->version,
                prepared.initial_speed_mps, prepared.stop_distance_m,
                stop->horizon->controls.size(), stop->rest_position.x,
                stop->rest_position.y, stop->rest_position.z, stop->clearance_reduction,
                productionMppiExecutionReasonName(reason),
                refreshed.has_value() ? "true" : "false");
    return publication;
  }
}

} // namespace drone_city_nav
