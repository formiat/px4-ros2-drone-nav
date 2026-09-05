#include "drone_city_nav/execution_horizon_contract_ros.hpp"

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>

#include "production_mppi_node_execution_internal.hpp"
#include "raw_world_ingress_ros_3d.hpp"

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kExecutionRevocationReasonMask{0xffU};
constexpr std::uint64_t kExecutionRevocationRequestStep{kExecutionRevocationReasonMask +
                                                        1U};

[[nodiscard]] bool
failClosedExecutionReason(const ProductionMppiExecutionReason reason) noexcept {
  return reason == ProductionMppiExecutionReason::kNoExecutableHorizon ||
         reason == ProductionMppiExecutionReason::kNoExecutableRoute ||
         reason == ProductionMppiExecutionReason::kUnavailableWorld;
}

} // namespace

ProductionMppiExecutionPublication ProductionMppiNode::publishPositionHold(
    const ProductionMppiExecutionCycle& cycle, const Point3& hold_position,
    const ProductionMppiExecutionReason reason, const ExecutionHoldIntent3D intent) {
  ProductionMppiExecutionPublication& publication = cycle.publicationRef();
  const auto evidence_lock = evidence_boundary_.evidenceWithLatestLidar();
  const RawWorldIngressSnapshot3D world_input = raw_world_ingress_->snapshot();
  const std::shared_ptr<const VersionedObservedRawWorld3D> current_observed_raw_world =
      world_input.latest_raw_world != nullptr
          ? world_input.latest_raw_world->authoritativeOwner()
          : nullptr;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      latest_lidar_evidence_.load(std::memory_order_acquire);
  const ExecutionHoldPreparation3D prepared =
      execution_supervisor_.prepareHold(ExecutionHoldRequest3D{
          .intent = intent,
          .requested_position = hold_position,
          .cycle_source_plan = cycle.route.execution.source_snapshot,
          .execution_input = cycle.evidence.execution_input,
          .latest_lidar_evidence = cycle.evidence.latest_lidar_evidence,
          .current_lidar_evidence = current_lidar,
          .current_observed_raw_world = current_observed_raw_world,
          .stationary_capture_observed_raw_world = cycle.evidence.direct_observed_world,
          .stationary_capture_static_world = cycle.evidence.direct_static_world,
          .selected_validation_policy = cycle.evidence.selected_policy,
          .stationary_capture_validation_policy = config_.execution.validation_policy,
          .validation_now_ns = cycle.evidence.lidar_validation_now_ns,
          .raw_world_identity_conflicted = world_input.raw_world_identity_conflicted,
          .latest_lidar_identity_conflicted =
              latest_lidar_evidence_identity_conflicted_.load(
                  std::memory_order_acquire),
      });
  if (!prepared.prepared() ||
      prepared.executionInput() != cycle.evidence.execution_input) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HOLD prepared=false stage=%s kind=%s transition=%.*s detail=%.*s",
        executionHoldPreparationStatus3DName(prepared.status),
        executionHoldPreparationKind3DName(prepared.kind),
        static_cast<int>(
            executionRouteTransitionStatus3DName(prepared.transition_status).size()),
        executionRouteTransitionStatus3DName(prepared.transition_status).data(),
        static_cast<int>(
            executionRouteTransitionDetail3DName(prepared.transition_detail).size()),
        executionRouteTransitionDetail3DName(prepared.transition_detail).data());
    return publication;
  }
  const Point3 owned_hold_position = prepared.position;
  if (!insideFlightEnvelope(owned_hold_position, config_.world.flight_envelope)) {
    RCLCPP_ERROR(get_logger(),
                 "EXECUTION_HORIZON rejected reason=hold_outside_flight_envelope "
                 "target_z=%.3f",
                 owned_hold_position.z);
    return publication;
  }
  if (cycle.controller.finite_path_control_interval_ns <= 0 ||
      cycle.controller.finite_path_control_interval_ns >
          std::numeric_limits<std::int64_t>::max() / 2) {
    return publication;
  }
  const std::int64_t requested_hold_duration_ns =
      reason == ProductionMppiExecutionReason::kGoalCapture
          ? config_.execution.mission_goal_capture_hold_validity_ns
          : config_.execution.stationary_hold_validity_ns;
  const std::int64_t hold_duration_ns = std::max(
      requested_hold_duration_ns, 2 * cycle.controller.finite_path_control_interval_ns);
  const std::optional<std::int64_t> hold_valid_until_ns =
      production_mppi_execution_detail::canonicalHorizonEndTime(cycle.controller.now_ns,
                                                                hold_duration_ns);
  if (!hold_valid_until_ns.has_value()) {
    return publication;
  }
  msg::MppiTrajectoryHorizon horizon = makeExecutionHorizon(
      cycle, *hold_valid_until_ns, ProductionMppiExecutionMode::kPositionHold, reason);
  horizon.stationary_position_hold = true;
  horizon.stationary_hold_position.x = owned_hold_position.x;
  horizon.stationary_hold_position.y = owned_hold_position.y;
  horizon.stationary_hold_position.z = owned_hold_position.z;
  horizon.points.reserve(2U);
  production_mppi_execution_detail::appendStationaryHoldPoint(
      horizon, owned_hold_position, 0, cycle.evidence.exact_initial_state.yaw);
  production_mppi_execution_detail::appendStationaryHoldPoint(
      horizon, owned_hold_position, cycle.controller.finite_path_control_interval_ns,
      cycle.evidence.exact_initial_state.yaw);

  ExecutionHorizonLeaseCandidate3D candidate;
  const std::shared_ptr<const ExecutionPlan3D> hold_expected = prepared.expectedPlan();
  if (hold_expected == nullptr) {
    return publication;
  }
  if (prepared.kind == ExecutionHoldPreparationKind3D::kTransition &&
      prepared.transition != nullptr) {
    candidate.kind = ExecutionHorizonCommitKind3D::kTransition;
    candidate.expected_plan = hold_expected;
    candidate.transition = prepared.transition;
  } else if (prepared.kind == ExecutionHoldPreparationKind3D::kUnchangedPlan) {
    candidate.kind = ExecutionHorizonCommitKind3D::kUnchangedPlan;
    candidate.expected_plan = hold_expected;
  } else {
    return publication;
  }
  candidate.expected_authority = prepared.expected_authority;
  candidate.certification_plan = hold_expected;
  candidate.stationary_capture_rearm_intent = prepared.stationary_capture_rearm;
  if (commitAndPublishExecutionHorizon(cycle, horizon, std::move(candidate)) !=
      ProductionMppiHorizonCommitStatus::kPublished) {
    return publication;
  }
  publication.horizon = {
      mppi::State{.x = static_cast<float>(owned_hold_position.x),
                  .y = static_cast<float>(owned_hold_position.y),
                  .z = static_cast<float>(owned_hold_position.z),
                  .yaw = cycle.evidence.exact_initial_state.yaw},
      mppi::State{.x = static_cast<float>(owned_hold_position.x),
                  .y = static_cast<float>(owned_hold_position.y),
                  .z = static_cast<float>(owned_hold_position.z),
                  .yaw = cycle.evidence.exact_initial_state.yaw},
  };
  publication.mode = ProductionMppiExecutionMode::kPositionHold;
  publication.reason = reason;
  publication.latest_lidar_obstacle_sequence =
      cycle.evidence.latest_lidar_obstacle_sequence;
  publication.latest_lidar_obstacle_hit_count =
      cycle.evidence.latest_lidar_obstacle_points.size();
  publication.latest_lidar_obstacle_age_ms =
      cycle.evidence.latest_lidar_obstacle_age_ms;
  publication.latest_lidar_obstacle_fresh = cycle.evidence.latest_lidar_obstacle_fresh;
  publication.latest_lidar_obstacle_receive_time_fallback =
      cycle.evidence.latest_lidar_obstacle_receive_time_fallback;
  publication.published = true;
  return publication;
}

ProductionMppiExecutionPublication ProductionMppiNode::publishNoExecutablePathHold(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason reason,
    const bool physical_candidate_rejection) {
  const bool mission_goal_hold = cycle.route.planning_state ==
                                 ProductionMppiPlanningState::kMissionGoalPositionHold;
  if (cycle.route.execution.source_snapshot != nullptr &&
      cycle.route.execution.source_snapshot->stationaryHold() != nullptr) {
    // A resident hold is re-leased only when its lease is about to run out.
    // Every re-lease is a new horizon on the wire: it restarts the waypoint
    // capture continuity, and it moves the execution base a pending certified
    // route has to hand off from, so a per-tick refresh keeps a new route from
    // ever activating out of the hold.
    if (!residentHoldLeaseNearExpiry(cycle.controller.now_ns)) {
      return residentOwnerContinuation(reason, cycle.controller.now_ns, {},
                                       mission_goal_hold);
    }
    ProductionMppiExecutionPublication hold = publishPositionHold(
        cycle, cycle.route.execution.source_snapshot->stationaryHold()->position,
        reason, ExecutionHoldIntent3D::kRefreshResident);
    if (hold.published) {
      return hold;
    }
  }
  bool retention_physically_rejected{false};
  // At the mission goal the route is finished: re-leasing its finite path
  // would keep extending the lease the goal hold has to outlive.
  if (!mission_goal_hold) {
    if (std::optional<ProductionMppiExecutionPublication> retained =
            retainActiveFinitePath(cycle, reason, &retention_physically_rejected);
        retained.has_value()) {
      return *retained;
    }
  }
  const bool physical_route_invalidation =
      physical_candidate_rejection || retention_physically_rejected ||
      cycle.route.execution.physical_trajectory_invalidated ||
      (cycle.route.execution.lifecycle_event.has_value() &&
       (cycle.route.execution.lifecycle_event->kind ==
            RouteLifecycleEventKind3D::kRawInvalidated ||
        cycle.route.execution.lifecycle_event->kind ==
            RouteLifecycleEventKind3D::kLatestLidarInvalidated));
  // Any lifecycle event that ends the resident path's claim on the vehicle
  // leaves the vehicle without a plan to execute, physical or not.
  const bool path_claim_ended =
      cycle.route.execution.lifecycle_event.has_value() &&
      routeLifecycleEventEndsPathClaim3D(cycle.route.execution.lifecycle_event->kind);
  // Physical evidence against the resident path ends that path's claim on the
  // vehicle. Revoking it would hand the moving vehicle to the offboard's local
  // hold, which knows nothing about obstacles; a certified stop is the same
  // decision carried out along a trajectory the world was actually checked
  // against.
  if (physical_route_invalidation || path_claim_ended) {
    ProductionMppiExecutionPublication stop = publishStopExecution(cycle, reason);
    if (stop.published) {
      return stop;
    }
  }
  ProductionMppiExecutionPublication revocation = publishExecutionRevocation(
      reason, cycle.controller.now_ns, physical_route_invalidation);
  if (revocation.published ||
      executionRevocationAllowed(physical_route_invalidation,
                                 config_.planning.optional_constraints
                                     .nonphysical_execution_revocation_enabled)) {
    return revocation;
  }
  ProductionMppiExecutionPublication continuation = residentOwnerContinuation(
      reason, cycle.controller.now_ns, revocation,
      cycle.route.planning_state ==
          ProductionMppiPlanningState::kMissionGoalPositionHold);
  if (continuation.resident_owner_continues) {
    return continuation;
  }
  // Nothing owns the vehicle any more: no replacement, no retained path and no
  // lease left to continue. Whatever the vehicle is still carrying, it is
  // carrying it without a plan, so it is stopped along a validated trajectory
  // instead of being left to coast into the offboard's blind hold.
  ProductionMppiExecutionPublication stop = publishStopExecution(cycle, reason);
  return stop.published ? stop : continuation;
}

bool ProductionMppiNode::residentHoldLeaseNearExpiry(const std::int64_t now_ns) const {
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      execution_supervisor_.authority();
  if (authority == nullptr || !authority->valid() || !authority->owner().valid) {
    return true;
  }
  // Two nominal ticks, or two measured ones when the planning tick runs
  // slower than its nominal rate, so the renewal lands before the lease ends.
  const std::int64_t nominal_margin_ns = static_cast<std::int64_t>(
      std::llround(2.0e9 / std::max(config_.planning.tick_rate_hz, 1.0)));
  const std::int64_t renewal_margin_ns =
      std::max(nominal_margin_ns, 2 * last_planning_tick_period_ns_);
  return authority->owner().valid_until_ns - now_ns <= renewal_margin_ns;
}

ProductionMppiExecutionPublication ProductionMppiNode::residentOwnerContinuation(
    const ProductionMppiExecutionReason replacement_failure_reason,
    const std::int64_t now_ns,
    const ProductionMppiExecutionPublication& unpublished_revocation,
    const bool retire_route_on_expiry) {
  // A non-physical replacement failure leaves the last committed lease in
  // force by policy. Report that state honestly: the vehicle keeps executing
  // the resident planned horizon, nothing was revoked on the wire.
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      execution_supervisor_.authority();
  if (authority == nullptr || !authority->valid()) {
    return unpublished_revocation;
  }
  const ExecutionOwnerIdentity3D& owner = authority->owner();
  const std::shared_ptr<const ExecutionPlan3D>& plan = authority->plan();
  if (owner.valid && now_ns >= owner.valid_until_ns) {
    expireResidentOwner(now_ns, retire_route_on_expiry);
    return unpublished_revocation;
  }
  if (!owner.valid || owner.execution_mode != ExecutionAuthorityMode3D::kPlanned ||
      now_ns < owner.valid_from_ns || plan == nullptr) {
    return unpublished_revocation;
  }
  const mppi::FiniteHorizon* resident_horizon{nullptr};
  if (const FiniteExecutionState3D* const finite = plan->finiteExecution()) {
    resident_horizon = finite->horizon.get();
  } else if (const DirectTrackingFiniteExecution3D* const direct =
                 plan->directTrackingExecution()) {
    resident_horizon = direct->horizon.get();
  }
  if (resident_horizon == nullptr || resident_horizon->controls.empty()) {
    return unpublished_revocation;
  }
  resident_owner_continuation_ticks_.fetch_add(1U, std::memory_order_relaxed);
  ProductionMppiExecutionPublication continuation;
  continuation.horizon = resident_horizon->states;
  continuation.mode = ProductionMppiExecutionMode::kPlanned;
  continuation.reason = replacement_failure_reason;
  continuation.planned_control_count = resident_horizon->controls.size();
  continuation.nominal_prefix_control_count =
      resident_horizon->nominal_prefix_control_count;
  continuation.arrival_control_count = resident_horizon->arrival_control_count;
  continuation.first_control = resident_horizon->controls.front();
  continuation.first_control_available = true;
  continuation.resident_owner_continues = true;
  continuation.terminal_rest_state = true;
  continuation.published = false;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "EXECUTION_HORIZON resident_owner_continues=true sequence=%" PRIu64
      " replacement_failure=%s remaining_lease_ms=%.1f",
      owner.sequence, productionMppiExecutionReasonName(replacement_failure_reason),
      static_cast<double>(owner.valid_until_ns - now_ns) * 1.0e-6);
  return continuation;
}

ProductionMppiExecutionPublication ProductionMppiNode::publishExecutionRevocation(
    const ProductionMppiExecutionReason reason, const std::int64_t now_ns,
    const bool physical_route_invalidation) {
  ProductionMppiExecutionPublication publication;
  publication.mode = ProductionMppiExecutionMode::kRevoked;
  publication.reason = reason;
  if (!executionRevocationAllowed(physical_route_invalidation,
                                  config_.planning.optional_constraints
                                      .nonphysical_execution_revocation_enabled)) {
    return publication;
  }
  if (!failClosedExecutionReason(reason) || execution_horizon_pub_ == nullptr ||
      now_ns <= 0 ||
      execution_horizon_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return publication;
  }

  const auto lock = evidence_boundary_.evidenceWithInput();
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority =
      execution_supervisor_.authority();
  if (expected_authority == nullptr || !expected_authority->valid()) {
    return publication;
  }
  const std::shared_ptr<const ExecutionPlan3D> expected = expected_authority->plan();
  if (expected == nullptr) {
    return publication;
  }
  const ExecutionRouteTransitionResult3D transition = [&] {
    if (expected->route() != nullptr) {
      ExecutionRouteTransitionResult3D suspension =
          suspendFiniteExecution3D(*expected, expected->version);
      if (suspension.applied() ||
          suspension.status == ExecutionRouteTransitionStatus3D::kNoChange) {
        return suspension;
      }
      return ExecutionRouteTransitionResult3D{};
    }
    return revokeExecution3D(*expected, expected->version);
  }();
  const bool certified_route_preserved =
      expected->route() != nullptr &&
      ((transition.applied() && transition.next != nullptr &&
        transition.next->route() != nullptr) ||
       transition.status == ExecutionRouteTransitionStatus3D::kNoChange);
  const bool transition_required = transition.applied();
  if (!transition_required &&
      transition.status != ExecutionRouteTransitionStatus3D::kNoChange) {
    return publication;
  }
  const std::int64_t publication_now_ns = get_clock()->now().nanoseconds();
  // A revoked snapshot with no live horizon owner already represents the
  // requested tombstone. Do not emit a fresh transport sequence for every
  // duplicate callback; publish only while there is an owner to revoke.
  if (!transition_required && !expected_authority->owner().valid) {
    return publication;
  }
  const bool current_session =
      offboard_session_admission_.valid() &&
      offboard_session_admission_.latest_source_stamp_ns > 0 &&
      offboard_session_receive_stamp_ns_ > 0 &&
      publication_now_ns >= offboard_session_admission_.latest_source_stamp_ns &&
      publication_now_ns >= offboard_session_receive_stamp_ns_ &&
      static_cast<double>(publication_now_ns -
                          offboard_session_admission_.latest_source_stamp_ns) *
              1.0e-6 <=
          config_.execution.maximum_control_feedback_age_ms &&
      static_cast<double>(publication_now_ns - offboard_session_receive_stamp_ns_) *
              1.0e-6 <=
          config_.execution.maximum_control_feedback_age_ms;
  if (!current_session) {
    return publication;
  }
  const std::uint64_t target_offboard_instance_id =
      offboard_session_admission_.current_producer_instance_id;
  if (target_offboard_instance_id == 0U) {
    return publication;
  }

  msg::MppiTrajectoryHorizon revocation;
  revocation.header.stamp = now();
  revocation.header.frame_id = config_.world.frame_id;
  revocation.producer_instance_id = execution_horizon_producer_instance_id_;
  revocation.target_offboard_instance_id = target_offboard_instance_id;
  revocation.sequence = execution_horizon_sequence_ + 1U;
  revocation.valid_from = production_mppi_execution_detail::timeFromNanoseconds(now_ns);
  revocation.valid_until = revocation.valid_from;
  revocation.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
  revocation.execution_reason = static_cast<std::uint8_t>(reason);
  if (assessExecutionHorizonPayload(revocation,
                                    ExecutionHorizonPayloadValidationConfig{
                                        .expected_frame_id = config_.world.frame_id}) !=
      ExecutionHorizonPayloadStatus::kValid) {
    return publication;
  }

  const bool authority_cleared =
      transition_required ? execution_supervisor_.commitDetachedTransition(
                                expected_authority, transition) ==
                                ExecutionRoutePublicationStatus3D::kPublished
                          : execution_supervisor_.clearLeaseIfSame(expected_authority);
  if (!authority_cleared) {
    return publication;
  }
  execution_horizon_sequence_ = revocation.sequence;
  if (expected_authority->control().valid) {
    recordAppliedControlDiscontinuityLocked();
  }
  execution_horizon_pub_->publish(revocation);
  publication.published = true;
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "EXECUTION_HORIZON revoked=true snapshot_version=%" PRIu64 " owner_epoch=%" PRIu64
      " sequence=%" PRIu64 " reason=%s certified_route_preserved=%s",
      transition_required ? transition.next->version : expected->version,
      transition_required ? transition.next->execution_owner_epoch
                          : expected->execution_owner_epoch,
      revocation.sequence, productionMppiExecutionReasonName(reason),
      certified_route_preserved ? "true" : "false");
  return publication;
}

void ProductionMppiNode::expireResidentOwner(const std::int64_t now_ns,
                                             const bool retire_route) {
  const auto lock = evidence_boundary_.evidenceWithInput();
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority =
      execution_supervisor_.authority();
  if (expected_authority == nullptr || !expected_authority->valid()) {
    return;
  }
  const ExecutionOwnerIdentity3D& owner = expected_authority->owner();
  const std::shared_ptr<const ExecutionPlan3D> expected = expected_authority->plan();
  if (!owner.valid || now_ns < owner.valid_until_ns || expected == nullptr) {
    return;
  }
  // A suspended route plan is not publishable, so a detached suspension can
  // never be committed. Mid-route the plan therefore keeps its sticky route
  // and stale finite execution exactly as before the lease expired: the next
  // executable horizon replaces them, and a certified replacement can be
  // handed off against them. Only a finished route, whose mission goal capture
  // is latched, is revoked outright so the stationary capture rearm can take
  // the revoked plan over.
  if (!retire_route) {
    return;
  }
  const ExecutionRouteTransitionResult3D transition =
      revokeExecution3D(*expected, expected->version);
  const bool transition_required = transition.applied();
  if (!transition_required &&
      transition.status != ExecutionRouteTransitionStatus3D::kNoChange) {
    const std::string_view status_name =
        executionRouteTransitionStatus3DName(transition.status);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "EXECUTION_HORIZON owner_expired=true sequence=%" PRIu64
                         " plan_transition=rejected status=%.*s retire_route=%s",
                         owner.sequence, static_cast<int>(status_name.size()),
                         status_name.data(), retire_route ? "true" : "false");
    return;
  }
  ExecutionRoutePublicationStatus3D publication_status{
      ExecutionRoutePublicationStatus3D::kPublished};
  bool authority_cleared{false};
  if (transition_required) {
    publication_status =
        execution_supervisor_.commitDetachedTransition(expected_authority, transition);
    authority_cleared =
        publication_status == ExecutionRoutePublicationStatus3D::kPublished;
  } else {
    authority_cleared = execution_supervisor_.clearLeaseIfSame(expected_authority);
  }
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "EXECUTION_HORIZON owner_expired=true sequence=%" PRIu64
      " lease_age_ms=%.1f plan_transition=%s publication=%s "
      "authority_cleared=%s retire_route=%s",
      owner.sequence, static_cast<double>(now_ns - owner.valid_until_ns) * 1.0e-6,
      transition_required ? "applied" : "unchanged",
      executionRoutePublicationStatus3DName(publication_status),
      authority_cleared ? "true" : "false", retire_route ? "true" : "false");
}

void ProductionMppiNode::retireGoalHoldForSuccessorLeg() {
  // The completed leg's goal hold has served its purpose once the waypoint is
  // acknowledged. Left resident, it would be re-leased indefinitely and every
  // successor route would have to hand off from it under the hold-evidence
  // ordering; revoked here, the successor activates from an empty base.
  const auto lock = evidence_boundary_.evidenceWithInput();
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority =
      execution_supervisor_.authority();
  if (expected_authority == nullptr || !expected_authority->valid()) {
    return;
  }
  const std::shared_ptr<const ExecutionPlan3D> expected = expected_authority->plan();
  if (expected == nullptr || expected->stationaryHold() == nullptr ||
      expected->finiteExecution() != nullptr ||
      expected->directTrackingExecution() != nullptr) {
    return;
  }
  const ExecutionRouteTransitionResult3D transition =
      revokeExecution3D(*expected, expected->version);
  if (!transition.applied()) {
    const std::string_view status_name =
        executionRouteTransitionStatus3DName(transition.status);
    RCLCPP_WARN(get_logger(), "MISSION_GOAL_HOLD_RETIRED=false status=%.*s",
                static_cast<int>(status_name.size()), status_name.data());
    return;
  }
  const ExecutionRoutePublicationStatus3D publication_status =
      execution_supervisor_.commitDetachedTransition(expected_authority, transition);
  RCLCPP_INFO(get_logger(), "MISSION_GOAL_HOLD_RETIRED=true publication=%s",
              executionRoutePublicationStatus3DName(publication_status));
}

bool ProductionMppiNode::handleRequestedExecutionRevocation(const std::int64_t now_ns) {
  const std::uint64_t requested_revocation =
      requested_execution_revocation_.load(std::memory_order_acquire);
  if (!config_.planning.optional_constraints.nonphysical_execution_revocation_enabled) {
    // Callback requests describe evidence/freshness discontinuities, not a
    // certified raw-occupancy intersection. Drain them without turning a
    // transient publication race into a permanent planning barrier.
    handled_execution_revocation_request_ = requested_revocation;
    return false;
  }
  if (requested_revocation == handled_execution_revocation_request_) {
    return false;
  }

  const auto requested_reason = static_cast<ProductionMppiExecutionReason>(
      requested_revocation & kExecutionRevocationReasonMask);
  const ProductionMppiExecutionPublication revocation =
      publishExecutionRevocation(requested_reason, now_ns);
  if (revocation.published) {
    handled_execution_revocation_request_ = requested_revocation;
    return true;
  }

  bool revocation_already_satisfied{false};
  {
    const auto lock = evidence_boundary_.evidenceWithInput();
    const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
        execution_supervisor_.authority();
    const std::shared_ptr<const ExecutionPlan3D> snapshot =
        authority != nullptr ? authority->plan() : nullptr;
    const bool snapshot_has_executable_authority =
        snapshot != nullptr && (snapshot->finiteExecution() != nullptr ||
                                snapshot->directTrackingExecution() != nullptr ||
                                snapshot->stationaryHold() != nullptr);
    revocation_already_satisfied = !snapshot_has_executable_authority &&
                                   authority != nullptr && !authority->owner().valid;
  }
  if (revocation_already_satisfied) {
    handled_execution_revocation_request_ = requested_revocation;
  }
  return true;
}

void ProductionMppiNode::publishFailClosedExecutionRevocation(
    const ProductionMppiExecutionReason reason, const std::int64_t now_ns) {
  if (!config_.planning.optional_constraints.nonphysical_execution_revocation_enabled) {
    return;
  }
  const std::shared_ptr<const ExecutionPlan3D> snapshot = execution_supervisor_.plan();
  const bool authority_present =
      snapshot != nullptr && (snapshot->phase() == ExecutionRoutePhase3D::kRevoked ||
                              snapshot->finiteExecution() != nullptr ||
                              snapshot->directTrackingExecution() != nullptr ||
                              snapshot->stationaryHold() != nullptr);
  if (authority_present) {
    static_cast<void>(publishExecutionRevocation(reason, now_ns));
  }
}

void ProductionMppiNode::requestExecutionRevocation(
    const ProductionMppiExecutionReason reason) noexcept {
  if (!config_.planning.optional_constraints.nonphysical_execution_revocation_enabled ||
      !failClosedExecutionReason(reason)) {
    return;
  }
  const std::uint64_t encoded_reason = static_cast<std::uint8_t>(reason);
  std::uint64_t current =
      requested_execution_revocation_.load(std::memory_order_relaxed);
  while (current <=
         std::numeric_limits<std::uint64_t>::max() - kExecutionRevocationRequestStep) {
    const std::uint64_t desired = ((current & ~kExecutionRevocationReasonMask) +
                                   kExecutionRevocationRequestStep) |
                                  encoded_reason;
    if (requested_execution_revocation_.compare_exchange_weak(
            current, desired, std::memory_order_release, std::memory_order_relaxed)) {
      return;
    }
  }
}

ProductionMppiExecutionPublication
ProductionMppiNode::publishExplicitHold(const ProductionMppiExecutionCycle& cycle,
                                        const Point3& hold_position,
                                        const ProductionMppiExecutionReason reason) {
  const ExecutionHoldIntent3D intent =
      reason == ProductionMppiExecutionReason::kGoalCapture &&
              cycle.route.planning_state ==
                  ProductionMppiPlanningState::kMissionGoalPositionHold
          ? ExecutionHoldIntent3D::kExplicitTransferWithStationaryCaptureRearm
          : ExecutionHoldIntent3D::kExplicitTransfer;
  return publishPositionHold(cycle, hold_position, reason, intent);
}

} // namespace drone_city_nav
