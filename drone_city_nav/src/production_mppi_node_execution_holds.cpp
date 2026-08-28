#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/execution_publication_currentness_3d.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include "execution_publication_navigation_rebase_3d.hpp"
#include "production_mppi_node_execution_internal.hpp"
#include "production_mppi_node_planning_tick_rearm.hpp"

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
    const ProductionMppiExecutionReason reason,
    const ProductionMppiHoldOwnershipTransition3D ownership_transition) {
  ProductionMppiExecutionPublication& publication = cycle.publication;
  if (!insideFlightEnvelope(hold_position, flight_envelope_config_)) {
    RCLCPP_ERROR(get_logger(),
                 "EXECUTION_HORIZON rejected reason=hold_outside_flight_envelope "
                 "target_z=%.3f",
                 hold_position.z);
    return publication;
  }
  Point3 owned_hold_position = hold_position;
  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_,
                                       latest_lidar_evidence_commit_mutex_};
  std::shared_ptr<const ExecutionRouteSnapshot3D> hold_expected;
  std::optional<ExecutionRouteTransitionResult3D> hold_transition;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
      latest_lidar_evidence_.load(std::memory_order_acquire);
  if (!cycle.latest_lidar_obstacle_fresh || cycle.latest_lidar_evidence == nullptr ||
      current_lidar == nullptr ||
      current_lidar->evidenceId() != cycle.latest_lidar_evidence->evidenceId() ||
      current_lidar->contentFingerprint() !=
          cycle.latest_lidar_evidence->contentFingerprint()) {
    return publication;
  }
  hold_expected = execution_route_store_.snapshot();
  if (hold_expected == nullptr ||
      hold_expected != cycle.route_execution.source_snapshot) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON published=false reason=hold_source_not_current "
        "snapshot_present=%s",
        hold_expected != nullptr ? "true" : "false");
    return publication;
  }
  const bool stationary_capture_rearm =
      reason == ProductionMppiExecutionReason::kGoalCapture &&
      ownership_transition ==
          ProductionMppiHoldOwnershipTransition3D::kExplicitTransfer &&
      hold_expected->phase == ExecutionRoutePhase3D::kRevoked &&
      !hold_expected->route.has_value() &&
      !hold_expected->finite_execution.has_value() &&
      !hold_expected->direct_tracking_execution.has_value() &&
      !hold_expected->stationary_hold.has_value() &&
      cycle.planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold &&
      cycle.execution_input != nullptr &&
      cycle.execution_input->stationaryCaptureStateAuthoritative() &&
      cycle.selected_policy != nullptr && execution_validation_policy_ != nullptr &&
      cycle.selected_policy->policyId() == execution_validation_policy_->policyId() &&
      (cycle.direct_observed_world == nullptr) !=
          (cycle.direct_static_world == nullptr);
  if (cycle.execution_input != nullptr &&
      cycle.execution_input->stationaryCaptureStateAuthoritative() &&
      !stationary_capture_rearm) {
    return publication;
  }
  if (stationary_capture_rearm && cycle.direct_observed_world != nullptr &&
      latest_raw_world_3d_.load(std::memory_order_acquire) !=
          cycle.latest_raw_world_3d) {
    return publication;
  }
  if (hold_expected->stationary_hold.has_value() &&
      ownership_transition ==
          ProductionMppiHoldOwnershipTransition3D::kEnterEmptyOwner) {
    owned_hold_position = hold_expected->stationary_hold->position;
  }
  const StationaryExecutionHold3D* const resident_hold =
      hold_expected->stationary_hold.has_value()
          ? std::addressof(*hold_expected->stationary_hold)
          : nullptr;
  const FiniteExecutionState3D* const route_execution =
      hold_expected->finite_execution.has_value()
          ? std::addressof(*hold_expected->finite_execution)
          : nullptr;
  const DirectTrackingFiniteExecution3D* const direct_execution =
      hold_expected->direct_tracking_execution.has_value()
          ? std::addressof(*hold_expected->direct_tracking_execution)
          : nullptr;
  const std::shared_ptr<const VersionedObservedRawWorld3D> source_observed =
      stationary_capture_rearm      ? cycle.direct_observed_world
      : resident_hold != nullptr    ? resident_hold->observed_raw_world
      : route_execution != nullptr  ? route_execution->observed_raw_world
      : direct_execution != nullptr ? direct_execution->observed_raw_world
                                    : nullptr;
  std::shared_ptr<const VersionedObservedRawWorld3D> current_observed;
  if (stationary_capture_rearm) {
    current_observed = source_observed;
  } else if (source_observed != nullptr) {
    const std::shared_ptr<const ProductionMppiRawWorld3D> current_raw =
        latest_raw_world_3d_.load(std::memory_order_acquire);
    if (current_raw == nullptr || current_raw->execution_owner == nullptr ||
        current_raw->execution_owner->version().producer_instance_id !=
            source_observed->version().producer_instance_id) {
      return publication;
    }
    current_observed = current_raw->execution_owner;
  }
  const std::shared_ptr<const VersionedStaticWorld3D> current_static =
      stationary_capture_rearm      ? cycle.direct_static_world
      : resident_hold != nullptr    ? resident_hold->static_world
      : route_execution != nullptr  ? route_execution->static_world
      : direct_execution != nullptr ? direct_execution->static_world
                                    : nullptr;
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D> policy =
      stationary_capture_rearm      ? execution_validation_policy_
      : resident_hold != nullptr    ? resident_hold->validation_policy
      : route_execution != nullptr  ? route_execution->validation_policy
      : direct_execution != nullptr ? direct_execution->validation_policy
                                    : nullptr;
  const auto make_hold_certification = [&]() {
    return StationaryExecutionHoldCertification3D{
        .position = owned_hold_position,
        .execution_input = cycle.execution_input,
        .observed_raw_world = current_observed,
        .static_world = current_static,
        .validation_policy = policy,
        .latest_lidar_evidence = cycle.latest_lidar_evidence,
    };
  };
  const ExecutionRouteTransitionResult3D hold =
      stationary_capture_rearm
          ? armStationaryCaptureHold3D(*hold_expected, hold_expected->version,
                                       make_hold_certification())
          : transferToExecutionHold3D(*hold_expected, hold_expected->version,
                                      make_hold_certification());
  std::shared_ptr<const ExecutionRouteSnapshot3D> owned_snapshot;
  if (hold.applied()) {
    hold_transition.emplace(hold);
    owned_snapshot = hold.next;
  } else if (hold.status == ExecutionRouteTransitionStatus3D::kNoChange) {
    owned_snapshot = hold_expected;
  } else {
    return publication;
  }
  if (owned_snapshot == nullptr || !owned_snapshot->stationary_hold.has_value() ||
      owned_snapshot->route.has_value() ||
      owned_snapshot->finite_execution.has_value() ||
      owned_snapshot->direct_tracking_execution.has_value()) {
    return publication;
  }
  owned_hold_position = owned_snapshot->stationary_hold->position;
  if (!insideFlightEnvelope(owned_hold_position, flight_envelope_config_)) {
    RCLCPP_ERROR(get_logger(),
                 "EXECUTION_HORIZON rejected reason=hold_outside_flight_envelope "
                 "target_z=%.3f",
                 owned_hold_position.z);
    return publication;
  }
  if (cycle.finite_path_control_interval_ns <= 0 ||
      cycle.finite_path_control_interval_ns >
          std::numeric_limits<std::int64_t>::max() / 2) {
    return publication;
  }
  const std::int64_t requested_hold_duration_ns =
      reason == ProductionMppiExecutionReason::kGoalCapture
          ? mission_goal_capture_hold_validity_ns_
          : stationary_hold_validity_ns_;
  const std::int64_t hold_duration_ns =
      std::max(requested_hold_duration_ns, 2 * cycle.finite_path_control_interval_ns);
  const std::optional<std::int64_t> hold_valid_until_ns =
      production_mppi_execution_detail::canonicalHorizonEndTime(cycle.now_ns,
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
      horizon, owned_hold_position, 0, cycle.exact_initial_state.yaw);
  production_mppi_execution_detail::appendStationaryHoldPoint(
      horizon, owned_hold_position, cycle.finite_path_control_interval_ns,
      cycle.exact_initial_state.yaw);

  ProductionMppiHorizonCommit commit;
  if (hold_expected == nullptr) {
    return publication;
  } else if (hold_transition.has_value()) {
    commit.kind = ProductionMppiHorizonCommitKind::kPublishSnapshotTransition;
    commit.expected_snapshot = hold_expected;
    commit.transition = &*hold_transition;
  } else {
    commit.kind = ProductionMppiHorizonCommitKind::kConfirmSnapshotUnchanged;
    commit.expected_snapshot = hold_expected;
  }
  if (commitAndPublishExecutionHorizon(cycle, horizon, commit) !=
      ProductionMppiHorizonCommitStatus::kPublished) {
    return publication;
  }
  publication.horizon = {
      mppi::State{.x = static_cast<float>(owned_hold_position.x),
                  .y = static_cast<float>(owned_hold_position.y),
                  .z = static_cast<float>(owned_hold_position.z),
                  .yaw = cycle.exact_initial_state.yaw},
      mppi::State{.x = static_cast<float>(owned_hold_position.x),
                  .y = static_cast<float>(owned_hold_position.y),
                  .z = static_cast<float>(owned_hold_position.z),
                  .yaw = cycle.exact_initial_state.yaw},
  };
  publication.mode = ProductionMppiExecutionMode::kPositionHold;
  publication.reason = reason;
  publication.latest_lidar_obstacle_sequence = cycle.latest_lidar_obstacle_sequence;
  publication.latest_lidar_obstacle_hit_count =
      cycle.latest_lidar_obstacle_points.size();
  publication.latest_lidar_obstacle_age_ms = cycle.latest_lidar_obstacle_age_ms;
  publication.latest_lidar_obstacle_fresh = cycle.latest_lidar_obstacle_fresh;
  publication.latest_lidar_obstacle_receive_time_fallback =
      cycle.latest_lidar_obstacle_receive_time_fallback;
  publication.published = true;
  return publication;
}

ProductionMppiExecutionPublication ProductionMppiNode::publishNoExecutablePathHold(
    const ProductionMppiExecutionCycle& cycle,
    const ProductionMppiExecutionReason reason) {
  if (cycle.route_execution.source_snapshot != nullptr &&
      cycle.route_execution.source_snapshot->stationary_hold.has_value()) {
    ProductionMppiExecutionPublication hold = publishPositionHold(
        cycle, cycle.route_execution.source_snapshot->stationary_hold->position, reason,
        ProductionMppiHoldOwnershipTransition3D::kEnterEmptyOwner);
    if (hold.published) {
      return hold;
    }
  }
  if (std::optional<ProductionMppiExecutionPublication> retained =
          retainActiveFinitePath(cycle, reason);
      retained.has_value()) {
    return *retained;
  }
  return publishExecutionRevocation(reason, cycle.now_ns);
}

ProductionMppiExecutionPublication ProductionMppiNode::publishExecutionRevocation(
    const ProductionMppiExecutionReason reason, const std::int64_t now_ns) {
  ProductionMppiExecutionPublication publication;
  publication.mode = ProductionMppiExecutionMode::kRevoked;
  publication.reason = reason;
  if (!failClosedExecutionReason(reason) || execution_horizon_pub_ == nullptr ||
      now_ns <= 0 ||
      execution_horizon_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return publication;
  }

  const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_};
  const std::shared_ptr<const ExecutionRouteSnapshot3D> expected =
      execution_route_store_.snapshot();
  if (expected == nullptr) {
    return publication;
  }
  const ExecutionRouteTransitionResult3D transition =
      revokeExecution3D(*expected, expected->version);
  const bool transition_required = transition.applied();
  if (!transition_required &&
      transition.status != ExecutionRouteTransitionStatus3D::kNoChange) {
    return publication;
  }

  msg::MppiTrajectoryHorizon revocation;
  {
    const std::scoped_lock input_lock{input_mutex_};
    const std::int64_t publication_now_ns = get_clock()->now().nanoseconds();
    // A revoked snapshot with no live horizon owner already represents the
    // requested tombstone. Do not emit a fresh transport sequence for every
    // duplicate callback; publish only while there is an owner to revoke.
    if (!transition_required && !execution_horizon_owner_.valid) {
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
            maximum_control_feedback_age_ms_ &&
        static_cast<double>(publication_now_ns - offboard_session_receive_stamp_ns_) *
                1.0e-6 <=
            maximum_control_feedback_age_ms_;
    if (!current_session) {
      return publication;
    }
    const std::uint64_t target_offboard_instance_id =
        offboard_session_admission_.current_producer_instance_id;
    if (target_offboard_instance_id == 0U) {
      return publication;
    }

    revocation.header.stamp = now();
    revocation.header.frame_id = frame_id_;
    revocation.producer_instance_id = execution_horizon_producer_instance_id_;
    revocation.target_offboard_instance_id = target_offboard_instance_id;
    revocation.sequence = execution_horizon_sequence_ + 1U;
    revocation.valid_from =
        production_mppi_execution_detail::timeFromNanoseconds(now_ns);
    revocation.valid_until = revocation.valid_from;
    revocation.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
    revocation.execution_reason = static_cast<std::uint8_t>(reason);
    if (assessExecutionHorizonPayload(
            revocation,
            ExecutionHorizonPayloadValidationConfig{.expected_frame_id = frame_id_}) !=
        ExecutionHorizonPayloadStatus::kValid) {
      return publication;
    }

    const ExecutionRoutePublicationStatus3D snapshot_status =
        transition_required ? execution_route_store_.publish(expected, transition)
        : execution_route_store_.snapshot() == expected
            ? ExecutionRoutePublicationStatus3D::kPublished
            : ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
    if (snapshot_status != ExecutionRoutePublicationStatus3D::kPublished) {
      return publication;
    }
    execution_horizon_sequence_ = revocation.sequence;
    applied_control_ = {};
    execution_horizon_owner_ = {};
    execution_horizon_pub_->publish(revocation);
    publication.published = true;
  }
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                       "EXECUTION_HORIZON revoked=true snapshot_version=%" PRIu64
                       " owner_epoch=%" PRIu64 " sequence=%" PRIu64 " reason=%s",
                       transition_required ? transition.next->version
                                           : expected->version,
                       transition_required ? transition.next->execution_owner_epoch
                                           : expected->execution_owner_epoch,
                       revocation.sequence, productionMppiExecutionReasonName(reason));
  return publication;
}

bool ProductionMppiNode::handleRequestedExecutionRevocation(const std::int64_t now_ns) {
  const std::uint64_t requested_revocation =
      requested_execution_revocation_.load(std::memory_order_acquire);
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
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    const std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot =
        execution_route_store_.snapshot();
    const bool snapshot_has_executable_authority =
        snapshot != nullptr && (snapshot->finite_execution.has_value() ||
                                snapshot->direct_tracking_execution.has_value() ||
                                snapshot->stationary_hold.has_value());
    revocation_already_satisfied =
        !snapshot_has_executable_authority && !execution_horizon_owner_.valid;
  }
  if (revocation_already_satisfied) {
    handled_execution_revocation_request_ = requested_revocation;
  }
  return true;
}

void ProductionMppiNode::publishFailClosedExecutionRevocation(
    const ProductionMppiExecutionReason reason, const std::int64_t now_ns) {
  const std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot =
      execution_route_store_.snapshot();
  const bool authority_present =
      snapshot != nullptr && (snapshot->phase == ExecutionRoutePhase3D::kRevoked ||
                              snapshot->finite_execution.has_value() ||
                              snapshot->direct_tracking_execution.has_value() ||
                              snapshot->stationary_hold.has_value());
  if (authority_present) {
    static_cast<void>(publishExecutionRevocation(reason, now_ns));
  }
}

void ProductionMppiNode::requestExecutionRevocation(
    const ProductionMppiExecutionReason reason) noexcept {
  if (!failClosedExecutionReason(reason)) {
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
  return publishPositionHold(
      cycle, hold_position, reason,
      ProductionMppiHoldOwnershipTransition3D::kExplicitTransfer);
}

} // namespace drone_city_nav
