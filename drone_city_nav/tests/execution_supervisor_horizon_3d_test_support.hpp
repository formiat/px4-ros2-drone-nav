#pragma once

#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

namespace drone_city_nav {

struct ExecutionHorizonTestTransaction3D {
  ExecutionHorizonCommitKind3D kind{ExecutionHorizonCommitKind3D::kUnchangedPlan};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionPlan3D> expected_plan;
  std::optional<ExecutionRouteTransitionResult3D> transition;
  std::shared_ptr<const PendingCertifiedRoute3D> expected_pending;
  ExecutionOwnerIdentity3D owner{};
  std::shared_ptr<const VersionedExecutionInput3D> input;
  bool stationary_capture_rearm_intent{false};
};

[[nodiscard]] inline std::shared_ptr<const VersionedObservedRawWorld3D>
executionHorizonTestRawOwner(const ExecutionPlan3D& plan) {
  if (const FiniteExecutionState3D* const execution = plan.finiteExecution()) {
    return execution->observed_raw_world;
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          plan.directTrackingExecution()) {
    return execution->observed_raw_world;
  }
  if (const StationaryExecutionHold3D* const hold = plan.stationaryHold()) {
    return hold->observed_raw_world;
  }
  return nullptr;
}

[[nodiscard]] inline std::shared_ptr<const VersionedLatestLidarEvidence3D>
executionHorizonTestLidarOwner(const ExecutionPlan3D& plan) {
  if (const FiniteExecutionState3D* const execution = plan.finiteExecution()) {
    return execution->latest_lidar_evidence;
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          plan.directTrackingExecution()) {
    return execution->latest_lidar_evidence;
  }
  if (const StationaryExecutionHold3D* const hold = plan.stationaryHold()) {
    return hold->latest_lidar_evidence;
  }
  return nullptr;
}

[[nodiscard]] inline ExecutionHorizonNavigationWitness3D
executionHorizonTestNavigationWitness(const VersionedExecutionInput3D& input) {
  return ExecutionHorizonNavigationWitness3D{
      .state = input.state(),
      .measured_equivalent_control = input.previousControl(),
      .pose_revision = input.poseRevision(),
      .source_timestamp_us = input.poseSourceTimestampUs(),
      .receive_stamp_ns = input.poseReceiveStampNs(),
      .measured_control_source_sequence = input.previousControlSourceSequence(),
      .measured_control_source_stamp_ns = input.previousControlSourceStampNs(),
      .measured_control_receive_stamp_ns = input.previousControlReceiveStampNs(),
      .measured_acceleration_authoritative = true,
  };
}

[[nodiscard]] inline ExecutionHorizonCommitRequest3D
makeExecutionHorizonTestRequest(ExecutionHorizonTestTransaction3D transaction) {
  const std::shared_ptr<const ExecutionPlan3D> publication_plan = [&] {
    if (transaction.kind == ExecutionHorizonCommitKind3D::kUnchangedPlan) {
      return transaction.expected_plan;
    }
    return transaction.transition.has_value() ? transaction.transition->next : nullptr;
  }();
  const std::shared_ptr<const VersionedObservedRawWorld3D> raw =
      publication_plan != nullptr ? executionHorizonTestRawOwner(*publication_plan)
                                  : nullptr;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> lidar =
      publication_plan != nullptr ? executionHorizonTestLidarOwner(*publication_plan)
                                  : nullptr;
  const ExecutionHorizonNavigationWitness3D navigation =
      transaction.input != nullptr
          ? executionHorizonTestNavigationWitness(*transaction.input)
          : ExecutionHorizonNavigationWitness3D{};
  const std::shared_ptr<const ExecutionPlan3D> certification_plan =
      transaction.expected_plan;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> transition;
  if (transaction.transition.has_value()) {
    transition = std::make_shared<const ExecutionRouteTransitionResult3D>(
        std::move(*transaction.transition));
  }
  return ExecutionHorizonCommitRequest3D{
      .candidate =
          ExecutionHorizonLeaseCandidate3D{
              .kind = transaction.kind,
              .expected_authority = std::move(transaction.expected_authority),
              .expected_plan = std::move(transaction.expected_plan),
              .certification_plan = certification_plan,
              .progress_preparation = nullptr,
              .transition = std::move(transition),
              .expected_pending = std::move(transaction.expected_pending),
              .owner = transaction.owner,
              .expected_horizon_producer_instance_id =
                  transaction.owner.producer_instance_id,
              .execution_input = std::move(transaction.input),
              .stationary_capture_rearm_intent =
                  transaction.stationary_capture_rearm_intent,
          },
      .runtime =
          ExecutionHorizonRuntimeCurrentness3D{
              .vehicle_status_authoritative = true,
              .current_raw_age_ms = 0.0,
              .maximum_raw_age_ms = 1000.0,
              .revocation_epoch_current = true,
              .objective_current = true,
              .navigation_authoritative = true,
              .offboard_session_currentness =
                  OffboardSessionPublicationCurrentnessStatus::kCurrent,
          },
      .navigation = navigation,
      .current_observed_raw_world = raw,
      .current_lidar_evidence = lidar,
      .publication_now_ns = transaction.owner.valid_from_ns,
      .maximum_control_feedback_age_ms = 1000.0,
  };
}

[[nodiscard]] inline ExecutionHorizonCommitResult3D
commitExecutionHorizonForTest(ExecutionSupervisor3D& supervisor,
                              ExecutionHorizonTestTransaction3D transaction) {
  return supervisor.commitHorizon(
      makeExecutionHorizonTestRequest(std::move(transaction)));
}

} // namespace drone_city_nav
