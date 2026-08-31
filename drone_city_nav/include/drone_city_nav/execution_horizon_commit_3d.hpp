#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/execution_publication_currentness_3d.hpp"
#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/offboard_session_admission.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

enum class ExecutionHorizonCommitKind3D : std::uint8_t {
  kTransition,
  kUnchangedPlan,
  kPendingTransition,
};

enum class ExecutionHorizonCommitStatus3D : std::uint8_t {
  kCommitted,
  kInvalidRequest,
  kAuthorityNotCurrent,
  kVehicleStatusNotAuthoritative,
  kRawWorldNotCurrent,
  kHorizonTimeNotCurrent,
  kRevocationEpochChanged,
  kObjectiveChanged,
  kNavigationNotAuthoritative,
  kOffboardSessionNotCurrent,
  kExecutionInputNotCurrent,
  kExecutionInputNotFresh,
  kEvidenceNotCurrent,
  kLidarEvidenceNotCurrent,
  kControlEvidenceNotCurrent,
  kOwnerInvalid,
  kOwnerPlanIdentityInvalid,
  kLeaseRejected,
};

enum class ExecutionHorizonRevocationRequest3D : std::uint8_t {
  kNone,
  kUnavailableWorld,
  kNoExecutableHorizon,
};

// Adapter-owned runtime facts captured under its input/evidence locks. The
// supervisor evaluates them in the same ordered transaction as plan evidence
// and the final lease compare-and-swap.
struct ExecutionHorizonRuntimeCurrentness3D {
  bool vehicle_status_authoritative{false};
  bool raw_world_identity_conflicted{false};
  double current_raw_age_ms{-1.0};
  double maximum_raw_age_ms{0.0};
  bool revocation_epoch_current{false};
  bool objective_current{false};
  bool navigation_authoritative{false};
  OffboardSessionPublicationCurrentnessStatus offboard_session_currentness{
      OffboardSessionPublicationCurrentnessStatus::kInvalidInput};
};

// Exact navigation/control witness captured by the runtime adapter. No mutable
// node state or ROS message crosses the execution-service boundary.
struct ExecutionHorizonNavigationWitness3D {
  MotionState3D state{};
  MotionControl3D measured_equivalent_control{};
  std::uint64_t pose_revision{0U};
  std::uint64_t source_timestamp_us{0U};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t measured_control_source_sequence{0U};
  std::int64_t measured_control_source_stamp_ns{0};
  std::int64_t measured_control_receive_stamp_ns{0};
  bool measured_acceleration_authoritative{false};
};

// Owned candidate for one controller-visible lease. A progress-only snapshot
// may certify a transition but can never be published independently of the
// exact resident predecessor and composed transition stored here.
struct ExecutionHorizonLeaseCandidate3D {
  ExecutionHorizonCommitKind3D kind{ExecutionHorizonCommitKind3D::kUnchangedPlan};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionPlan3D> expected_plan;
  std::shared_ptr<const ExecutionPlan3D> certification_plan;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> progress_preparation;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> transition;
  std::shared_ptr<const PendingCertifiedRoute3D> expected_pending;
  ExecutionOwnerIdentity3D owner{};
  std::uint64_t expected_horizon_producer_instance_id{0U};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  bool stationary_capture_rearm_intent{false};
};

struct ExecutionHorizonCommitRequest3D {
  ExecutionHorizonLeaseCandidate3D candidate{};
  ExecutionHorizonRuntimeCurrentness3D runtime{};
  ExecutionHorizonNavigationWitness3D navigation{};
  std::shared_ptr<const VersionedObservedRawWorld3D> current_observed_raw_world;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar_evidence;
  std::int64_t publication_now_ns{0};
  double maximum_control_feedback_age_ms{0.0};
  bool latest_lidar_identity_conflicted{false};
};

struct ExecutionHorizonCommitResult3D {
  ExecutionHorizonCommitStatus3D status{
      ExecutionHorizonCommitStatus3D::kInvalidRequest};
  ExecutionHorizonRevocationRequest3D revocation_request{
      ExecutionHorizonRevocationRequest3D::kNone};
  ExecutionPublicationCurrentnessStatus3D publication_currentness{
      ExecutionPublicationCurrentnessStatus3D::kSnapshotMissing};
  ExecutionRoutePublicationStatus3D lease_status{
      ExecutionRoutePublicationStatus3D::kInvalidCandidate};
  bool latest_evidence_revalidated{false};
  bool replaced_applied_control{false};

  [[nodiscard]] bool committed() const noexcept;
};

[[nodiscard]] bool appliedControlAuthoritativeForExecution3D(
    const AppliedControlEvidence3D& control, const ExecutionOwnerIdentity3D& owner,
    std::int64_t now_ns, double maximum_age_ms) noexcept;

[[nodiscard]] const char*
executionHorizonCommitStatus3DName(ExecutionHorizonCommitStatus3D status) noexcept;

} // namespace drone_city_nav
