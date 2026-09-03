#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"

#include <cstdint>
#include <memory>

namespace drone_city_nav {

// The caller maps mission/runtime policy to one of these domain intents. The
// supervisor still decides whether the captured resident state can satisfy it.
enum class ExecutionHoldIntent3D : std::uint8_t {
  kRefreshResident,
  kExplicitTransfer,
  kExplicitTransferWithStationaryCaptureRearm,
};

enum class ExecutionHoldPreparationKind3D : std::uint8_t {
  kNone,
  kTransition,
  kUnchangedPlan,
};

enum class ExecutionHoldPreparationStatus3D : std::uint8_t {
  kPrepared,
  kMissingAuthority,
  kSourceNotCurrent,
  kIntentNotApplicable,
  kExecutionInputInvalid,
  kLidarEvidenceNotCurrent,
  kValidationWorldUnavailable,
  kTransitionRejected,
};

[[nodiscard]] const char*
executionHoldPreparationKind3DName(ExecutionHoldPreparationKind3D kind) noexcept;

[[nodiscard]] const char*
executionHoldPreparationStatus3DName(ExecutionHoldPreparationStatus3D status) noexcept;

// Owned evidence for one stationary-hold preparation. The cycle source proves
// that the caller and supervisor refer to the same plan. Current raw/lidar
// owners are captured by the runtime adapter under its evidence lock; no raw
// views or caller-owned temporaries cross this boundary.
struct ExecutionHoldRequest3D {
  ExecutionHoldIntent3D intent{ExecutionHoldIntent3D::kRefreshResident};
  Point3 requested_position{};
  std::shared_ptr<const ExecutionPlan3D> cycle_source_plan;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar_evidence;
  std::shared_ptr<const VersionedObservedRawWorld3D> current_observed_raw_world;
  std::shared_ptr<const VersionedObservedRawWorld3D>
      stationary_capture_observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> stationary_capture_static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D>
      selected_validation_policy;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D>
      stationary_capture_validation_policy;
  std::int64_t validation_now_ns{0};
  bool raw_world_identity_conflicted{false};
  bool latest_lidar_identity_conflicted{false};
};

// Preparation is side-effect free. A later wire publication commits either the
// owned transition or the unchanged exact plan against expected_authority.
struct ExecutionHoldPreparation3D {
  ExecutionHoldPreparationKind3D kind{ExecutionHoldPreparationKind3D::kNone};
  ExecutionHoldPreparationStatus3D status{
      ExecutionHoldPreparationStatus3D::kMissingAuthority};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> transition;
  ExecutionRouteTransitionStatus3D transition_status{
      ExecutionRouteTransitionStatus3D::kInvalidCandidate};
  ExecutionRouteTransitionDetail3D transition_detail{
      ExecutionRouteTransitionDetail3D::kNone};
  Point3 position{};
  bool stationary_capture_rearm{false};

  [[nodiscard]] bool prepared() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> expectedPlan() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> preparedPlan() const noexcept;
  [[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D>
  executionInput() const noexcept;
};

} // namespace drone_city_nav
