#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace drone_city_nav {

enum class ExecutionStopStatus3D : std::uint8_t {
  kPrepared,
  kMissingAuthority,
  kSourceNotCurrent,
  kExecutionInputInvalid,
  kAtRest,
  kResidentStopCurrent,
  kValidationWorldUnavailable,
  kLidarEvidenceNotCurrent,
  kHorizonUnavailable,
  kRevisionExhausted,
  kTransitionRejected,
};

[[nodiscard]] const char*
executionStopStatus3DName(ExecutionStopStatus3D status) noexcept;

// Owned evidence for one stop preparation. A stop is derived from the exact
// state the vehicle is in now and the newest world owned by the caller: it is
// never derived from the route, because the situations that need a stop are
// exactly the ones where the route produced nothing.
struct ExecutionStopRequest3D {
  std::shared_ptr<const ExecutionPlan3D> cycle_source_plan;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::shared_ptr<const VersionedStaticWorld3D> static_world;
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy;
  MotionState3D exact_initial_state{};
  MotionControl3D exact_previous_control{};
  FiniteMotionHorizonConfig3D finite_horizon_config{};
  std::size_t maximum_control_count{0U};
  std::int64_t now_ns{0};
};

// Preparation is side-effect free. The publication commits the owned
// transition against expected_authority, exactly like every other lease.
struct ExecutionStopPreparation3D {
  ExecutionStopStatus3D status{ExecutionStopStatus3D::kMissingAuthority};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> transition;
  ExecutionRouteTransitionStatus3D transition_status{
      ExecutionRouteTransitionStatus3D::kInvalidCandidate};
  ExecutionRouteTransitionDetail3D transition_detail{
      ExecutionRouteTransitionDetail3D::kNone};
  double initial_speed_mps{0.0};
  double stop_distance_m{0.0};

  [[nodiscard]] bool prepared() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> expectedPlan() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> preparedPlan() const noexcept;
  [[nodiscard]] const StopExecution3D* stopExecution() const noexcept;
};

} // namespace drone_city_nav
