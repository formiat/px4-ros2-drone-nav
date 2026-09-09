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

// The profile a stop is built with. The guaranteed deceleration in `config` is
// what the speed policy plans against, so a routine braking tail fits what the
// sensors see; a stop is the last motion the vehicle can be given, and it
// brakes with the full authority the dynamics admit, the same dynamics it is
// certified and executed under. A stop shaped to the guaranteed vertical
// deceleration once stretched a descending vehicle's brake to five metres and
// rested it below the floor it was diving towards.
[[nodiscard]] FiniteMotionHorizonConfig3D
stopMotionHorizonConfig3D(const FiniteMotionHorizonConfig3D& config,
                          const MotionDynamicsConfig3D& dynamics) noexcept;

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
  // A floor on the trajectory length. The stop derives its own length from
  // the state and the dynamics; this only keeps it at least as long as the
  // control sequence the caller is replacing.
  std::size_t minimum_control_count{0U};
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
  // Why the stop's certification refused it, when the transition rejected it
  // as an invalid candidate.
  StopCertificationResult3D certification{};
  double initial_speed_mps{0.0};
  double stop_distance_m{0.0};

  [[nodiscard]] bool prepared() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> expectedPlan() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> preparedPlan() const noexcept;
  [[nodiscard]] const StopExecution3D* stopExecution() const noexcept;
};

} // namespace drone_city_nav
