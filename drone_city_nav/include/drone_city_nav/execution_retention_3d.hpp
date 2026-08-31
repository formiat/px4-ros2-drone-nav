#pragma once

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

enum class ExecutionRetentionKind3D : std::uint8_t {
  kNone,
  kRoute,
  kDirectTracking,
};

enum class ExecutionRetentionStatus3D : std::uint8_t {
  kPrepared,
  kMissingResidentOwner,
  kInvalidLifecycleOwner,
  kInvalidActivePath,
  kValidationWorldUnavailable,
  kTrajectoryRevisionExhausted,
  kRebuildRejected,
  kCertificationRejected,
  kTransitionRejected,
};

[[nodiscard]] const char*
executionRetentionKind3DName(ExecutionRetentionKind3D kind) noexcept;

[[nodiscard]] const char*
executionRetentionStatus3DName(ExecutionRetentionStatus3D status) noexcept;

// Exact, owned evidence for one attempt to recertify the resident finite
// execution. The supervisor captures the current authority itself; a lifecycle
// source is used only to prove that a braking event belongs to that authority.
struct ExecutionRetentionRequest3D {
  std::shared_ptr<const ExecutionPlan3D> lifecycle_source_plan;
  std::optional<RouteLifecycleEvent3D> lifecycle_event;
  std::shared_ptr<const VersionedObservedRawWorld3D> lifecycle_observed_raw_world;
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  mppi::State exact_initial_state{};
  mppi::Control exact_previous_control{};
  mppi::FiniteHorizonConfig finite_horizon_config{};
  std::int64_t now_ns{0};
  std::int64_t lidar_validation_now_ns{0};
};

// Prepared retention never mutates the resident authority. The exact captured
// authority and owned transition are returned together so later horizon
// publication can perform one compare-and-swap lease commit.
struct ExecutionRetentionResult3D {
  ExecutionRetentionKind3D kind{ExecutionRetentionKind3D::kNone};
  ExecutionRetentionStatus3D status{ExecutionRetentionStatus3D::kMissingResidentOwner};
  std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority;
  std::shared_ptr<const ExecutionRouteTransitionResult3D> transition;
  mppi::FiniteExecutionPathValidation actual_state_validation{};
  mppi::FiniteExecutionPathValidation trajectory_validation{};
  mppi::FiniteExecutionPathValidation rebuild_validation{};
  FiniteExecutionCertificationResult3D certification{};
  std::optional<RouteLifecycleEventKind3D> braking_event;
  std::size_t arrival_shaping_attempts{0U};
  std::uint64_t source_trajectory_revision{0U};
  std::uint64_t prepared_trajectory_revision{0U};

  [[nodiscard]] bool prepared() const noexcept;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> expectedPlan() const noexcept;
};

} // namespace drone_city_nav
