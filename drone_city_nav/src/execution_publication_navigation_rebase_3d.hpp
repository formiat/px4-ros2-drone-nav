#pragma once

#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/finite_execution_path_3d.hpp"
#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/pending_certified_route_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

enum class ExecutionPublicationNavigationRebaseStatus3D : std::uint8_t {
  kRebased,
  kInvalidRequest,
  kEvidenceUnavailable,
  kPathUnavailable,
  kPathRejected,
  kCertificationRejected,
};

struct ExecutionPublicationNavigationRebaseRequest3D {
  const ExecutionPlan3D* expected_snapshot{nullptr};
  // A progress-only preparation is a valid certification base, but never a
  // publishable owner. When present it must be composed back onto the resident
  // expected_snapshot in the same atomic publication transition.
  const ExecutionPlan3D* certification_snapshot{nullptr};
  const ExecutionRouteTransitionResult3D* progress_preparation{nullptr};
  const ExecutionPlan3D* candidate_snapshot{nullptr};
  const PendingCertifiedRoute3D* expected_pending{nullptr};
  // Required when the candidate has already transferred ownership to an
  // emergency braking tail. The event is replayed against the exact navigation
  // and sensor evidence captured under the publication lock.
  const RouteLifecycleEvent3D* lifecycle_event{nullptr};
  std::shared_ptr<const VersionedExecutionInput3D> current_execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar_evidence;
  std::shared_ptr<const VersionedObservedRawWorld3D> current_observed_raw_world;
  std::int64_t publication_now_ns{0};
  std::size_t arrival_search_step_controls{0U};
  const FiniteMotionHorizonConfig3D* finite_horizon_config{nullptr};
  std::optional<FiniteExecutionPathTerminalBoundary3D> terminal_boundary;
};

struct ExecutionPublicationNavigationRebaseResult3D {
  ExecutionPublicationNavigationRebaseStatus3D status{
      ExecutionPublicationNavigationRebaseStatus3D::kInvalidRequest};
  std::optional<ExecutionRouteTransitionResult3D> transition;
  FiniteExecutionPathStatus3D path_validation_status{
      FiniteExecutionPathStatus3D::kInvalidContract};
  FiniteExecutionCertificationStatus3D route_certification_status{
      FiniteExecutionCertificationStatus3D::kInvalidInput};
  FiniteExecutionRouteAdherenceStatus3D route_adherence_status{
      FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated};
  ExecutionRouteTransitionStatus3D transition_status{
      ExecutionRouteTransitionStatus3D::kInvalidCandidate};
  std::size_t source_control_index{0U};
  std::size_t route_adherence_failure_state_index{0U};
  double route_adherence_failure_distance_m{-1.0};

  [[nodiscard]] bool rebased() const noexcept;
};

[[nodiscard]] ExecutionPublicationNavigationRebaseResult3D
rebaseExecutionPublicationForCurrentNavigation3D(
    const ExecutionPublicationNavigationRebaseRequest3D& request);

[[nodiscard]] const char* executionPublicationNavigationRebaseStatus3DName(
    ExecutionPublicationNavigationRebaseStatus3D status) noexcept;

} // namespace drone_city_nav
