#pragma once

#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/finite_execution_path.hpp"
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
  const ExecutionRouteSnapshot3D* expected_snapshot{nullptr};
  const ExecutionRouteSnapshot3D* candidate_snapshot{nullptr};
  const PendingCertifiedRoute3D* expected_pending{nullptr};
  std::shared_ptr<const VersionedExecutionInput3D> current_execution_input;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar_evidence;
  std::shared_ptr<const VersionedObservedRawWorld3D> current_observed_raw_world;
  std::int64_t publication_now_ns{0};
  std::size_t arrival_search_step_controls{0U};
  const mppi::FiniteHorizonConfig* finite_horizon_config{nullptr};
  std::optional<mppi::FiniteExecutionPathTerminalBoundary> terminal_boundary;
};

struct ExecutionPublicationNavigationRebaseResult3D {
  ExecutionPublicationNavigationRebaseStatus3D status{
      ExecutionPublicationNavigationRebaseStatus3D::kInvalidRequest};
  std::optional<ExecutionRouteTransitionResult3D> transition;

  [[nodiscard]] bool rebased() const noexcept;
};

[[nodiscard]] ExecutionPublicationNavigationRebaseResult3D
rebaseExecutionPublicationForCurrentNavigation3D(
    const ExecutionPublicationNavigationRebaseRequest3D& request);

[[nodiscard]] const char* executionPublicationNavigationRebaseStatus3DName(
    ExecutionPublicationNavigationRebaseStatus3D status) noexcept;

} // namespace drone_city_nav
