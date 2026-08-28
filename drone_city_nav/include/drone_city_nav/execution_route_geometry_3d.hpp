#pragma once

#include "drone_city_nav/cooperative_passage_route.hpp"
#include "drone_city_nav/observation_frontier.hpp"
#include "drone_city_nav/passage_volume.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

struct ExecutionRouteGeometry3D {
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_route;
  std::shared_ptr<const std::vector<RouteSample3D>> route;
  std::shared_ptr<const TrackingErrorTubeProfile3D> tracking_error_tube;
  std::shared_ptr<const std::vector<Point2>> route_2d_projection;
  std::shared_ptr<const std::vector<ConstrainedRouteSpan>> constrained_spans;
  std::shared_ptr<const std::vector<PassageVolume>> passage_volumes;
  std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      cooperative_passage_assignments;
  std::shared_ptr<const std::vector<PassageTraversalId>> selected_passage_traversal_ids;
  PassageVolumeConfig passage_volume_config{};
  Lattice3DRoutePurpose route_purpose{Lattice3DRoutePurpose::kMissionTransit};
  std::optional<ObservationFrontier> observation_frontier;
  std::uint64_t materialized_route_fingerprint{0U};
  std::uint64_t physical_route_fingerprint{0U};
  std::uint64_t executable_geometry_revision{0U};
};

enum class ExecutionRouteGeometryFailureReason3D : std::uint8_t {
  kNotAttempted,
  kValid,
  kMissingRoute,
  kTooFewSamples,
  kNonFiniteSample,
  kInvalidTangent,
  kNonMonotonicStation,
  kSegmentStationMismatch,
  kIncomingTangentMismatch,
  kTerminalTangentMismatch,
  kInvalidTimeProfile,
  kInvalidProjection,
  kInvalidConstrainedSpans,
  kInvalidPassageResources,
  kInvalidFingerprint,
  kDerivedResourceMismatch,
};

struct ExecutionRouteGeometryValidation3D {
  ExecutionRouteGeometryFailureReason3D reason{
      ExecutionRouteGeometryFailureReason3D::kNotAttempted};
  std::size_t sample_index{0U};

  [[nodiscard]] bool valid() const noexcept {
    return reason == ExecutionRouteGeometryFailureReason3D::kValid;
  }
};

[[nodiscard]] const char* executionRouteGeometryFailureReasonName3D(
    ExecutionRouteGeometryFailureReason3D reason) noexcept;

[[nodiscard]] ExecutionRouteGeometryValidation3D
validateExecutionRouteGeometrySamples3D(std::span<const RouteSample3D> route) noexcept;

using ProductionRouteGeometry3D = ExecutionRouteGeometry3D;

// Covers every executable route resource except the resulting revision field.
// Returns zero when a required resource or scalar is invalid.
[[nodiscard]] std::uint64_t
executionRouteGeometryRevision3D(const ExecutionRouteGeometry3D& geometry) noexcept;

// Covers the exact physical route, constrained spans, derived passage volumes,
// ordered traversal identities, and derivation configuration. Returns zero
// when any required passage resource or configuration is invalid.
[[nodiscard]] std::uint64_t
executionPassageGeometryRevision3D(const ExecutionRouteGeometry3D& geometry) noexcept;

} // namespace drone_city_nav
