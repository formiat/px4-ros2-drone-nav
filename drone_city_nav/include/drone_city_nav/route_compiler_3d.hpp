#pragma once

#include "drone_city_nav/execution_route_geometry_3d.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {

struct RouteCompilerConfig3D {
  double unconstrained_speed_mps{20.0};
  double constrained_speed_mps{10.0};
  double minimum_continuous_turn_alignment{0.7071067811865476};
  MppiSpeedPolicyConfig speed_policy{};
  mppi::DynamicsConfig dynamics{};
  SweptFootprintConfig physical_footprint{};
  TrackingErrorTubeConfig3D tracking_error_tube{};
};

struct RouteCompilerInput3D {
  std::vector<RouteSample3D> route;
  std::vector<ConstrainedRouteSpan> constrained_spans;
  std::vector<PassageVolume> passage_volumes;
  std::vector<CooperativePassageAssignment> cooperative_passage_assignments;
  std::vector<PassageTraversalId> selected_passage_traversal_ids;
  PassageVolumeConfig passage_volume_config{};
  RouteEndpointSemantics3D endpoint_semantics{RouteEndpointSemantics3D::kContinuation};
  std::uint64_t materialized_route_fingerprint{0U};
  TrackingErrorTubeWorld3D tracking_world{};
  RouteCompilerConfig3D config{};
};

struct RouteCompilationResult3D {
  std::shared_ptr<const ExecutionRouteGeometry3D> geometry;
  ExecutionRouteGeometryValidation3D validation{};
  std::size_t stop_turn_count{0U};
  std::shared_ptr<const TrackingErrorTubeProfile3D> tracking_error_tube;

  [[nodiscard]] bool compiled() const noexcept {
    return geometry != nullptr && validation.valid() &&
           tracking_error_tube != nullptr && tracking_error_tube->valid;
  }
};

// The only constructor for a candidate's executable geometry. It owns
// canonical stations/tangents, the time profile, projection, and every passage
// sidecar, then seals them under one immutable geometry revision.
[[nodiscard]] RouteCompilationResult3D
compileExecutionRoute3D(RouteCompilerInput3D input);

// Recompile the dynamic tracking tube and time profile for an unchanged spatial
// route. Passage resources remain sealed to the source geometry and are still
// checked against the activation world by route certification.
[[nodiscard]] RouteCompilationResult3D recompileExecutionRouteDynamics3D(
    const ExecutionRouteGeometry3D& source, RouteEndpointSemantics3D endpoint_semantics,
    TrackingErrorTubeWorld3D tracking_world, const RouteCompilerConfig3D& config);

} // namespace drone_city_nav
