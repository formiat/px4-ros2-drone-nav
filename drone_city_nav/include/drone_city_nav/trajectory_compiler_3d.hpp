#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/flight_time_model_3d.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {

struct TrajectoryCompilerConfig3D {
  double unconstrained_speed_mps{5.0};
  double constrained_speed_mps{3.0};
  double maximum_lateral_acceleration_mps2{4.0};
  double minimum_continuous_turn_alignment{0.7071067811865476};
  FlightTimeModel3D time_model{};
  SweptFootprintConfig physical_footprint{};
  TrackingErrorTubeConfig3D tracking_error_tube{};
};

struct TrajectoryCompilerInput3D {
  VehicleState3D exact_initial_state{};
  std::uint64_t route_generation{0U};
  std::vector<RouteSample3D> route;
  std::vector<ConstrainedRouteSpan> constrained_spans;
  std::vector<PassageVolume> passage_volumes;
  std::vector<CooperativePassageAssignment> cooperative_passage_assignments;
  std::vector<PassageTraversalId> selected_passage_traversal_ids;
  PassageVolumeConfig passage_volume_config{};
  RouteEndpointSemantics3D endpoint_semantics{RouteEndpointSemantics3D::kContinuation};
  std::uint64_t materialized_route_fingerprint{0U};
  TrackingErrorTubeWorld3D tracking_world{};
  TrajectoryCompilerConfig3D config{};
};

struct TrajectoryCompilationResult3D {
  std::shared_ptr<const CompiledTrajectory3D> trajectory;
  CompiledTrajectoryValidation3D validation{};
  std::size_t stop_turn_count{0U};

  [[nodiscard]] bool compiled() const noexcept {
    return trajectory != nullptr && validation.valid();
  }
};

// The only constructor for executable route trajectories. Compilation receives
// one exact measured initial state, canonicalizes the spatial route, creates
// the tracking tube and time profile once, and seals every resource under one
// immutable revision.
class TrajectoryCompiler3D final {
public:
  [[nodiscard]] static TrajectoryCompilationResult3D
  compile(TrajectoryCompilerInput3D input);
};

} // namespace drone_city_nav
