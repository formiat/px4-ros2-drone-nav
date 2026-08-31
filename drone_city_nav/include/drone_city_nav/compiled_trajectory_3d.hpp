#pragma once

#include "drone_city_nav/cooperative_passage_route.hpp"
#include "drone_city_nav/passage_volume.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"
#include "drone_city_nav/tracking_error_tube_3d.hpp"
#include "drone_city_nav/vehicle_state_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

class TrajectoryCompiler3D;

struct CompiledTrajectoryTimeProfile3D {
  double travel_time_s{0.0};
  double translation_time_s{0.0};
  double stationary_turn_time_s{0.0};
  std::vector<double> arrival_times_s;
  std::vector<double> departure_times_s;

  [[nodiscard]] bool valid() const noexcept;
};

// Sealed controller-neutral executable trajectory. Only TrajectoryCompiler3D
// can construct an instance; every field is const and every owned collection is
// exposed through shared_ptr<const>. MPPI references and planar visualizations
// are derived adapters and are intentionally absent.
class CompiledTrajectory3D final {
public:
  CompiledTrajectory3D(const CompiledTrajectory3D&) = delete;
  CompiledTrajectory3D& operator=(const CompiledTrajectory3D&) = delete;
  CompiledTrajectory3D(CompiledTrajectory3D&&) = delete;
  CompiledTrajectory3D& operator=(CompiledTrajectory3D&&) = delete;
  ~CompiledTrajectory3D() = default;

  const VehicleState3D exact_initial_state;
  const RouteEndpointSemantics3D endpoint_semantics;
  const std::shared_ptr<const std::vector<RouteSample3D>> route;
  const std::shared_ptr<const TrackingErrorTubeProfile3D> tracking_error_tube;
  const std::shared_ptr<const std::vector<ConstrainedRouteSpan>> constrained_spans;
  const std::shared_ptr<const std::vector<PassageVolume>> passage_volumes;
  const std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      cooperative_passage_assignments;
  const std::shared_ptr<const std::vector<PassageTraversalId>>
      selected_passage_traversal_ids;
  const PassageVolumeConfig passage_volume_config;
  const CompiledTrajectoryTimeProfile3D time_profile;
  const std::uint64_t materialized_route_fingerprint;
  const std::uint64_t physical_route_fingerprint;
  const std::uint64_t compiled_trajectory_revision;

private:
  friend class TrajectoryCompiler3D;

  CompiledTrajectory3D(
      VehicleState3D initial_state,
      RouteEndpointSemantics3D compiled_endpoint_semantics,
      std::shared_ptr<const std::vector<RouteSample3D>> compiled_route,
      std::shared_ptr<const TrackingErrorTubeProfile3D> compiled_tracking_error_tube,
      std::shared_ptr<const std::vector<ConstrainedRouteSpan>>
          compiled_constrained_spans,
      std::shared_ptr<const std::vector<PassageVolume>> compiled_passage_volumes,
      std::shared_ptr<const std::vector<CooperativePassageAssignment>>
          compiled_cooperative_passage_assignments,
      std::shared_ptr<const std::vector<PassageTraversalId>>
          compiled_selected_passage_traversal_ids,
      PassageVolumeConfig compiled_passage_volume_config,
      CompiledTrajectoryTimeProfile3D compiled_time_profile,
      std::uint64_t compiled_materialized_route_fingerprint,
      std::uint64_t compiled_physical_route_fingerprint);
};

enum class CompiledTrajectoryFailureReason3D : std::uint8_t {
  kNotAttempted,
  kValid,
  kMissingRoute,
  kInvalidInitialState,
  kTooFewSamples,
  kNonFiniteSample,
  kInvalidTangent,
  kNonMonotonicStation,
  kSegmentStationMismatch,
  kIncomingTangentMismatch,
  kTerminalTangentMismatch,
  kInvalidTrackingErrorTube,
  kInvalidTimeProfile,
  kInvalidConstrainedSpans,
  kInvalidPassageResources,
  kInvalidFingerprint,
  kDerivedResourceMismatch,
};

struct CompiledTrajectoryValidation3D {
  CompiledTrajectoryFailureReason3D reason{
      CompiledTrajectoryFailureReason3D::kNotAttempted};
  std::size_t sample_index{0U};

  [[nodiscard]] bool valid() const noexcept {
    return reason == CompiledTrajectoryFailureReason3D::kValid;
  }
};

[[nodiscard]] const char* compiledTrajectoryFailureReason3DName(
    CompiledTrajectoryFailureReason3D reason) noexcept;

[[nodiscard]] CompiledTrajectoryValidation3D
validateCompiledTrajectorySamples3D(std::span<const RouteSample3D> route) noexcept;

// Validates the sealed controller-neutral resource graph, including endpoint
// semantics, speed/tube consistency, route generation, passage IDs, frames,
// volumes, and cooperative assignments. No execution identity is required.
[[nodiscard]] bool compiledTrajectoryResourcesValid3D(
    const CompiledTrajectory3D& trajectory,
    std::uint64_t expected_route_generation = 0U) noexcept;

// Covers the exact initial state and every controller-neutral executable
// resource. Returns zero when any required resource or scalar is invalid.
[[nodiscard]] std::uint64_t
compiledTrajectoryRevision3D(const CompiledTrajectory3D& trajectory) noexcept;

// Covers physical route and passage decorators independently of the exact
// initial dynamics. This is used only for continuity certification.
[[nodiscard]] std::uint64_t
compiledTrajectoryPassageRevision3D(const CompiledTrajectory3D& trajectory) noexcept;

} // namespace drone_city_nav
