#include "route_trajectory_compiler_3d.hpp"

#include <memory>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  return value.has_value() ? std::addressof(value.value()) : nullptr;
}

[[nodiscard]] bool
materializedResourcesAvailable(const MaterializedRoute3D& materialized) noexcept {
  return materialized.world != nullptr && materialized.route != nullptr &&
         materialized.constrained_spans != nullptr &&
         materialized.passage_volumes != nullptr &&
         materialized.cooperative_passage_assignments != nullptr &&
         materialized.selected_passage_traversal_ids != nullptr;
}

[[nodiscard]] std::optional<TrackingErrorTubeWorld3D> trackingWorld(
    const MaterializedRoute3D& materialized,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_raw_world) {
  if (materialized.world->observed_occupancy != nullptr) {
    if (observed_raw_world == nullptr || !observed_raw_world->valid()) {
      return std::nullopt;
    }
    return TrackingErrorTubeWorld3D{
        .observed_occupancy = &observed_raw_world->occupancy(),
        .occupied_content_fingerprint =
            observed_raw_world->occupiedContentFingerprint(),
        .launch_support_contact =
            optionalAddress(materialized.world->launch_support_contact),
        .proprioceptive_free_space_seed =
            optionalAddress(materialized.world->proprioceptive_free_space_seed),
    };
  }
  return TrackingErrorTubeWorld3D{
      .occupancy = materialized.world->static_occupancy.get(),
      .occupied_content_fingerprint =
          materialized.world->static_occupancy != nullptr
              ? materialized.world->static_occupancy->contentFingerprint()
              : 0U,
  };
}

} // namespace

RouteTrajectoryCompiler3D::RouteTrajectoryCompiler3D(
    const RouteTrajectoryCompilerConfig3D& config)
    : config_{config} {
}

RouteTrajectoryCompilationResult3D RouteTrajectoryCompiler3D::compile(
    const RouteTrajectoryCompilationRequest3D& request) const {
  using Failure = CompiledTrajectoryFailureReason3D;
  if (!materializedResourcesAvailable(request.materialized)) {
    return RouteTrajectoryCompilationResult3D{
        .trajectory = nullptr,
        .decorations = nullptr,
        .validation = {.reason = Failure::kMissingRoute},
        .decoration_validation = {},
        .stop_turn_count = 0U,
    };
  }
  const std::optional<TrackingErrorTubeWorld3D> tracking_world =
      trackingWorld(request.materialized, request.observed_raw_world);
  if (!tracking_world.has_value()) {
    return RouteTrajectoryCompilationResult3D{
        .trajectory = nullptr,
        .decorations = nullptr,
        .validation = {.reason = Failure::kInvalidTrackingErrorTube},
        .decoration_validation = {},
        .stop_turn_count = 0U,
    };
  }

  const MaterializedRoute3D& materialized = request.materialized;
  TrajectoryCompilationResult3D trajectory =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state = request.exact_initial_state,
          .route_generation = materialized.candidate_generation,
          .route = *materialized.route,
          .constrained_spans = *materialized.constrained_spans,
          .endpoint_semantics = request.endpoint_semantics,
          .materialized_route_fingerprint = materialized.fingerprint,
          .tracking_world = tracking_world.value(),
          .config = config_.trajectory,
      });
  RouteTrajectoryCompilationResult3D result{
      .trajectory = trajectory.trajectory,
      .decorations = nullptr,
      .validation = trajectory.validation,
      .decoration_validation = {},
      .stop_turn_count = trajectory.stop_turn_count,
  };
  if (!trajectory.compiled()) {
    return result;
  }
  RouteDecorationCompilationResult3D decorations =
      RouteDecorationCompiler3D::compile(RouteDecorationCompilerInput3D{
          .trajectory = trajectory.trajectory,
          .route_generation = materialized.candidate_generation,
          .passage_volumes = *materialized.passage_volumes,
          .cooperative_passage_assignments =
              *materialized.cooperative_passage_assignments,
          .selected_passage_traversal_ids =
              *materialized.selected_passage_traversal_ids,
          .passage_volume_config = config_.passage_volume,
      });
  result.decorations = std::move(decorations.decorations);
  result.decoration_validation = decorations.validation;
  return result;
}

} // namespace drone_city_nav
