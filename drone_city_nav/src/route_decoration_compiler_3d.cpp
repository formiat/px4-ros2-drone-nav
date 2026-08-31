#include "drone_city_nav/route_decoration_compiler_3d.hpp"

#include <memory>
#include <utility>

namespace drone_city_nav {

const char* routeDecorationFailureReason3DName(
    const RouteDecorationFailureReason3D reason) noexcept {
  switch (reason) {
    case RouteDecorationFailureReason3D::kNotAttempted:
      return "not_attempted";
    case RouteDecorationFailureReason3D::kValid:
      return "valid";
    case RouteDecorationFailureReason3D::kMissingTrajectory:
      return "missing_trajectory";
    case RouteDecorationFailureReason3D::kInvalidTrajectory:
      return "invalid_trajectory";
    case RouteDecorationFailureReason3D::kInvalidRouteGeneration:
      return "invalid_route_generation";
    case RouteDecorationFailureReason3D::kInvalidPassageResources:
      return "invalid_passage_resources";
    case RouteDecorationFailureReason3D::kDerivedResourceMismatch:
      return "derived_resource_mismatch";
  }
  return "unknown";
}

RouteDecorationCompilationResult3D
RouteDecorationCompiler3D::compile(RouteDecorationCompilerInput3D input) {
  using Failure = RouteDecorationFailureReason3D;
  RouteDecorationCompilationResult3D result;
  if (input.trajectory == nullptr) {
    result.validation = {.reason = Failure::kMissingTrajectory};
    return result;
  }
  if (input.route_generation == 0U) {
    result.validation = {.reason = Failure::kInvalidRouteGeneration};
    return result;
  }
  if (!compiledTrajectoryResourcesValid3D(*input.trajectory, input.route_generation) ||
      input.trajectory->compiled_trajectory_revision == 0U ||
      input.trajectory->compiled_trajectory_revision !=
          compiledTrajectoryRevision3D(*input.trajectory)) {
    result.validation = {.reason = Failure::kInvalidTrajectory};
    return result;
  }

  const auto passage_volumes = std::make_shared<const std::vector<PassageVolume>>(
      std::move(input.passage_volumes));
  const auto cooperative_assignments =
      std::make_shared<const std::vector<CooperativePassageAssignment>>(
          std::move(input.cooperative_passage_assignments));
  const auto traversal_ids = std::make_shared<const std::vector<PassageTraversalId>>(
      std::move(input.selected_passage_traversal_ids));
  auto decorations = std::shared_ptr<const RouteDecorations3D>{new RouteDecorations3D(
      input.route_generation, input.trajectory->compiled_trajectory_revision,
      input.trajectory->physical_route_fingerprint, passage_volumes,
      cooperative_assignments, traversal_ids, input.passage_volume_config)};
  if (decorations->route_decorations_revision == 0U) {
    result.validation = {.reason = Failure::kDerivedResourceMismatch};
    return result;
  }
  if (!routeDecorationsValid3D(*decorations, *input.trajectory,
                               input.route_generation)) {
    result.validation = {.reason = Failure::kInvalidPassageResources};
    return result;
  }
  result.decorations = std::move(decorations);
  result.validation = {.reason = Failure::kValid};
  return result;
}

} // namespace drone_city_nav
