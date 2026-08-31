#pragma once

#include "drone_city_nav/route_decorations_3d.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {

enum class RouteDecorationFailureReason3D : std::uint8_t {
  kNotAttempted,
  kValid,
  kMissingTrajectory,
  kInvalidTrajectory,
  kInvalidRouteGeneration,
  kInvalidPassageResources,
  kDerivedResourceMismatch,
};

struct RouteDecorationValidation3D {
  RouteDecorationFailureReason3D reason{RouteDecorationFailureReason3D::kNotAttempted};

  [[nodiscard]] bool valid() const noexcept {
    return reason == RouteDecorationFailureReason3D::kValid;
  }
};

[[nodiscard]] const char*
routeDecorationFailureReason3DName(RouteDecorationFailureReason3D reason) noexcept;

struct RouteDecorationCompilerInput3D {
  std::shared_ptr<const CompiledTrajectory3D> trajectory;
  std::uint64_t route_generation{0U};
  std::vector<PassageVolume> passage_volumes;
  std::vector<CooperativePassageAssignment> cooperative_passage_assignments;
  std::vector<PassageTraversalId> selected_passage_traversal_ids;
  PassageVolumeConfig passage_volume_config{};
};

struct RouteDecorationCompilationResult3D {
  std::shared_ptr<const RouteDecorations3D> decorations;
  RouteDecorationValidation3D validation{};

  [[nodiscard]] bool compiled() const noexcept {
    return decorations != nullptr && validation.valid();
  }
};

// The only constructor for immutable route decorations. Compilation validates
// every cross-resource relation before binding the result to one trajectory.
class RouteDecorationCompiler3D final {
public:
  [[nodiscard]] static RouteDecorationCompilationResult3D
  compile(RouteDecorationCompilerInput3D input);
};

} // namespace drone_city_nav
