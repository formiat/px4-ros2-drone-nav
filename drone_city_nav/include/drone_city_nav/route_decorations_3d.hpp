#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/cooperative_passage_route.hpp"
#include "drone_city_nav/passage_volume.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {

class RouteDecorationCompiler3D;

// Optional passage and cooperative metadata sealed independently from the
// controller-neutral trajectory. A decoration is immutable and is bound to one
// exact compiled geometry revision and route generation.
class RouteDecorations3D final {
public:
  RouteDecorations3D(const RouteDecorations3D&) = delete;
  RouteDecorations3D& operator=(const RouteDecorations3D&) = delete;
  RouteDecorations3D(RouteDecorations3D&&) = delete;
  RouteDecorations3D& operator=(RouteDecorations3D&&) = delete;
  ~RouteDecorations3D() = default;

  const std::uint64_t route_generation;
  const std::uint64_t geometry_revision;
  const std::uint64_t physical_route_fingerprint;
  const std::shared_ptr<const std::vector<PassageVolume>> passage_volumes;
  const std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      cooperative_passage_assignments;
  const std::shared_ptr<const std::vector<PassageTraversalId>>
      selected_passage_traversal_ids;
  const PassageVolumeConfig passage_volume_config;
  const std::uint64_t route_decorations_revision;

private:
  friend class RouteDecorationCompiler3D;

  RouteDecorations3D(
      std::uint64_t compiled_route_generation, std::uint64_t compiled_geometry_revision,
      std::uint64_t compiled_physical_route_fingerprint,
      std::shared_ptr<const std::vector<PassageVolume>> compiled_passage_volumes,
      std::shared_ptr<const std::vector<CooperativePassageAssignment>>
          compiled_cooperative_passage_assignments,
      std::shared_ptr<const std::vector<PassageTraversalId>>
          compiled_selected_passage_traversal_ids,
      PassageVolumeConfig compiled_passage_volume_config);
};

// Recomputes the immutable decoration revision. Returns zero for structurally
// invalid or incomplete resources.
[[nodiscard]] std::uint64_t
routeDecorationsRevision3D(const RouteDecorations3D& decorations) noexcept;

// Validates both the decoration graph and its binding to the exact compiled
// trajectory. When expected_route_generation is non-zero it is also enforced.
[[nodiscard]] bool
routeDecorationsValid3D(const RouteDecorations3D& decorations,
                        const CompiledTrajectory3D& trajectory,
                        std::uint64_t expected_route_generation = 0U) noexcept;

} // namespace drone_city_nav
