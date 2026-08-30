#pragma once

#include "drone_city_nav/portal_graph.hpp"

#include <span>
#include <vector>

namespace drone_city_nav {

struct PassageTraversalSelectionConfig3D {
  double maximum_endpoint_distance_m{1.0};
  double maximum_centerline_distance_m{1.0};
  double minimum_direction_alignment{0.5};
  double minimum_traversal_length_m{0.5};
};

[[nodiscard]] bool passageTraversalSelectionConfig3DValid(
    const PassageTraversalSelectionConfig3D& config) noexcept;

// Associates optional topology decorations with the exact spatial route. A
// topology miss never invalidates base route geometry. Ambiguous overlapping
// matches are resolved deterministically to one best geometric association so
// downstream constrained spans remain ordered and non-overlapping.
[[nodiscard]] std::vector<SelectedPassageTraversal> selectRoutePassageTraversals3D(
    std::span<const RouteSample3D> route,
    std::span<const PassageTraversalEdge> topology_traversals,
    const PassageTraversalSelectionConfig3D& config = {});

} // namespace drone_city_nav
