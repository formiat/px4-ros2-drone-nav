#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_lattice_level_zero.hpp"

// Edge cost cache maintenance and statistics of the lattice: forgetting edge
// costs when the world changes beside them, traversability of paths against
// the cached edges, and the counters the telemetry reports.

namespace drone_city_nav::detail {

void PlannerLattice3D::resetEdgeStatistics() noexcept {
  clearances_rederived_ = 0U;
  edge_queries_ = 0U;
  raw_edge_validation_checks_ = 0U;
  adaptive_edge_queries_ = 0U;
  maximum_queried_level_ = 0U;
}

const GridBounds3D& PlannerLattice3D::bounds() const noexcept {
  return raw_bounds_;
}

bool PlannerLattice3D::edgeTraversable(const PersistentPlannerNode3D first,
                                       const PersistentPlannerNode3D second) {
  return std::isfinite(rawEdgeCost(first, second));
}

bool PlannerLattice3D::pathTraversable(const std::vector<Point3>& path) const {
  return !firstInvalidSegment(path).has_value();
}

std::optional<std::size_t>
PlannerLattice3D::firstInvalidSegment(const std::vector<Point3>& path) const {
  if (path.size() < 2U || !std::ranges::all_of(path, [&](const Point3& point) {
        return pointInsideFlightEnvelope(point);
      })) {
    return 0U;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const bool segment_valid =
        index == 1U ? departureSegmentValid(path[index - 1U], path[index])
                    : rawSegmentValid(path[index - 1U], path[index]);
    if (!segment_valid) {
      return index;
    }
  }
  return std::nullopt;
}

std::size_t PlannerLattice3D::nodeSpan() const noexcept {
  return static_cast<std::size_t>(width_) + static_cast<std::size_t>(height_) +
         static_cast<std::size_t>(depth_);
}

bool PlannerLattice3D::forgetEdgeCost(const PersistentPlannerEdge3D& edge) {
  if (!nodeInside(edge.first) || !nodeInside(edge.second) ||
      edge.first == edge.second) {
    return false;
  }
  if (level(edge.first, edge.second) == 0U) {
    const PersistentPlannerEdge3D canonical = canonicalEdge(edge.first, edge.second);
    const LevelZeroSlot slot = levelZeroSlot(canonical);
    if (levelZeroState(slot) == kLevelZeroEdgeUnknown) {
      return false;
    }
    setLevelZeroState(slot, kLevelZeroEdgeUnknown);
    forgetRefinedWaypoint(canonical);
    return true;
  }
  return adaptive_edge_cost_cache_.erase(edge) != 0U;
}

bool PlannerLattice3D::forgetEdgeCostForChange(const PersistentPlannerEdge3D& edge,
                                               const bool occupied_cell_added,
                                               const bool occupied_cell_removed) {
  if (!nodeInside(edge.first) || !nodeInside(edge.second) ||
      edge.first == edge.second || (!occupied_cell_added && !occupied_cell_removed)) {
    return false;
  }
  if (level(edge.first, edge.second) == 0U) {
    const LevelZeroSlot slot = levelZeroSlot(canonicalEdge(edge.first, edge.second));
    const unsigned state = levelZeroState(slot);
    const bool movable =
        ((state == kLevelZeroEdgeClear || state == kLevelZeroEdgeRefined) &&
         occupied_cell_added) ||
        (state == kLevelZeroEdgeBlocked && occupied_cell_removed);
    if (!movable) {
      return false;
    }
    setLevelZeroState(slot, kLevelZeroEdgeUnknown);
    if (state == kLevelZeroEdgeRefined) {
      forgetRefinedWaypoint(canonicalEdge(edge.first, edge.second));
    }
    return true;
  }
  const auto cost = adaptive_edge_cost_cache_.find(edge);
  if (cost == adaptive_edge_cost_cache_.end()) {
    return false;
  }
  const bool clear = std::isfinite(cost->second);
  if (!((clear && occupied_cell_added) || (!clear && occupied_cell_removed))) {
    return false;
  }
  adaptive_edge_cost_cache_.erase(cost);
  unindexAdaptiveEdge(edge);
  return true;
}

bool PlannerLattice3D::forgetNodeEdgesForChange(const PersistentPlannerNode3D node,
                                                const bool occupied_cell_added,
                                                const bool occupied_cell_removed) {
  if (!nodeInside(node) || (!occupied_cell_added && !occupied_cell_removed)) {
    return false;
  }
  bool forgotten = false;
  for (int z_offset = -1; z_offset <= 1; ++z_offset) {
    for (int y_offset = -1; y_offset <= 1; ++y_offset) {
      for (int x_offset = -1; x_offset <= 1; ++x_offset) {
        if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
          continue;
        }
        const PersistentPlannerNode3D neighbor{node.x + x_offset, node.y + y_offset,
                                               node.z + z_offset};
        if (!nodeInside(neighbor)) {
          continue;
        }
        const PersistentPlannerEdge3D edge = canonicalEdge(node, neighbor);
        const LevelZeroSlot slot = levelZeroSlot(edge);
        const unsigned state = levelZeroState(slot);
        const bool movable =
            ((state == kLevelZeroEdgeClear || state == kLevelZeroEdgeRefined) &&
             occupied_cell_added) ||
            (state == kLevelZeroEdgeBlocked && occupied_cell_removed);
        if (movable) {
          setLevelZeroState(slot, kLevelZeroEdgeUnknown);
          if (state == kLevelZeroEdgeRefined) {
            forgetRefinedWaypoint(edge);
          }
          forgotten = true;
        }
      }
    }
  }
  return forgotten;
}

bool PlannerLattice3D::hasEdgeCost(const PersistentPlannerEdge3D& edge) const noexcept {
  if (!nodeInside(edge.first) || !nodeInside(edge.second) ||
      edge.first == edge.second) {
    return false;
  }
  if (level(edge.first, edge.second) == 0U) {
    return levelZeroState(levelZeroSlot(canonicalEdge(edge.first, edge.second))) !=
           kLevelZeroEdgeUnknown;
  }
  return adaptive_edge_cost_cache_.contains(edge);
}

bool PlannerLattice3D::configured() const noexcept {
  return width_ > 0 && height_ > 0 && depth_ > 0;
}

std::size_t PlannerLattice3D::edgeQueries() const noexcept {
  return edge_queries_;
}

std::size_t PlannerLattice3D::rawEdgeValidationChecks() const noexcept {
  return raw_edge_validation_checks_;
}

std::size_t PlannerLattice3D::adaptiveEdgeQueries() const noexcept {
  return adaptive_edge_queries_;
}

std::size_t PlannerLattice3D::maximumQueriedLevel() const noexcept {
  return maximum_queried_level_;
}

} // namespace drone_city_nav::detail
