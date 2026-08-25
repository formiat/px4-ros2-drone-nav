#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <cmath>
#include <cstdint>
#include <functional>

#include "incremental_topology_graph_3d_internal.hpp"

namespace drone_city_nav {

std::size_t IncrementalTopologyNodeIdHash::operator()(
    const IncrementalTopologyNodeId id) const noexcept {
  return std::hash<std::uint64_t>{}(id.value);
}

std::size_t IncrementalTopologyEdgeIdHash::operator()(
    const IncrementalTopologyEdgeId id) const noexcept {
  return std::hash<std::uint64_t>{}(id.value);
}

std::size_t IncrementalTopologyBlockIndex3DHash::operator()(
    const IncrementalTopologyBlockIndex3D index) const noexcept {
  std::uint64_t hash{incremental_topology_detail::kFnvOffset};
  incremental_topology_detail::hashInteger(hash, index.x);
  incremental_topology_detail::hashInteger(hash, index.y);
  incremental_topology_detail::hashInteger(hash, index.z);
  return static_cast<std::size_t>(hash);
}

bool incrementalTopologyGraph3DConfigIsValid(
    const IncrementalTopologyGraph3DConfig& config) noexcept {
  return config.block_size_cells > 0 && config.coarse_sample_stride_cells > 0 &&
         config.refined_sample_stride_cells > 0 &&
         config.refined_sample_stride_cells <= config.coarse_sample_stride_cells &&
         config.coarse_sample_stride_cells <= config.block_size_cells &&
         config.coarse_sample_stride_cells % config.refined_sample_stride_cells == 0 &&
         config.maximum_observed_blocks_per_update > 0U &&
         config.maximum_backlog_blocks_per_update > 0U &&
         config.backlog_boost_threshold_blocks > 0U &&
         config.minimum_oldest_blocks_per_update <=
             config.maximum_observed_blocks_per_update &&
         std::isfinite(config.local_priority_radius_m) &&
         config.local_priority_radius_m > 0.0 &&
         std::isfinite(config.forward_corridor_radius_m) &&
         config.forward_corridor_radius_m > 0.0 &&
         std::isfinite(config.forward_corridor_lookahead_m) &&
         config.forward_corridor_lookahead_m > 0.0 &&
         std::isfinite(config.footprint.radius_m) &&
         std::isfinite(config.footprint.lower_extent_m) &&
         std::isfinite(config.footprint.upper_extent_m) &&
         std::isfinite(config.footprint.sweep_step_m) &&
         config.footprint.radius_m >= 0.0 && config.footprint.lower_extent_m >= 0.0 &&
         config.footprint.upper_extent_m >= 0.0 &&
         config.footprint.perimeter_samples > 0U &&
         config.footprint.radial_rings > 0U && config.footprint.axial_samples > 0U &&
         config.footprint.sweep_step_m > 0.0;
}

const char* incrementalTopologyTransitionKind3DName(
    const IncrementalTopologyTransitionKind3D kind) noexcept {
  switch (kind) {
    case IncrementalTopologyTransitionKind3D::kObservedFree:
      return "observed_free";
    case IncrementalTopologyTransitionKind3D::kOptimisticUnknown:
      return "optimistic_unknown";
  }
  return "unknown";
}

} // namespace drone_city_nav
