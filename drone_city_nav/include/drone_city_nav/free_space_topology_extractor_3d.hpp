#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/portal_graph.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace drone_city_nav {

class DistanceField3D;

struct FreeSpaceTopologyExtractorConfig {
  double maximum_clearance_m{6.0};
  double open_space_clearance_m{3.0};
  double speed_limit_mps{10.0};
  double medial_clearance_weight{2.0};
  double medial_ridge_prominence_m{0.1};
  std::size_t medial_band_radius_cells{1U};
  std::size_t chunk_size_cells{32U};
  std::size_t minimum_open_region_voxels{16U};
  std::size_t minimum_constrained_component_voxels{16U};
  std::size_t minimum_portal_voxels{4U};
  SweptFootprintConfig footprint{};
  std::optional<double> minimum_center_z_m;
  std::optional<double> maximum_center_z_m;
};

struct FreeSpaceTopologyExtractionStats {
  std::size_t processed_chunks{0U};
  std::size_t free_voxels{0U};
  std::size_t footprint_feasible_voxels{0U};
  std::size_t open_space_voxels{0U};
  std::size_t constrained_voxels{0U};
  std::size_t medial_ridge_voxels{0U};
  std::size_t medial_band_voxels{0U};
  std::size_t open_space_components{0U};
  std::size_t constrained_components{0U};
  std::size_t rejected_constrained_components{0U};
  std::size_t portal_patches{0U};
  std::size_t rejected_for_insufficient_portals{0U};
  std::size_t rejected_for_disconnected_medial_graph{0U};
  std::size_t raw_unsafe_segments{0U};
  std::size_t rejected_for_no_safe_segments{0U};
  double clearance_ms{0.0};
  double classification_ms{0.0};
  double component_labeling_ms{0.0};
  double topology_build_ms{0.0};
  double duration_ms{0.0};
};

struct ExtractedFreeSpaceTopology3D {
  std::vector<FreeSpaceRegion> regions;
  std::vector<PassagePortal> portals;
  std::vector<PassageSegment> segments;
  FreeSpaceTopologyExtractionStats stats{};
};

[[nodiscard]] bool freeSpaceTopologyExtractorConfigIsValid(
    const FreeSpaceTopologyExtractorConfig& config) noexcept;

[[nodiscard]] ExtractedFreeSpaceTopology3D
extractFreeSpaceTopology3D(const OccupancyGrid3D& occupancy,
                           const FreeSpaceTopologyExtractorConfig& config = {});

[[nodiscard]] ExtractedFreeSpaceTopology3D
extractFreeSpaceTopology3D(const OccupancyGrid3D& occupancy,
                           const DistanceField3D& clearance_field,
                           const FreeSpaceTopologyExtractorConfig& config = {});

} // namespace drone_city_nav
