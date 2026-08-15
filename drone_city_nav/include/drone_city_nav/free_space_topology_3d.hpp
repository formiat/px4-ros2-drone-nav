#pragma once

#include "drone_city_nav/portal_graph.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

namespace drone_city_nav {

class OccupancyGrid3D;

class FreeSpaceTopology3D {
public:
  FreeSpaceTopology3D(std::uint64_t occupancy_fingerprint,
                      const GridBounds3D& occupancy_bounds,
                      std::vector<FreeSpaceRegion> regions,
                      std::vector<PassagePortal> portals,
                      std::vector<PassageTraversalEdge> traversal_edges);
  FreeSpaceTopology3D(std::uint64_t occupancy_fingerprint,
                      const GridBounds3D& occupancy_bounds,
                      std::vector<FreeSpaceRegion> regions,
                      std::vector<PassagePortal> portals,
                      std::vector<PassageSegment> segments,
                      std::vector<PassageTraversalEdge> legacy_traversal_edges = {});

  [[nodiscard]] static FreeSpaceTopology3D load(const std::filesystem::path& path);
  void write(const std::filesystem::path& path) const;

  [[nodiscard]] std::uint64_t occupancyFingerprint() const noexcept;
  [[nodiscard]] const GridBounds3D& occupancyBounds() const noexcept;
  [[nodiscard]] bool compatibleWith(const OccupancyGrid3D& occupancy) const noexcept;
  [[nodiscard]] const std::vector<FreeSpaceRegion>& regions() const noexcept;
  [[nodiscard]] const std::vector<PassagePortal>& portals() const noexcept;
  [[nodiscard]] const std::vector<PassageSegment>& segments() const noexcept;
  [[nodiscard]] const std::vector<PassageTraversalEdge>&
  traversalEdges() const noexcept;

private:
  void validate() const;

  std::uint64_t occupancy_fingerprint_{0U};
  GridBounds3D occupancy_bounds_{};
  std::vector<FreeSpaceRegion> regions_;
  std::vector<PassagePortal> portals_;
  std::vector<PassageSegment> segments_;
  std::vector<PassageTraversalEdge> traversal_edges_;
};

} // namespace drone_city_nav
