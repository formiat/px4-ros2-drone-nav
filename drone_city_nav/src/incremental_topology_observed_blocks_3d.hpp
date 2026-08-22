#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav::incremental_topology_detail {

struct ObservedBlockChanges3D {
  std::vector<IncrementalTopologyBlockIndex3D> geometry;
  std::vector<IncrementalTopologyBlockIndex3D> observation_evidence;
};

class ObservedBlockLifecycle3D {
public:
  explicit ObservedBlockLifecycle3D(const IncrementalTopologyGraph3DConfig& config);

  [[nodiscard]] std::vector<IncrementalTopologyBlockIndex3D>
  allObservedBlocks(const ObservedOccupancyGrid3D& occupancy) const;
  [[nodiscard]] ObservedBlockChanges3D
  dirtyObservedBlocks(const ObservedOccupancyGrid3D& occupancy,
                      std::span<const OccupancyChunkIndex3D> dirty_chunks);
  [[nodiscard]] std::vector<OccupancyChunkIndex3D>
  completeSnapshotChunks(const ObservedOccupancyGrid3D& occupancy) const;
  [[nodiscard]] std::vector<IncrementalTopologyBlockIndex3D>
  allStaticBlocks(const GridBounds3D& bounds) const;

  void clearPending();
  void enqueue(std::span<const IncrementalTopologyBlockIndex3D> dirty_blocks);
  [[nodiscard]] std::vector<IncrementalTopologyBlockIndex3D>
  takePending(const GridBounds3D& bounds,
              const std::optional<IncrementalTopologyBuildPriority3D>& priority,
              bool preserve_oldest_work);
  [[nodiscard]] const std::unordered_set<IncrementalTopologyBlockIndex3D,
                                         IncrementalTopologyBlockIndex3DHash>&
  pending() const noexcept;
  [[nodiscard]] std::size_t pendingCount() const noexcept;

  void replaceObservedSnapshot(const ObservedOccupancyGrid3D::ChunkMap& chunks);
  void clearObservedSnapshot();

private:
  void addBlockRange(std::unordered_set<IncrementalTopologyBlockIndex3D,
                                        IncrementalTopologyBlockIndex3DHash>& output,
                     GridIndex3D minimum, GridIndex3D maximum) const;
  [[nodiscard]] static std::vector<IncrementalTopologyBlockIndex3D>
  sortedBlocks(const std::unordered_set<IncrementalTopologyBlockIndex3D,
                                        IncrementalTopologyBlockIndex3DHash>& blocks);

  IncrementalTopologyGraph3DConfig config_{};
  ObservedOccupancyGrid3D::ChunkMap observed_chunks_;
  std::unordered_set<IncrementalTopologyBlockIndex3D,
                     IncrementalTopologyBlockIndex3DHash>
      pending_blocks_;
  std::unordered_map<IncrementalTopologyBlockIndex3D, std::uint64_t,
                     IncrementalTopologyBlockIndex3DHash>
      pending_sequence_;
  std::uint64_t next_pending_sequence_{1U};
};

} // namespace drone_city_nav::incremental_topology_detail
