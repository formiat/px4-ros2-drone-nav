#pragma once

#include "drone_city_nav/known_obstacle_distance_3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace drone_city_nav::detail {

constexpr std::uint64_t kNoKnownObstacleSource3D{
    std::numeric_limits<std::uint64_t>::max()};
constexpr std::size_t kKnownObstacleChunkVoxelCount3D{
    static_cast<std::size_t>(KnownObstacleDistance3D::kChunkSize) *
    static_cast<std::size_t>(KnownObstacleDistance3D::kChunkSize) *
    static_cast<std::size_t>(KnownObstacleDistance3D::kChunkSize)};

struct KnownObstacleSourcePoint3D {
  GridIndex3D cell{};
  std::uint64_t key{0U};

  [[nodiscard]] bool
  operator==(const KnownObstacleSourcePoint3D&) const noexcept = default;
};

using KnownObstacleSourceChunk3D = std::vector<KnownObstacleSourcePoint3D>;

struct KnownObstacleDistanceChunk3D {
  std::array<float, kKnownObstacleChunkVoxelCount3D> distances_m{};
  std::size_t finite_voxels{0U};

  [[nodiscard]] bool
  operator==(const KnownObstacleDistanceChunk3D&) const noexcept = default;
};

using KnownObstacleSourceChunkMap3D =
    std::unordered_map<OccupancyChunkIndex3D,
                       std::shared_ptr<const KnownObstacleSourceChunk3D>,
                       OccupancyChunkIndex3DHash>;
using KnownObstacleDistanceChunkMap3D =
    std::unordered_map<OccupancyChunkIndex3D,
                       std::shared_ptr<const KnownObstacleDistanceChunk3D>,
                       OccupancyChunkIndex3DHash>;

struct KnownObstacleDistanceStorage3D {
  GridBounds3D world_bounds{};
  GridBounds3D output_bounds{};
  GridBounds3D source_bounds{};
  int output_offset_x{0};
  int output_offset_y{0};
  int output_offset_z{0};
  int source_minimum_x{0};
  int source_minimum_y{0};
  int source_minimum_z{0};
  int source_maximum_x_exclusive{0};
  int source_maximum_y_exclusive{0};
  int source_maximum_z_exclusive{0};
  int radius_cells{0};
  double maximum_distance_m{0.0};
  double maximum_squared_cells{0.0};
  KnownObstacleSourceChunkMap3D source_chunks;
  KnownObstacleDistanceChunkMap3D distance_chunks;
  std::vector<GridIndex3D> suppressed_source_cells;
  std::uint64_t source_fingerprint{0U};
  std::size_t source_voxels{0U};
  std::size_t finite_distance_voxels{0U};
};

struct KnownObstacleNearestSource3D {
  std::uint64_t key{kNoKnownObstacleSource3D};
  double squared_cells{std::numeric_limits<double>::infinity()};

  [[nodiscard]] bool found() const noexcept {
    return key != kNoKnownObstacleSource3D;
  }
};

class KnownObstacleSourceIndex3D final {
public:
  explicit KnownObstacleSourceIndex3D(
      const KnownObstacleSourceChunkMap3D& source_chunks);
  explicit KnownObstacleSourceIndex3D(
      std::span<const KnownObstacleSourcePoint3D> sources);

  [[nodiscard]] KnownObstacleNearestSource3D
  nearest(GridIndex3D global_cell, double maximum_squared_cells) const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;

private:
  struct Node {
    KnownObstacleSourcePoint3D source{};
    int left{-1};
    int right{-1};
    int axis{0};
  };

  int build(std::vector<KnownObstacleSourcePoint3D>& sources, std::size_t begin,
            std::size_t end, int depth);
  void nearestRecursive(int node_index, GridIndex3D cell,
                        KnownObstacleNearestSource3D& nearest) const noexcept;

  std::vector<Node> nodes_;
  int root_{-1};
};

[[nodiscard]] bool sameKnownObstacleBounds3D(const GridBounds3D& first,
                                             const GridBounds3D& second) noexcept;
[[nodiscard]] std::size_t knownObstacleVoxelCount3D(const GridBounds3D& bounds);
[[nodiscard]] std::size_t
knownObstacleLocalLinearIndex3D(const GridBounds3D& bounds,
                                GridIndex3D local_cell) noexcept;
[[nodiscard]] std::size_t knownObstacleChunkBitIndex3D(GridIndex3D local_cell) noexcept;
[[nodiscard]] bool
knownObstacleSourceCanInfluence3D(const KnownObstacleDistanceStorage3D& storage,
                                  GridIndex3D global_cell) noexcept;
[[nodiscard]] std::uint64_t knownObstacleSourceKey3D(const GridBounds3D& world_bounds,
                                                     GridIndex3D global_cell) noexcept;
[[nodiscard]] OccupancyChunkIndex3D
knownObstacleOutputChunk3D(GridIndex3D local_cell) noexcept;
[[nodiscard]] std::vector<OccupancyChunkIndex3D>
knownObstacleOutputChunksAffectedBySources3D(
    const KnownObstacleDistanceStorage3D& storage,
    std::span<const KnownObstacleSourcePoint3D> sources);
[[nodiscard]] KnownObstacleSourceChunk3D collectKnownObstacleSourceChunk3D(
    const ObservedOccupancyGrid3D& occupancy,
    const KnownObstacleDistanceStorage3D& storage, OccupancyChunkIndex3D chunk_index,
    const std::unordered_set<std::uint64_t>& suppressed_keys);
[[nodiscard]] KnownObstacleSourceChunkMap3D collectKnownObstacleSourceChunks3D(
    const ObservedOccupancyGrid3D& occupancy,
    const KnownObstacleDistanceStorage3D& storage,
    const std::unordered_set<std::uint64_t>& suppressed_keys);
[[nodiscard]] std::vector<KnownObstacleSourcePoint3D>
flattenKnownObstacleSources3D(const KnownObstacleSourceChunkMap3D& source_chunks);
[[nodiscard]] std::uint64_t
knownObstacleSourceFingerprint3D(const KnownObstacleDistanceStorage3D& storage,
                                 const KnownObstacleSourceChunkMap3D& source_chunks);
[[nodiscard]] std::shared_ptr<const KnownObstacleDistanceChunk3D>
computeKnownObstacleDistanceChunk3D(const KnownObstacleDistanceStorage3D& storage,
                                    const KnownObstacleSourceIndex3D& source_index,
                                    OccupancyChunkIndex3D output_chunk);
[[nodiscard]] std::vector<std::shared_ptr<const KnownObstacleDistanceChunk3D>>
computeKnownObstacleDistanceChunks3D(
    const KnownObstacleDistanceStorage3D& storage,
    const KnownObstacleSourceIndex3D& source_index,
    std::span<const OccupancyChunkIndex3D> output_chunks,
    BoundedWorkerPool* worker_pool);
[[nodiscard]] KnownObstacleDistanceRegion3D
knownObstacleChunkRegion3D(const GridBounds3D& bounds,
                           OccupancyChunkIndex3D output_chunk) noexcept;
[[nodiscard]] std::size_t
knownObstacleRegionVoxelCount3D(const KnownObstacleDistanceRegion3D& region) noexcept;
[[nodiscard]] bool
knownObstacleRawChangesCovered3D(const ObservedOccupancyGrid3D& previous,
                                 const ObservedOccupancyGrid3D& current,
                                 const KnownObstacleDistanceStorage3D& storage,
                                 std::span<const OccupancyChunkIndex3D> dirty_chunks);
[[nodiscard]] std::unordered_set<std::uint64_t>
knownObstacleSuppressedKeys3D(const GridBounds3D& world_bounds,
                              std::span<const GridIndex3D> suppressed_cells);
[[nodiscard]] std::shared_ptr<KnownObstacleDistanceStorage3D>
makeKnownObstacleDistanceStorage3D(const ObservedOccupancyGrid3D& occupancy,
                                   const GridBounds3D& local_bounds,
                                   double maximum_distance_m,
                                   std::span<const GridIndex3D> suppressed_cells);

} // namespace drone_city_nav::detail
