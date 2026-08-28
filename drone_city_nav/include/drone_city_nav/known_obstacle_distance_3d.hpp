#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace drone_city_nav {

class BoundedWorkerPool;

enum class KnownObstacleDistance3DBuildMode : std::uint8_t {
  kFull,
  kIncremental,
  kReused,
};

struct KnownObstacleDistanceRegion3D {
  int minimum_x{0};
  int minimum_y{0};
  int minimum_z{0};
  int maximum_x_exclusive{0};
  int maximum_y_exclusive{0};
  int maximum_z_exclusive{0};
};

struct KnownObstacleDistance3DBuildStats {
  std::size_t source_voxels{0U};
  std::size_t source_chunks{0U};
  std::size_t stored_distance_chunks{0U};
  std::size_t finite_distance_voxels{0U};
  std::size_t inserted_sources{0U};
  std::size_t removed_sources{0U};
  std::size_t recomputed_chunks{0U};
  std::size_t reused_chunks{0U};
  std::size_t changed_chunks{0U};
  std::size_t queried_voxels{0U};
  double source_index_ms{0.0};
  double distance_query_ms{0.0};
  double duration_ms{0.0};
};

namespace detail {
struct KnownObstacleDistanceStorage3D;
}

struct KnownObstacleDistance3DBuildResult;

// Immutable sparse distance-to-confirmed-occupied evidence backed by shared
// 8-cubed chunks. sourceBounds() is the conservative rectangular influence halo;
// only sources within the exact capped radius of an output cell are indexed.
// Missing chunks and cells mean that no confirmed obstacle exists within
// maximumDistanceM(); they are neutral rather than unknown or forbidden.
class KnownObstacleDistance3D final {
public:
  static constexpr int kChunkSize{8};

  class ConstructionKey final {
  private:
    ConstructionKey() = default;
    friend class KnownObstacleDistance3D;
  };

  KnownObstacleDistance3D(
      ConstructionKey construction_key,
      std::shared_ptr<const detail::KnownObstacleDistanceStorage3D> storage);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] const GridBounds3D& sourceBounds() const noexcept;
  [[nodiscard]] double maximumDistanceM() const noexcept;
  [[nodiscard]] std::uint64_t sourceFingerprint() const noexcept;
  [[nodiscard]] std::size_t sourceVoxelCount() const noexcept;
  [[nodiscard]] std::size_t sourceChunkCount() const noexcept;
  [[nodiscard]] std::size_t storedDistanceChunkCount() const noexcept;
  [[nodiscard]] std::size_t finiteDistanceVoxelCount() const noexcept;
  [[nodiscard]] float distanceAt(GridIndex3D local_cell) const noexcept;
  [[nodiscard]] std::shared_ptr<const std::vector<float>> materializeDense() const;

private:
  [[nodiscard]] static std::shared_ptr<const KnownObstacleDistance3D>
  create(std::shared_ptr<const detail::KnownObstacleDistanceStorage3D> storage);

  std::shared_ptr<const detail::KnownObstacleDistanceStorage3D> storage_;

  friend struct KnownObstacleDistance3DBuildResult;
  friend KnownObstacleDistance3DBuildResult
  buildKnownObstacleDistance3D(const ObservedOccupancyGrid3D&, const GridBounds3D&,
                               double, std::span<const GridIndex3D>,
                               BoundedWorkerPool*);
  friend KnownObstacleDistance3DBuildResult updateKnownObstacleDistance3D(
      const ObservedOccupancyGrid3D&, const GridBounds3D&, double,
      const std::shared_ptr<const KnownObstacleDistance3D>&,
      const ObservedOccupancyGrid3D*, std::span<const OccupancyChunkIndex3D>, bool,
      double, std::span<const GridIndex3D>, BoundedWorkerPool*);
};

struct KnownObstacleDistance3DBuildResult {
  std::shared_ptr<const KnownObstacleDistance3D> field;
  std::vector<KnownObstacleDistanceRegion3D> dirty_regions;
  KnownObstacleDistance3DBuildStats stats{};
  KnownObstacleDistance3DBuildMode mode{KnownObstacleDistance3DBuildMode::kFull};
  bool incremental_fallback{false};
};

[[nodiscard]] GridBounds3D
knownObstacleDistanceSourceBounds3D(const GridBounds3D& world_bounds,
                                    const GridBounds3D& local_bounds,
                                    double maximum_distance_m);

[[nodiscard]] KnownObstacleDistance3DBuildResult
buildKnownObstacleDistance3D(const ObservedOccupancyGrid3D& occupancy,
                             const GridBounds3D& local_bounds,
                             double maximum_distance_m,
                             std::span<const GridIndex3D> suppressed_source_cells = {},
                             BoundedWorkerPool* worker_pool = nullptr);

[[nodiscard]] KnownObstacleDistance3DBuildResult updateKnownObstacleDistance3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    double maximum_distance_m,
    const std::shared_ptr<const KnownObstacleDistance3D>& previous,
    const ObservedOccupancyGrid3D* previous_source_occupancy,
    std::span<const OccupancyChunkIndex3D> dirty_chunks, bool full_reset,
    double maximum_rebuild_ratio,
    std::span<const GridIndex3D> suppressed_source_cells = {},
    BoundedWorkerPool* worker_pool = nullptr);

[[nodiscard]] const char*
knownObstacleDistance3DBuildModeName(KnownObstacleDistance3DBuildMode mode) noexcept;

} // namespace drone_city_nav
