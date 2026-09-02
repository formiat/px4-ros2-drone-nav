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
  kReused,
};

struct KnownObstacleDistance3DBuildStats {
  std::size_t source_voxels{0U};
  std::size_t transform_voxels{0U};
  std::size_t finite_distance_voxels{0U};
  double source_collection_ms{0.0};
  double transform_ms{0.0};
  double duration_ms{0.0};
};

// Immutable dense distance-to-confirmed-occupied evidence over one local output
// window. It is an exact Euclidean distance transform, capped at
// maximumDistanceM(): sources inside the rectangular halo sourceBounds() are
// transformed with three separable passes, and only the output window is
// stored. Infinite cells mean that no confirmed obstacle exists within the cap;
// they are neutral rather than unknown or forbidden. Free and unknown cells are
// equivalent inputs.
class KnownObstacleDistance3D final {
public:
  class ConstructionKey final {
  private:
    ConstructionKey() = default;
    friend class KnownObstacleDistance3D;
  };

  struct Storage final {
    GridBounds3D output_bounds{};
    GridBounds3D source_bounds{};
    double maximum_distance_m{0.0};
    std::uint64_t source_fingerprint{0U};
    std::size_t source_voxels{0U};
    std::size_t finite_distance_voxels{0U};
    std::shared_ptr<const std::vector<float>> distances_m;
  };

  KnownObstacleDistance3D(ConstructionKey construction_key, Storage storage);

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] const GridBounds3D& sourceBounds() const noexcept;
  [[nodiscard]] double maximumDistanceM() const noexcept;
  // Identity of the exact influencing occupied sources and the window geometry.
  [[nodiscard]] std::uint64_t sourceFingerprint() const noexcept;
  [[nodiscard]] std::size_t sourceVoxelCount() const noexcept;
  [[nodiscard]] std::size_t finiteDistanceVoxelCount() const noexcept;
  [[nodiscard]] float distanceAt(GridIndex3D local_cell) const noexcept;
  // The dense output projection in z-major order. It is shared, not copied.
  [[nodiscard]] const std::shared_ptr<const std::vector<float>>&
  denseDistances() const noexcept;

  // Only the build functions in this component can produce a Storage whose
  // distances are the exact transform of its fingerprinted sources.
  [[nodiscard]] static std::shared_ptr<const KnownObstacleDistance3D>
  create(Storage storage);

private:
  Storage storage_{};
};

struct KnownObstacleDistance3DBuildResult {
  std::shared_ptr<const KnownObstacleDistance3D> field;
  KnownObstacleDistance3DBuildStats stats{};
  KnownObstacleDistance3DBuildMode mode{KnownObstacleDistance3DBuildMode::kFull};
};

// The rectangular halo of world cells whose occupied evidence can influence
// any output cell of the local window under the distance cap.
[[nodiscard]] GridBounds3D
knownObstacleDistanceSourceBounds3D(const GridBounds3D& world_bounds,
                                    const GridBounds3D& local_bounds,
                                    double maximum_distance_m);

// Exact full transform. Suppressed source cells are treated as free.
[[nodiscard]] KnownObstacleDistance3DBuildResult
buildKnownObstacleDistance3D(const ObservedOccupancyGrid3D& occupancy,
                             const GridBounds3D& local_bounds,
                             double maximum_distance_m,
                             std::span<const GridIndex3D> suppressed_source_cells = {},
                             BoundedWorkerPool* worker_pool = nullptr);

// Returns the previous field unchanged when its exact source identity still
// matches the occupancy, otherwise runs the full transform.
[[nodiscard]] KnownObstacleDistance3DBuildResult updateKnownObstacleDistance3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    double maximum_distance_m,
    const std::shared_ptr<const KnownObstacleDistance3D>& previous,
    std::span<const GridIndex3D> suppressed_source_cells = {},
    BoundedWorkerPool* worker_pool = nullptr);

[[nodiscard]] const char*
knownObstacleDistance3DBuildModeName(KnownObstacleDistance3DBuildMode mode) noexcept;

} // namespace drone_city_nav
