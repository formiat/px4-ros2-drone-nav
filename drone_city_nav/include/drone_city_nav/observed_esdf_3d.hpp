#pragma once

#include "drone_city_nav/esdf_grid_3d.hpp"
#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/known_obstacle_distance_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class ObservedEsdf3DBuildMode : std::uint8_t {
  kFull,
  kReused,
};

struct ObservedEsdf3DBuildStats {
  KnownObstacleDistance3DBuildStats distance_field{};
  std::size_t known_voxels{0U};
  std::size_t free_voxels{0U};
  std::size_t occupied_voxels{0U};
  std::size_t unknown_voxels{0U};
  std::size_t proprioceptive_free_voxels{0U};
  std::size_t launch_support_voxels{0U};
  std::size_t classified_voxels{0U};
  std::size_t recomputed_voxels{0U};
  std::size_t reused_voxels{0U};
  double classification_ms{0.0};
  ObservedEsdf3DBuildMode mode{ObservedEsdf3DBuildMode::kFull};
};

struct ObservedEsdf3D {
  EsdfGrid3D grid{};
  std::shared_ptr<const std::vector<float>> distances_m;
  std::shared_ptr<const KnownObstacleDistance3D> known_obstacle_distance;
  std::shared_ptr<const ObservedOccupancyGrid3D> local_occupancy;
  std::vector<GridIndex3D> classification_override_cells;
  std::uint64_t occupancy_fingerprint{0U};
  double maximum_distance_m{0.0};
  ObservedEsdf3DBuildStats stats{};
};

// The resident field a new build may reuse when the exact influencing source
// set is unchanged. Free/unknown relabeling never changes that identity.
struct PreviousObservedEsdf3D {
  EsdfGrid3D grid{};
  std::shared_ptr<const std::vector<float>> distances_m;
  std::shared_ptr<const KnownObstacleDistance3D> known_obstacle_distance;
  std::uint64_t occupancy_fingerprint{0U};
  double maximum_distance_m{0.0};
};

struct ObservedEsdfCoverage3D {
  RawMapVersion source_raw_version{};
  RawMapVersion parent_raw_version{};
  std::uint64_t raw_local_fingerprint{0U};
  std::uint64_t esdf_fingerprint{0U};
  std::uint64_t parent_esdf_fingerprint{0U};
  std::size_t total_voxels{0U};
  std::size_t recomputed_voxels{0U};
  std::size_t reused_voxels{0U};
  double maximum_distance_m{0.0};
  ObservedEsdf3DBuildMode mode{ObservedEsdf3DBuildMode::kFull};

  [[nodiscard]] bool coherent() const noexcept;
};

struct ObservedEsdfResource3D {
  std::shared_ptr<const ObservedOccupancyGrid3D> local_occupancy;
  std::shared_ptr<const KnownObstacleDistance3D> known_obstacle_distance;
  std::shared_ptr<const std::vector<GridIndex3D>> classification_override_cells;
  ObservedEsdfCoverage3D coverage{};
};

struct ObservedEsdf3DRuntimeCounters {
  std::atomic<std::uint64_t> full_builds{0U};
  std::atomic<std::uint64_t> reused_builds{0U};
  std::atomic<std::uint64_t> recomputed_voxels{0U};
  std::atomic<std::uint64_t> reused_voxels{0U};
};

struct LocalObservedEsdfWindow3D {
  double horizontal_half_extent_m{20.0};
  double vertical_half_extent_m{15.0};
  double horizontal_recenter_margin_m{12.0};
  double vertical_recenter_margin_m{9.0};
};

// Selects the local window around the vehicle. The window is aligned to the
// occupancy chunk lattice so its classified copy is a chunk copy rather than a
// per-voxel walk; it may therefore extend the requested extent by less than one
// chunk per side, clamped to the world.
[[nodiscard]] GridBounds3D
selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds, const Point3& position,
                              const LocalObservedEsdfWindow3D& window);

[[nodiscard]] bool
localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                               const GridBounds3D& world_bounds, const Point3& position,
                               const LocalObservedEsdfWindow3D& window) noexcept;

[[nodiscard]] bool
localObservedEsdfWindow3DIsValid(const LocalObservedEsdfWindow3D& window) noexcept;

// Identifies only confirmed occupied geometry inside the supplied distance region.
// Free and unknown labels are deliberately absent from this identity: they are
// equivalent inputs to KnownObstacleDistance3D.
[[nodiscard]] std::uint64_t
knownObstacleFingerprint3D(const ObservedOccupancyGrid3D& occupancy,
                           const GridBounds3D& local_bounds);

[[nodiscard]] ObservedEsdf3D
buildObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                    const GridBounds3D& local_bounds, double maximum_distance_m,
                    BoundedWorkerPool* worker_pool = nullptr,
                    const LaunchSupportContact3D* launch_support_contact = nullptr);

// Rebuilds the exact field, or reuses the previous one when its influencing
// source identity is unchanged and no full reset is requested. Classification
// of the local window is always refreshed; it is a chunk copy.
[[nodiscard]] ObservedEsdf3D
updateObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                     const GridBounds3D& local_bounds, double maximum_distance_m,
                     const PreviousObservedEsdf3D* previous, bool full_reset,
                     BoundedWorkerPool* worker_pool = nullptr,
                     const LaunchSupportContact3D* launch_support_contact = nullptr);

[[nodiscard]] const char*
observedEsdf3DBuildModeName(ObservedEsdf3DBuildMode mode) noexcept;

} // namespace drone_city_nav
