#pragma once

#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
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
  kIncremental,
  kReused,
};

using ObservedEsdfDirtyRegion3D = mppi::EsdfDirtyRegion;

struct ObservedEsdf3DBuildStats {
  DistanceField3DBuildStats distance_field{};
  std::size_t known_voxels{0U};
  std::size_t free_voxels{0U};
  std::size_t occupied_voxels{0U};
  std::size_t unknown_voxels{0U};
  std::size_t proprioceptive_free_voxels{0U};
  std::size_t launch_support_voxels{0U};
  std::size_t classified_voxels{0U};
  std::size_t reused_classification_voxels{0U};
  std::size_t changed_voxels{0U};
  std::size_t recomputed_voxels{0U};
  std::size_t reused_voxels{0U};
  std::size_t dependency_invalidated_voxels{0U};
  std::size_t lowered_voxels{0U};
  std::size_t dirty_chunks{0U};
  double classification_ms{0.0};
  ObservedEsdf3DBuildMode mode{ObservedEsdf3DBuildMode::kFull};
  bool incremental_fallback{false};
};

struct ObservedEsdf3D {
  mppi::EsdfGrid grid{};
  std::vector<float> distances_m;
  std::vector<std::size_t> nearest_obstacle_indices;
  std::shared_ptr<const ObservedOccupancyGrid3D> local_occupancy;
  std::vector<GridIndex3D> classification_override_cells;
  std::vector<ObservedEsdfDirtyRegion3D> dirty_regions;
  std::uint64_t occupancy_fingerprint{0U};
  double maximum_distance_m{0.0};
  ObservedEsdf3DBuildStats stats{};
};

struct PreviousObservedEsdf3D {
  mppi::EsdfGrid grid{};
  std::span<const float> distances_m;
  std::span<const std::size_t> nearest_obstacle_indices;
  std::shared_ptr<const ObservedOccupancyGrid3D> source_occupancy;
  std::shared_ptr<const ObservedOccupancyGrid3D> local_occupancy;
  std::span<const GridIndex3D> classification_override_cells;
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
  std::shared_ptr<const std::vector<std::size_t>> nearest_obstacle_indices;
  std::shared_ptr<const std::vector<GridIndex3D>> classification_override_cells;
  ObservedEsdfCoverage3D coverage{};
};

struct ObservedEsdf3DRuntimeCounters {
  std::atomic<std::uint64_t> full_builds{0U};
  std::atomic<std::uint64_t> incremental_builds{0U};
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

[[nodiscard]] GridBounds3D
selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds, const Point3& position,
                              const LocalObservedEsdfWindow3D& window);

[[nodiscard]] bool
localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                               const GridBounds3D& world_bounds, const Point3& position,
                               const LocalObservedEsdfWindow3D& window) noexcept;

[[nodiscard]] bool
localObservedEsdfWindow3DIsValid(const LocalObservedEsdfWindow3D& window) noexcept;

[[nodiscard]] std::uint64_t
observedOccupancyFingerprint(const ObservedOccupancyGrid3D& occupancy,
                             const GridBounds3D& local_bounds);

[[nodiscard]] std::optional<LaunchSupportContact3D>
detectLaunchSupportContact3D(const ObservedOccupancyGrid3D& occupancy,
                             const ProprioceptiveFreeSpaceSeed3D& seed);

[[nodiscard]] LaunchSupportContact3D
makeVehicleLandedSupportContact3D(const GridBounds3D& bounds,
                                  const ProprioceptiveFreeSpaceSeed3D& seed);

struct LaunchSupportDeparture3D {
  Point3 target{};
  SweptFootprintResult validation{};
  double axial_departure_m{0.0};
  bool executable{false};
};

[[nodiscard]] LaunchSupportDeparture3D planLaunchSupportDeparture3D(
    const ObservedOccupancyGrid3D& occupancy, const Point3& current_position,
    const LaunchSupportContact3D& contact, double minimum_departure_m);

[[nodiscard]] ObservedEsdf3D
buildObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                    const GridBounds3D& local_bounds, double maximum_distance_m,
                    BoundedWorkerPool* worker_pool = nullptr,
                    const LaunchSupportContact3D* launch_support_contact = nullptr);

[[nodiscard]] ObservedEsdf3D updateObservedEsdf3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    double maximum_distance_m, const PreviousObservedEsdf3D* previous,
    std::span<const OccupancyChunkIndex3D> dirty_chunks, bool full_reset,
    double maximum_rebuild_ratio, BoundedWorkerPool* worker_pool = nullptr,
    const LaunchSupportContact3D* launch_support_contact = nullptr);

[[nodiscard]] double
requiredObservedEsdfMaximumDistanceM(double preferred_distance_m,
                                     const SweptFootprintConfig& footprint,
                                     double resolution_m) noexcept;

[[nodiscard]] bool observedEsdfFullAuditDue(std::uint64_t completed_builds,
                                            std::size_t audit_interval_builds) noexcept;

[[nodiscard]] const char*
observedEsdf3DBuildModeName(ObservedEsdf3DBuildMode mode) noexcept;

} // namespace drone_city_nav
