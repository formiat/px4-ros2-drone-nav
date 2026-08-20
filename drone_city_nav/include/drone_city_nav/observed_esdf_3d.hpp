#pragma once

#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {

struct ObservedEsdf3DBuildStats {
  DistanceField3DBuildStats distance_field{};
  std::size_t known_voxels{0U};
  std::size_t free_voxels{0U};
  std::size_t occupied_voxels{0U};
  std::size_t unknown_voxels{0U};
  double classification_ms{0.0};
};

struct ObservedEsdf3D {
  mppi::EsdfGrid grid{};
  std::vector<float> distances_m;
  std::shared_ptr<const ObservedOccupancyGrid3D> local_occupancy;
  std::uint64_t occupancy_fingerprint{0U};
  ObservedEsdf3DBuildStats stats{};
};

[[nodiscard]] GridBounds3D
selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds, const Point3& position,
                              double half_extent_m);

[[nodiscard]] bool localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                                                  const GridBounds3D& world_bounds,
                                                  const Point3& position,
                                                  double recenter_margin_m) noexcept;

[[nodiscard]] std::uint64_t
observedOccupancyFingerprint(const ObservedOccupancyGrid3D& occupancy,
                             const GridBounds3D& local_bounds);

[[nodiscard]] ObservedEsdf3D
buildObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                    const GridBounds3D& local_bounds, double maximum_distance_m,
                    BoundedWorkerPool* worker_pool = nullptr);

} // namespace drone_city_nav
