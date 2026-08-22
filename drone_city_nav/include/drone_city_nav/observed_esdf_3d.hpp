#pragma once

#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace drone_city_nav {

struct ObservedEsdf3DBuildStats {
  DistanceField3DBuildStats distance_field{};
  std::size_t known_voxels{0U};
  std::size_t free_voxels{0U};
  std::size_t occupied_voxels{0U};
  std::size_t unknown_voxels{0U};
  std::size_t proprioceptive_free_voxels{0U};
  std::size_t launch_support_voxels{0U};
  double classification_ms{0.0};
};

struct ObservedEsdf3D {
  mppi::EsdfGrid grid{};
  std::vector<float> distances_m;
  std::shared_ptr<const ObservedOccupancyGrid3D> local_occupancy;
  std::uint64_t occupancy_fingerprint{0U};
  ObservedEsdf3DBuildStats stats{};
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
                    const ProprioceptiveFreeSpaceSeed3D* free_space_seed = nullptr,
                    const LaunchSupportContact3D* launch_support_contact = nullptr);

} // namespace drone_city_nav
