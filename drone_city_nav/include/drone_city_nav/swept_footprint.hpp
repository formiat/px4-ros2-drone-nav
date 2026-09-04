#pragma once

#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>

namespace drone_city_nav {

[[nodiscard]] bool proprioceptiveSeedAllowsSupportContact(
    const ProprioceptiveFreeSpaceSeed3D& seed, const Point3& box_minimum,
    const Point3& box_maximum, double occupancy_resolution_m) noexcept;

enum class SweptFootprintStatus {
  kValid,
  kInvalidInput,
  kRawCollision,
};

[[nodiscard]] const char*
sweptFootprintStatusName(SweptFootprintStatus status) noexcept;

struct SweptFootprintResult {
  SweptFootprintStatus status{SweptFootprintStatus::kInvalidInput};
  Point3 failure_point{};

  [[nodiscard]] bool accepted() const noexcept {
    return status == SweptFootprintStatus::kValid;
  }
};

[[nodiscard]] SweptFootprintResult
validateRawFootprintAt(const OccupancyGrid2D& occupancy, const Point3& position,
                       const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult
validateRawFootprintAt(const RawOccupancyGridView2D& occupancy, const Point3& position,
                       const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult
validateRawSweptFootprint(const OccupancyGrid2D& occupancy, const Point3& first,
                          const Point3& second,
                          const SweptFootprintConfig& config) noexcept;

// Contact exemption of a proprioceptive seed. The body demonstrably occupies
// its volume at the seed, so occupied evidence overlapping that volume (widened
// by the seed's contact tolerance) is contact rather than an obstacle. A
// candidate body position keeps a contact box or point suppressed while it
// stays at least the seed's own stand-off away from it, so holding, departing
// and moving along the touched surface stay valid while closing on the contact
// collides; evidence the body does not overlap at the seed is never exempt.
// The tolerance widens what counts as contact, never how far the body may
// close on it: a seed that renews every tick would otherwise ratchet the body
// deeper one tolerance at a time.
[[nodiscard]] bool proprioceptiveSeedExemptsBox(
    const ProprioceptiveFreeSpaceSeed3D& seed, const Point3& candidate_position,
    const Point3& box_minimum, const Point3& box_maximum) noexcept;

[[nodiscard]] bool
proprioceptiveSeedExemptsPoint(const ProprioceptiveFreeSpaceSeed3D& seed,
                               const Point3& candidate_position,
                               const Point3& obstacle_point) noexcept;

// The 3D validators accept two kinds of contact evidence: the anchored launch
// support and the proprioceptive seed. Either exempts the occupied evidence it
// covers; a sweep interval is exempt only when its begin, midpoint, and end
// body positions all are.
[[nodiscard]] SweptFootprintResult validateRawFootprintAt(
    const OccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawSweptFootprint(
    const OccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawFootprintAt(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawSweptFootprint(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] bool footprintIntersectsAxisAlignedBox(
    const Point3& position, const FootprintBodyAxis& body_axis,
    const SweptFootprintConfig& config, const Point3& box_minimum,
    const Point3& box_maximum) noexcept;

[[nodiscard]] SweptFootprintResult validateRawPointCloudFootprintAt(
    std::span<const Point3> obstacle_points, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawPointCloudSweptFootprint(
    std::span<const Point3> obstacle_points, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] FootprintBodyAxis
bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                              double gravity_mps2 = 9.80665) noexcept;

} // namespace drone_city_nav
