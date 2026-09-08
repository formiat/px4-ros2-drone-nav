#pragma once

#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/indexed_point_cloud_3d.hpp"
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
// contact box or point stays suppressed for every candidate pose of that
// validation, with no condition on where the body moves. A condition of that
// kind — no closer than now, only away, not along the surface — is a
// prohibition on moving through free space and does not belong in a collision
// contract. Evidence the body does not overlap at the seed is never exempt,
// and the tolerance widens what counts as contact and nothing else.
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

// The footprint reduced to the physical body: the validation radius drops to
// the body radius while the axial extents and sampling stay as configured.
// This is the volume contact evidence is judged against, and the volume a
// motion answers to when the clearance envelope has nowhere left to go.
[[nodiscard]] SweptFootprintConfig
physicalBodyFootprint(const SweptFootprintConfig& footprint) noexcept;

[[nodiscard]] bool footprintIntersectsAxisAlignedBox(
    const Point3& position, const FootprintBodyAxis& body_axis,
    const SweptFootprintConfig& config, const Point3& box_minimum,
    const Point3& box_maximum) noexcept;

[[nodiscard]] SweptFootprintResult validateRawPointCloudFootprintAt(
    const IndexedPointCloudView3D& obstacle_points, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawPointCloudSweptFootprint(
    const IndexedPointCloudView3D& obstacle_points, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_seed = nullptr) noexcept;

[[nodiscard]] FootprintBodyAxis
bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                              double gravity_mps2 = 9.80665) noexcept;

// The largest tilt the body axis reaches under the dynamics: the thrust axis
// of a multirotor accelerating at the horizontal limit while its vertical
// acceleration is at the limit too, the pose bodyAxisFromWorldAcceleration
// derives for that control.
[[nodiscard]] double maximumBodyTiltRad(double maximum_horizontal_acceleration_mps2,
                                        double maximum_vertical_acceleration_mps2,
                                        double gravity_mps2 = 9.80665) noexcept;

// The footprint whose physical body is the body at every tilt up to
// `tilt_rad`, and whose envelope contains it: the body radius grows to the
// tilted rim's horizontal reach, the extents to the rim's dip and rise, and
// the envelope radius stays at least the body's. Every validator then stands
// the body upright, planner and execution alike, and a route clears whatever
// the airframe reaches at any acceleration the dynamics let the controller
// command along it. The sampling and the safe-clearance threshold are kept.
[[nodiscard]] SweptFootprintConfig
tiltEnvelopedFootprint(const SweptFootprintConfig& footprint, double tilt_rad) noexcept;

} // namespace drone_city_nav
