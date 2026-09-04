#pragma once

#include "drone_city_nav/types.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace drone_city_nav {

// Physical body geometry and the proprioceptive support evidence expressed in
// terms of it. These are value contracts with no occupancy dependency, so the
// world layer can publish them without reaching into collision checking.
struct SweptFootprintConfig {
  double radius_m{0.82};
  double lower_extent_m{0.23};
  double upper_extent_m{0.35};
  std::size_t perimeter_samples{12U};
  std::size_t radial_rings{2U};
  std::size_t axial_samples{3U};
  double sweep_step_m{0.25};
  double safe_clearance_threshold_m{0.0};
};

struct FootprintBodyAxis {
  double x{0.0};
  double y{0.0};
  double z{1.0};
};

// The body demonstrably occupies its own volume at the seed, so occupied
// evidence overlapping that volume is contact rather than an obstacle for a
// validation run from that pose. The exemption suppresses exactly that
// evidence and constrains nothing about where the body then moves: free space
// is always traversable, and a rule of the form "no closer than now" or "only
// away from the surface" is a prohibition on moving through it. The contact
// tolerance is the quantization of the evidence the seed is judged against and
// widens the contact volume only.
struct ProprioceptiveFreeSpaceSeed3D {
  Point3 position{};
  FootprintBodyAxis body_axis{};
  SweptFootprintConfig footprint{};
  double contact_tolerance_m{0.0};
};

struct AxisAlignedBox3D {
  Point3 minimum{};
  Point3 maximum{};
};

enum class LaunchSupportEvidenceSource {
  kObservedOccupancy,
  kVehicleLandDetector,
};

struct LaunchSupportContact3D {
  ProprioceptiveFreeSpaceSeed3D seed{};
  std::vector<AxisAlignedBox3D> contact_cells;
  std::size_t occupied_evidence_cells{0U};
  LaunchSupportEvidenceSource evidence_source{
      LaunchSupportEvidenceSource::kObservedOccupancy};
  double maximum_lateral_departure_m{0.0};
  double minimum_axial_departure_m{0.0};
  double maximum_axial_settling_m{0.0};
};

[[nodiscard]] inline FootprintBodyAxis
normalizedFootprintBodyAxis(const FootprintBodyAxis& axis) noexcept {
  const double length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  if (!(length > 1.0e-9) || !std::isfinite(length)) {
    return FootprintBodyAxis{.x = 0.0, .y = 0.0, .z = 0.0};
  }
  return FootprintBodyAxis{axis.x / length, axis.y / length, axis.z / length};
}

[[nodiscard]] bool
sameProprioceptiveFreeSpaceSeed3D(const ProprioceptiveFreeSpaceSeed3D& first,
                                  const ProprioceptiveFreeSpaceSeed3D& second) noexcept;

[[nodiscard]] bool
sameLaunchSupportContact3D(const LaunchSupportContact3D& first,
                           const LaunchSupportContact3D& second) noexcept;

[[nodiscard]] bool
launchSupportContactValid3D(const LaunchSupportContact3D& contact) noexcept;

// True while a candidate body position is still inside the departure envelope
// of the launch support: contact cells suppress occupied evidence only there.
[[nodiscard]] bool
launchSupportEnvelopeContains3D(const LaunchSupportContact3D& contact,
                                const Point3& candidate_position) noexcept;

// True when the voxel box is one of the recorded launch support contact cells.
[[nodiscard]] bool
launchSupportContactContainsCell3D(const LaunchSupportContact3D& contact,
                                   const Point3& box_minimum,
                                   const Point3& box_maximum) noexcept;

[[nodiscard]] bool
updateLaunchSupportSettling(LaunchSupportContact3D& contact,
                            const Point3& observed_position) noexcept;

} // namespace drone_city_nav
