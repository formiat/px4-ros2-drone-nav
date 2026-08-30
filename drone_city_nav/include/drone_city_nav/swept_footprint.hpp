#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

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

struct ProprioceptiveFreeSpaceSeed3D {
  Point3 position{};
  FootprintBodyAxis body_axis{};
  SweptFootprintConfig footprint{};
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

[[nodiscard]] bool
sameProprioceptiveFreeSpaceSeed3D(const ProprioceptiveFreeSpaceSeed3D& first,
                                  const ProprioceptiveFreeSpaceSeed3D& second) noexcept;

[[nodiscard]] bool
sameLaunchSupportContact3D(const LaunchSupportContact3D& first,
                           const LaunchSupportContact3D& second) noexcept;

[[nodiscard]] bool
launchSupportContactValid3D(const LaunchSupportContact3D& contact) noexcept;

[[nodiscard]] bool
updateLaunchSupportSettling(LaunchSupportContact3D& contact,
                            const Point3& observed_position) noexcept;

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

[[nodiscard]] SweptFootprintResult
validateRawFootprintAt(const OccupancyGrid3D& occupancy, const Point3& position,
                       const FootprintBodyAxis& body_axis,
                       const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult
validateRawSweptFootprint(const OccupancyGrid3D& occupancy, const Point3& first,
                          const FootprintBodyAxis& first_body_axis,
                          const Point3& second,
                          const FootprintBodyAxis& second_body_axis,
                          const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult validateRawFootprintAt(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawSweptFootprint(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] bool footprintIntersectsAxisAlignedBox(
    const Point3& position, const FootprintBodyAxis& body_axis,
    const SweptFootprintConfig& config, const Point3& box_minimum,
    const Point3& box_maximum) noexcept;

[[nodiscard]] SweptFootprintResult validateRawPointCloudFootprintAt(
    std::span<const Point3> obstacle_points, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawPointCloudSweptFootprint(
    std::span<const Point3> obstacle_points, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] FootprintBodyAxis
bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                              double gravity_mps2 = 9.80665) noexcept;

} // namespace drone_city_nav
