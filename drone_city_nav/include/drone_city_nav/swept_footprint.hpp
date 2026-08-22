#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
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
updateLaunchSupportSettling(LaunchSupportContact3D& contact,
                            const Point3& observed_position) noexcept;

[[nodiscard]] bool proprioceptiveSeedAllowsSupportContact(
    const ProprioceptiveFreeSpaceSeed3D& seed, const Point3& box_minimum,
    const Point3& box_maximum, double occupancy_resolution_m) noexcept;

enum class SweptFootprintStatus {
  kValid,
  kOutsideGrid,
  kUnknownSpace,
  kInvalidEsdf,
  kRawCollision,
};

[[nodiscard]] const char*
sweptFootprintStatusName(SweptFootprintStatus status) noexcept;

struct SweptFootprintEvidence {
  bool raw_collision{false};
  bool outside_grid_exposure{false};
  bool unknown_exposure{false};
  bool invalid_esdf_exposure{false};
  bool known_clearance_observed{false};
  double minimum_known_clearance_m{std::numeric_limits<double>::infinity()};
};

struct SweptFootprintResult {
  SweptFootprintStatus status{SweptFootprintStatus::kInvalidEsdf};
  Point3 failure_point{};
  SweptFootprintEvidence evidence{};

  [[nodiscard]] bool accepted() const noexcept {
    return status == SweptFootprintStatus::kValid;
  }
};

struct SweptFootprintClearanceProfile {
  SweptFootprintResult validation{};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
};

[[nodiscard]] SweptFootprintResult
validateFootprintAt(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                    const Point3& position,
                    const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult
validateFootprintAt(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                    const Point3& position, const FootprintBodyAxis& body_axis,
                    const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult
validateSweptFootprint(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                       const Point3& first, const Point3& second,
                       const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintClearanceProfile profileSweptFootprintClearance(
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m, const Point3& first,
    const Point3& second, const SweptFootprintConfig& config,
    double critical_distance_m, double preferred_distance_m) noexcept;

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
    const ProprioceptiveFreeSpaceSeed3D* free_space_seed = nullptr,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] SweptFootprintResult validateRawSweptFootprint(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* free_space_seed = nullptr,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] bool
rawFootprintIsNavigableAt(const OccupancyGrid3D& occupancy, const Point3& position,
                          const FootprintBodyAxis& body_axis,
                          const SweptFootprintConfig& config) noexcept;

[[nodiscard]] bool
rawSweptFootprintIsNavigable(const OccupancyGrid3D& occupancy, const Point3& first,
                             const FootprintBodyAxis& first_body_axis,
                             const Point3& second,
                             const FootprintBodyAxis& second_body_axis,
                             const SweptFootprintConfig& config) noexcept;

[[nodiscard]] bool rawFootprintIsNavigableAt(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* free_space_seed = nullptr,
    const LaunchSupportContact3D* launch_support_contact = nullptr) noexcept;

[[nodiscard]] bool rawSweptFootprintIsNavigable(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* free_space_seed = nullptr,
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

[[nodiscard]] SweptFootprintResult
validateSweptFootprint(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                       const Point3& first, const FootprintBodyAxis& first_body_axis,
                       const Point3& second, const FootprintBodyAxis& second_body_axis,
                       const SweptFootprintConfig& config) noexcept;

[[nodiscard]] FootprintBodyAxis
bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                              double gravity_mps2 = 9.80665) noexcept;

} // namespace drone_city_nav
