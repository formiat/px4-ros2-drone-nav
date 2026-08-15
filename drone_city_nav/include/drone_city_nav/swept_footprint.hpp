#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>

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

enum class SweptFootprintStatus {
  kValid,
  kOutsideGrid,
  kInvalidEsdf,
  kRawCollision,
};

struct SweptFootprintResult {
  SweptFootprintStatus status{SweptFootprintStatus::kInvalidEsdf};
  Point3 failure_point{};
  double minimum_clearance_m{0.0};

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

[[nodiscard]] SweptFootprintResult validateRawPointCloudFootprintAt(
    std::span<const Point3> obstacle_points, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult validateRawPointCloudSweptFootprint(
    std::span<const Point3> obstacle_points, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis,
    const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult
validateSweptFootprint(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                       const Point3& first, const FootprintBodyAxis& first_body_axis,
                       const Point3& second, const FootprintBodyAxis& second_body_axis,
                       const SweptFootprintConfig& config) noexcept;

[[nodiscard]] FootprintBodyAxis
bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                              double gravity_mps2 = 9.80665) noexcept;

} // namespace drone_city_nav
