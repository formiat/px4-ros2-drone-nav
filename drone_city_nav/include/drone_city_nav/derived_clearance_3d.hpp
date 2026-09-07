#pragma once

#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <limits>
#include <span>

namespace drone_city_nav {

// Derived clearance is an annotation only. This contract deliberately has no
// collision verdict: only OccupiedCollisionOracle3D may classify raw occupied
// evidence as a hard obstacle.
struct DerivedClearanceEvidence3D {
  bool outside_grid_exposure{false};
  bool unknown_exposure{false};
  bool invalid_esdf_exposure{false};
  bool known_clearance_observed{false};
  double minimum_known_clearance_m{std::numeric_limits<double>::infinity()};
  // A body sample lies inside a raw occupied voxel. The conservative clearance
  // is a bound; this is an exact fact about the sampled envelope.
  bool inside_occupied{false};
};

struct DerivedFootprintClearance3D {
  EsdfQueryStatus status{EsdfQueryStatus::kInvalidDistance};
  Point3 diagnostic_point{};
  DerivedClearanceEvidence3D evidence{};

  [[nodiscard]] bool fullyAvailable() const noexcept {
    return status == EsdfQueryStatus::kValid;
  }
};

struct DerivedSweptClearanceProfile3D {
  DerivedFootprintClearance3D clearance{};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
};

[[nodiscard]] DerivedFootprintClearance3D
queryFootprintClearance3D(const EsdfGrid3D& grid, std::span<const float> esdf_m,
                          const Point3& position,
                          const SweptFootprintConfig& config) noexcept;

[[nodiscard]] DerivedFootprintClearance3D
queryFootprintClearance3D(const EsdfGrid3D& grid, std::span<const float> esdf_m,
                          const Point3& position, const FootprintBodyAxis& body_axis,
                          const SweptFootprintConfig& config) noexcept;

[[nodiscard]] DerivedFootprintClearance3D
querySweptFootprintClearance3D(const EsdfGrid3D& grid, std::span<const float> esdf_m,
                               const Point3& first, const Point3& second,
                               const SweptFootprintConfig& config) noexcept;

[[nodiscard]] DerivedFootprintClearance3D querySweptFootprintClearance3D(
    const EsdfGrid3D& grid, std::span<const float> esdf_m, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis,
    const SweptFootprintConfig& config) noexcept;

[[nodiscard]] DerivedSweptClearanceProfile3D profileSweptFootprintClearance3D(
    const EsdfGrid3D& grid, std::span<const float> esdf_m, const Point3& first,
    const Point3& second, const SweptFootprintConfig& config,
    double critical_distance_m, double preferred_distance_m) noexcept;

} // namespace drone_city_nav
