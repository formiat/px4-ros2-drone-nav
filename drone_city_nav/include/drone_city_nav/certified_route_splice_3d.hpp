#pragma once

#include "drone_city_nav/execution_route_snapshot_3d.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

namespace drone_city_nav {

struct CertifiedRouteSpliceConfig3D {
  double required_overlap_m{8.0};
  double sample_step_m{0.5};
  // The successor must retain the executable prefix, not merely follow a
  // nearby corridor.  These defaults bound any numerical resampling error
  // while rejecting a route that would require a lateral or heading step at
  // handoff.
  double maximum_position_separation_m{0.05};
  double minimum_tangent_alignment{0.995};
  double activation_station_tolerance_m{1.0};
};

[[nodiscard]] bool
certifiedRouteSpliceConfig3DValid(const CertifiedRouteSpliceConfig3D& config) noexcept;

enum class RouteSpliceCertificationStatus3D : std::uint8_t {
  kCertified,
  kInvalidConfig,
  kInvalidRoute,
  kGenerationMismatch,
  kContinuityMismatch,
  kProjectionUnavailable,
  kInsufficientOverlap,
  kGeometryDiverged,
  kTangentDiscontinuity,
};

struct CertifiedRouteSplice3D {
  std::uint64_t base_route_generation{0U};
  std::uint64_t base_geometry_revision{0U};
  std::uint64_t base_continuity_id{0U};
  std::uint64_t successor_route_generation{0U};
  std::uint64_t successor_geometry_revision{0U};
  std::uint64_t successor_continuity_id{0U};
  RouteContinuityLineage3D continuity_lineage{};
  double base_begin_station_m{0.0};
  double base_end_station_m{0.0};
  double successor_begin_station_m{0.0};
  double successor_end_station_m{0.0};
  double required_overlap_m{0.0};
  double sample_step_m{0.0};
  double maximum_position_separation_m{0.0};
  double minimum_tangent_alignment{0.0};
  double activation_station_tolerance_m{0.0};
  double measured_maximum_position_separation_m{0.0};
  double measured_minimum_tangent_alignment{0.0};
  bool terminal_overlap_capped{false};

  [[nodiscard]] bool structurallyValid() const noexcept;
  [[nodiscard]] bool validFor(const CertifiedRouteSuffix3D& base,
                              const CertifiedRouteSuffix3D& successor) const noexcept;
};

struct RouteSpliceCertificationResult3D {
  RouteSpliceCertificationStatus3D status{
      RouteSpliceCertificationStatus3D::kInvalidRoute};
  std::optional<CertifiedRouteSplice3D> splice;
  double available_base_overlap_m{0.0};
  double available_successor_overlap_m{0.0};
  double measured_maximum_position_separation_m{0.0};
  double measured_minimum_tangent_alignment{0.0};

  [[nodiscard]] bool certified() const noexcept;
};

[[nodiscard]] RouteSpliceCertificationResult3D
certifyRouteSplice3D(const CertifiedRouteSuffix3D& base,
                     const CertifiedRouteSuffix3D& successor,
                     const Point3& activation_position,
                     const CertifiedRouteSpliceConfig3D& config) noexcept;

enum class RouteSpliceReadinessStatus3D : std::uint8_t {
  kReady,
  kInvalidProof,
  kBeforeWindow,
  kExpired,
  kBaseProjectionUnavailable,
  kSuccessorProjectionUnavailable,
  kStationMismatch,
  kGeometryDiverged,
  kTangentDiscontinuity,
};

struct RouteSpliceReadiness3D {
  RouteSpliceReadinessStatus3D status{RouteSpliceReadinessStatus3D::kInvalidProof};
  double base_station_m{0.0};
  double successor_station_m{0.0};
  double position_separation_m{0.0};
  double tangent_alignment{0.0};

  [[nodiscard]] bool ready() const noexcept;
  [[nodiscard]] bool canStillBecomeReady() const noexcept;
};

[[nodiscard]] RouteSpliceReadiness3D
assessRouteSpliceReadiness3D(const CertifiedRouteSplice3D& splice,
                             const CertifiedRouteSuffix3D& base,
                             const CertifiedRouteSuffix3D& successor,
                             const Point3& activation_position) noexcept;

[[nodiscard]] bool
routeSpliceWindowExpired3D(const CertifiedRouteSplice3D& splice,
                           const CertifiedRouteSuffix3D& base) noexcept;

[[nodiscard]] std::string_view
routeSpliceCertificationStatus3DName(RouteSpliceCertificationStatus3D status) noexcept;

[[nodiscard]] std::string_view
routeSpliceReadinessStatus3DName(RouteSpliceReadinessStatus3D status) noexcept;

} // namespace drone_city_nav
