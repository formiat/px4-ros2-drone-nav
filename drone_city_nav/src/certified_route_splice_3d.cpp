#include "drone_city_nav/certified_route_splice_3d.hpp"

#include "drone_city_nav/route_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>

namespace drone_city_nav {
namespace {

constexpr double kStationEpsilonM{1.0e-6};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool sameLineage(const RouteContinuityLineage3D& first,
                               const RouteContinuityLineage3D& second) noexcept {
  return first.mission_epoch == second.mission_epoch &&
         first.assignment_generation == second.assignment_generation &&
         first.target_detection_id == second.target_detection_id &&
         first.target_track_id == second.target_track_id;
}

[[nodiscard]] bool
configValidImpl(const CertifiedRouteSpliceConfig3D& config) noexcept {
  return std::isfinite(config.required_overlap_m) && config.required_overlap_m > 0.0 &&
         std::isfinite(config.sample_step_m) && config.sample_step_m > 0.0 &&
         std::isfinite(config.maximum_position_separation_m) &&
         config.maximum_position_separation_m > 0.0 &&
         std::isfinite(config.minimum_tangent_alignment) &&
         config.minimum_tangent_alignment >= -1.0 &&
         config.minimum_tangent_alignment <= 1.0 &&
         std::isfinite(config.activation_station_tolerance_m) &&
         config.activation_station_tolerance_m > 0.0;
}

[[nodiscard]] double tangentAlignment(const Vec3& first, const Vec3& second) noexcept {
  const double first_norm =
      std::sqrt(first.x * first.x + first.y * first.y + first.z * first.z);
  const double second_norm =
      std::sqrt(second.x * second.x + second.y * second.y + second.z * second.z);
  if (!(first_norm > 0.0) || !(second_norm > 0.0) || !std::isfinite(first_norm) ||
      !std::isfinite(second_norm)) {
    return -1.0;
  }
  return std::clamp((first.x * second.x + first.y * second.y + first.z * second.z) /
                        (first_norm * second_norm),
                    -1.0, 1.0);
}

[[nodiscard]] bool routeIdentityMatches(const CertifiedRouteSuffix3D& route,
                                        const std::uint64_t generation,
                                        const std::uint64_t geometry_revision,
                                        const std::uint64_t continuity_id) noexcept {
  return route.valid() && route.geometry != nullptr &&
         route.identity.generation == generation &&
         route.geometry->executable_geometry_revision == geometry_revision &&
         route.continuity_id == continuity_id;
}

[[nodiscard]] double
effectiveRequiredOverlap(const CertifiedRouteSuffix3D& successor,
                         const RouteProjection3D& successor_projection,
                         const CertifiedRouteSpliceConfig3D& config,
                         bool& terminal_overlap_capped) noexcept {
  const double successor_remaining_m =
      std::max(0.0, successor.endStationM() - successor_projection.station_m);
  terminal_overlap_capped =
      routeEndpointHasTerminalStop3D(successor.planned_endpoint_semantics) &&
      successor_remaining_m < config.required_overlap_m;
  return terminal_overlap_capped ? successor_remaining_m : config.required_overlap_m;
}

[[nodiscard]] RouteProjection3D
projectInsideRouteSuffix(const CertifiedRouteSuffix3D& route,
                         const Point3& position) noexcept {
  if (route.geometry == nullptr || route.geometry->route == nullptr) {
    return {};
  }
  return projectOntoRoute3DWithinStationWindow(
      *route.geometry->route, position, route.progress.station_m, route.endStationM());
}

} // namespace

bool certifiedRouteSpliceConfig3DValid(
    const CertifiedRouteSpliceConfig3D& config) noexcept {
  return configValidImpl(config);
}

bool CertifiedRouteSplice3D::structurallyValid() const noexcept {
  return base_route_generation != 0U && base_geometry_revision != 0U &&
         base_continuity_id != 0U && successor_route_generation != 0U &&
         successor_geometry_revision != 0U && successor_continuity_id != 0U &&
         base_route_generation != std::numeric_limits<std::uint64_t>::max() &&
         successor_route_generation == base_route_generation + 1U &&
         std::isfinite(base_begin_station_m) && std::isfinite(base_end_station_m) &&
         std::isfinite(successor_begin_station_m) &&
         std::isfinite(successor_end_station_m) && std::isfinite(required_overlap_m) &&
         required_overlap_m > 0.0 && std::isfinite(sample_step_m) &&
         sample_step_m > 0.0 && std::isfinite(maximum_position_separation_m) &&
         maximum_position_separation_m > 0.0 &&
         std::isfinite(minimum_tangent_alignment) &&
         minimum_tangent_alignment >= -1.0 && minimum_tangent_alignment <= 1.0 &&
         std::isfinite(activation_station_tolerance_m) &&
         activation_station_tolerance_m > 0.0 &&
         std::isfinite(measured_maximum_position_separation_m) &&
         measured_maximum_position_separation_m <=
             maximum_position_separation_m + kStationEpsilonM &&
         std::isfinite(measured_minimum_tangent_alignment) &&
         measured_minimum_tangent_alignment + kStationEpsilonM >=
             minimum_tangent_alignment &&
         base_end_station_m + kStationEpsilonM >=
             base_begin_station_m + required_overlap_m &&
         successor_end_station_m + kStationEpsilonM >=
             successor_begin_station_m + required_overlap_m;
}

bool CertifiedRouteSplice3D::validFor(
    const CertifiedRouteSuffix3D& base,
    const CertifiedRouteSuffix3D& successor) const noexcept {
  return structurallyValid() &&
         routeIdentityMatches(base, base_route_generation, base_geometry_revision,
                              base_continuity_id) &&
         routeIdentityMatches(successor, successor_route_generation,
                              successor_geometry_revision, successor_continuity_id) &&
         sameLineage(base.continuity_lineage, continuity_lineage) &&
         sameLineage(successor.continuity_lineage, continuity_lineage) &&
         base.progress.station_m <=
             base_end_station_m + activation_station_tolerance_m &&
         base_end_station_m <= base.endStationM() + kStationEpsilonM &&
         successor.progress.station_m <=
             successor_end_station_m + activation_station_tolerance_m &&
         successor_end_station_m <= successor.endStationM() + kStationEpsilonM;
}

bool RouteSpliceCertificationResult3D::certified() const noexcept {
  return status == RouteSpliceCertificationStatus3D::kCertified && splice.has_value();
}

RouteSpliceCertificationResult3D
certifyRouteSplice3D(const CertifiedRouteSuffix3D& base,
                     const CertifiedRouteSuffix3D& successor,
                     const Point3& activation_position,
                     const CertifiedRouteSpliceConfig3D& config) noexcept {
  RouteSpliceCertificationResult3D result;
  if (!certifiedRouteSpliceConfig3DValid(config)) {
    result.status = RouteSpliceCertificationStatus3D::kInvalidConfig;
    return result;
  }
  if (!base.valid() || !successor.valid() || !finitePoint(activation_position) ||
      base.geometry == nullptr || base.geometry->route == nullptr ||
      successor.geometry == nullptr || successor.geometry->route == nullptr) {
    result.status = RouteSpliceCertificationStatus3D::kInvalidRoute;
    return result;
  }
  if (base.identity.generation == std::numeric_limits<std::uint64_t>::max() ||
      successor.identity.generation != base.identity.generation + 1U) {
    result.status = RouteSpliceCertificationStatus3D::kGenerationMismatch;
    return result;
  }
  if (!sameLineage(base.continuity_lineage, successor.continuity_lineage)) {
    result.status = RouteSpliceCertificationStatus3D::kContinuityMismatch;
    return result;
  }

  const RouteProjection3D base_projection =
      projectInsideRouteSuffix(base, activation_position);
  const RouteProjection3D successor_projection =
      projectInsideRouteSuffix(successor, activation_position);
  if (!base_projection.valid || !successor_projection.valid) {
    result.status = RouteSpliceCertificationStatus3D::kProjectionUnavailable;
    return result;
  }

  bool terminal_overlap_capped{false};
  const double required_overlap_m = effectiveRequiredOverlap(
      successor, successor_projection, config, terminal_overlap_capped);
  result.available_base_overlap_m =
      std::max(0.0, base.endStationM() - base_projection.station_m);
  result.available_successor_overlap_m =
      std::max(0.0, successor.endStationM() - successor_projection.station_m);
  if (!(required_overlap_m > config.activation_station_tolerance_m) ||
      result.available_base_overlap_m + kStationEpsilonM < required_overlap_m ||
      result.available_successor_overlap_m + kStationEpsilonM < required_overlap_m) {
    result.status = RouteSpliceCertificationStatus3D::kInsufficientOverlap;
    return result;
  }

  result.measured_minimum_tangent_alignment = 1.0;
  const std::size_t interval_count = std::max<std::size_t>(
      1U,
      static_cast<std::size_t>(std::ceil(required_overlap_m / config.sample_step_m)));
  for (std::size_t interval = 0U; interval <= interval_count; ++interval) {
    const double offset_m = required_overlap_m * static_cast<double>(interval) /
                            static_cast<double>(interval_count);
    const RouteSample3D base_sample = sampleRoute3DAtStation(
        *base.geometry->route, base_projection.station_m + offset_m);
    const RouteSample3D successor_sample = sampleRoute3DAtStation(
        *successor.geometry->route, successor_projection.station_m + offset_m);
    const double separation_m =
        distance3D(base_sample.position, successor_sample.position);
    const double alignment =
        tangentAlignment(base_sample.tangent, successor_sample.tangent);
    result.measured_maximum_position_separation_m =
        std::max(result.measured_maximum_position_separation_m, separation_m);
    result.measured_minimum_tangent_alignment =
        std::min(result.measured_minimum_tangent_alignment, alignment);
    if (!std::isfinite(separation_m) ||
        separation_m > config.maximum_position_separation_m) {
      result.status = RouteSpliceCertificationStatus3D::kGeometryDiverged;
      return result;
    }
    if (alignment < config.minimum_tangent_alignment) {
      result.status = RouteSpliceCertificationStatus3D::kTangentDiscontinuity;
      return result;
    }
  }

  result.splice = CertifiedRouteSplice3D{
      .base_route_generation = base.identity.generation,
      .base_geometry_revision = base.geometry->executable_geometry_revision,
      .base_continuity_id = base.continuity_id,
      .successor_route_generation = successor.identity.generation,
      .successor_geometry_revision = successor.geometry->executable_geometry_revision,
      .successor_continuity_id = successor.continuity_id,
      .continuity_lineage = base.continuity_lineage,
      .base_begin_station_m = base_projection.station_m,
      .base_end_station_m = base_projection.station_m + required_overlap_m,
      .successor_begin_station_m = successor_projection.station_m,
      .successor_end_station_m = successor_projection.station_m + required_overlap_m,
      .required_overlap_m = required_overlap_m,
      .sample_step_m = config.sample_step_m,
      .maximum_position_separation_m = config.maximum_position_separation_m,
      .minimum_tangent_alignment = config.minimum_tangent_alignment,
      .activation_station_tolerance_m = config.activation_station_tolerance_m,
      .measured_maximum_position_separation_m =
          result.measured_maximum_position_separation_m,
      .measured_minimum_tangent_alignment = result.measured_minimum_tangent_alignment,
      .terminal_overlap_capped = terminal_overlap_capped,
  };
  if (!result.splice->validFor(base, successor)) {
    result.splice.reset();
    result.status = RouteSpliceCertificationStatus3D::kInvalidRoute;
    return result;
  }
  result.status = RouteSpliceCertificationStatus3D::kCertified;
  return result;
}

bool RouteSpliceReadiness3D::ready() const noexcept {
  return status == RouteSpliceReadinessStatus3D::kReady;
}

bool RouteSpliceReadiness3D::canStillBecomeReady() const noexcept {
  return status == RouteSpliceReadinessStatus3D::kReady ||
         status == RouteSpliceReadinessStatus3D::kBeforeWindow ||
         status == RouteSpliceReadinessStatus3D::kBaseProjectionUnavailable ||
         status == RouteSpliceReadinessStatus3D::kSuccessorProjectionUnavailable;
}

RouteSpliceReadiness3D
assessRouteSpliceReadiness3D(const CertifiedRouteSplice3D& splice,
                             const CertifiedRouteSuffix3D& base,
                             const CertifiedRouteSuffix3D& successor,
                             const Point3& activation_position) noexcept {
  RouteSpliceReadiness3D result;
  if (!splice.validFor(base, successor) || !finitePoint(activation_position)) {
    return result;
  }
  if (base.progress.station_m + splice.activation_station_tolerance_m <
      splice.base_begin_station_m) {
    result.status = RouteSpliceReadinessStatus3D::kBeforeWindow;
    return result;
  }
  if (base.progress.station_m >
      splice.base_end_station_m + splice.activation_station_tolerance_m) {
    result.status = RouteSpliceReadinessStatus3D::kExpired;
    return result;
  }

  const RouteProjection3D base_projection = projectOntoRoute3DWithinStationWindow(
      *base.geometry->route, activation_position,
      std::min(std::max(base.progress.station_m, splice.base_begin_station_m),
               splice.base_end_station_m),
      splice.base_end_station_m);
  if (!base_projection.valid ||
      base_projection.distance_m > splice.maximum_position_separation_m) {
    result.status = RouteSpliceReadinessStatus3D::kBaseProjectionUnavailable;
    return result;
  }
  const RouteProjection3D successor_projection = projectOntoRoute3DWithinStationWindow(
      *successor.geometry->route, activation_position,
      std::min(std::max(successor.progress.station_m, splice.successor_begin_station_m),
               splice.successor_end_station_m),
      splice.successor_end_station_m);
  if (!successor_projection.valid ||
      successor_projection.distance_m > splice.maximum_position_separation_m) {
    result.status = RouteSpliceReadinessStatus3D::kSuccessorProjectionUnavailable;
    return result;
  }
  result.base_station_m = base_projection.station_m;
  result.successor_station_m = successor_projection.station_m;
  const double base_advance_m = base_projection.station_m - splice.base_begin_station_m;
  const double successor_advance_m =
      successor_projection.station_m - splice.successor_begin_station_m;
  if (std::abs(base_advance_m - successor_advance_m) >
      splice.activation_station_tolerance_m) {
    result.status = RouteSpliceReadinessStatus3D::kStationMismatch;
    return result;
  }
  const RouteSample3D base_sample =
      sampleRoute3DAtStation(*base.geometry->route, base_projection.station_m);
  const RouteSample3D successor_sample = sampleRoute3DAtStation(
      *successor.geometry->route, successor_projection.station_m);
  result.position_separation_m =
      distance3D(base_sample.position, successor_sample.position);
  result.tangent_alignment =
      tangentAlignment(base_sample.tangent, successor_sample.tangent);
  if (!std::isfinite(result.position_separation_m) ||
      result.position_separation_m > splice.maximum_position_separation_m) {
    result.status = RouteSpliceReadinessStatus3D::kGeometryDiverged;
    return result;
  }
  if (result.tangent_alignment < splice.minimum_tangent_alignment) {
    result.status = RouteSpliceReadinessStatus3D::kTangentDiscontinuity;
    return result;
  }
  result.status = RouteSpliceReadinessStatus3D::kReady;
  return result;
}

bool routeSpliceWindowExpired3D(const CertifiedRouteSplice3D& splice,
                                const CertifiedRouteSuffix3D& base) noexcept {
  return base.geometry != nullptr &&
         base.identity.generation == splice.base_route_generation &&
         base.geometry->executable_geometry_revision == splice.base_geometry_revision &&
         base.continuity_id == splice.base_continuity_id &&
         std::isfinite(base.progress.station_m) &&
         base.progress.station_m >
             splice.base_end_station_m + splice.activation_station_tolerance_m;
}

std::string_view routeSpliceCertificationStatus3DName(
    const RouteSpliceCertificationStatus3D status) noexcept {
  switch (status) {
    case RouteSpliceCertificationStatus3D::kNotAttempted:
      return "not_attempted";
    case RouteSpliceCertificationStatus3D::kCertified:
      return "certified";
    case RouteSpliceCertificationStatus3D::kInvalidConfig:
      return "invalid_config";
    case RouteSpliceCertificationStatus3D::kInvalidRoute:
      return "invalid_route";
    case RouteSpliceCertificationStatus3D::kGenerationMismatch:
      return "generation_mismatch";
    case RouteSpliceCertificationStatus3D::kContinuityMismatch:
      return "continuity_mismatch";
    case RouteSpliceCertificationStatus3D::kProjectionUnavailable:
      return "projection_unavailable";
    case RouteSpliceCertificationStatus3D::kInsufficientOverlap:
      return "insufficient_overlap";
    case RouteSpliceCertificationStatus3D::kGeometryDiverged:
      return "geometry_diverged";
    case RouteSpliceCertificationStatus3D::kTangentDiscontinuity:
      return "tangent_discontinuity";
  }
  return "unknown";
}

std::string_view
routeSpliceReadinessStatus3DName(const RouteSpliceReadinessStatus3D status) noexcept {
  switch (status) {
    case RouteSpliceReadinessStatus3D::kReady:
      return "ready";
    case RouteSpliceReadinessStatus3D::kInvalidProof:
      return "invalid_proof";
    case RouteSpliceReadinessStatus3D::kBeforeWindow:
      return "before_window";
    case RouteSpliceReadinessStatus3D::kExpired:
      return "expired";
    case RouteSpliceReadinessStatus3D::kBaseProjectionUnavailable:
      return "base_projection_unavailable";
    case RouteSpliceReadinessStatus3D::kSuccessorProjectionUnavailable:
      return "successor_projection_unavailable";
    case RouteSpliceReadinessStatus3D::kStationMismatch:
      return "station_mismatch";
    case RouteSpliceReadinessStatus3D::kGeometryDiverged:
      return "geometry_diverged";
    case RouteSpliceReadinessStatus3D::kTangentDiscontinuity:
      return "tangent_discontinuity";
  }
  return "unknown";
}

} // namespace drone_city_nav
