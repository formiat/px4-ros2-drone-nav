#pragma once

#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageVolumeConfig testPassageVolumeConfig() noexcept {
  PassageVolumeConfig config;
  config.footprint = SweptFootprintConfig{
      .radius_m = 0.0,
      .perimeter_samples = 0U,
      .radial_rings = 0U,
      .axial_samples = 1U,
      .sweep_step_m = 0.25,
  };
  return config;
}

[[nodiscard]] VehicleState3D testVehicleState(const Point3& position,
                                              const Vec3& velocity = {}) noexcept {
  return VehicleState3D{
      .identity =
          VehicleStateIdentity3D{
              .revision = 1U,
              .source_timestamp_us = 1U,
              .receive_stamp_ns = 1,
          },
      .position = position,
      .velocity = velocity,
  };
}

[[nodiscard]] RouteEndpointSemantics3D
testEndpointSemantics(const std::vector<RouteSample3D>& route) noexcept {
  return !route.empty() && route.back().reference_speed_mps <= 1.0e-9
             ? RouteEndpointSemantics3D::kMissionStop
             : RouteEndpointSemantics3D::kContinuation;
}

[[nodiscard]] std::shared_ptr<const CompiledTrajectory3D>
makeGeometry(const std::vector<RouteSample3D>& route,
             const std::uint64_t physical_route_fingerprint,
             const TrackingErrorTubeWorld3D tracking_world = {}) {
  if (route.empty()) {
    throw std::logic_error{"test trajectory requires a route"};
  }
  TrajectoryCompilerConfig3D config;
  config.physical_footprint = SweptFootprintConfig{};
  const TrajectoryCompilationResult3D compilation =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state = testVehicleState(route.front().position),
          .route_generation = 1U,
          .route = route,
          .constrained_spans = {},
          .passage_volumes = {},
          .cooperative_passage_assignments = {},
          .selected_passage_traversal_ids = {},
          .passage_volume_config = testPassageVolumeConfig(),
          .endpoint_semantics = testEndpointSemantics(route),
          .materialized_route_fingerprint = physical_route_fingerprint,
          .tracking_world = tracking_world,
          .config = config,
      });
  if (!compilation.compiled()) {
    throw std::logic_error{
        compiledTrajectoryFailureReason3DName(compilation.validation.reason)};
  }
  return compilation.trajectory;
}

[[nodiscard, maybe_unused]] CertifiedRouteSplice3D
testRouteSplice(const CertifiedRouteSuffix3D& base,
                const CertifiedRouteSuffix3D& successor) {
  const RouteSpliceCertificationResult3D certification =
      certifyRouteSplice3D(base, successor, successor.progress.last_observed_position,
                           CertifiedRouteSpliceConfig3D{
                               .required_overlap_m = 2.0,
                               .sample_step_m = 0.5,
                               .maximum_position_separation_m = 2.0,
                               .minimum_tangent_alignment = 0.5,
                               .activation_station_tolerance_m = 1.0,
                           });
  if (!certification.certified()) {
    throw std::runtime_error{"failed to create test route splice"};
  }
  return *certification.splice;
}

[[nodiscard, maybe_unused]] std::shared_ptr<const CompiledTrajectory3D>
withEndpointSemantics(const std::shared_ptr<const CompiledTrajectory3D>& source,
                      const RouteEndpointSemantics3D endpoint_semantics,
                      const TrackingErrorTubeWorld3D tracking_world = {}) {
  if (source == nullptr || source->route == nullptr || source->route->empty()) {
    throw std::logic_error{"test trajectory source is unavailable"};
  }
  TrajectoryCompilerConfig3D config;
  config.physical_footprint = source->tracking_error_tube->physical_footprint;
  config.tracking_error_tube = source->tracking_error_tube->config;
  const TrajectoryCompilationResult3D compilation =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state = source->exact_initial_state,
          .route_generation = 1U,
          .route = *source->route,
          .constrained_spans = *source->constrained_spans,
          .passage_volumes = *source->passage_volumes,
          .cooperative_passage_assignments = *source->cooperative_passage_assignments,
          .selected_passage_traversal_ids = *source->selected_passage_traversal_ids,
          .passage_volume_config = source->passage_volume_config,
          .endpoint_semantics = endpoint_semantics,
          .materialized_route_fingerprint = source->materialized_route_fingerprint,
          .tracking_world = tracking_world,
          .config = config,
      });
  if (!compilation.compiled()) {
    throw std::logic_error{
        compiledTrajectoryFailureReason3DName(compilation.validation.reason)};
  }
  return compilation.trajectory;
}

[[nodiscard, maybe_unused]] std::shared_ptr<const CompiledTrajectory3D>
makeConstrainedGeometry(const std::vector<RouteSample3D>& route,
                        const std::uint64_t physical_route_fingerprint,
                        const std::uint64_t route_generation,
                        const OccupancyGrid3D& occupancy,
                        const PassageVolumeConfig& passage_volume_config,
                        const double constrained_begin_station_m = 0.0,
                        const double constrained_end_station_m = -1.0) {
  if (route.empty()) {
    throw std::logic_error{"test constrained trajectory requires a route"};
  }
  const PassageTraversalId passage_traversal_id{"test_passage:forward"};
  const double route_end_station_m = route.back().station_m;
  const double span_end_station_m = constrained_end_station_m >= 0.0
                                        ? constrained_end_station_m
                                        : route_end_station_m;
  const RouteSample3D span_begin =
      sampleRoute3DAtStation(route, constrained_begin_station_m);
  const RouteSample3D span_end = sampleRoute3DAtStation(route, span_end_station_m);
  const ConstrainedRouteSpan span{
      .passage_traversal_id = passage_traversal_id,
      .route_generation = route_generation,
      .direction_sign = 1,
      .begin_station_m = constrained_begin_station_m,
      .end_station_m = span_end_station_m,
      .envelope =
          {
              RouteEnvelopeSample{
                  .station_m = constrained_begin_station_m,
                  .lateral_free_left_m = 2.0,
                  .lateral_free_right_m = 2.0,
                  .min_z_m = 4.0,
                  .max_z_m = 6.0,
                  .minimum_clearance_m = 1.0,
                  .reference_z_m = span_begin.position.z,
                  .reference_speed_mps = span_begin.reference_speed_mps,
              },
              RouteEnvelopeSample{
                  .station_m = span_end_station_m,
                  .lateral_free_left_m = 2.0,
                  .lateral_free_right_m = 2.0,
                  .min_z_m = 4.0,
                  .max_z_m = 6.0,
                  .minimum_clearance_m = 1.0,
                  .reference_z_m = span_end.position.z,
                  .reference_speed_mps = span_end.reference_speed_mps,
              },
          },
      .segment_spans = {},
  };

  std::vector<ConstrainedRouteSpan> spans{span};
  std::vector<PassageVolume> volumes =
      derivePassageVolumes(route, spans, occupancy, passage_volume_config);
  if (volumes.size() == spans.size()) {
    static_cast<void>(
        projectPassageVolumeEnvelopes(spans, volumes, passage_volume_config.footprint));
  }
  const PassageVolume volume = volumes.front();
  const CooperativePassageAssignment assignment{
      .passage_traversal_id = passage_traversal_id,
      .route_generation = route_generation,
      .span_index = 0U,
      .physical_width_m = volume.minimum_physical_width_m,
      .minimum_lateral_offset_m = volume.minimum_lateral_offset_m,
      .maximum_lateral_offset_m = volume.maximum_lateral_offset_m,
      .minimum_secondary_offset_m = volume.minimum_secondary_offset_m,
      .maximum_secondary_offset_m = volume.maximum_secondary_offset_m,
      .requested_lateral_offset_m = 0.0,
      .applied_lateral_offset_m = 0.0,
      .desired_center_separation_m = 1.0,
      .passage_cross_section_count = volume.cross_sections.size(),
      .passage_volume_raw_validated = volume.raw_validated,
      .status = CooperativePassageRouteStatus::kCentered,
  };

  TrajectoryCompilerConfig3D config;
  config.physical_footprint = SweptFootprintConfig{};
  const TrajectoryCompilationResult3D compilation =
      TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
          .exact_initial_state = testVehicleState(route.front().position),
          .route_generation = route_generation,
          .route = route,
          .constrained_spans = std::move(spans),
          .passage_volumes = std::move(volumes),
          .cooperative_passage_assignments = {assignment},
          .selected_passage_traversal_ids = {passage_traversal_id},
          .passage_volume_config = passage_volume_config,
          .endpoint_semantics = testEndpointSemantics(route),
          .materialized_route_fingerprint = physical_route_fingerprint,
          .tracking_world =
              TrackingErrorTubeWorld3D{
                  .occupancy = &occupancy,
                  .occupied_content_fingerprint = occupancy.contentFingerprint(),
              },
          .config = config,
      });
  if (!compilation.compiled()) {
    throw std::logic_error{
        compiledTrajectoryFailureReason3DName(compilation.validation.reason)};
  }
  return compilation.trajectory;
}

} // namespace
} // namespace drone_city_nav
