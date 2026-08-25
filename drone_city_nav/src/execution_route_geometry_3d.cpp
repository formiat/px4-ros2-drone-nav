#include "drone_city_nav/execution_route_geometry_3d.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace drone_city_nav {
namespace {

class GeometryHasher final {
public:
  void value(const std::uint64_t input) noexcept {
    for (std::size_t byte = 0U; byte < sizeof(input); ++byte) {
      hash_ ^= (input >> (byte * 8U)) & 0xffU;
      hash_ *= kFnvPrime;
    }
  }

  void boolean(const bool input) noexcept {
    value(input ? 1U : 0U);
  }

  void number(const double input) noexcept {
    if (!std::isfinite(input)) {
      valid_ = false;
      return;
    }
    value(input == 0.0 ? 0U : std::bit_cast<std::uint64_t>(input));
  }

  void number(const float input) noexcept {
    if (!std::isfinite(input)) {
      valid_ = false;
      return;
    }
    value(input == 0.0F
              ? 0U
              : static_cast<std::uint64_t>(std::bit_cast<std::uint32_t>(input)));
  }

  void text(const std::string_view input) noexcept {
    value(static_cast<std::uint64_t>(input.size()));
    for (const char character : input) {
      value(static_cast<std::uint8_t>(character));
    }
  }

  void point(const Point3& input) noexcept {
    number(input.x);
    number(input.y);
    number(input.z);
  }

  void vector(const Vec3& input) noexcept {
    number(input.x);
    number(input.y);
    number(input.z);
  }

  [[nodiscard]] std::uint64_t result() const noexcept {
    if (!valid_) {
      return 0U;
    }
    return hash_ == 0U ? 1U : hash_;
  }

private:
  static constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
  static constexpr std::uint64_t kFnvPrime{1099511628211ULL};

  std::uint64_t hash_{kFnvOffset};
  bool valid_{true};
};

void hashSegmentSpan(GeometryHasher& hash,
                     const PassageTraversalSegmentSpan& span) noexcept {
  hash.text(span.passage_segment_id.value());
  hash.number(span.begin_station_m);
  hash.number(span.end_station_m);
}

void hashConstrainedSpan(GeometryHasher& hash,
                         const ConstrainedRouteSpan& span) noexcept {
  hash.text(span.passage_traversal_id.value());
  hash.value(span.route_generation);
  hash.value(static_cast<std::uint64_t>(span.direction_sign));
  hash.number(span.begin_station_m);
  hash.number(span.end_station_m);
  hash.value(static_cast<std::uint64_t>(span.envelope.size()));
  for (const RouteEnvelopeSample& sample : span.envelope) {
    hash.number(sample.station_m);
    hash.number(sample.lateral_free_left_m);
    hash.number(sample.lateral_free_right_m);
    hash.number(sample.min_z_m);
    hash.number(sample.max_z_m);
    hash.number(sample.minimum_clearance_m);
    hash.number(sample.reference_z_m);
    hash.number(sample.reference_speed_mps);
  }
  hash.value(static_cast<std::uint64_t>(span.segment_spans.size()));
  for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
    hashSegmentSpan(hash, segment);
  }
}

void hashPassageVolume(GeometryHasher& hash, const PassageVolume& volume) noexcept {
  hash.text(volume.passage_traversal_id.value());
  hash.value(static_cast<std::uint64_t>(volume.span_index));
  hash.number(volume.begin_station_m);
  hash.number(volume.end_station_m);
  hash.number(volume.minimum_lateral_offset_m);
  hash.number(volume.maximum_lateral_offset_m);
  hash.number(volume.minimum_secondary_offset_m);
  hash.number(volume.maximum_secondary_offset_m);
  hash.number(volume.minimum_physical_width_m);
  hash.number(volume.minimum_physical_secondary_extent_m);
  hash.value(static_cast<std::uint64_t>(volume.cross_sections.size()));
  for (const PassageCrossSection& section : volume.cross_sections) {
    hash.number(section.station_m);
    hash.point(section.center);
    hash.vector(section.tangent);
    hash.vector(section.lateral_axis);
    hash.vector(section.secondary_axis);
    hash.number(section.minimum_lateral_offset_m);
    hash.number(section.maximum_lateral_offset_m);
    hash.number(section.minimum_secondary_offset_m);
    hash.number(section.maximum_secondary_offset_m);
    hash.boolean(section.raw_validated);
  }
  hash.boolean(volume.raw_validated);
  hash.value(static_cast<std::uint64_t>(volume.segment_spans.size()));
  for (const PassageTraversalSegmentSpan& segment : volume.segment_spans) {
    hashSegmentSpan(hash, segment);
  }
}

void hashPassageAssignment(GeometryHasher& hash,
                           const CooperativePassageAssignment& assignment) noexcept {
  hash.text(assignment.passage_traversal_id.value());
  hash.value(assignment.route_generation);
  hash.value(static_cast<std::uint64_t>(assignment.span_index));
  hash.number(assignment.physical_width_m);
  hash.number(assignment.minimum_lateral_offset_m);
  hash.number(assignment.maximum_lateral_offset_m);
  hash.number(assignment.minimum_secondary_offset_m);
  hash.number(assignment.maximum_secondary_offset_m);
  hash.number(assignment.requested_lateral_offset_m);
  hash.number(assignment.applied_lateral_offset_m);
  hash.number(assignment.desired_center_separation_m);
  hash.value(static_cast<std::uint64_t>(assignment.passage_cross_section_count));
  hash.boolean(assignment.passage_volume_raw_validated);
  hash.value(static_cast<std::uint64_t>(assignment.status));
}

void hashPassageVolumeConfig(GeometryHasher& hash,
                             const PassageVolumeConfig& config) noexcept {
  hash.number(config.cross_section_spacing_m);
  hash.number(config.lateral_probe_step_m);
  hash.number(config.secondary_probe_step_m);
  hash.number(config.maximum_cross_section_probe_m);
  hash.number(config.minimum_wall_clearance_m);
  hash.number(config.flight_envelope.minimum_target_z_m);
  hash.number(config.flight_envelope.maximum_target_z_m);
  hash.number(config.footprint.radius_m);
  hash.number(config.footprint.lower_extent_m);
  hash.number(config.footprint.upper_extent_m);
  hash.value(static_cast<std::uint64_t>(config.footprint.perimeter_samples));
  hash.value(static_cast<std::uint64_t>(config.footprint.radial_rings));
  hash.value(static_cast<std::uint64_t>(config.footprint.axial_samples));
  hash.number(config.footprint.sweep_step_m);
  hash.number(config.footprint.safe_clearance_threshold_m);
}

void hashRoute(GeometryHasher& hash, const std::vector<RouteSample3D>& route) noexcept {
  hash.value(static_cast<std::uint64_t>(route.size()));
  for (const RouteSample3D& sample : route) {
    hash.point(sample.position);
    hash.vector(sample.tangent);
    hash.number(sample.station_m);
    hash.number(sample.reference_speed_mps);
    hash.value(static_cast<std::uint64_t>(sample.required_risk_tier));
    hash.value(static_cast<std::uint64_t>(sample.transition));
  }
}

void hashPassageResources(GeometryHasher& hash,
                          const ExecutionRouteGeometry3D& geometry) noexcept {
  hashRoute(hash, *geometry.route);
  hash.value(static_cast<std::uint64_t>(geometry.constrained_spans->size()));
  for (const ConstrainedRouteSpan& span : *geometry.constrained_spans) {
    hashConstrainedSpan(hash, span);
  }
  hash.value(static_cast<std::uint64_t>(geometry.passage_volumes->size()));
  for (const PassageVolume& volume : *geometry.passage_volumes) {
    hashPassageVolume(hash, volume);
  }
  hash.value(
      static_cast<std::uint64_t>(geometry.selected_passage_traversal_ids->size()));
  for (const PassageTraversalId& traversal_id :
       *geometry.selected_passage_traversal_ids) {
    hash.text(traversal_id.value());
  }
  hashPassageVolumeConfig(hash, geometry.passage_volume_config);
}

void hashObservationFrontier(GeometryHasher& hash,
                             const ObservationFrontier& frontier) noexcept {
  hash.value(frontier.id.value);
  hash.point(frontier.supporting_viewpoint);
  hash.point(frontier.observation_pose);
  hash.point(frontier.boundary_centroid);
  hash.vector(frontier.observation_direction);
  hash.value(frontier.supporting_map_revision);
  hash.value(static_cast<std::uint64_t>(frontier.supporting_rays));
  hash.value(static_cast<std::uint64_t>(frontier.information_gain_voxels));
  hash.value(static_cast<std::uint64_t>(frontier.required_information_gain_voxels));
  hash.number(frontier.minimum_known_free_ray_m);
}

} // namespace

std::uint64_t
executionRouteGeometryRevision3D(const ExecutionRouteGeometry3D& geometry) noexcept {
  if (geometry.mppi_route == nullptr || geometry.route == nullptr ||
      geometry.route_2d_projection == nullptr ||
      geometry.constrained_spans == nullptr || geometry.passage_volumes == nullptr ||
      geometry.cooperative_passage_assignments == nullptr ||
      geometry.selected_passage_traversal_ids == nullptr ||
      !passageVolumeConfigIsValid(geometry.passage_volume_config)) {
    return 0U;
  }

  GeometryHasher hash;
  hashRoute(hash, *geometry.route);

  hash.value(static_cast<std::uint64_t>(geometry.mppi_route->size()));
  for (const mppi::RouteSample3D& sample : *geometry.mppi_route) {
    hash.number(sample.x_m);
    hash.number(sample.y_m);
    hash.number(sample.z_m);
    hash.number(sample.tangent_x);
    hash.number(sample.tangent_y);
    hash.number(sample.tangent_z);
    hash.number(sample.station_m);
    hash.number(sample.reference_speed_mps);
    hash.value(static_cast<std::uint64_t>(sample.required_risk_tier));
  }

  hash.value(static_cast<std::uint64_t>(geometry.route_2d_projection->size()));
  for (const Point2& point : *geometry.route_2d_projection) {
    hash.number(point.x);
    hash.number(point.y);
  }

  hash.value(
      static_cast<std::uint64_t>(geometry.cooperative_passage_assignments->size()));
  for (const CooperativePassageAssignment& assignment :
       *geometry.cooperative_passage_assignments) {
    hashPassageAssignment(hash, assignment);
  }

  hashPassageResources(hash, geometry);
  hash.value(static_cast<std::uint64_t>(geometry.route_purpose));
  hash.boolean(geometry.observation_frontier.has_value());
  if (geometry.observation_frontier.has_value()) {
    hashObservationFrontier(hash, *geometry.observation_frontier);
  }
  hash.value(geometry.materialized_route_fingerprint);
  return hash.result();
}

std::uint64_t
executionPassageGeometryRevision3D(const ExecutionRouteGeometry3D& geometry) noexcept {
  if (geometry.route == nullptr || geometry.constrained_spans == nullptr ||
      geometry.passage_volumes == nullptr ||
      geometry.selected_passage_traversal_ids == nullptr ||
      !passageVolumeConfigIsValid(geometry.passage_volume_config)) {
    return 0U;
  }
  GeometryHasher hash;
  hashPassageResources(hash, geometry);
  return hash.result();
}

} // namespace drone_city_nav
