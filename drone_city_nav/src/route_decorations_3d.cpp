#include "drone_city_nav/route_decorations_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>
#include <string_view>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kStationToleranceM{1.0e-6};
constexpr double kGeometryTolerance{1.0e-4};

class DecorationHasher final {
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

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finiteVector(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool nearlyEqual(const double first, const double second,
                               const double tolerance) noexcept {
  return std::abs(first - second) <= tolerance;
}

[[nodiscard]] double vectorNorm(const Vec3& vector) noexcept {
  return std::hypot(std::hypot(vector.x, vector.y), vector.z);
}

[[nodiscard]] double vectorDot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

void hashSegmentSpan(DecorationHasher& hash,
                     const PassageTraversalSegmentSpan& span) noexcept {
  hash.text(span.passage_segment_id.value());
  hash.number(span.begin_station_m);
  hash.number(span.end_station_m);
}

void hashPassageVolume(DecorationHasher& hash, const PassageVolume& volume) noexcept {
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

void hashPassageAssignment(DecorationHasher& hash,
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

[[nodiscard]] bool
validPassageCrossSection(const PassageCrossSection& section) noexcept {
  const double tangent_norm = vectorNorm(section.tangent);
  const double lateral_norm = vectorNorm(section.lateral_axis);
  const double secondary_norm = vectorNorm(section.secondary_axis);
  const Vec3 tangent_cross_lateral{section.tangent.y * section.lateral_axis.z -
                                       section.tangent.z * section.lateral_axis.y,
                                   section.tangent.z * section.lateral_axis.x -
                                       section.tangent.x * section.lateral_axis.z,
                                   section.tangent.x * section.lateral_axis.y -
                                       section.tangent.y * section.lateral_axis.x};
  return std::isfinite(section.station_m) && finitePoint(section.center) &&
         finiteVector(section.tangent) && finiteVector(section.lateral_axis) &&
         finiteVector(section.secondary_axis) &&
         nearlyEqual(tangent_norm, 1.0, 1.0e-3) &&
         nearlyEqual(lateral_norm, 1.0, 1.0e-3) &&
         nearlyEqual(secondary_norm, 1.0, 1.0e-3) &&
         std::abs(vectorDot(section.tangent, section.lateral_axis)) <= 1.0e-3 &&
         std::abs(vectorDot(section.tangent, section.secondary_axis)) <= 1.0e-3 &&
         std::abs(vectorDot(section.lateral_axis, section.secondary_axis)) <= 1.0e-3 &&
         vectorDot(tangent_cross_lateral, section.secondary_axis) >= 0.999 &&
         std::isfinite(section.minimum_lateral_offset_m) &&
         std::isfinite(section.maximum_lateral_offset_m) &&
         std::isfinite(section.minimum_secondary_offset_m) &&
         std::isfinite(section.maximum_secondary_offset_m) &&
         section.minimum_lateral_offset_m <= section.maximum_lateral_offset_m &&
         section.minimum_secondary_offset_m <= section.maximum_secondary_offset_m &&
         section.raw_validated;
}

[[nodiscard]] bool validPassageVolume(const PassageVolume& volume,
                                      const ConstrainedRouteSpan& span,
                                      const std::size_t span_index) noexcept {
  if (volume.span_index != span_index ||
      volume.passage_traversal_id != span.passage_traversal_id ||
      !volume.raw_validated || volume.cross_sections.empty() ||
      !std::isfinite(volume.begin_station_m) || !std::isfinite(volume.end_station_m) ||
      !nearlyEqual(volume.begin_station_m, span.begin_station_m, kStationToleranceM) ||
      !nearlyEqual(volume.end_station_m, span.end_station_m, kStationToleranceM) ||
      !std::isfinite(volume.minimum_lateral_offset_m) ||
      !std::isfinite(volume.maximum_lateral_offset_m) ||
      !std::isfinite(volume.minimum_secondary_offset_m) ||
      !std::isfinite(volume.maximum_secondary_offset_m) ||
      volume.minimum_lateral_offset_m > volume.maximum_lateral_offset_m ||
      volume.minimum_secondary_offset_m > volume.maximum_secondary_offset_m ||
      !std::isfinite(volume.minimum_physical_width_m) ||
      !std::isfinite(volume.minimum_physical_secondary_extent_m) ||
      volume.minimum_physical_width_m < 0.0 ||
      volume.minimum_physical_secondary_extent_m < 0.0 ||
      volume.segment_spans != span.segment_spans) {
    return false;
  }
  double previous_station_m{-std::numeric_limits<double>::infinity()};
  const PassageCrossSection* previous_section{nullptr};
  for (const PassageCrossSection& section : volume.cross_sections) {
    if (!validPassageCrossSection(section) || section.station_m <= previous_station_m ||
        section.station_m + kStationToleranceM < volume.begin_station_m ||
        section.station_m > volume.end_station_m + kStationToleranceM ||
        (previous_section != nullptr &&
         (vectorDot(previous_section->lateral_axis, section.lateral_axis) <= 0.0 ||
          vectorDot(previous_section->secondary_axis, section.secondary_axis) <=
              0.0))) {
      return false;
    }
    previous_station_m = section.station_m;
    previous_section = &section;
  }
  return volume.cross_sections.front().station_m <=
             volume.begin_station_m + kStationToleranceM &&
         volume.cross_sections.back().station_m + kStationToleranceM >=
             volume.end_station_m;
}

[[nodiscard]] bool
assignmentStatusValid(const CooperativePassageRouteStatus status) noexcept {
  return status == CooperativePassageRouteStatus::kCentered ||
         status == CooperativePassageRouteStatus::kApplied;
}

[[nodiscard]] std::uint64_t
calculateRouteDecorationsRevision(const RouteDecorations3D& decorations) noexcept {
  const std::uint64_t config_fingerprint =
      passageVolumeConfigFingerprint(decorations.passage_volume_config);
  if (decorations.route_generation == 0U || decorations.geometry_revision == 0U ||
      decorations.physical_route_fingerprint == 0U ||
      decorations.passage_volumes == nullptr ||
      decorations.cooperative_passage_assignments == nullptr ||
      decorations.selected_passage_traversal_ids == nullptr ||
      config_fingerprint == 0U) {
    return 0U;
  }
  DecorationHasher hash;
  hash.text("route_decorations_3d:v1");
  hash.value(decorations.route_generation);
  hash.value(decorations.geometry_revision);
  hash.value(decorations.physical_route_fingerprint);
  hash.value(config_fingerprint);
  hash.value(static_cast<std::uint64_t>(decorations.passage_volumes->size()));
  for (const PassageVolume& volume : *decorations.passage_volumes) {
    hashPassageVolume(hash, volume);
  }
  hash.value(
      static_cast<std::uint64_t>(decorations.cooperative_passage_assignments->size()));
  for (const CooperativePassageAssignment& assignment :
       *decorations.cooperative_passage_assignments) {
    hashPassageAssignment(hash, assignment);
  }
  hash.value(
      static_cast<std::uint64_t>(decorations.selected_passage_traversal_ids->size()));
  for (const PassageTraversalId& traversal_id :
       *decorations.selected_passage_traversal_ids) {
    hash.text(traversal_id.value());
  }
  return hash.result();
}

} // namespace

RouteDecorations3D::RouteDecorations3D(
    const std::uint64_t compiled_route_generation,
    const std::uint64_t compiled_geometry_revision,
    const std::uint64_t compiled_physical_route_fingerprint,
    std::shared_ptr<const std::vector<PassageVolume>> compiled_passage_volumes,
    std::shared_ptr<const std::vector<CooperativePassageAssignment>>
        compiled_cooperative_passage_assignments,
    std::shared_ptr<const std::vector<PassageTraversalId>>
        compiled_selected_passage_traversal_ids,
    PassageVolumeConfig compiled_passage_volume_config)
    : route_generation{compiled_route_generation},
      geometry_revision{compiled_geometry_revision},
      physical_route_fingerprint{compiled_physical_route_fingerprint},
      passage_volumes{std::move(compiled_passage_volumes)},
      cooperative_passage_assignments{
          std::move(compiled_cooperative_passage_assignments)},
      selected_passage_traversal_ids{
          std::move(compiled_selected_passage_traversal_ids)},
      passage_volume_config{compiled_passage_volume_config},
      route_decorations_revision{calculateRouteDecorationsRevision(*this)} {
}

std::uint64_t
routeDecorationsRevision3D(const RouteDecorations3D& decorations) noexcept {
  // Sealed at construction over immutable, non-copyable resources; see
  // compiledTrajectoryRevision3D.
  return decorations.route_decorations_revision;
}

bool routeDecorationsValid3D(const RouteDecorations3D& decorations,
                             const CompiledTrajectory3D& trajectory,
                             const std::uint64_t expected_route_generation) noexcept {
  if (decorations.route_generation == 0U ||
      (expected_route_generation != 0U &&
       decorations.route_generation != expected_route_generation) ||
      !compiledTrajectoryResourcesValid3D(trajectory, decorations.route_generation) ||
      trajectory.compiled_trajectory_revision == 0U ||
      trajectory.compiled_trajectory_revision !=
          compiledTrajectoryRevision3D(trajectory) ||
      decorations.geometry_revision != trajectory.compiled_trajectory_revision ||
      decorations.physical_route_fingerprint != trajectory.physical_route_fingerprint ||
      decorations.passage_volumes == nullptr ||
      decorations.cooperative_passage_assignments == nullptr ||
      decorations.selected_passage_traversal_ids == nullptr ||
      !passageVolumeConfigIsValid(decorations.passage_volume_config) ||
      decorations.route_decorations_revision == 0U ||
      decorations.route_decorations_revision !=
          routeDecorationsRevision3D(decorations)) {
    return false;
  }

  const std::vector<PassageTraversalId>& selected_ids =
      *decorations.selected_passage_traversal_ids;
  for (std::size_t index = 0U; index < selected_ids.size(); ++index) {
    if (selected_ids[index].empty() ||
        std::ranges::find(selected_ids.begin(),
                          selected_ids.begin() + static_cast<std::ptrdiff_t>(index),
                          selected_ids[index]) !=
            selected_ids.begin() + static_cast<std::ptrdiff_t>(index)) {
      return false;
    }
  }

  const std::vector<ConstrainedRouteSpan>& spans = *trajectory.constrained_spans;
  std::vector<PassageTraversalId> expected_ids;
  expected_ids.reserve(spans.size());
  for (const ConstrainedRouteSpan& span : spans) {
    if (span.route_generation != decorations.route_generation) {
      return false;
    }
    if (std::ranges::find(expected_ids, span.passage_traversal_id) ==
        expected_ids.end()) {
      expected_ids.push_back(span.passage_traversal_id);
    }
  }
  if (selected_ids != expected_ids ||
      decorations.passage_volumes->size() != spans.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < spans.size(); ++index) {
    const PassageVolume& volume = (*decorations.passage_volumes)[index];
    if (!validPassageVolume(volume, spans[index], index)) {
      return false;
    }
    for (const PassageCrossSection& section : volume.cross_sections) {
      const RouteSample3D route_sample =
          sampleRoute3DAtStation(*trajectory.route, section.station_m);
      if (vectorDot(route_sample.tangent, section.tangent) <= 0.0) {
        return false;
      }
    }
  }

  const std::vector<CooperativePassageAssignment>& assignments =
      *decorations.cooperative_passage_assignments;
  if (!assignments.empty() && assignments.size() != spans.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < assignments.size(); ++index) {
    const CooperativePassageAssignment& assignment = assignments[index];
    const ConstrainedRouteSpan& span = spans[index];
    const PassageVolume& volume = (*decorations.passage_volumes)[index];
    const double first_lateral_bound_m =
        volume.minimum_lateral_offset_m * static_cast<double>(span.direction_sign) +
        assignment.applied_lateral_offset_m;
    const double second_lateral_bound_m =
        volume.maximum_lateral_offset_m * static_cast<double>(span.direction_sign) +
        assignment.applied_lateral_offset_m;
    if (assignment.route_generation != span.route_generation ||
        assignment.span_index != index ||
        assignment.passage_traversal_id != span.passage_traversal_id ||
        !assignmentStatusValid(assignment.status) ||
        !std::isfinite(assignment.requested_lateral_offset_m) ||
        !std::isfinite(assignment.applied_lateral_offset_m) ||
        !std::isfinite(assignment.desired_center_separation_m) ||
        assignment.physical_width_m != volume.minimum_physical_width_m ||
        assignment.minimum_lateral_offset_m !=
            std::min(first_lateral_bound_m, second_lateral_bound_m) ||
        assignment.maximum_lateral_offset_m !=
            std::max(first_lateral_bound_m, second_lateral_bound_m) ||
        assignment.minimum_secondary_offset_m != volume.minimum_secondary_offset_m ||
        assignment.maximum_secondary_offset_m != volume.maximum_secondary_offset_m ||
        assignment.passage_cross_section_count != volume.cross_sections.size() ||
        assignment.passage_volume_raw_validated != volume.raw_validated) {
      return false;
    }
  }
  return true;
}

} // namespace drone_city_nav
