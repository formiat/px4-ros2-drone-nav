#include "drone_city_nav/passage_volume.hpp"

#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <ranges>
#include <sstream>
#include <string>
#include <unordered_map>

namespace drone_city_nav {
namespace {

constexpr double kAxisEpsilon{1.0e-9};
constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};
constexpr std::size_t kPassageVolumeRegistryMaximumEntries{256U};
constexpr std::size_t kPassageVolumeRegistrySweepInterval{32U};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t byte = 0U; byte < sizeof(value); ++byte) {
    hash ^= (value >> (byte * 8U)) & 0xffU;
    hash *= kFnvPrime;
  }
}

void hashDouble(std::uint64_t& hash, const double value) noexcept {
  hashValue(hash, value == 0.0 ? 0U : std::bit_cast<std::uint64_t>(value));
}

void hashFootprint(std::uint64_t& hash,
                   const SweptFootprintConfig& footprint) noexcept {
  hashDouble(hash, footprint.radius_m);
  hashDouble(hash, footprint.lower_extent_m);
  hashDouble(hash, footprint.upper_extent_m);
  hashValue(hash, static_cast<std::uint64_t>(footprint.perimeter_samples));
  hashValue(hash, static_cast<std::uint64_t>(footprint.radial_rings));
  hashValue(hash, static_cast<std::uint64_t>(footprint.axial_samples));
  hashDouble(hash, footprint.sweep_step_m);
  hashDouble(hash, footprint.safe_clearance_threshold_m);
}

struct CrossSectionFrame {
  Vec3 tangent{};
  Vec3 lateral{};
  Vec3 secondary{};
};

[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] Vec3 cross(const Vec3& first, const Vec3& second) noexcept {
  return Vec3{first.y * second.z - first.z * second.y,
              first.z * second.x - first.x * second.z,
              first.x * second.y - first.y * second.x};
}

[[nodiscard]] double norm(const Vec3& value) noexcept {
  return std::hypot(std::hypot(value.x, value.y), value.z);
}

[[nodiscard]] Vec3 normalized(const Vec3& value) noexcept {
  const double length = norm(value);
  return length > kAxisEpsilon
             ? Vec3{value.x / length, value.y / length, value.z / length}
             : Vec3{};
}

[[nodiscard]] Vec3 scaled(const Vec3& value, const double scale) noexcept {
  return Vec3{scale * value.x, scale * value.y, scale * value.z};
}

[[nodiscard]] Point3 translated(const Point3& point, const Vec3& direction,
                                const double distance_m) noexcept {
  return Point3{point.x + direction.x * distance_m, point.y + direction.y * distance_m,
                point.z + direction.z * distance_m};
}

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool validConfig(const PassageVolumeConfig& config) noexcept {
  return finitePositive(config.cross_section_spacing_m) &&
         finitePositive(config.lateral_probe_step_m) &&
         finitePositive(config.secondary_probe_step_m) &&
         finitePositive(config.maximum_cross_section_probe_m) &&
         std::isfinite(config.minimum_wall_clearance_m) &&
         config.minimum_wall_clearance_m >= 0.0 &&
         std::isfinite(config.flight_envelope.minimum_target_z_m) &&
         std::isfinite(config.flight_envelope.maximum_target_z_m) &&
         config.flight_envelope.maximum_target_z_m >
             config.flight_envelope.minimum_target_z_m &&
         std::isfinite(config.footprint.radius_m) && config.footprint.radius_m >= 0.0 &&
         std::isfinite(config.footprint.lower_extent_m) &&
         config.footprint.lower_extent_m >= 0.0 &&
         std::isfinite(config.footprint.upper_extent_m) &&
         config.footprint.upper_extent_m >= 0.0 &&
         std::isfinite(config.footprint.sweep_step_m) &&
         config.footprint.sweep_step_m > 0.0 &&
         std::isfinite(config.footprint.safe_clearance_threshold_m) &&
         config.footprint.safe_clearance_threshold_m >= 0.0 &&
         config.footprint.axial_samples != 0U;
}

[[nodiscard]] CrossSectionFrame makeFrame(const RouteSample3D& sample,
                                          const Vec3& previous_lateral) noexcept {
  Vec3 tangent = normalized(sample.tangent);
  if (norm(tangent) <= kAxisEpsilon) {
    tangent = Vec3{1.0, 0.0, 0.0};
  }
  const Vec3 world_up{0.0, 0.0, 1.0};
  Vec3 lateral = normalized(cross(world_up, tangent));
  if (norm(lateral) <= kAxisEpsilon) {
    const Vec3 projected_previous{
        previous_lateral.x - dot(previous_lateral, tangent) * tangent.x,
        previous_lateral.y - dot(previous_lateral, tangent) * tangent.y,
        previous_lateral.z - dot(previous_lateral, tangent) * tangent.z,
    };
    lateral = normalized(projected_previous);
  }
  if (norm(lateral) <= kAxisEpsilon) {
    const Vec3 fallback =
        std::abs(tangent.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
    lateral = normalized(Vec3{fallback.x - dot(fallback, tangent) * tangent.x,
                              fallback.y - dot(fallback, tangent) * tangent.y,
                              fallback.z - dot(fallback, tangent) * tangent.z});
  }
  if (norm(previous_lateral) > kAxisEpsilon && dot(lateral, previous_lateral) < 0.0) {
    lateral = scaled(lateral, -1.0);
  }
  Vec3 secondary = normalized(cross(tangent, lateral));
  if (secondary.z < 0.0 && std::hypot(tangent.x, tangent.y) > 1.0e-6) {
    lateral = scaled(lateral, -1.0);
    secondary = scaled(secondary, -1.0);
  }
  return CrossSectionFrame{
      .tangent = tangent,
      .lateral = lateral,
      .secondary = secondary,
  };
}

[[nodiscard]] bool rawFootprintAccepted(const Point3& position,
                                        const OccupancyGrid3D& occupancy,
                                        const PassageVolumeConfig& config) noexcept {
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = nullptr,
      .static_occupancy = std::addressof(occupancy),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = config.footprint,
      .flight_envelope = config.flight_envelope,
  }};
  return oracle.validatePoint(position).clear();
}

[[nodiscard]] double probeFreeDistance(const Point3& center, const Vec3& axis,
                                       const double sign, const double step_m,
                                       const OccupancyGrid3D& occupancy,
                                       const PassageVolumeConfig& config) noexcept {
  double accepted_distance_m = 0.0;
  const std::size_t step_count = static_cast<std::size_t>(
      std::ceil(config.maximum_cross_section_probe_m / step_m));
  for (std::size_t step = 1U; step <= step_count; ++step) {
    const double distance_m = std::min(config.maximum_cross_section_probe_m,
                                       step_m * static_cast<double>(step));
    if (!rawFootprintAccepted(translated(center, axis, sign * distance_m), occupancy,
                              config)) {
      break;
    }
    accepted_distance_m = distance_m;
    if (distance_m >= config.maximum_cross_section_probe_m - 1.0e-9) {
      break;
    }
  }
  return accepted_distance_m;
}

[[nodiscard]] double conservativeMinimumOffset(const double raw_minimum_offset_m,
                                               const double wall_clearance_m) noexcept {
  return std::min(0.0, raw_minimum_offset_m + wall_clearance_m);
}

[[nodiscard]] double conservativeMaximumOffset(const double raw_maximum_offset_m,
                                               const double wall_clearance_m) noexcept {
  return std::max(0.0, raw_maximum_offset_m - wall_clearance_m);
}

[[nodiscard]] PassageCrossSection
deriveCrossSection(const RouteSample3D& sample, const CrossSectionFrame& frame,
                   const OccupancyGrid3D& occupancy,
                   const PassageVolumeConfig& config) noexcept {
  PassageCrossSection result{
      .station_m = sample.station_m,
      .center = sample.position,
      .tangent = frame.tangent,
      .lateral_axis = frame.lateral,
      .secondary_axis = frame.secondary,
  };
  if (!rawFootprintAccepted(sample.position, occupancy, config)) {
    return result;
  }
  result.minimum_lateral_offset_m =
      -probeFreeDistance(sample.position, frame.lateral, -1.0,
                         config.lateral_probe_step_m, occupancy, config);
  result.maximum_lateral_offset_m =
      probeFreeDistance(sample.position, frame.lateral, 1.0,
                        config.lateral_probe_step_m, occupancy, config);
  result.minimum_secondary_offset_m =
      -probeFreeDistance(sample.position, frame.secondary, -1.0,
                         config.secondary_probe_step_m, occupancy, config);
  result.maximum_secondary_offset_m =
      probeFreeDistance(sample.position, frame.secondary, 1.0,
                        config.secondary_probe_step_m, occupancy, config);
  result.raw_validated = true;
  return result;
}

[[nodiscard]] PassageVolume
derivePassageVolume(const std::span<const RouteSample3D> route,
                    const ConstrainedRouteSpan& span, const std::size_t span_index,
                    const OccupancyGrid3D& occupancy,
                    const PassageVolumeConfig& config) {
  PassageVolume result{
      .passage_traversal_id = span.passage_traversal_id,
      .span_index = span_index,
      .begin_station_m = span.begin_station_m,
      .end_station_m = span.end_station_m,
      .minimum_lateral_offset_m = -std::numeric_limits<double>::infinity(),
      .maximum_lateral_offset_m = std::numeric_limits<double>::infinity(),
      .minimum_secondary_offset_m = -std::numeric_limits<double>::infinity(),
      .maximum_secondary_offset_m = std::numeric_limits<double>::infinity(),
      .minimum_physical_width_m = std::numeric_limits<double>::infinity(),
      .minimum_physical_secondary_extent_m = std::numeric_limits<double>::infinity(),
      .cross_sections = {},
      .raw_validated = false,
      .segment_spans = span.segment_spans,
  };
  if (route.size() < 2U || span.passage_traversal_id.empty() ||
      !(span.end_station_m > span.begin_station_m)) {
    return result;
  }
  const double length_m = span.end_station_m - span.begin_station_m;
  const std::size_t interval_count = std::max<std::size_t>(
      1U,
      static_cast<std::size_t>(std::ceil(length_m / config.cross_section_spacing_m)));
  result.cross_sections.reserve(interval_count + 1U);
  Vec3 previous_lateral{};
  for (std::size_t index = 0U; index <= interval_count; ++index) {
    const double ratio =
        static_cast<double>(index) / static_cast<double>(interval_count);
    const RouteSample3D sample = sampleRoute3DAtStation(
        route, std::lerp(span.begin_station_m, span.end_station_m, ratio));
    const CrossSectionFrame frame = makeFrame(sample, previous_lateral);
    previous_lateral = frame.lateral;
    PassageCrossSection section = deriveCrossSection(sample, frame, occupancy, config);
    if (!section.raw_validated) {
      result.cross_sections.push_back(section);
      return result;
    }
    result.minimum_lateral_offset_m =
        std::max(result.minimum_lateral_offset_m,
                 conservativeMinimumOffset(section.minimum_lateral_offset_m,
                                           config.minimum_wall_clearance_m));
    result.maximum_lateral_offset_m =
        std::min(result.maximum_lateral_offset_m,
                 conservativeMaximumOffset(section.maximum_lateral_offset_m,
                                           config.minimum_wall_clearance_m));
    result.minimum_secondary_offset_m =
        std::max(result.minimum_secondary_offset_m,
                 conservativeMinimumOffset(section.minimum_secondary_offset_m,
                                           config.minimum_wall_clearance_m));
    result.maximum_secondary_offset_m =
        std::min(result.maximum_secondary_offset_m,
                 conservativeMaximumOffset(section.maximum_secondary_offset_m,
                                           config.minimum_wall_clearance_m));
    result.minimum_physical_width_m =
        std::min(result.minimum_physical_width_m,
                 section.usableWidthM() + 2.0 * config.footprint.radius_m);
    result.minimum_physical_secondary_extent_m =
        std::min(result.minimum_physical_secondary_extent_m,
                 section.usableSecondaryExtentM() + config.footprint.lower_extent_m +
                     config.footprint.upper_extent_m + 2.0 * config.footprint.radius_m);
    result.cross_sections.push_back(section);
  }
  result.raw_validated =
      !result.cross_sections.empty() &&
      result.maximum_lateral_offset_m >= result.minimum_lateral_offset_m &&
      result.maximum_secondary_offset_m >= result.minimum_secondary_offset_m;
  return result;
}

[[nodiscard]] const PassageVolume*
matchingVolume(const std::span<const PassageVolume> passage_volumes,
               const ConstrainedRouteSpan& span,
               const std::size_t span_index) noexcept {
  const auto found =
      std::ranges::find_if(passage_volumes, [&](const PassageVolume& volume) {
        return volume.span_index == span_index &&
               volume.passage_traversal_id == span.passage_traversal_id;
      });
  return found == passage_volumes.end() ? nullptr : &*found;
}

[[nodiscard]] const RouteEnvelopeSample*
nearestEnvelope(const std::span<const RouteEnvelopeSample> envelope,
                const double station_m) noexcept {
  if (envelope.empty()) {
    return nullptr;
  }
  const auto upper = std::ranges::lower_bound(envelope, station_m, {},
                                              &RouteEnvelopeSample::station_m);
  if (upper == envelope.begin()) {
    return &envelope.front();
  }
  if (upper == envelope.end()) {
    return &envelope.back();
  }
  const RouteEnvelopeSample& previous = *std::prev(upper);
  return station_m - previous.station_m <= upper->station_m - station_m ? &previous
                                                                        : &*upper;
}

[[nodiscard]] std::pair<double, double>
verticalCenterRange(const PassageCrossSection& section) noexcept {
  double minimum_z_m = std::numeric_limits<double>::infinity();
  double maximum_z_m = -std::numeric_limits<double>::infinity();
  for (const double lateral_m :
       {section.minimum_lateral_offset_m, section.maximum_lateral_offset_m}) {
    for (const double secondary_m :
         {section.minimum_secondary_offset_m, section.maximum_secondary_offset_m}) {
      const double z_m = section.center.z + section.lateral_axis.z * lateral_m +
                         section.secondary_axis.z * secondary_m;
      minimum_z_m = std::min(minimum_z_m, z_m);
      maximum_z_m = std::max(maximum_z_m, z_m);
    }
  }
  return {minimum_z_m, maximum_z_m};
}

[[nodiscard]] std::string
resourceKey(const std::span<const RouteSample3D> route,
            const std::span<const ConstrainedRouteSpan> constrained_spans,
            const OccupancyGrid3D& occupancy,
            const std::uint64_t occupancy_content_fingerprint,
            const PassageVolumeConfig& config) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << occupancy.fingerprint() << '|' << occupancy_content_fingerprint << '|'
         << routeFingerprint(route) << '|' << route.size() << '|' << std::hexfloat;
  for (const RouteSample3D& sample : route) {
    stream << sample.position.x << '|' << sample.position.y << '|' << sample.position.z
           << '|' << sample.tangent.x << '|' << sample.tangent.y << '|'
           << sample.tangent.z << '|' << sample.station_m << '|';
  }
  stream << config.cross_section_spacing_m << '|' << config.lateral_probe_step_m << '|'
         << config.secondary_probe_step_m << '|' << config.maximum_cross_section_probe_m
         << '|' << config.minimum_wall_clearance_m << '|'
         << config.flight_envelope.minimum_target_z_m << '|'
         << config.flight_envelope.maximum_target_z_m << '|'
         << config.footprint.radius_m << '|' << config.footprint.lower_extent_m << '|'
         << config.footprint.upper_extent_m << '|' << config.footprint.perimeter_samples
         << '|' << config.footprint.radial_rings << '|'
         << config.footprint.axial_samples << '|' << config.footprint.sweep_step_m
         << '|' << config.footprint.safe_clearance_threshold_m << '|'
         << std::defaultfloat << constrained_spans.size();
  for (const ConstrainedRouteSpan& span : constrained_spans) {
    stream << '|' << span.passage_traversal_id.value().size() << ':'
           << span.passage_traversal_id.value() << '|' << std::hexfloat
           << span.begin_station_m << '|' << span.end_station_m << '|'
           << std::defaultfloat << span.segment_spans.size() << '|' << std::hexfloat;
    for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
      stream << '|' << segment.passage_segment_id.value().size() << ':'
             << segment.passage_segment_id.value() << '|' << segment.begin_station_m
             << '|' << segment.end_station_m;
    }
  }
  return stream.str();
}

} // namespace

bool passageVolumeConfigIsValid(const PassageVolumeConfig& config) noexcept {
  return validConfig(config);
}

bool samePassageVolumeConfig(const PassageVolumeConfig& first,
                             const PassageVolumeConfig& second) noexcept {
  return first.cross_section_spacing_m == second.cross_section_spacing_m &&
         first.lateral_probe_step_m == second.lateral_probe_step_m &&
         first.secondary_probe_step_m == second.secondary_probe_step_m &&
         first.maximum_cross_section_probe_m == second.maximum_cross_section_probe_m &&
         first.minimum_wall_clearance_m == second.minimum_wall_clearance_m &&
         first.flight_envelope.minimum_target_z_m ==
             second.flight_envelope.minimum_target_z_m &&
         first.flight_envelope.maximum_target_z_m ==
             second.flight_envelope.maximum_target_z_m &&
         first.footprint.radius_m == second.footprint.radius_m &&
         first.footprint.lower_extent_m == second.footprint.lower_extent_m &&
         first.footprint.upper_extent_m == second.footprint.upper_extent_m &&
         first.footprint.perimeter_samples == second.footprint.perimeter_samples &&
         first.footprint.radial_rings == second.footprint.radial_rings &&
         first.footprint.axial_samples == second.footprint.axial_samples &&
         first.footprint.sweep_step_m == second.footprint.sweep_step_m &&
         first.footprint.safe_clearance_threshold_m ==
             second.footprint.safe_clearance_threshold_m;
}

std::uint64_t
passageVolumeConfigFingerprint(const PassageVolumeConfig& config) noexcept {
  if (!validConfig(config)) {
    return 0U;
  }
  std::uint64_t hash{kFnvOffset};
  hashDouble(hash, config.cross_section_spacing_m);
  hashDouble(hash, config.lateral_probe_step_m);
  hashDouble(hash, config.secondary_probe_step_m);
  hashDouble(hash, config.maximum_cross_section_probe_m);
  hashDouble(hash, config.minimum_wall_clearance_m);
  hashDouble(hash, config.flight_envelope.minimum_target_z_m);
  hashDouble(hash, config.flight_envelope.maximum_target_z_m);
  hashFootprint(hash, config.footprint);
  return hash == 0U ? 1U : hash;
}

std::vector<PassageVolume>
derivePassageVolumes(const std::span<const RouteSample3D> route,
                     const std::span<const ConstrainedRouteSpan> constrained_spans,
                     const OccupancyGrid3D& occupancy,
                     const PassageVolumeConfig& config) {
  std::vector<PassageVolume> result;
  if (!validConfig(config) || route.size() < 2U) {
    return result;
  }
  result.reserve(constrained_spans.size());
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    result.push_back(
        derivePassageVolume(route, constrained_spans[index], index, occupancy, config));
  }
  return result;
}

PassageVolumeResource acquireDerivedPassageVolumes(
    const std::span<const RouteSample3D> route,
    const std::span<const ConstrainedRouteSpan> constrained_spans,
    const OccupancyGrid3D& occupancy, const std::uint64_t occupancy_content_fingerprint,
    const PassageVolumeConfig& config) {
  if (!validConfig(config)) {
    return {};
  }
  using VolumeVector = std::vector<PassageVolume>;
  const std::optional<std::uint64_t> cached_content_fingerprint =
      occupancy.cachedContentFingerprint();
  if (occupancy_content_fingerprint == 0U || !cached_content_fingerprint.has_value() ||
      *cached_content_fingerprint != occupancy_content_fingerprint) {
    return PassageVolumeResource{
        .volumes = std::make_shared<const VolumeVector>(
            derivePassageVolumes(route, constrained_spans, occupancy, config)),
        .shared_resource_reused = false,
    };
  }
  const std::string key = resourceKey(route, constrained_spans, occupancy,
                                      occupancy_content_fingerprint, config);
  static std::mutex registry_mutex;
  static std::unordered_map<std::string, std::weak_ptr<const VolumeVector>> registry;
  static std::size_t acquisitions_since_sweep{0U};
  const std::scoped_lock registry_lock{registry_mutex};
  ++acquisitions_since_sweep;
  if (acquisitions_since_sweep >= kPassageVolumeRegistrySweepInterval ||
      registry.size() >= kPassageVolumeRegistryMaximumEntries) {
    std::erase_if(registry, [](const auto& entry) { return entry.second.expired(); });
    acquisitions_since_sweep = 0U;
  }
  if (const auto found = registry.find(key); found != registry.end()) {
    if (std::shared_ptr<const VolumeVector> existing = found->second.lock()) {
      return PassageVolumeResource{
          .volumes = std::move(existing),
          .shared_resource_reused = true,
      };
    }
    registry.erase(found);
  }
  auto volumes = std::make_shared<const VolumeVector>(
      derivePassageVolumes(route, constrained_spans, occupancy, config));
  // Live resources are never evicted: removing one would defeat exact sharing
  // for a route still owned by an active world. Once the hard bound is full,
  // derive an uncached resource until owners release entries for the next sweep.
  if (registry.size() < kPassageVolumeRegistryMaximumEntries) {
    registry.emplace(key, volumes);
  }
  return PassageVolumeResource{
      .volumes = std::move(volumes),
      .shared_resource_reused = false,
  };
}

std::size_t
projectPassageVolumeEnvelopes(const std::span<ConstrainedRouteSpan> constrained_spans,
                              const std::span<const PassageVolume> passage_volumes,
                              const SweptFootprintConfig& footprint) noexcept {
  std::size_t projected_count = 0U;
  for (std::size_t index = 0U; index < constrained_spans.size(); ++index) {
    ConstrainedRouteSpan& span = constrained_spans[index];
    const PassageVolume* const volume = matchingVolume(passage_volumes, span, index);
    if (volume == nullptr || !volume->raw_validated || volume->cross_sections.empty()) {
      continue;
    }
    const std::vector<RouteEnvelopeSample> original_envelope = span.envelope;
    span.envelope.clear();
    span.envelope.reserve(volume->cross_sections.size());
    for (const PassageCrossSection& section : volume->cross_sections) {
      const RouteEnvelopeSample* const original =
          nearestEnvelope(original_envelope, section.station_m);
      const auto [minimum_center_z_m, maximum_center_z_m] =
          verticalCenterRange(section);
      const double minimum_center_clearance_m =
          std::max(0.0, std::min({-section.minimum_lateral_offset_m,
                                  section.maximum_lateral_offset_m,
                                  -section.minimum_secondary_offset_m,
                                  section.maximum_secondary_offset_m}));
      span.envelope.push_back(RouteEnvelopeSample{
          .station_m = section.station_m,
          .lateral_free_left_m = footprint.radius_m + section.maximum_lateral_offset_m,
          .lateral_free_right_m = footprint.radius_m - section.minimum_lateral_offset_m,
          .min_z_m = minimum_center_z_m - footprint.lower_extent_m,
          .max_z_m = maximum_center_z_m + footprint.upper_extent_m,
          .minimum_clearance_m = footprint.radius_m + minimum_center_clearance_m,
          .reference_z_m = section.center.z,
          .reference_speed_mps =
              original != nullptr ? original->reference_speed_mps : 0.0,
      });
    }
    ++projected_count;
  }
  return projected_count;
}

const PassageCrossSection* nearestPassageCrossSection(const PassageVolume& volume,
                                                      const double station_m) noexcept {
  if (volume.cross_sections.empty() || !std::isfinite(station_m)) {
    return nullptr;
  }
  const auto upper = std::ranges::lower_bound(volume.cross_sections, station_m, {},
                                              &PassageCrossSection::station_m);
  if (upper == volume.cross_sections.begin()) {
    return &volume.cross_sections.front();
  }
  if (upper == volume.cross_sections.end()) {
    return &volume.cross_sections.back();
  }
  const PassageCrossSection& previous = *std::prev(upper);
  return station_m - previous.station_m <= upper->station_m - station_m ? &previous
                                                                        : &*upper;
}

} // namespace drone_city_nav
