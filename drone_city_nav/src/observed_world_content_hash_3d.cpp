#include "drone_city_nav/observed_world_content_hash_3d.hpp"

#include "drone_city_nav/launch_support_contact_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <optional>
#include <ranges>
#include <span>

namespace drone_city_nav::observed_world_content_3d {
namespace {

// Conservative-containment slack for published footprint geometry. Content
// identity is exact; only the containment predicate admits this tolerance.
inline constexpr double kGeometryTolerance{1.0e-4};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

} // namespace

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t byte = 0U; byte < sizeof(value); ++byte) {
    hash ^= (value >> (byte * 8U)) & 0xffU;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept {
  return value == 0.0 ? 0U : std::bit_cast<std::uint64_t>(value);
}

void hashPoint(std::uint64_t& hash, const Point3& point) noexcept {
  hashValue(hash, canonicalDoubleBits(point.x));
  hashValue(hash, canonicalDoubleBits(point.y));
  hashValue(hash, canonicalDoubleBits(point.z));
}

void hashAxis(std::uint64_t& hash, const FootprintBodyAxis& axis) noexcept {
  hashValue(hash, canonicalDoubleBits(axis.x));
  hashValue(hash, canonicalDoubleBits(axis.y));
  hashValue(hash, canonicalDoubleBits(axis.z));
}

[[nodiscard]] bool footprintValid(const SweptFootprintConfig& footprint) noexcept {
  return std::isfinite(footprint.radius_m) && footprint.radius_m >= 0.0 &&
         std::isfinite(footprint.body_radius_m) && footprint.body_radius_m >= 0.0 &&
         std::isfinite(footprint.body_lower_extent_m) &&
         footprint.body_lower_extent_m >= 0.0 &&
         std::isfinite(footprint.body_upper_extent_m) &&
         footprint.body_upper_extent_m >= 0.0 &&
         std::isfinite(footprint.lower_extent_m) && footprint.lower_extent_m >= 0.0 &&
         std::isfinite(footprint.upper_extent_m) && footprint.upper_extent_m >= 0.0 &&
         std::isfinite(footprint.sweep_step_m) && footprint.sweep_step_m > 0.0 &&
         std::isfinite(footprint.safe_clearance_threshold_m) &&
         footprint.safe_clearance_threshold_m >= 0.0 && footprint.axial_samples != 0U;
}

[[nodiscard]] bool sameFootprintConfig(const SweptFootprintConfig& first,
                                       const SweptFootprintConfig& second) noexcept {
  return first.radius_m == second.radius_m &&
         first.body_radius_m == second.body_radius_m &&
         first.body_lower_extent_m == second.body_lower_extent_m &&
         first.body_upper_extent_m == second.body_upper_extent_m &&
         first.lower_extent_m == second.lower_extent_m &&
         first.upper_extent_m == second.upper_extent_m &&
         first.perimeter_samples == second.perimeter_samples &&
         first.radial_rings == second.radial_rings &&
         first.axial_samples == second.axial_samples &&
         first.sweep_step_m == second.sweep_step_m &&
         first.safe_clearance_threshold_m == second.safe_clearance_threshold_m;
}

[[nodiscard]] bool
footprintConservativelyContains(const SweptFootprintConfig& outer,
                                const SweptFootprintConfig& inner) noexcept {
  return footprintValid(outer) && footprintValid(inner) &&
         outer.radius_m + kGeometryTolerance >= inner.radius_m &&
         outer.body_radius_m + kGeometryTolerance >= inner.body_radius_m &&
         outer.body_lower_extent_m + kGeometryTolerance >= inner.body_lower_extent_m &&
         outer.body_upper_extent_m + kGeometryTolerance >= inner.body_upper_extent_m &&
         outer.lower_extent_m + kGeometryTolerance >= inner.lower_extent_m &&
         outer.upper_extent_m + kGeometryTolerance >= inner.upper_extent_m &&
         outer.perimeter_samples >= inner.perimeter_samples &&
         outer.radial_rings >= inner.radial_rings &&
         outer.axial_samples >= inner.axial_samples &&
         outer.sweep_step_m <= inner.sweep_step_m + kGeometryTolerance &&
         outer.safe_clearance_threshold_m + kGeometryTolerance >=
             inner.safe_clearance_threshold_m;
}

[[nodiscard]] bool
sameFreeSpaceSeed(const ProprioceptiveFreeSpaceSeed3D& first,
                  const ProprioceptiveFreeSpaceSeed3D& second) noexcept {
  return first.position.x == second.position.x &&
         first.position.y == second.position.y &&
         first.position.z == second.position.z &&
         first.body_axis.x == second.body_axis.x &&
         first.body_axis.y == second.body_axis.y &&
         first.body_axis.z == second.body_axis.z &&
         sameFootprintConfig(first.footprint, second.footprint) &&
         first.contact_tolerance_m == second.contact_tolerance_m &&
         first.departure_chain.size() == second.departure_chain.size() &&
         std::equal(first.departure_chain.begin(), first.departure_chain.end(),
                    second.departure_chain.begin(),
                    [](const Point3& left, const Point3& right) noexcept {
                      return left.x == right.x && left.y == right.y &&
                             left.z == right.z;
                    });
}

[[nodiscard]] bool sameAxisAlignedBox(const AxisAlignedBox3D& first,
                                      const AxisAlignedBox3D& second) noexcept {
  return first.minimum.x == second.minimum.x && first.minimum.y == second.minimum.y &&
         first.minimum.z == second.minimum.z && first.maximum.x == second.maximum.x &&
         first.maximum.y == second.maximum.y && first.maximum.z == second.maximum.z;
}

[[nodiscard]] bool
sameCanonicalLaunchSupport(const LaunchSupportContact3D& candidate,
                           const LaunchSupportContact3D& canonical) noexcept {
  return launchSupportContactValid3D(candidate) &&
         launchSupportContactValid3D(canonical) &&
         sameFreeSpaceSeed(candidate.seed, canonical.seed) &&
         candidate.contact_cells.size() == canonical.contact_cells.size() &&
         std::ranges::equal(candidate.contact_cells, canonical.contact_cells,
                            sameAxisAlignedBox) &&
         candidate.occupied_evidence_cells == canonical.occupied_evidence_cells &&
         candidate.evidence_source == canonical.evidence_source &&
         candidate.maximum_lateral_departure_m ==
             canonical.maximum_lateral_departure_m &&
         candidate.maximum_axial_settling_m == canonical.maximum_axial_settling_m;
}

[[nodiscard]] bool
launchSupportMatchesOwnedOccupancy(const LaunchSupportContact3D& support,
                                   const ObservedOccupancyGrid3D& occupancy) {
  if (!launchSupportContactValid3D(support)) {
    return false;
  }
  if (support.evidence_source == LaunchSupportEvidenceSource::kObservedOccupancy) {
    const std::optional<LaunchSupportContact3D> canonical =
        detectLaunchSupportContact3D(occupancy, support.seed);
    return canonical.has_value() && sameCanonicalLaunchSupport(support, *canonical);
  }
  const LaunchSupportContact3D canonical =
      makeVehicleLandedSupportContact3D(occupancy.bounds(), support.seed);
  return sameCanonicalLaunchSupport(support, canonical);
}

void hashFootprint(std::uint64_t& hash,
                   const SweptFootprintConfig& footprint) noexcept {
  hashValue(hash, canonicalDoubleBits(footprint.radius_m));
  hashValue(hash, canonicalDoubleBits(footprint.body_radius_m));
  hashValue(hash, canonicalDoubleBits(footprint.body_lower_extent_m));
  hashValue(hash, canonicalDoubleBits(footprint.body_upper_extent_m));
  hashValue(hash, canonicalDoubleBits(footprint.lower_extent_m));
  hashValue(hash, canonicalDoubleBits(footprint.upper_extent_m));
  hashValue(hash, static_cast<std::uint64_t>(footprint.perimeter_samples));
  hashValue(hash, static_cast<std::uint64_t>(footprint.radial_rings));
  hashValue(hash, static_cast<std::uint64_t>(footprint.axial_samples));
  hashValue(hash, canonicalDoubleBits(footprint.sweep_step_m));
  hashValue(hash, canonicalDoubleBits(footprint.safe_clearance_threshold_m));
}

[[nodiscard]] bool hashProprioceptiveFreeSpaceSeed(
    std::uint64_t& hash,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed) noexcept {
  hashValue(hash, free_space_seed == nullptr ? 0U : 1U);
  if (free_space_seed == nullptr) {
    return true;
  }
  const double axis_norm =
      std::hypot(std::hypot(free_space_seed->body_axis.x, free_space_seed->body_axis.y),
                 free_space_seed->body_axis.z);
  if (!finitePoint(free_space_seed->position) || !std::isfinite(axis_norm) ||
      std::abs(axis_norm - 1.0) > 1.0e-6 ||
      !std::isfinite(free_space_seed->body_axis.x) ||
      !std::isfinite(free_space_seed->body_axis.y) ||
      !std::isfinite(free_space_seed->body_axis.z) ||
      !footprintValid(free_space_seed->footprint) ||
      !std::isfinite(free_space_seed->contact_tolerance_m) ||
      free_space_seed->contact_tolerance_m < 0.0) {
    return false;
  }
  if (!std::ranges::all_of(free_space_seed->departure_chain, finitePoint)) {
    return false;
  }
  hashPoint(hash, free_space_seed->position);
  hashAxis(hash, free_space_seed->body_axis);
  hashFootprint(hash, free_space_seed->footprint);
  hashValue(hash, canonicalDoubleBits(free_space_seed->contact_tolerance_m));
  hashValue(hash, static_cast<std::uint64_t>(free_space_seed->departure_chain.size()));
  for (const Point3& pose : free_space_seed->departure_chain) {
    hashPoint(hash, pose);
  }
  return true;
}

[[nodiscard]] bool hashLaunchSupportContact(
    std::uint64_t& hash,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  hashValue(hash, launch_support_contact == nullptr ? 0U : 1U);
  if (launch_support_contact == nullptr) {
    return true;
  }
  const LaunchSupportContact3D& support = *launch_support_contact;
  if (!launchSupportContactValid3D(support)) {
    return false;
  }
  hashPoint(hash, support.seed.position);
  hashAxis(hash, support.seed.body_axis);
  hashFootprint(hash, support.seed.footprint);
  hashValue(hash, static_cast<std::uint64_t>(support.contact_cells.size()));
  for (const AxisAlignedBox3D& cell : support.contact_cells) {
    if (!finitePoint(cell.minimum) || !finitePoint(cell.maximum)) {
      return false;
    }
    hashPoint(hash, cell.minimum);
    hashPoint(hash, cell.maximum);
  }
  hashValue(hash, static_cast<std::uint64_t>(support.occupied_evidence_cells));
  hashValue(hash, static_cast<std::uint64_t>(support.evidence_source));
  hashValue(hash, canonicalDoubleBits(support.maximum_lateral_departure_m));
  hashValue(hash, canonicalDoubleBits(support.minimum_axial_departure_m));
  hashValue(hash, canonicalDoubleBits(support.maximum_axial_settling_m));
  return true;
}

[[nodiscard]] std::uint64_t validationPolicyFingerprint(
    const SweptFootprintConfig& footprint,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  if (!footprintValid(footprint)) {
    return 0U;
  }

  std::uint64_t hash{kFnvOffset};
  hashFootprint(hash, footprint);
  if (!hashLaunchSupportContact(hash, launch_support_contact)) {
    return 0U;
  }
  return hash == 0U ? 1U : hash;
}

void hashGridBounds(std::uint64_t& hash, const GridBounds3D& bounds) noexcept {
  hashValue(hash, canonicalDoubleBits(bounds.origin_x));
  hashValue(hash, canonicalDoubleBits(bounds.origin_y));
  hashValue(hash, canonicalDoubleBits(bounds.origin_z));
  hashValue(hash, canonicalDoubleBits(bounds.resolution_m));
  hashValue(hash,
            static_cast<std::uint64_t>(static_cast<std::int64_t>(bounds.width_cells)));
  hashValue(hash,
            static_cast<std::uint64_t>(static_cast<std::int64_t>(bounds.height_cells)));
  hashValue(hash,
            static_cast<std::uint64_t>(static_cast<std::int64_t>(bounds.depth_cells)));
}

void hashObservedOccupancy(std::uint64_t& hash,
                           const ObservedOccupancyGrid3D& occupancy) {
  hashGridBounds(hash, occupancy.bounds());
  std::vector<std::pair<OccupancyChunkIndex3D, const ObservedOccupancyChunk3D*>> chunks;
  chunks.reserve(occupancy.chunks().size());
  for (const auto& [index, storage] : occupancy.chunks()) {
    chunks.emplace_back(index, &storage.get());
  }
  std::ranges::sort(chunks, {}, [](const auto& entry) {
    return std::tuple{entry.first.x, entry.first.y, entry.first.z};
  });
  hashValue(hash, static_cast<std::uint64_t>(chunks.size()));
  for (const auto& [index, chunk] : chunks) {
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(index.x)));
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(index.y)));
    hashValue(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(index.z)));
    for (const std::uint64_t word : chunk->observed) {
      hashValue(hash, word);
    }
    for (const std::uint64_t word : chunk->occupied) {
      hashValue(hash, word);
    }
  }
}

[[nodiscard]] std::uint64_t
observedOccupancyContentFingerprint(const ObservedOccupancyGrid3D& occupancy) {
  std::uint64_t hash{kFnvOffset};
  hashObservedOccupancy(hash, occupancy);
  return hash;
}

[[nodiscard]] std::uint64_t observedWorldContentFingerprintFromObservation(
    const std::uint64_t observation_content_fingerprint,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) {
  std::uint64_t hash{observation_content_fingerprint};
  if (!hashProprioceptiveFreeSpaceSeed(hash, free_space_seed) ||
      !hashLaunchSupportContact(hash, launch_support_contact)) {
    return 0U;
  }
  return hash == 0U ? 1U : hash;
}

} // namespace drone_city_nav::observed_world_content_3d
