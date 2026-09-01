#pragma once

#include "drone_city_nav/esdf_grid_3d.hpp"
#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"

#include <cstdint>

namespace drone_city_nav::observed_world_content_3d {

// Canonical content identity of an observed world: the same observation always
// hashes the same way, and two observations compare equal only when every
// published field matches. Owning this in the world layer lets world evidence
// seal its own identity instead of borrowing an execution-private helper.
inline constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
inline constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept;

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept;

void hashPoint(std::uint64_t& hash, const Point3& point) noexcept;

void hashAxis(std::uint64_t& hash, const FootprintBodyAxis& axis) noexcept;

[[nodiscard]] bool footprintValid(const SweptFootprintConfig& footprint) noexcept;

[[nodiscard]] bool sameFootprintConfig(const SweptFootprintConfig& first,
                                       const SweptFootprintConfig& second) noexcept;

[[nodiscard]] bool
footprintConservativelyContains(const SweptFootprintConfig& outer,
                                const SweptFootprintConfig& inner) noexcept;

[[nodiscard]] bool
sameFreeSpaceSeed(const ProprioceptiveFreeSpaceSeed3D& first,
                  const ProprioceptiveFreeSpaceSeed3D& second) noexcept;

[[nodiscard]] bool sameAxisAlignedBox(const AxisAlignedBox3D& first,
                                      const AxisAlignedBox3D& second) noexcept;

[[nodiscard]] bool
sameCanonicalLaunchSupport(const LaunchSupportContact3D& candidate,
                           const LaunchSupportContact3D& canonical) noexcept;

[[nodiscard]] bool
launchSupportMatchesOwnedOccupancy(const LaunchSupportContact3D& support,
                                   const ObservedOccupancyGrid3D& occupancy);

void hashFootprint(std::uint64_t& hash, const SweptFootprintConfig& footprint) noexcept;

[[nodiscard]] bool hashProprioceptiveFreeSpaceSeed(
    std::uint64_t& hash,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed) noexcept;

[[nodiscard]] bool hashLaunchSupportContact(
    std::uint64_t& hash,
    const LaunchSupportContact3D* const launch_support_contact) noexcept;

[[nodiscard]] std::uint64_t validationPolicyFingerprint(
    const SweptFootprintConfig& footprint,
    const LaunchSupportContact3D* const launch_support_contact) noexcept;

void hashGridBounds(std::uint64_t& hash, const GridBounds3D& bounds) noexcept;

void hashObservedOccupancy(std::uint64_t& hash,
                           const ObservedOccupancyGrid3D& occupancy);

[[nodiscard]] std::uint64_t
observedOccupancyContentFingerprint(const ObservedOccupancyGrid3D& occupancy);

[[nodiscard]] std::uint64_t observedWorldContentFingerprintFromObservation(
    const std::uint64_t observation_content_fingerprint,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact);

} // namespace drone_city_nav::observed_world_content_3d
