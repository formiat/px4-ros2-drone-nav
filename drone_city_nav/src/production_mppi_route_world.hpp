#pragma once

#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/world_generation.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>

namespace drone_city_nav {

struct PersistentPlannerWorld3D;
struct ProductionMppiRawWorld3D;

enum class ProductionWorldGenerationStatus : std::uint8_t {
  kCoherent,
  kInvalidGeneration,
  kMissingEsdfResources,
  kEsdfRevisionMismatch,
  kRawVersionMismatch,
  kObservedOwnerMismatch,
  kObservedEsdfCoverageMismatch,
};

[[nodiscard]] ProductionWorldGenerationStatus
assessProductionWorldGeneration(const WorldSnapshot3D& world) noexcept;
[[nodiscard]] bool
productionWorldGenerationCoherent(const WorldSnapshot3D& world) noexcept;
[[nodiscard]] std::string_view
productionWorldGenerationStatusName(ProductionWorldGenerationStatus status) noexcept;

[[nodiscard]] NavigationWorldCertificate3D
navigationWorldCertificate3D(const WorldSnapshot3D& world) noexcept;

// Initial recovery and physical-collision searches need current hard occupancy;
// choosing it never schedules a search by itself.
[[nodiscard]] bool
routeSearchRequiresLatestRawOverlay3D(RouteReleaseReason3D reason) noexcept;

// Captures an exact raw occupancy snapshot for one route-search transaction.
// The overlay has no delta-lineage assumption relative to the persistent
// planner's previous transaction, so it intentionally requests a full reset.
[[nodiscard]] std::shared_ptr<const PersistentPlannerWorld3D>
captureObservedRouteSearchWorld3D(
    const ProductionMppiRawWorld3D& raw_world,
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact);

[[nodiscard]] std::shared_ptr<const PersistentPlannerWorld3D> routeSearchPlannerWorld3D(
    const std::shared_ptr<const PersistentPlannerWorld3D>& resident_world,
    const std::shared_ptr<const PersistentPlannerWorld3D>& raw_overlay,
    bool use_raw_overlay) noexcept;

} // namespace drone_city_nav
