#pragma once

#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>

namespace drone_city_nav {

struct PersistentPlannerWorld3D;
struct ProductionMppiPreparedEsdf;
struct ProductionMppiRawWorld3D;

enum class ProductionWorldGenerationStatus : std::uint8_t {
  kCoherent,
  kInvalidGeneration,
  kMissingEsdfResources,
  kEsdfRevisionMismatch,
  kRawVersionMismatch,
  kObservedOwnerMismatch,
  kObservedPlannerWorldMismatch,
  kObservedEsdfCoverageMismatch,
};

[[nodiscard]] ProductionWorldGenerationStatus
assessProductionWorldGeneration(const ProductionMppiPreparedEsdf& world) noexcept;
[[nodiscard]] bool
productionWorldGenerationCoherent(const ProductionMppiPreparedEsdf& world) noexcept;
[[nodiscard]] std::string_view
productionWorldGenerationStatusName(ProductionWorldGenerationStatus status) noexcept;

[[nodiscard]] NavigationWorldCertificate3D
navigationWorldCertificate3D(const ProductionMppiPreparedEsdf& world) noexcept;

// Captures an exact raw occupancy snapshot for a physical-collision search.
// The overlay has no delta-lineage assumption relative to the persistent
// planner's previous transaction, so it intentionally requests a full reset.
[[nodiscard]] std::shared_ptr<const PersistentPlannerWorld3D>
captureObservedRouteSearchWorld3D(
    const ProductionMppiRawWorld3D& raw_world,
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact);

[[nodiscard]] std::shared_ptr<const PersistentPlannerWorld3D>
routeSearchPlannerWorld3D(const ProductionMppiPreparedEsdf& world) noexcept;

void adoptWorldResources(ProductionMppiPreparedEsdf& target,
                         const ProductionMppiPreparedEsdf& source);

} // namespace drone_city_nav
