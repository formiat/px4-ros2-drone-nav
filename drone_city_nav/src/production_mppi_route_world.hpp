#pragma once

#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <string_view>

namespace drone_city_nav {

struct ProductionMppiPreparedEsdf;

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
assessProductionWorldGeneration(const ProductionMppiPreparedEsdf& world) noexcept;
[[nodiscard]] bool
productionWorldGenerationCoherent(const ProductionMppiPreparedEsdf& world) noexcept;
[[nodiscard]] std::string_view
productionWorldGenerationStatusName(ProductionWorldGenerationStatus status) noexcept;

[[nodiscard]] NavigationWorldCertificate3D
navigationWorldCertificate3D(const ProductionMppiPreparedEsdf& world) noexcept;

void adoptWorldResources(ProductionMppiPreparedEsdf& target,
                         const ProductionMppiPreparedEsdf& source);

} // namespace drone_city_nav
