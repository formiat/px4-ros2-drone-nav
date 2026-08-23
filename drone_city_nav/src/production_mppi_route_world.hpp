#pragma once

#include "drone_city_nav/route_lifecycle_3d.hpp"

namespace drone_city_nav {

struct ProductionMppiPreparedEsdf;

[[nodiscard]] NavigationWorldCertificate3D
navigationWorldCertificate3D(const ProductionMppiPreparedEsdf& world) noexcept;

void adoptWorldResources(ProductionMppiPreparedEsdf& target,
                         const ProductionMppiPreparedEsdf& source);

} // namespace drone_city_nav
