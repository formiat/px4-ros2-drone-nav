#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <optional>
#include <span>

namespace drone_city_nav {

struct ProductionMppiPreparedEsdf;

void adoptWorldResources(ProductionMppiPreparedEsdf& target,
                         const ProductionMppiPreparedEsdf& source);

[[nodiscard]] std::optional<StaticRouteCandidateValidation>
validateRouteAgainstLatestObservedRawOccupancy(
    std::span<const RouteSample3D> route, const ObservedOccupancyGrid3D& occupancy,
    const SweptFootprintConfig& footprint_config,
    const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed = nullptr,
    const LaunchSupportContact3D* launch_support_contact = nullptr);

} // namespace drone_city_nav
