#pragma once

#include <cstdint>
#include <string>

namespace drone_city_nav {

struct Point3;
struct MppiDebugMarkerInput;
struct ProductionNavigationObjective;

namespace detail {

[[nodiscard]] std::string
trackingObjectiveJsonFields(const ProductionNavigationObjective* navigation_objective,
                            const Point3& resolved_position, std::int64_t now_ns);

void populateTrackingObjectiveMarkers(
    const ProductionNavigationObjective* navigation_objective,
    MppiDebugMarkerInput& marker_input);

} // namespace detail
} // namespace drone_city_nav
