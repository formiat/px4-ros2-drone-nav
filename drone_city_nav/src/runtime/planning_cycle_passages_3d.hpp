#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "planning_cycle_coordinator_3d.hpp"

// What a planning cycle reads of the passages it flies through and of the
// traffic it shares them with: the cooperative command adapted for this
// vehicle, and where the vehicle stands against the passages' geometry.

namespace drone_city_nav {

[[nodiscard]] ProductionMppiCooperativeUpdate prepareCooperativeUpdate(
    const PlanningCycleCoordinatorConfig3D& config,
    std::span<const CooperativePassageAssignment> passage_assignments,
    const ConstrainedRouteObservation& route_observation,
    const std::optional<ProductionMppiCooperativeCommand>& command, std::int64_t now_ns,
    double planned_speed_mps);

[[nodiscard]] std::optional<PassageGeometryProximity3D>
observePassageGeometry(const WorldSnapshot3D& world, const Point3& actual_position,
                       std::vector<PassageGeometryObservation>& observations);

} // namespace drone_city_nav
