#include "planning_cycle_passages_3d.hpp"

#include "drone_city_nav/control_route_projection_3d.hpp"
#include "drone_city_nav/cooperative_mppi_adapter.hpp"

#include <limits>
#include <memory>

namespace drone_city_nav {

[[nodiscard]] ProductionMppiCooperativeUpdate prepareCooperativeUpdate(
    const PlanningCycleCoordinatorConfig3D& config,
    const std::span<const CooperativePassageAssignment> passage_assignments,
    const ConstrainedRouteObservation& route_observation,
    const std::optional<ProductionMppiCooperativeCommand>& command,
    const std::int64_t now_ns, const double planned_speed_mps) {
  ProductionMppiCooperativeUpdate result;
  if (!config.cooperative_traffic_enabled) {
    return result;
  }

  const CooperativePassageAssignment* assignment = nullptr;
  if (route_observation.span_available &&
      route_observation.span_index < passage_assignments.size()) {
    const CooperativePassageAssignment& candidate =
        passage_assignments[route_observation.span_index];
    if (candidate.span_index == route_observation.span_index &&
        candidate.route_generation == route_observation.route_generation &&
        candidate.passage_traversal_id == route_observation.passage_traversal_id) {
      assignment = std::addressof(candidate);
    }
  }
  if (assignment != nullptr) {
    result.passage =
        makeCooperativePassageUse(route_observation, *assignment, now_ns,
                                  planned_speed_mps, config.cooperative_timing);
  }
  if (!command.has_value()) {
    return result;
  }

  result.command_generation = command->data.command_generation;
  result.command_age_ms =
      now_ns >= command->data.stamp_ns
          ? static_cast<double>(now_ns - command->data.stamp_ns) / 1.0e6
          : -1.0;
  result.mppi = adaptCooperativeMppiCommand(command->data, config.vehicle_id, now_ns,
                                            config.horizon_steps, config.dynamics.dt_s);
  result.yield = evaluateCooperativePassageYield(
      command->data, result.passage, route_observation, config.vehicle_id, now_ns,
      route_observation.actual_horizontal_speed_mps, config.cooperative_yield);
  return result;
}

[[nodiscard]] std::optional<PassageGeometryProximity3D>
observePassageGeometry(const WorldSnapshot3D& world, const Point3& actual_position,
                       std::vector<PassageGeometryObservation>& observations) {
  if (world.topology_passage_traversals == nullptr) {
    return std::nullopt;
  }
  observations.reserve(world.topology_passage_traversals->size());
  const PassageTraversalEdge* nearest = nullptr;
  RouteProjection3D nearest_projection;
  double nearest_entry_distance_m = std::numeric_limits<double>::infinity();
  for (const PassageTraversalEdge& passage : *world.topology_passage_traversals) {
    const RouteProjection3D projection =
        projectOntoRoute3D(passage.centerline, actual_position);
    const double entry_distance_m = distance3D(actual_position, passage.entry);
    if (entry_distance_m < nearest_entry_distance_m) {
      nearest = std::addressof(passage);
      nearest_projection = projection;
      nearest_entry_distance_m = entry_distance_m;
    }
    observations.push_back(PassageGeometryObservation{
        .passage_traversal_id = passage.id,
        .within_corridor =
            projection.valid && projection.distance_m <= passage.minimum_clearance_m,
        .station_m = projection.station_m,
        .traversal_length_m =
            passage.centerline.empty() ? 0.0 : passage.centerline.back().station_m,
        .cross_track_error_m = projection.distance_m,
    });
  }
  if (nearest == nullptr || nearest_entry_distance_m >= 8.0) {
    return std::nullopt;
  }
  return PassageGeometryProximity3D{
      .passage_traversal_id = nearest->id,
      .entry_distance_m = nearest_entry_distance_m,
      .projection_station_m = nearest_projection.station_m,
      .projection_cross_track_m = nearest_projection.distance_m,
      .minimum_clearance_m = nearest->minimum_clearance_m,
      .projection_valid = nearest_projection.valid,
      .within_corridor = nearest_projection.valid &&
                         nearest_projection.distance_m <= nearest->minimum_clearance_m,
  };
}

} // namespace drone_city_nav
