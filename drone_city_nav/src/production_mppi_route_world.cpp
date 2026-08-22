#include "production_mppi_route_world.hpp"

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void adoptWorldResources(ProductionMppiPreparedEsdf& target,
                         const ProductionMppiPreparedEsdf& source) {
  target.producer_instance_id = source.producer_instance_id;
  target.revision = source.revision;
  target.source_raw_revision = source.source_raw_revision;
  target.source_occupied_fingerprint = source.source_occupied_fingerprint;
  target.source_stamp_ns = source.source_stamp_ns;
  target.ready_stamp_ns = source.ready_stamp_ns;
  target.build_ms = source.build_ms;
  target.esdf_x_pass_ms = source.esdf_x_pass_ms;
  target.esdf_y_pass_ms = source.esdf_y_pass_ms;
  target.esdf_z_pass_ms = source.esdf_z_pass_ms;
  target.esdf_finalize_ms = source.esdf_finalize_ms;
  target.conversion_ms = source.conversion_ms;
  target.upload_ms = source.upload_ms;
  target.grid = source.grid;
  target.distances_m = source.distances_m;
  target.raw_occupancy = source.raw_occupancy;
  target.observed_occupancy = source.observed_occupancy;
  target.proprioceptive_free_space_seed = source.proprioceptive_free_space_seed;
  target.launch_support_contact = source.launch_support_contact;
  target.launch_support_resolution_pending = source.launch_support_resolution_pending;
  target.topological_graph = source.topological_graph;
  target.topological_graph_update = source.topological_graph_update;
}

std::optional<StaticRouteCandidateValidation>
validateRouteAgainstLatestObservedRawOccupancy(
    const std::span<const RouteSample3D> route,
    const ObservedOccupancyGrid3D& occupancy,
    const SweptFootprintConfig& footprint_config) {
  for (std::size_t index = 1U; index < route.size(); ++index) {
    const SweptFootprintResult validation = validateRawSweptFootprint(
        occupancy, route[index - 1U].position, FootprintBodyAxis{},
        route[index].position, FootprintBodyAxis{}, footprint_config);
    if (validation.status == SweptFootprintStatus::kRawCollision) {
      return StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kRawCollision,
          .failure_segment_index = index - 1U,
          .failure_point = validation.failure_point};
    }
  }
  return std::nullopt;
}

} // namespace drone_city_nav
