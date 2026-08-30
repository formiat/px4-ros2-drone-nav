#include <memory>
#include <optional>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
void ProductionMppiNode::queueLatestObservedWorldForPose(
    const ProductionMppiNavigation& navigation) {
  if (use_static_map_ || !navigation.world_state_authoritative) {
    return;
  }
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world =
      world_pipeline_->latestRawWorld();
  if (!raw_world || !raw_world->occupancy) {
    return;
  }

  bool local_world_required{false};
  const WorldPipelineResidentSnapshot3D resident = world_pipeline_->residentSnapshot();
  {
    if (!resident.world || !productionWorldGenerationCoherent(*resident.world) ||
        resident.world->producer_instance_id !=
            raw_world->version.producer_instance_id ||
        resident.world->grid.depth <= 1 || !resident.world->grid.outside_is_unknown) {
      local_world_required = true;
    } else {
      const GridBounds3D local_bounds =
          resident.world->observed_esdf_resource.local_occupancy
              ? resident.world->observed_esdf_resource.local_occupancy->bounds()
              : GridBounds3D{
                    .origin_x = resident.world->grid.origin_x_m,
                    .origin_y = resident.world->grid.origin_y_m,
                    .origin_z = resident.world->grid.origin_z_m,
                    .resolution_m = resident.world->grid.resolution_m,
                    .width_cells = resident.world->grid.width,
                    .height_cells = resident.world->grid.height,
                    .depth_cells = resident.world->grid.depth,
                };
      const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
      local_world_required =
          localObservedEsdfNeedsRecenter(local_bounds, raw_world->occupancy->bounds(),
                                         position, no_static_3d_esdf_window_);
    }
  }
  if (!local_world_required) {
    return;
  }

  static_cast<void>(world_pipeline_->scheduleLatestRawWorldUrgently());
}

} // namespace drone_city_nav
