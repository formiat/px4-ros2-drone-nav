#include <memory>
#include <optional>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
void ProductionMppiNode::queueLatestObservedWorldForPose(
    const ProductionMppiNavigation& navigation) {
  if (use_static_map_ || !navigation.world_state_authoritative) {
    return;
  }
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  if (!raw_world || !raw_world->occupancy) {
    return;
  }

  bool local_world_required{false};
  {
    const std::scoped_lock lock{world_generation_publication_mutex_, esdf_state_mutex_};
    if (!resident_world_ || !productionWorldGenerationCoherent(*resident_world_) ||
        resident_world_->producer_instance_id !=
            raw_world->version.producer_instance_id ||
        resident_world_->grid.depth <= 1 || !resident_world_->grid.outside_is_unknown) {
      local_world_required = true;
    } else {
      const GridBounds3D local_bounds =
          resident_world_->observed_esdf_resource.local_occupancy
              ? resident_world_->observed_esdf_resource.local_occupancy->bounds()
              : GridBounds3D{
                    .origin_x = resident_world_->grid.origin_x_m,
                    .origin_y = resident_world_->grid.origin_y_m,
                    .origin_z = resident_world_->grid.origin_z_m,
                    .resolution_m = resident_world_->grid.resolution_m,
                    .width_cells = resident_world_->grid.width,
                    .height_cells = resident_world_->grid.height,
                    .depth_cells = resident_world_->grid.depth,
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

  {
    const std::scoped_lock lock{raw_queue_mutex_};
    static_cast<void>(raw_world_scheduler_3d_.submit(raw_world, true));
  }
  raw_queue_condition_.notify_all();
}

} // namespace drone_city_nav
