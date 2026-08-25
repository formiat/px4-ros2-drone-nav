#include <memory>
#include <optional>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
void ProductionMppiNode::queueLatestObservedWorldForPose(
    const ProductionMppiNavigation& navigation) {
  if (use_static_map_ || !navigation.valid ||
      no_static_world_model_ != ProductionNoStaticWorldModel::kObservedOccupancy3D) {
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
    if (!prepared_esdf_ || !productionWorldGenerationCoherent(*prepared_esdf_) ||
        prepared_esdf_->producer_instance_id !=
            raw_world->version.producer_instance_id ||
        prepared_esdf_->grid.depth <= 1 || !prepared_esdf_->grid.outside_is_unknown) {
      local_world_required = true;
    } else {
      const GridBounds3D local_bounds =
          prepared_esdf_->observed_esdf_resource.local_occupancy
              ? prepared_esdf_->observed_esdf_resource.local_occupancy->bounds()
              : GridBounds3D{
                    .origin_x = prepared_esdf_->grid.origin_x_m,
                    .origin_y = prepared_esdf_->grid.origin_y_m,
                    .origin_z = prepared_esdf_->grid.origin_z_m,
                    .resolution_m = prepared_esdf_->grid.resolution_m,
                    .width_cells = prepared_esdf_->grid.width,
                    .height_cells = prepared_esdf_->grid.height,
                    .depth_cells = prepared_esdf_->grid.depth,
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
