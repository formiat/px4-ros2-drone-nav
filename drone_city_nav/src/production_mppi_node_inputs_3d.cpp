#include "production_mppi_node.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
void ProductionMppiNode::queueLatestObservedWorldForPose(
    const ProductionMppiNavigation& navigation) {
  if (config_.world.use_static_map || !navigation.world_state_authoritative) {
    return;
  }
  const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
  if (!world_pipeline_->observedWorldNeedsRefresh(position)) {
    return;
  }

  static_cast<void>(world_pipeline_->scheduleLatestRawWorldUrgently());
}

} // namespace drone_city_nav
