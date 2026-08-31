#include <cinttypes>
#include <string_view>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {

bool ProductionMppiNode::worldGenerationAvailableForPlanning(
    const WorldSnapshot3D& world, const std::int64_t now_ns) {
  const ProductionWorldGenerationStatus status = assessProductionWorldGeneration(world);
  if (status == ProductionWorldGenerationStatus::kCoherent) {
    return true;
  }
  const std::string_view status_name = productionWorldGenerationStatusName(status);
  RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "PRODUCTION_MPPI_UNAVAILABLE_WORLD action=wait_for_coherent_generation "
      "local_world_generation=%" PRIu64 " reason=%.*s",
      world.local_world_generation.generation, static_cast<int>(status_name.size()),
      status_name.data());
  publishFailClosedExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld,
                                       now_ns);
  return false;
}

} // namespace drone_city_nav
