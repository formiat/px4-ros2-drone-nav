#include <cinttypes>
#include <memory>
#include <optional>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

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

std::optional<mppi::MppiTickResult>
ProductionMppiNode::planOnCapturedWorldGeneration(const WorldSnapshot3D& world,
                                                  const mppi::MppiTickInput& input) {
  WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
  ProductionWorldGenerationStatus resident_status{
      ProductionWorldGenerationStatus::kInvalidGeneration};
  bool captured_generation_is_resident{false};
  if (resident.world()) {
    resident_status = assessProductionWorldGeneration(*resident.world());
    captured_generation_is_resident =
        resident_status == ProductionWorldGenerationStatus::kCoherent &&
        resident.world()->local_world_generation.sameSnapshot(
            world.local_world_generation);
  }
  if (!captured_generation_is_resident) {
    world_pipeline_->recordSupersededPlanningGeneration();
    const std::string_view status_name =
        productionWorldGenerationStatusName(resident_status);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PRODUCTION_MPPI_WORLD_SNAPSHOT status=superseded "
                         "captured_generation=%" PRIu64 " resident_status=%.*s "
                         "action=retry_next_tick",
                         world.local_world_generation.generation,
                         static_cast<int>(status_name.size()), status_name.data());
    return std::nullopt;
  }
  return engine_->plan(input);
}

} // namespace drone_city_nav
