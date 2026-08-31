#include <cinttypes>
#include <optional>
#include <string_view>
#include <utility>

#include "mppi_controller_3d.hpp"
#include "production_mppi_node.hpp"
#include "production_mppi_node_planning_tick_context.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

std::optional<MppiControllerResult3D>
ProductionMppiNode::runPlanningController(ProductionMppiControllerTick tick) {
  if (mppi_controller_ == nullptr) {
    return std::nullopt;
  }

  MppiControllerResult3D output;
  if (tick.request.mode == MppiControllerMode3D::kPlan) {
    WorldPipeline3D::ResidentLease resident = world_pipeline_->lockResident();
    const MppiControllerWorldCurrentness3D currentness =
        assessMppiControllerWorldCurrentness3D(tick.world, resident.world());
    if (!currentness.current()) {
      world_pipeline_->recordSupersededPlanningGeneration();
      const std::string_view status_name =
          productionWorldGenerationStatusName(currentness.resident_generation_status);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "PRODUCTION_MPPI_WORLD_SNAPSHOT status=superseded "
                           "captured_generation=%" PRIu64 " resident_status=%.*s "
                           "action=retry_next_tick",
                           tick.world.local_world_generation.generation,
                           static_cast<int>(status_name.size()), status_name.data());
      return std::nullopt;
    }
    // The resident lease excludes GPU-world replacement for the complete
    // controller transaction, keeping the captured CPU world and CUDA resource
    // on one generation.
    output = mppi_controller_->run(std::move(tick.request));
  } else {
    output = mppi_controller_->run(std::move(tick.request));
  }

  if (!output.executable()) {
    RCLCPP_ERROR(get_logger(), "PRODUCTION_MPPI_TICK failed: status=%s error=%s",
                 mppiControllerStatus3DName(output.status),
                 output.failure_message.empty() ? "none"
                                                : output.failure_message.c_str());
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, tick.now_ns);
    return std::nullopt;
  }
  mppi::MppiTickResult& result = output.result;
  if (result.route_directed_candidate_injected &&
      !result.route_directed_candidate_device_feasible &&
      !tick.direct_tracking_interception) {
    // A route-directed seed is only one controller candidate. Its rejection does
    // not invalidate the certified route geometry while the remaining MPPI
    // rollouts can still provide an executable control result.
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ROUTE_EXECUTION status=seed_not_executable route_generation=%" PRIu64
        " cross_track_m=%.2f alternative_rollout_available=%s "
        "action=reject_control_candidate",
        tick.route_generation, tick.route_cross_track_m,
        result.feasibility_contract.available ? "true" : "false");
  }
  if (output.no_eligible_recovery.route_replan_requested &&
      !tick.direct_tracking_interception &&
      optional_constraints_.no_eligible_route_replan_enabled) {
    requestRouteRelease(RouteReleaseReason3D::kNoEligibleRollouts,
                        tick.route_generation);
  }
  return output;
}

} // namespace drone_city_nav
