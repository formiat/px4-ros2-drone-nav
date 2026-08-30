#include <chrono>
#include <cinttypes>
#include <exception>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_node_planning_tick_context.hpp"

namespace drone_city_nav {

std::optional<ProductionMppiControllerTickResult>
ProductionMppiNode::runPlanningController(const ProductionMppiControllerTick& tick) {
  ProductionMppiControllerTickResult output{
      .result = {},
      .no_eligible_recovery =
          MppiEligibleRolloutUpdate{
              .no_eligible_recovery_generation =
                  tick.nominal_reseed.no_eligible_recovery_generation,
              .phase = tick.nominal_reseed.no_eligible_phase,
          },
  };
  mppi::MppiTickResult& result = output.result;
  if (tick.planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold ||
      tick.planning_state == ProductionMppiPlanningState::kNoExecutableRouteHold) {
    result.horizon = {tick.target, tick.target};
    result.controls = {mppi::Control{}};
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.esdf_revision = tick.esdf.revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  tick.snapshot_started)
            .count();
    return output;
  }

  try {
    std::optional<mppi::MppiTickResult> planned =
        planOnCapturedWorldGeneration(tick.esdf, tick.input);
    if (!planned.has_value()) {
      return std::nullopt;
    }
    result = std::move(*planned);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "PRODUCTION_MPPI_TICK failed: %s", error.what());
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, tick.now_ns);
    return std::nullopt;
  }
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
  output.no_eligible_recovery = nominal_reseed_tracker_.observeEligibleRolloutResult(
      result.feasibility_contract.available, result.nominal_reseeded);
  if (output.no_eligible_recovery.route_replan_requested &&
      !tick.direct_tracking_interception &&
      optional_constraints_.no_eligible_route_replan_enabled) {
    requestRouteRelease(RouteReleaseReason3D::kNoEligibleRollouts,
                        tick.route_generation);
  }
  return output;
}

} // namespace drone_city_nav
