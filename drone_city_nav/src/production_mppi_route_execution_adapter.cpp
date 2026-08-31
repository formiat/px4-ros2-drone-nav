#include <cinttypes>
#include <memory>
#include <utility>

#include "production_mppi_node.hpp"
#include "route_execution_selector_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] const char* residentObstacleSource(
    const ProductionMppiResidentObstacleDisposition disposition) noexcept {
  switch (disposition) {
    case ProductionMppiResidentObstacleDisposition::kRouteSuffixReplacementRequired:
      return "resident_route_suffix_persistent_raw";
    case ProductionMppiResidentObstacleDisposition::
        kPersistentRawFiniteExecutionInvalidated:
      return "active_finite_trajectory_persistent_raw";
    case ProductionMppiResidentObstacleDisposition::
        kLatestLidarFiniteExecutionInvalidated:
      return "active_finite_trajectory_latest_lidar";
    case ProductionMppiResidentObstacleDisposition::kClear:
      return "none";
  }
  return "none";
}

} // namespace

ProductionRouteExecutionSelection3D ProductionMppiNode::resolveRouteExecution3D(
    const WorldSnapshot3D& world, const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& latest_lidar_evidence,
    const std::int64_t validation_stamp_ns,
    const std::uint64_t minimum_tracking_sample_sequence,
    std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity,
    const bool observed_3d_world) {
  if (route_execution_selector_ == nullptr) {
    return {};
  }
  RouteExecutionSelectorResult3D outcome =
      route_execution_selector_->select(RouteExecutionSelectorRequest3D{
          .world = std::addressof(world),
          .objective = objective,
          .navigation = navigation,
          .execution_input = execution_input,
          .latest_raw_world = latest_raw_world,
          .latest_lidar_evidence = latest_lidar_evidence,
          .validation_stamp_ns = validation_stamp_ns,
          .minimum_tracking_sample_sequence = minimum_tracking_sample_sequence,
          .physically_invalidated_through_generation =
              physical_trajectory_replan_route_generation_.load(
                  std::memory_order_acquire),
          .direct_tracking_identity = direct_tracking_identity,
          .observed_3d_world = observed_3d_world,
      });
  for (const RouteExecutionSelectorEffect3D& effect : outcome.effects) {
    switch (effect.kind) {
      case RouteExecutionSelectorEffectKind3D::kRequestRouteRelease:
        requestRouteRelease(effect.release_reason, effect.route_generation);
        break;
      case RouteExecutionSelectorEffectKind3D::kHandlePhysicalTrajectoryCollision:
        handlePhysicalTrajectoryCollision(
            effect.route_generation, effect.observed_raw_world,
            residentObstacleSource(effect.obstacle_disposition),
            ProductionMppiPhysicalTrajectoryAuthority::kResidentOwner);
        break;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ROUTE_EXECUTION3D route_generation=%" PRIu64
        " status=%.*s effect=%s obstacle_source=%s",
        effect.route_generation,
        static_cast<int>(routeExecutionStatus3DName(outcome.selection.status).size()),
        routeExecutionStatus3DName(outcome.selection.status).data(),
        effect.kind == RouteExecutionSelectorEffectKind3D::kRequestRouteRelease
            ? "request_route_release"
            : "handle_physical_trajectory_collision",
        residentObstacleSource(effect.obstacle_disposition));
  }
  return std::move(outcome.selection);
}

} // namespace drone_city_nav
