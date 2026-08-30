#include "drone_city_nav/route_compiler_3d.hpp"

#include <memory>
#include <optional>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

} // namespace

RouteCompilerConfig3D ProductionMppiNode::routeCompilerConfig3D() const noexcept {
  return RouteCompilerConfig3D{
      .unconstrained_speed_mps = speed_policy_config_.cruise_speed_mps,
      .constrained_speed_mps = constrained_route_speed_limit_mps_,
      .minimum_continuous_turn_alignment =
          future_route_connector_config_.minimum_continuous_turn_alignment,
      .speed_policy = speed_policy_config_,
      .dynamics = mppi_config_.dynamics,
      .physical_footprint = physical_footprint_config_,
      .tracking_error_tube = tracking_error_tube_config_,
  };
}

TrackingErrorTubeWorld3D ProductionMppiNode::trackingErrorTubeWorld3D(
    const WorldSnapshot3D& world) const noexcept {
  if (world.observed_occupancy != nullptr) {
    const bool exact_observation_owner =
        world.observed_raw_world_owner != nullptr &&
        std::addressof(world.observed_raw_world_owner->occupancy()) ==
            world.observed_occupancy.get();
    return TrackingErrorTubeWorld3D{
        .observed_occupancy = world.observed_occupancy.get(),
        .occupied_content_fingerprint =
            exact_observation_owner
                ? world.observed_raw_world_owner->occupiedContentFingerprint()
                : 0U,
        .launch_support_contact = optionalAddress(world.launch_support_contact),
    };
  }
  return TrackingErrorTubeWorld3D{
      .occupancy = world.static_occupancy.get(),
      .occupied_content_fingerprint = world.static_occupancy != nullptr
                                          ? world.static_occupancy->contentFingerprint()
                                          : 0U,
  };
}

} // namespace drone_city_nav
