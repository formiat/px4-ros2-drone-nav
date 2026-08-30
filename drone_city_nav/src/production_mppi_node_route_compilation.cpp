#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <algorithm>
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

TrajectoryCompilerConfig3D
ProductionMppiNode::trajectoryCompilerConfig3D() const noexcept {
  return TrajectoryCompilerConfig3D{
      .unconstrained_speed_mps = speed_policy_config_.cruise_speed_mps,
      .constrained_speed_mps = constrained_route_speed_limit_mps_,
      .maximum_lateral_acceleration_mps2 =
          speed_policy_config_.maximum_lateral_acceleration_mps2,
      .minimum_continuous_turn_alignment =
          future_route_connector_config_.minimum_continuous_turn_alignment,
      .time_model =
          FlightTimeModel3D{
              .maximum_horizontal_speed_mps =
                  std::min({speed_policy_config_.cruise_speed_mps,
                            speed_policy_config_.absolute_speed_limit_mps,
                            static_cast<double>(
                                mppi_config_.dynamics.maximum_horizontal_speed_mps)}),
              .maximum_vertical_speed_mps =
                  static_cast<double>(mppi_config_.dynamics.maximum_vertical_speed_mps),
              .maximum_translational_speed_mps = static_cast<double>(
                  mppi_config_.dynamics.maximum_translational_speed_mps),
              .maximum_horizontal_acceleration_mps2 = static_cast<double>(
                  mppi_config_.dynamics.maximum_horizontal_acceleration_mps2),
              .maximum_vertical_acceleration_mps2 = static_cast<double>(
                  mppi_config_.dynamics.maximum_vertical_acceleration_mps2),
              .maximum_control_jerk_mps3 =
                  static_cast<double>(mppi_config_.dynamics.maximum_control_jerk_mps3),
              .maximum_yaw_acceleration_radps2 = static_cast<double>(
                  mppi_config_.dynamics.maximum_yaw_acceleration_radps2),
              .maximum_yaw_rate_radps =
                  static_cast<double>(mppi_config_.dynamics.maximum_yaw_rate_radps),
          },
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
