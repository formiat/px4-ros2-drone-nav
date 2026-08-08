#pragma once

#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/msg/simulation_truth_state.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <cstdint>
#include <optional>

namespace drone_city_nav::detail {

[[nodiscard]] inline std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] inline builtin_interfaces::msg::Time
timeMessage(const std::int64_t stamp_ns) noexcept {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  return stamp;
}

[[nodiscard]] inline TimedVehicleState
vehicleState(const msg::VehicleNavigationState& message) noexcept {
  return TimedVehicleState{
      .position = Point3{message.position.x, message.position.y, message.position.z},
      .velocity = Vec3{message.velocity.x, message.velocity.y, message.velocity.z},
      .stamp_ns = timeNanoseconds(message.stamp),
      .heading_rad = message.heading_rad,
      .position_valid = message.position_valid,
      .velocity_valid = message.velocity_valid,
      .heading_valid = message.heading_valid,
      .armed = message.armed,
      .airborne = message.airborne,
      .navigation_ready = message.navigation_ready,
  };
}

[[nodiscard]] inline TimedVehicleState
physicalTruthState(const msg::SimulationTruthState& message) noexcept {
  return TimedVehicleState{
      .position = Point3{message.position.x, message.position.y, message.position.z},
      .velocity = Vec3{message.velocity.x, message.velocity.y, message.velocity.z},
      .stamp_ns = timeNanoseconds(message.header.stamp),
      .position_valid = message.position_valid,
      .velocity_valid = message.velocity_valid,
  };
}

[[nodiscard]] inline TimedVehicleState
physicalStateWithNavigationStatus(const TimedVehicleState& physical_truth,
                                  const TimedVehicleState& navigation) noexcept {
  TimedVehicleState result = physical_truth;
  result.heading_rad = navigation.heading_rad;
  result.heading_valid = navigation.heading_valid;
  result.armed = navigation.armed;
  result.airborne = navigation.airborne;
  result.navigation_ready = navigation.navigation_ready;
  return result;
}

[[nodiscard]] inline std::optional<TimedVehicleState>
physicalState(const std::optional<TimedVehicleState>& navigation,
              const std::optional<TimedVehicleState>& truth) noexcept {
  if (!navigation || !truth) {
    return std::nullopt;
  }
  return physicalStateWithNavigationStatus(*truth, *navigation);
}

[[nodiscard]] inline const char* vehicleRoleName(const std::uint8_t role) noexcept {
  switch (role) {
    case msg::VehicleDestroyed::ROLE_INTERCEPTOR:
      return "interceptor";
    case msg::VehicleDestroyed::ROLE_EVADER:
      return "evader";
    default:
      return "unspecified";
  }
}

[[nodiscard]] inline const char* deathCauseName(const std::uint8_t cause) noexcept {
  switch (cause) {
    case msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION:
      return "physical_collision";
    case msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT:
      return "proximity_intercept";
    case msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION:
      return "proximity_collision";
    default:
      return "invalid";
  }
}

[[nodiscard]] inline bool validDeathCause(const std::uint8_t cause) noexcept {
  return cause == msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION ||
         cause == msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT ||
         cause == msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION;
}

[[nodiscard]] inline double speed(const TimedVehicleState& state) noexcept {
  return state.velocity_valid
             ? std::hypot(std::hypot(state.velocity.x, state.velocity.y),
                          state.velocity.z)
             : 0.0;
}

} // namespace drone_city_nav::detail
