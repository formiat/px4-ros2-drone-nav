#pragma once

#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstdint>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] double interpolate(const double first, const double second,
                                 const double ratio) {
  return first + (second - first) * ratio;
}

// The vehicle as RViz follows and draws it.
[[nodiscard]] geometry_msgs::msg::TransformStamped
droneFollowTransform(const std_msgs::msg::Header& header,
                     const std::string& child_frame, const Point3& position) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header = header;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = position.x;
  transform.transform.translation.y = position.y;
  transform.transform.translation.z = position.z;
  transform.transform.rotation.w = 1.0;
  return transform;
}

[[nodiscard]] visualization_msgs::msg::Marker
droneMarker(const std_msgs::msg::Header& header, const int marker_id,
            const Point3& position, const std_msgs::msg::ColorRGBA& color) {
  visualization_msgs::msg::Marker marker =
      makeMarker(header, "drone", marker_id, visualization_msgs::msg::Marker::SPHERE);
  marker.pose.position = markerPoint(position);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 0.45;
  marker.color = color;
  return marker;
}

[[nodiscard]] const char* executionModeName(const std::uint8_t mode) noexcept {
  switch (mode) {
    case msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED:
      return "planned";
    case msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD:
      return "position_hold";
    case msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED:
      return "revoked";
    default:
      return "invalid";
  }
}

[[nodiscard]] const char* executionReasonName(const std::uint8_t reason) noexcept {
  switch (reason) {
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_NONE:
      return "none";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_EXECUTABLE_HORIZON:
      return "no_executable_horizon";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_COOPERATIVE_PASSAGE_YIELD:
      return "cooperative_passage_yield";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_GOAL_CAPTURE:
      return "goal_capture";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_EXECUTABLE_ROUTE:
      return "no_executable_route";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_UNAVAILABLE_WORLD:
      return "unavailable_world";
    default:
      return "invalid";
  }
}

[[nodiscard]] const char* vehicleRoleName(const std::uint8_t role) noexcept {
  switch (role) {
    case msg::VehicleDestroyed::ROLE_UNSPECIFIED:
      return "unspecified";
    case msg::VehicleDestroyed::ROLE_CIVILIAN:
      return "civilian";
    default:
      return "invalid";
  }
}

[[nodiscard]] const char* vehicleDeathCauseName(const std::uint8_t cause) noexcept {
  switch (cause) {
    case msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION:
      return "physical_collision";
    case msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION:
      return "proximity_collision";
    default:
      return "invalid";
  }
}

[[nodiscard]] bool validVehicleDeathCause(const std::uint8_t cause) noexcept {
  return cause == msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION ||
         cause == msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION;
}

} // namespace
} // namespace drone_city_nav
