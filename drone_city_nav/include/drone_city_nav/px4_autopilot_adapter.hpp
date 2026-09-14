#pragma once

#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"

#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace drone_city_nav {

// The PX4 side of the autopilot contract: PX4 reports its local position in
// NED with a heading from north, the stack reads the map frame with z up.
[[nodiscard]] AutopilotLocalState
px4LocalPositionToAutopilotState(const px4_msgs::msg::VehicleLocalPosition& message,
                                 const Px4MapFrameTransform& transform) noexcept;

[[nodiscard]] AutopilotAttitude
px4AttitudeToAutopilotAttitude(const px4_msgs::msg::VehicleAttitude& message) noexcept;

[[nodiscard]] AutopilotClockSync
px4TimesyncToAutopilotClockSync(const px4_msgs::msg::TimesyncStatus& message) noexcept;

[[nodiscard]] AutopilotStatus
px4VehicleStatusToAutopilotStatus(const px4_msgs::msg::VehicleStatus& message) noexcept;

[[nodiscard]] AutopilotGroundContact px4LandDetectedToAutopilotGroundContact(
    const px4_msgs::msg::VehicleLandDetected& message) noexcept;

} // namespace drone_city_nav
