#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/cooperative_maneuver_command.hpp"
#include "drone_city_nav/msg/cooperative_passage_intent.hpp"
#include "drone_city_nav/msg/cooperative_peer_trajectory.hpp"

#include <rclcpp/qos.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>

namespace drone_city_nav {

[[nodiscard]] rclcpp::QoS cooperativeFlightIntentQos();

[[nodiscard]] std::int64_t
cooperativeTimeNanoseconds(const builtin_interfaces::msg::Time& time) noexcept;

[[nodiscard]] builtin_interfaces::msg::Time
cooperativeTimeMessage(std::int64_t nanoseconds) noexcept;

[[nodiscard]] CooperativeFlightIntentData
cooperativeFlightIntentData(const msg::CooperativeFlightIntent& message);

[[nodiscard]] CooperativePassageUse
cooperativePassageUseData(const msg::CooperativePassageIntent& message);

[[nodiscard]] msg::CooperativePassageIntent
cooperativePassageIntentMessage(const CooperativePassageUse& passage);

[[nodiscard]] msg::CooperativeFlightIntent
cooperativeFlightIntentMessage(const CooperativeFlightIntentData& intent);

[[nodiscard]] msg::CooperativePeerTrajectory
cooperativePeerTrajectoryMessage(const CooperativeFlightIntentData& intent);

[[nodiscard]] CooperativePeerTrajectoryData
cooperativePeerTrajectoryData(const msg::CooperativePeerTrajectory& message);

[[nodiscard]] CooperativeManeuverCommandData
cooperativeManeuverCommandData(const msg::CooperativeManeuverCommand& message);

} // namespace drone_city_nav
