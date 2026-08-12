#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/msg/cooperative_channel_intent.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/cooperative_peer_trajectory.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>

namespace drone_city_nav {

[[nodiscard]] std::int64_t
cooperativeTimeNanoseconds(const builtin_interfaces::msg::Time& time) noexcept;

[[nodiscard]] builtin_interfaces::msg::Time
cooperativeTimeMessage(std::int64_t nanoseconds) noexcept;

[[nodiscard]] CooperativeFlightIntentData
cooperativeFlightIntentData(const msg::CooperativeFlightIntent& message);

[[nodiscard]] CooperativeChannelUse
cooperativeChannelUseData(const msg::CooperativeChannelIntent& message);

[[nodiscard]] msg::CooperativeChannelIntent
cooperativeChannelIntentMessage(const CooperativeChannelUse& channel);

[[nodiscard]] msg::CooperativeFlightIntent
cooperativeFlightIntentMessage(const CooperativeFlightIntentData& intent);

[[nodiscard]] msg::CooperativePeerTrajectory
cooperativePeerTrajectoryMessage(const CooperativeFlightIntentData& intent);

} // namespace drone_city_nav
