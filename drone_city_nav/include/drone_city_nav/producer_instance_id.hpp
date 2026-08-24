#pragma once

#include <cstdint>

namespace drone_city_nav {

// Creates a nonzero process identity mixed with PID, wall/steady clocks, and a
// local sequence. PID separates simultaneous same-domain launch processes;
// clocks separate later PID reuse. Domains keep producer protocols disjoint.
[[nodiscard]] std::uint64_t createProducerInstanceId(std::uint64_t domain) noexcept;

// Raw-world transports use a process identity independent of ROS time so a
// simulator reset cannot recreate a retired producer epoch.
[[nodiscard]] std::uint64_t createRawObstacleProducerInstanceId() noexcept;

} // namespace drone_city_nav
