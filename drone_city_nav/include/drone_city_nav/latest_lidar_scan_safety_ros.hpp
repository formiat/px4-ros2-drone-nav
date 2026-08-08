#pragma once

#include "drone_city_nav/latest_lidar_scan_safety.hpp"
#include "drone_city_nav/msg/latest_lidar_safety_scan.hpp"

#include <std_msgs/msg/header.hpp>

#include <cstdint>
#include <string_view>

namespace drone_city_nav {

[[nodiscard]] msg::LatestLidarSafetyScan
makeLatestLidarSafetyScanMessage(const LatestLidarSafetyScanBuildResult& scan,
                                 const std_msgs::msg::Header& source_header,
                                 std::string_view frame_id,
                                 std::int64_t acquisition_stamp_ns,
                                 std::uint64_t sequence, std::uint64_t pose_generation);

} // namespace drone_city_nav
