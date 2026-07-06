#pragma once

#include "drone_city_nav/known_passage_map.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace drone_city_nav {

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildKnownPassageDebugMarkers(const std_msgs::msg::Header& header,
                              const KnownPassageMap& map);

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildKnownPassageDeleteMarkers(const std_msgs::msg::Header& header);

} // namespace drone_city_nav
