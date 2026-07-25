#pragma once

#include "drone_city_nav/directed_inflation_escape.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace drone_city_nav {

[[nodiscard]] visualization_msgs::msg::MarkerArray
buildDirectedInflationEscapeDebugMarkers(const std_msgs::msg::Header& header,
                                         const DirectedInflationEscapeResult& escape,
                                         double tunnel_width_m);

} // namespace drone_city_nav
