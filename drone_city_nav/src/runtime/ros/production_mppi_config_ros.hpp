#pragma once

#include "production_mppi_config.hpp"

namespace rclcpp {
class Node;
}

namespace drone_city_nav {

[[nodiscard]] ProductionMppiConfig declareProductionMppiConfig(rclcpp::Node& node);

} // namespace drone_city_nav
