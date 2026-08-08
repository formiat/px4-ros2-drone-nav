#pragma once

#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/spectator_diagnostics_selection.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace drone_city_nav {

[[nodiscard]] rclcpp::Subscription<msg::SpectatorTarget>::SharedPtr
subscribeSpectatorDiagnosticsSelection(rclcpp::Node& node, const std::string& topic,
                                       SpectatorDiagnosticsSelection& selection,
                                       const std::string& log_event);

} // namespace drone_city_nav
