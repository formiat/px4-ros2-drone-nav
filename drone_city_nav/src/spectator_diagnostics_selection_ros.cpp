#include "drone_city_nav/spectator_diagnostics_selection_ros.hpp"

#include <utility>

namespace drone_city_nav {

rclcpp::Subscription<msg::SpectatorTarget>::SharedPtr
subscribeSpectatorDiagnosticsSelection(rclcpp::Node& node, const std::string& topic,
                                       SpectatorDiagnosticsSelection& selection,
                                       const std::string& log_event) {
  if (!selection.gated()) {
    return nullptr;
  }
  return node.create_subscription<msg::SpectatorTarget>(
      topic, rclcpp::QoS{1}.reliable().transient_local(),
      [&node, &selection, log_event](const msg::SpectatorTarget::SharedPtr target) {
        if (!selection.select(target->vehicle_id)) {
          return;
        }
        RCLCPP_INFO(node.get_logger(), "%s vehicle_id='%s' selected=%s",
                    log_event.c_str(), target->vehicle_id.c_str(),
                    selection.selected() ? "true" : "false");
      });
}

} // namespace drone_city_nav
