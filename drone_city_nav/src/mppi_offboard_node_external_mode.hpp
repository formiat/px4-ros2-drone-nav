#pragma once

#include "drone_city_nav/px4_offboard_setpoint_io.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <px4_ros2/common/setpoint_base.hpp>
#include <px4_ros2/components/mode.hpp>

namespace drone_city_nav {

// The stack as a mode of PX4's own (roadmap item 44, specification K91), with
// px4-ros2-interface-lib (Auterion, BSD-3): registered with the autopilot, it
// is listed beside the autopilot's modes, the pilot switches it on and off
// like any of them, its requirements (a local position) enter the arming
// checks and the autopilot's failsafe, and a stack that stops answering is a
// failsafe of the autopilot's. The stack still commands no mode and no
// arming (K29): it registers, and says which control its setpoints are.
//
// The setpoints are the offboard tick's own, on the same topic. What a mode
// adds is the control they are flown by, which offboard derives from
// OffboardControlMode and a mode states once per change
// (`fmu/in/config_control_setpoints`). The flags are those PX4 derives for
// offboard (commander ModeUtil/control_mode.cpp, v1.17.0), so that the mode
// flies as offboard flies.
class StackExternalMode final : public px4_ros2::ModeBase {
public:
  static constexpr const char* kName{"Drone City Nav"};

  explicit StackExternalMode(rclcpp::Node& node)
      : ModeBase(node, Settings{kName}) {
    for (std::size_t index = 0U; index < controls_.size(); ++index) {
      controls_[index] =
          std::make_shared<Control>(*this, static_cast<OffboardSetpointMode>(index));
    }
  }

  // The control the tick commands; PX4 is told when it changes.
  void select(const OffboardSetpointMode mode) {
    controls_[static_cast<std::size_t>(mode)]->select();
  }

  void onActivate() override {
  }

  void onDeactivate() override {
  }

private:
  class Control final : public px4_ros2::SetpointBase {
  public:
    Control(px4_ros2::Context& context, const OffboardSetpointMode mode)
        : SetpointBase(context),
          mode_(mode) {
    }

    Configuration getConfiguration() override {
      Configuration configuration{};
      configuration.position_enabled = mode_ != OffboardSetpointMode::kVelocityCruise;
      configuration.climb_rate_enabled = true;
      return configuration;
    }

    // The tick publishes the setpoints; the library's timer is not started.
    float desiredUpdateRateHz() override {
      return 0.0F;
    }

    void select() {
      onUpdate();
    }

  private:
    OffboardSetpointMode mode_;
  };

  std::array<std::shared_ptr<Control>, 3> controls_;
};

} // namespace drone_city_nav
