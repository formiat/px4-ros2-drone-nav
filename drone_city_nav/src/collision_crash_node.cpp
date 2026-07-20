#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/msg/crash_state.hpp"

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <string>

namespace drone_city_nav {

class CollisionCrashNode final : public rclcpp::Node {
public:
  CollisionCrashNode()
      : Node{"collision_crash_node"} {
    airborne_altitude_m_ = declare_parameter<double>("airborne_altitude_m", 1.0);
    const std::string contacts_topic = declare_parameter<std::string>(
        "contacts_topic", "/drone_city_nav/drone_contacts");
    const std::string crash_state_topic = declare_parameter<std::string>(
        "crash_state_topic", "/drone_city_nav/crash_state");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    const std::string status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status");

    crash_state_pub_ = create_publisher<msg::CrashState>(
        crash_state_topic,
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local());
    contacts_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
        contacts_topic, rclcpp::QoS{rclcpp::KeepLast{10}}.reliable(),
        [this](const ros_gz_interfaces::msg::Contacts::SharedPtr contacts) {
          onContacts(*contacts);
        });
    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr position) {
          onLocalPosition(*position);
        });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr attitude) {
          const auto euler = quaternionToEuler(attitude->q);
          if (euler.has_value()) {
            attitude_ = *euler;
            attitude_valid_ = true;
          }
        });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        status_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr status) {
          armed_ =
              status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        });

    msg::CrashState initial_state;
    initial_state.stamp = now();
    initial_state.crashed = false;
    initial_state.reason = "none";
    crash_state_pub_->publish(initial_state);
    RCLCPP_INFO(get_logger(),
                "Physical collision detector ready: contacts='%s' crash_state='%s' "
                "airborne_altitude=%.2fm",
                contacts_topic.c_str(), crash_state_topic.c_str(),
                airborne_altitude_m_);
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& position) {
    if (position.z_valid && std::isfinite(position.z)) {
      altitude_m_ = -static_cast<double>(position.z);
      altitude_valid_ = true;
      if (armed_ && altitude_m_ >= airborne_altitude_m_) {
        airborne_seen_ = true;
      }
    }
    if (std::isfinite(position.vx) && std::isfinite(position.vy) &&
        std::isfinite(position.vz)) {
      speed_mps_ = std::sqrt(static_cast<double>(position.vx) * position.vx +
                             static_cast<double>(position.vy) * position.vy +
                             static_cast<double>(position.vz) * position.vz);
    }
  }

  void onContacts(const ros_gz_interfaces::msg::Contacts& contacts) {
    if (crashed_) {
      return;
    }
    if (!airborne_seen_) {
      if (!contacts.contacts.empty()) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                              "Ignoring physical contact before airborne");
      }
      return;
    }

    for (const auto& contact : contacts.contacts) {
      msg::CrashState state;
      state.stamp = contacts.header.stamp;
      if (state.stamp.sec == 0 && state.stamp.nanosec == 0U) {
        state.stamp = now();
      }
      state.crashed = true;
      state.reason = "physical_collision";
      state.drone_collision = contact.collision1.name;
      state.obstacle_collision = contact.collision2.name;
      state.altitude_m = altitude_m_;
      state.speed_mps = speed_mps_;
      if (!contact.positions.empty()) {
        state.contact_position.x = contact.positions.front().x;
        state.contact_position.y = contact.positions.front().y;
        state.contact_position.z = contact.positions.front().z;
      }

      crashed_ = true;
      crash_state_pub_->publish(state);
      const double roll = attitude_valid_ ? attitude_.roll_rad
                                          : std::numeric_limits<double>::quiet_NaN();
      const double pitch = attitude_valid_ ? attitude_.pitch_rad
                                           : std::numeric_limits<double>::quiet_NaN();
      const double yaw = attitude_valid_ ? attitude_.yaw_rad
                                         : std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(get_logger(),
                   "PHYSICAL_COLLISION crashed=true drone_collision='%s' "
                   "obstacle_collision='%s' contact=(%.3f, %.3f, %.3f) altitude=%.2f "
                   "speed=%.2f attitude_rpy=(%.3f, %.3f, %.3f)",
                   state.drone_collision.c_str(), state.obstacle_collision.c_str(),
                   state.contact_position.x, state.contact_position.y,
                   state.contact_position.z, state.altitude_m, state.speed_mps, roll,
                   pitch, yaw);
      return;
    }
  }

  double airborne_altitude_m_{1.0};
  double altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  AttitudeEuler attitude_{};
  bool altitude_valid_{false};
  bool attitude_valid_{false};
  bool armed_{false};
  bool airborne_seen_{false};
  bool crashed_{false};

  rclcpp::Publisher<msg::CrashState>::SharedPtr crash_state_pub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::CollisionCrashNode>());
  rclcpp::shutdown();
  return 0;
}
