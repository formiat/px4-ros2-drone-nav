#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <stdexcept>
#include <string>

namespace drone_city_nav {

class CollisionCrashNode final : public rclcpp::Node {
public:
  CollisionCrashNode()
      : Node{"collision_crash_node"} {
    airborne_altitude_m_ = declare_parameter<double>("airborne_altitude_m", 1.0);
    const std::string contacts_topic = declare_parameter<std::string>(
        "contacts_topic", "/drone_city_nav/drone_contacts");
    const std::string vehicle_destroyed_topic = declare_parameter<std::string>(
        "vehicle_destroyed_topic", "/drone_city_nav/vehicle_destroyed");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    const std::string status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status");
    drone_collision_filter_ =
        declare_parameter<std::string>("drone_collision_filter", "");
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 0));
    vehicle_role_ = static_cast<std::uint8_t>(declare_parameter<std::int64_t>(
        "vehicle_role", msg::VehicleDestroyed::ROLE_UNSPECIFIED));
    vehicle_id_ = declare_parameter<std::string>("vehicle_id", "");
    if (vehicle_role_ > msg::VehicleDestroyed::ROLE_EVADER) {
      throw std::invalid_argument{"invalid collision detector vehicle role"};
    }

    vehicle_destroyed_pub_ = create_publisher<msg::VehicleDestroyed>(
        vehicle_destroyed_topic,
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local());
    vehicle_destroyed_sub_ = create_subscription<msg::VehicleDestroyed>(
        vehicle_destroyed_topic,
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(),
        [this](const msg::VehicleDestroyed::SharedPtr destroyed) {
          const bool valid_cause =
              destroyed->death_cause ==
                  msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION ||
              destroyed->death_cause ==
                  msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT ||
              destroyed->death_cause ==
                  msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION;
          const bool matching_role =
              destroyed->vehicle_role == vehicle_role_ ||
              destroyed->vehicle_role == msg::VehicleDestroyed::ROLE_UNSPECIFIED;
          const bool matching_epoch =
              mission_epoch_ == 0U || destroyed->mission_epoch == mission_epoch_;
          const bool matching_id =
              vehicle_id_.empty() || destroyed->vehicle_id == vehicle_id_;
          if (valid_cause && matching_role && matching_epoch && matching_id) {
            destroyed_ = true;
          }
        });
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

    RCLCPP_INFO(get_logger(),
                "Physical collision detector ready: contacts='%s' "
                "vehicle_destroyed='%s' role=%u vehicle_id='%s' "
                "airborne_altitude=%.2fm",
                contacts_topic.c_str(), vehicle_destroyed_topic.c_str(),
                static_cast<unsigned>(vehicle_role_), vehicle_id_.c_str(),
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
    if (destroyed_) {
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
      if (!drone_collision_filter_.empty() &&
          contact.collision1.name.find(drone_collision_filter_) == std::string::npos) {
        continue;
      }
      msg::VehicleDestroyed event;
      event.stamp = contacts.header.stamp;
      if (event.stamp.sec == 0 && event.stamp.nanosec == 0U) {
        event.stamp = now();
      }
      event.mission_epoch = mission_epoch_;
      event.vehicle_role = vehicle_role_;
      event.vehicle_id = vehicle_id_;
      event.death_cause = msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION;
      event.detail = "gazebo_contact";
      event.drone_collision = contact.collision1.name;
      event.obstacle_collision = contact.collision2.name;
      event.altitude_m = altitude_m_;
      event.speed_mps = speed_mps_;
      if (!contact.positions.empty()) {
        event.event_position.x = contact.positions.front().x;
        event.event_position.y = contact.positions.front().y;
        event.event_position.z = contact.positions.front().z;
      }

      destroyed_ = true;
      vehicle_destroyed_pub_->publish(event);
      const double roll = attitude_valid_ ? attitude_.roll_rad
                                          : std::numeric_limits<double>::quiet_NaN();
      const double pitch = attitude_valid_ ? attitude_.pitch_rad
                                           : std::numeric_limits<double>::quiet_NaN();
      const double yaw = attitude_valid_ ? attitude_.yaw_rad
                                         : std::numeric_limits<double>::quiet_NaN();
      RCLCPP_ERROR(get_logger(),
                   "VEHICLE_DESTROYED role=%u vehicle_id='%s' "
                   "cause=physical_collision "
                   "drone_collision='%s' "
                   "obstacle_collision='%s' contact=(%.3f, %.3f, %.3f) altitude=%.2f "
                   "speed=%.2f attitude_rpy=(%.3f, %.3f, %.3f) mission_epoch=%lu",
                   static_cast<unsigned>(event.vehicle_role), event.vehicle_id.c_str(),
                   event.drone_collision.c_str(), event.obstacle_collision.c_str(),
                   event.event_position.x, event.event_position.y,
                   event.event_position.z, event.altitude_m, event.speed_mps, roll,
                   pitch, yaw, static_cast<unsigned long>(event.mission_epoch));
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
  bool destroyed_{false};
  std::string drone_collision_filter_;
  std::string vehicle_id_;
  std::uint64_t mission_epoch_{0U};
  std::uint8_t vehicle_role_{msg::VehicleDestroyed::ROLE_UNSPECIFIED};

  rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_pub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
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
