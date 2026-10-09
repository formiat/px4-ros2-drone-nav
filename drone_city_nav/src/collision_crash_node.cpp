#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
    if (vehicle_role_ > msg::VehicleDestroyed::ROLE_CIVILIAN) {
      throw std::invalid_argument{"invalid collision detector vehicle role"};
    }

    vehicle_destroyed_pub_ = create_publisher<msg::VehicleDestroyed>(
        vehicle_destroyed_topic,
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local());
    // The landing as an event of the mission (specification K26).
    mission_event_pub_ = create_publisher<std_msgs::msg::String>(
        declare_parameter<std::string>("mission_event_topic",
                                       "/drone_city_nav/mission_events"),
        rclcpp::QoS{50}.reliable().transient_local());
    // A contact with a floor is a landing only while the offboard is landing
    // the vehicle (specification A9), which its state says (the blind
    // descent of K19). r1184 sank onto the floor of the staging area in
    // flight, level and at 0.11 m/s, and read as a landing.
    navigation_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        declare_parameter<std::string>("vehicle_navigation_state_topic",
                                       "/drone_city_nav/vehicle_state"),
        rclcpp::QoS{10}.best_effort(),
        [this](const msg::VehicleNavigationState::SharedPtr state) {
          descending_ = state->landing;
        });
    vehicle_destroyed_sub_ = create_subscription<msg::VehicleDestroyed>(
        vehicle_destroyed_topic,
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(),
        [this](const msg::VehicleDestroyed::SharedPtr destroyed) {
          const bool valid_cause =
              destroyed->death_cause ==
                  msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION ||
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
    // The crash detector reads altitude and speed only, so the identity
    // transform reads the autopilot's own origin.
    autopilot_state_source_ = std::make_unique<AutopilotStateSource>(
        *this, Px4MapFrameTransform{},
        AutopilotStateTopics{
            .local_state = local_position_topic,
            .attitude = attitude_topic,
            .status = status_topic,
        },
        px4_qos,
        AutopilotStateCallbacks{
            .local_state =
                [this](const AutopilotLocalState& state) { onLocalState(state); },
            .attitude =
                [this](const AutopilotAttitude& attitude) {
                  const auto euler = quaternionToEuler(attitude.quaternion);
                  if (euler.has_value()) {
                    attitude_ = *euler;
                    attitude_valid_ = true;
                  }
                },
            .status = [this](const AutopilotStatus& status) { armed_ = status.armed; },
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
  void onLocalState(const AutopilotLocalState& state) {
    if (state.altitude_valid && std::isfinite(state.position.z)) {
      altitude_m_ = state.position.z;
      altitude_valid_ = true;
      if (armed_ && altitude_m_ >= airborne_altitude_m_) {
        airborne_seen_ = true;
      }
    }
    if (std::isfinite(state.velocity.x) && std::isfinite(state.velocity.y) &&
        std::isfinite(state.velocity.z)) {
      speed_mps_ =
          std::hypot(std::hypot(state.velocity.x, state.velocity.y), state.velocity.z);
      horizontal_speed_mps_ = std::hypot(state.velocity.x, state.velocity.y);
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

    const double contact_s = rclcpp::Time{contacts.header.stamp}.seconds();
    for (const auto& contact : contacts.contacts) {
      if (!drone_collision_filter_.empty() &&
          contact.collision1.name.find(drone_collision_filter_) == std::string::npos) {
        continue;
      }
      // A landed body stays in contact: r819 rested on the platform it had
      // landed on for 46 s, and when its position source ran out the
      // autopilot's velocity drifted past the landing's and the resting
      // contact read as a collision. Contact kept without a gap is the
      // landing going on.
      const bool body = contact.collision1.name.find("base_link") != std::string::npos;
      if (landed_ && body && contact_s - landed_contact_s_ <= kLandingContactGapS) {
        landed_contact_s_ = contact_s;
        continue;
      }
      // Roadmap item 17 stage 5: a vehicle with no position source lands, and
      // a landing is not a crash. A contact made level and at a landing's
      // speed is one, made by the body: the autopilot's blind landing met
      // the floor at 0.73 m/s (r790), and a contact at speed, tilted or by a
      // rotor (r792, a rotor on a wall at 0.23 m/s) is a collision.
      // Only the ground or a floor carries a landing (the owner's rule of
      // 2026-09-29: every other contact with a surface is a crash): every
      // normal of the contact within 25 degrees of the vertical.
      const bool on_floor =
          !contact.normals.empty() &&
          std::ranges::all_of(contact.normals, [](const auto& normal) {
            return std::abs(normal.z) >=
                   kFloorNormalVerticalShare *
                       std::sqrt(normal.x * normal.x + normal.y * normal.y +
                                 normal.z * normal.z);
          });
      if (body && on_floor && descending_ && attitude_valid_ &&
          std::abs(attitude_.roll_rad) < kLandingTiltRad &&
          std::abs(attitude_.pitch_rad) < kLandingTiltRad &&
          speed_mps_ < kLandingSpeedMps &&
          horizontal_speed_mps_ < kLandingHorizontalSpeedMps) {
        landed_contact_s_ = contact_s;
        if (!landed_) {
          landed_ = true;
          std_msgs::msg::String event;
          event.data = "VEHICLE_LANDED";
          mission_event_pub_->publish(event);
          RCLCPP_WARN(get_logger(),
                      "VEHICLE_LANDED drone_collision='%s' obstacle_collision='%s' "
                      "speed=%.2f horizontal_speed=%.2f attitude_rp=(%.3f, %.3f)",
                      contact.collision1.name.c_str(), contact.collision2.name.c_str(),
                      speed_mps_, horizontal_speed_mps_, attitude_.roll_rad,
                      attitude_.pitch_rad);
        }
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
      std_msgs::msg::String destroyed_event;
      destroyed_event.data = "VEHICLE_DESTROYED";
      mission_event_pub_->publish(destroyed_event);
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
  double horizontal_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  static constexpr double kLandingTiltRad{0.26};
  static constexpr double kLandingSpeedMps{1.0};
  static constexpr double kLandingHorizontalSpeedMps{0.5};
  bool landed_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
  static constexpr double kLandingContactGapS{0.5};
  static constexpr double kFloorNormalVerticalShare{0.9};
  double landed_contact_s_{-1.0};
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
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr navigation_state_sub_;
  bool descending_{false};
  std::unique_ptr<AutopilotStateSource> autopilot_state_source_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::CollisionCrashNode>());
  rclcpp::shutdown();
  return 0;
}
