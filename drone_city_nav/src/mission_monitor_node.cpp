#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/types.hpp"

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] double
speed2D(const px4_msgs::msg::VehicleLocalPosition& position) noexcept {
  if (!std::isfinite(position.vx) || !std::isfinite(position.vy)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::hypot(static_cast<double>(position.vx), static_cast<double>(position.vy));
}

[[nodiscard]] double
altitude(const px4_msgs::msg::VehicleLocalPosition& position) noexcept {
  return position.z_valid && std::isfinite(position.z)
             ? -static_cast<double>(position.z)
             : std::numeric_limits<double>::quiet_NaN();
}

} // namespace

class MissionMonitorNode final : public rclcpp::Node {
public:
  MissionMonitorNode()
      : Node{"mission_monitor_node"} {
    start_ = Point2{declare_parameter<double>("start_x_m", 54.0),
                    declare_parameter<double>("start_y_m", 54.0)};
    goal_ = Point2{declare_parameter<double>("goal_x_m", 216.0),
                   declare_parameter<double>("goal_y_m", 378.0)};
    px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 54.0),
                               declare_parameter<double>("px4_local_origin_y_m", 54.0)};
    spawn_tolerance_m_ = declare_parameter<double>("spawn_tolerance_m", 1.0);
    minimum_movement_m_ = declare_parameter<double>("min_movement_distance_m", 5.0);
    goal_radius_m_ = declare_parameter<double>("goal_radius_m", 2.0);
    stop_speed_mps_ = declare_parameter<double>("stop_speed_mps", 0.6);
    stop_hold_s_ = declare_parameter<double>("stop_hold_s", 2.0);

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        declare_parameter<std::string>("px4_local_position_topic",
                                       "/fmu/out/vehicle_local_position"),
        px4_qos, [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
          onLocalPosition(*message);
        });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        declare_parameter<std::string>("px4_vehicle_status_topic",
                                       "/fmu/out/vehicle_status"),
        px4_qos, [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
          onVehicleStatus(*message);
        });
    vehicle_destroyed_sub_ = create_subscription<msg::VehicleDestroyed>(
        declare_parameter<std::string>("vehicle_destroyed_topic",
                                       "/drone_city_nav/vehicle_destroyed"),
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(),
        [this](const msg::VehicleDestroyed::SharedPtr message) {
          if (!result_reported_) {
            RCLCPP_ERROR(get_logger(),
                         "MISSION_CHECK vehicle_destroyed=true role=%u cause=%u "
                         "drone_collision='%s' obstacle_collision='%s' "
                         "event_position=(%.3f, %.3f, %.3f)",
                         static_cast<unsigned>(message->vehicle_role),
                         static_cast<unsigned>(message->death_cause),
                         message->drone_collision.c_str(),
                         message->obstacle_collision.c_str(), message->event_position.x,
                         message->event_position.y, message->event_position.z);
            report(false, "vehicle_destroyed");
          }
        });
    summary_timer_ =
        create_wall_timer(std::chrono::seconds{5}, [this] { logSummary(); });
    RCLCPP_INFO(get_logger(),
                "Mission monitor ready: start=(%.2f, %.2f) goal=(%.2f, %.2f) "
                "goal_radius=%.2fm",
                start_.x, start_.y, goal_.x, goal_.y, goal_radius_m_);
  }

private:
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus& message) {
    if (message.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      armed_seen_ = true;
    }
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& message) {
    if (result_reported_ || !message.xy_valid || !std::isfinite(message.x) ||
        !std::isfinite(message.y)) {
      return;
    }
    latest_position_ = Point2{static_cast<double>(message.x) + px4_local_origin_.x,
                              static_cast<double>(message.y) + px4_local_origin_.y};
    latest_altitude_m_ = altitude(message);
    latest_speed_mps_ = speed2D(message);
    latest_position_valid_ = true;
    const double start_distance_m = distance(latest_position_, start_);
    const double goal_distance_m = distance(latest_position_, goal_);
    if (!first_position_seen_) {
      first_position_seen_ = true;
      spawn_distance_m_ = start_distance_m;
      spawn_ok_ = spawn_distance_m_ <= spawn_tolerance_m_;
    }
    maximum_distance_from_start_m_ =
        std::max(maximum_distance_from_start_m_, start_distance_m);
    minimum_goal_distance_m_ = std::min(minimum_goal_distance_m_, goal_distance_m);
    maximum_speed_mps_ = std::max(maximum_speed_mps_, latest_speed_mps_);
    if (std::isfinite(latest_speed_mps_)) {
      speed_sum_mps_ += latest_speed_mps_;
      ++speed_samples_;
    }
    moved_ = moved_ || maximum_distance_from_start_m_ >= minimum_movement_m_;
    const bool captured = goal_distance_m <= goal_radius_m_ &&
                          latest_speed_mps_ <= stop_speed_mps_ && armed_seen_;
    const auto now_time = now();
    if (!captured) {
      goal_hold_started_.reset();
      return;
    }
    if (!goal_hold_started_) {
      goal_hold_started_ = now_time;
      return;
    }
    if ((now_time - *goal_hold_started_).seconds() >= stop_hold_s_) {
      report(spawn_ok_ && moved_, spawn_ok_ && moved_ ? "none" : "mission_contract");
    }
  }

  [[nodiscard]] double meanSpeed() const noexcept {
    return speed_samples_ == 0U ? std::numeric_limits<double>::quiet_NaN()
                                : speed_sum_mps_ / static_cast<double>(speed_samples_);
  }

  void report(const bool success, const std::string& reason) {
    result_reported_ = true;
    const char* format =
        "MISSION_RESULT success=%s reason='%s' spawn_distance=%.2f "
        "max_distance_from_start=%.2f min_goal_distance=%.2f "
        "final_position=(%.2f, %.2f) final_altitude=%.2f final_speed=%.2f "
        "max_observed_speed=%.2f mean_observed_speed=%.2f";
    if (success) {
      RCLCPP_INFO(get_logger(), format, "true", reason.c_str(), spawn_distance_m_,
                  maximum_distance_from_start_m_, minimum_goal_distance_m_,
                  latest_position_.x, latest_position_.y, latest_altitude_m_,
                  latest_speed_mps_, maximum_speed_mps_, meanSpeed());
    } else {
      RCLCPP_ERROR(get_logger(), format, "false", reason.c_str(), spawn_distance_m_,
                   maximum_distance_from_start_m_, minimum_goal_distance_m_,
                   latest_position_.x, latest_position_.y, latest_altitude_m_,
                   latest_speed_mps_, maximum_speed_mps_, meanSpeed());
    }
  }

  void logSummary() {
    if (!latest_position_valid_ || result_reported_) {
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Mission summary: spawn_ok=%s moved=%s armed_seen=%s "
                "position=(%.2f, %.2f) altitude=%.2f speed=%.2f "
                "distance_to_goal=%.2f min_goal_distance=%.2f",
                spawn_ok_ ? "true" : "false", moved_ ? "true" : "false",
                armed_seen_ ? "true" : "false", latest_position_.x, latest_position_.y,
                latest_altitude_m_, latest_speed_mps_,
                distance(latest_position_, goal_), minimum_goal_distance_m_);
  }

  Point2 start_{};
  Point2 goal_{};
  Point2 px4_local_origin_{};
  Point2 latest_position_{};
  double spawn_tolerance_m_{1.0};
  double minimum_movement_m_{5.0};
  double goal_radius_m_{2.0};
  double stop_speed_mps_{0.6};
  double stop_hold_s_{2.0};
  double latest_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double latest_speed_mps_{std::numeric_limits<double>::infinity()};
  double spawn_distance_m_{std::numeric_limits<double>::infinity()};
  double maximum_distance_from_start_m_{0.0};
  double minimum_goal_distance_m_{std::numeric_limits<double>::infinity()};
  double maximum_speed_mps_{0.0};
  double speed_sum_mps_{0.0};
  std::size_t speed_samples_{0U};
  bool first_position_seen_{false};
  bool latest_position_valid_{false};
  bool spawn_ok_{false};
  bool moved_{false};
  bool armed_seen_{false};
  bool result_reported_{false};
  std::optional<rclcpp::Time> goal_hold_started_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MissionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
