#include "drone_city_nav/mission_waypoint_sequence.hpp"
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
#include <string>
#include <vector>

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
    px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 54.0),
                               declare_parameter<double>("px4_local_origin_y_m", 54.0)};
    spawn_tolerance_m_ = declare_parameter<double>("spawn_tolerance_m", 1.0);
    minimum_movement_m_ = declare_parameter<double>("min_movement_distance_m", 5.0);
    goal_radius_m_ = declare_parameter<double>("goal_radius_m", 2.0);
    stop_speed_mps_ = declare_parameter<double>("stop_speed_mps", 0.6);
    stop_hold_s_ = declare_parameter<double>("stop_hold_s", 2.0);
    shutdown_on_result_ = declare_parameter<bool>("shutdown_on_result", false);
    const std::vector<Point3> waypoints =
        missionWaypointsFromFlatParameters(declare_parameter<std::vector<double>>(
            "mission_goal_sequence_xyz_m", std::vector<double>{}));
    waypoint_sequence_ = std::make_unique<MissionWaypointSequence>(
        waypoints, MissionWaypointSequenceConfig{.goal_radius_m = goal_radius_m_,
                                                 .stop_speed_mps = stop_speed_mps_,
                                                 .stop_hold_s = stop_hold_s_});
    goal_ =
        Point2{waypoint_sequence_->activeGoal().x, waypoint_sequence_->activeGoal().y};

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
                "Mission monitor ready: start=(%.2f, %.2f) waypoint_count=%zu "
                "first_goal=(%.2f, %.2f) goal_radius=%.2fm",
                start_.x, start_.y, waypoint_sequence_->waypointCount(), goal_.x,
                goal_.y, goal_radius_m_);
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
    const auto now_time = now();
    const MissionWaypointUpdate waypoint_update =
        waypoint_sequence_->update(MissionWaypointObservation{
            .stamp_ns = now_time.nanoseconds(),
            .goal_captured = goal_distance_m <= goal_radius_m_ && armed_seen_,
            .horizontal_speed_mps = latest_speed_mps_});
    if (!waypoint_update.waypoint_completed) {
      return;
    }
    if (waypoint_update.advanced) {
      const Point3& next_goal = waypoint_sequence_->activeGoal();
      goal_ = Point2{next_goal.x, next_goal.y};
      minimum_goal_distance_m_ = std::numeric_limits<double>::infinity();
      RCLCPP_INFO(get_logger(),
                  "MISSION_WAYPOINT_REACHED completed_index=%zu waypoint_count=%zu "
                  "next_goal=(%.2f,%.2f,%.2f)",
                  waypoint_update.completed_index, waypoint_sequence_->waypointCount(),
                  next_goal.x, next_goal.y, next_goal.z);
      return;
    }
    if (waypoint_update.mission_completed) {
      RCLCPP_INFO(get_logger(),
                  "MISSION_WAYPOINT_REACHED completed_index=%zu waypoint_count=%zu "
                  "terminal=true",
                  waypoint_update.completed_index, waypoint_sequence_->waypointCount());
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
        "max_distance_from_start=%.2f min_goal_distance=%.2f waypoint_count=%zu "
        "completed_waypoints=%zu "
        "final_position=(%.2f, %.2f) final_altitude=%.2f final_speed=%.2f "
        "max_observed_speed=%.2f mean_observed_speed=%.2f";
    if (success) {
      RCLCPP_INFO(get_logger(), format, "true", reason.c_str(), spawn_distance_m_,
                  maximum_distance_from_start_m_, minimum_goal_distance_m_,
                  waypoint_sequence_->waypointCount(),
                  waypoint_sequence_->completedWaypointCount(), latest_position_.x,
                  latest_position_.y, latest_altitude_m_, latest_speed_mps_,
                  maximum_speed_mps_, meanSpeed());
    } else {
      RCLCPP_ERROR(get_logger(), format, "false", reason.c_str(), spawn_distance_m_,
                   maximum_distance_from_start_m_, minimum_goal_distance_m_,
                   waypoint_sequence_->waypointCount(),
                   waypoint_sequence_->completedWaypointCount(), latest_position_.x,
                   latest_position_.y, latest_altitude_m_, latest_speed_mps_,
                   maximum_speed_mps_, meanSpeed());
    }
    if (shutdown_on_result_) {
      shutdown_timer_ = create_wall_timer(std::chrono::milliseconds{100}, [this] {
        shutdown_timer_->cancel();
        rclcpp::shutdown();
      });
    }
  }

  void logSummary() {
    if (!latest_position_valid_ || result_reported_) {
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Mission summary: spawn_ok=%s moved=%s armed_seen=%s "
                "position=(%.2f, %.2f) altitude=%.2f speed=%.2f "
                "active_waypoint=%zu/%zu distance_to_goal=%.2f min_goal_distance=%.2f",
                spawn_ok_ ? "true" : "false", moved_ ? "true" : "false",
                armed_seen_ ? "true" : "false", latest_position_.x, latest_position_.y,
                latest_altitude_m_, latest_speed_mps_,
                waypoint_sequence_->activeIndex() + 1U,
                waypoint_sequence_->waypointCount(), distance(latest_position_, goal_),
                minimum_goal_distance_m_);
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
  bool shutdown_on_result_{false};
  std::unique_ptr<MissionWaypointSequence> waypoint_sequence_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MissionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
