#include "drone_city_nav/types.hpp"

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

struct BuildingFootprint {
  Point2 center{};
  double size_x_m{0.0};
  double size_y_m{0.0};
};

[[nodiscard]] double speed2D(
    const px4_msgs::msg::VehicleLocalPosition &position) noexcept {
  if (!std::isfinite(position.vx) || !std::isfinite(position.vy)) {
    return std::numeric_limits<double>::infinity();
  }

  return std::hypot(static_cast<double>(position.vx),
                    static_cast<double>(position.vy));
}

[[nodiscard]] double clearanceToFootprint(const Point2 point,
                                          const BuildingFootprint &building) {
  const double half_x = building.size_x_m * 0.5;
  const double half_y = building.size_y_m * 0.5;
  const double dx_inside = half_x - std::abs(point.x - building.center.x);
  const double dy_inside = half_y - std::abs(point.y - building.center.y);
  if (dx_inside >= 0.0 && dy_inside >= 0.0) {
    return -std::min(dx_inside, dy_inside);
  }

  const double dx = std::max(std::abs(point.x - building.center.x) - half_x,
                             0.0);
  const double dy = std::max(std::abs(point.y - building.center.y) - half_y,
                             0.0);
  return std::hypot(dx, dy);
}

[[nodiscard]] std::vector<BuildingFootprint>
parseBuildings(const std::vector<double> &values) {
  std::vector<BuildingFootprint> buildings;
  buildings.reserve(values.size() / 4U);
  for (std::size_t i = 0U; i + 3U < values.size(); i += 4U) {
    buildings.push_back(BuildingFootprint{
        Point2{values[i], values[i + 1U]}, values[i + 2U], values[i + 3U]});
  }
  return buildings;
}

} // namespace

class MissionMonitorNode final : public rclcpp::Node {
public:
  MissionMonitorNode() : Node{"mission_monitor_node"} {
    start_ = Point2{declare_parameter<double>("start_x_m", 0.0),
                    declare_parameter<double>("start_y_m", 0.0)};
    goal_ = Point2{declare_parameter<double>("goal_x_m", 85.0),
                   declare_parameter<double>("goal_y_m", 0.0)};
    spawn_tolerance_m_ = declare_parameter<double>("spawn_tolerance_m", 1.0);
    min_movement_distance_m_ =
        declare_parameter<double>("min_movement_distance_m", 5.0);
    goal_radius_m_ = declare_parameter<double>("goal_radius_m", 2.0);
    stop_speed_mps_ = declare_parameter<double>("stop_speed_mps", 0.6);
    stop_hold_s_ = declare_parameter<double>("stop_hold_s", 2.0);
    building_clearance_m_ =
        declare_parameter<double>("building_clearance_m", 1.0);

    const std::vector<double> building_values =
        declare_parameter<std::vector<double>>("building_footprints",
                                               std::vector<double>{});
    buildings_ = parseBuildings(building_values);
    if (building_values.size() % 4U != 0U) {
      RCLCPP_WARN(get_logger(),
                  "Ignoring trailing building footprint values: count=%zu",
                  building_values.size());
    }

    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string vehicle_status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status");
    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();

    local_position_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            local_position_topic, px4_qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
              onLocalPosition(*msg);
            });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          onVehicleStatus(*msg);
        });

    summary_timer_ = create_wall_timer(std::chrono::seconds{5},
                                       [this]() { logSummary(); });

    RCLCPP_INFO(
        get_logger(),
        "Mission monitor ready: start=(%.2f, %.2f) goal=(%.2f, %.2f) "
        "spawn_tolerance=%.2fm goal_radius=%.2fm buildings=%zu clearance=%.2fm",
        start_.x, start_.y, goal_.x, goal_.y, spawn_tolerance_m_,
        goal_radius_m_, buildings_.size(), building_clearance_m_);
  }

private:
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus &msg) {
    vehicle_status_ = msg;
    vehicle_status_valid_ = true;
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition &msg) {
    if (result_reported_ || !msg.xy_valid || !std::isfinite(msg.x) ||
        !std::isfinite(msg.y)) {
      return;
    }

    const Point2 position{static_cast<double>(msg.x),
                          static_cast<double>(msg.y)};
    const double current_speed_mps = speed2D(msg);
    latest_position_ = position;
    latest_speed_mps_ = current_speed_mps;
    latest_position_valid_ = true;

    if (!spawn_checked_) {
      spawn_checked_ = true;
      spawn_distance_m_ = distance(position, start_);
      spawn_ok_ = spawn_distance_m_ <= spawn_tolerance_m_;
      if (spawn_ok_) {
        RCLCPP_INFO(get_logger(),
                    "MISSION_CHECK spawn_ok=true position=(%.2f, %.2f) "
                    "distance_to_start=%.2f",
                    position.x, position.y, spawn_distance_m_);
      } else {
        reportFailure("spawn position is outside tolerance");
        return;
      }
    }

    const double movement_distance = distance(position, start_);
    max_distance_from_start_m_ =
        std::max(max_distance_from_start_m_, movement_distance);
    if (!movement_ok_ &&
        max_distance_from_start_m_ >= min_movement_distance_m_) {
      movement_ok_ = true;
      RCLCPP_INFO(get_logger(),
                  "MISSION_CHECK movement_ok=true max_distance_from_start=%.2f",
                  max_distance_from_start_m_);
    }

    updateBuildingClearance(position);
    if (collision_detected_) {
      reportFailure("building clearance violation");
      return;
    }

    const double goal_distance = distance(position, goal_);
    min_goal_distance_m_ = std::min(min_goal_distance_m_, goal_distance);
    const bool stopped_at_goal =
        goal_distance <= goal_radius_m_ && current_speed_mps <= stop_speed_mps_;
    const rclcpp::Time current_time = now();
    if (stopped_at_goal) {
      if (!goal_stop_started_) {
        goal_stop_started_ = true;
        goal_stop_start_time_ = current_time;
      }
    } else {
      goal_stop_started_ = false;
      goal_stop_start_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    }

    const bool goal_stop_held =
        goal_stop_started_ &&
        (current_time - goal_stop_start_time_).seconds() >= stop_hold_s_;
    if (spawn_ok_ && movement_ok_ && goal_stop_held && isArmedOrWasArmed()) {
      reportSuccess();
    }
  }

  void updateBuildingClearance(const Point2 position) {
    for (const BuildingFootprint &building : buildings_) {
      const double clearance_m = clearanceToFootprint(position, building);
      min_building_clearance_m_ =
          std::min(min_building_clearance_m_, clearance_m);
      if (clearance_m < building_clearance_m_) {
        collision_detected_ = true;
        RCLCPP_ERROR(get_logger(),
                     "MISSION_CHECK collision_risk=true position=(%.2f, %.2f) "
                     "building_center=(%.2f, %.2f) clearance=%.2f required=%.2f",
                     position.x, position.y, building.center.x,
                     building.center.y, clearance_m, building_clearance_m_);
      }
    }
  }

  [[nodiscard]] bool isArmedOrWasArmed() {
    if (vehicle_status_valid_ &&
        vehicle_status_.arming_state ==
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      armed_seen_ = true;
    }

    return armed_seen_;
  }

  void reportSuccess() {
    result_reported_ = true;
    RCLCPP_INFO(
        get_logger(),
        "MISSION_RESULT success=true spawn_distance=%.2f "
        "max_distance_from_start=%.2f min_goal_distance=%.2f "
        "min_building_clearance=%.2f final_position=(%.2f, %.2f) "
        "final_speed=%.2f",
        spawn_distance_m_, max_distance_from_start_m_, min_goal_distance_m_,
        min_building_clearance_m_, latest_position_.x, latest_position_.y,
        latest_speed_mps_);
  }

  void reportFailure(const std::string &reason) {
    result_reported_ = true;
    RCLCPP_ERROR(
        get_logger(),
        "MISSION_RESULT success=false reason='%s' spawn_distance=%.2f "
        "max_distance_from_start=%.2f min_goal_distance=%.2f "
        "min_building_clearance=%.2f latest_position=(%.2f, %.2f) "
        "latest_speed=%.2f",
        reason.c_str(), spawn_distance_m_, max_distance_from_start_m_,
        min_goal_distance_m_, min_building_clearance_m_, latest_position_.x,
        latest_position_.y, latest_speed_mps_);
  }

  void logSummary() {
    if (!latest_position_valid_ || result_reported_) {
      return;
    }
    const bool armed_seen = isArmedOrWasArmed();

    RCLCPP_INFO(
        get_logger(),
        "Mission summary: spawn_ok=%s moved=%s armed_seen=%s "
        "position=(%.2f, %.2f) speed=%.2f distance_to_start=%.2f "
        "distance_to_goal=%.2f max_distance_from_start=%.2f "
        "min_building_clearance=%.2f",
        spawn_ok_ ? "true" : "false", movement_ok_ ? "true" : "false",
        armed_seen ? "true" : "false", latest_position_.x,
        latest_position_.y, latest_speed_mps_,
        distance(latest_position_, start_), distance(latest_position_, goal_),
        max_distance_from_start_m_, min_building_clearance_m_);
  }

  std::vector<BuildingFootprint> buildings_;
  px4_msgs::msg::VehicleStatus vehicle_status_{};
  Point2 start_{};
  Point2 goal_{};
  Point2 latest_position_{};
  rclcpp::Time goal_stop_start_time_{0, 0, RCL_ROS_TIME};
  double spawn_tolerance_m_{1.0};
  double min_movement_distance_m_{5.0};
  double goal_radius_m_{2.0};
  double stop_speed_mps_{0.6};
  double stop_hold_s_{2.0};
  double building_clearance_m_{1.0};
  double spawn_distance_m_{std::numeric_limits<double>::infinity()};
  double max_distance_from_start_m_{0.0};
  double min_goal_distance_m_{std::numeric_limits<double>::infinity()};
  double min_building_clearance_m_{std::numeric_limits<double>::infinity()};
  double latest_speed_mps_{std::numeric_limits<double>::infinity()};
  bool latest_position_valid_{false};
  bool vehicle_status_valid_{false};
  bool spawn_checked_{false};
  bool spawn_ok_{false};
  bool movement_ok_{false};
  bool collision_detected_{false};
  bool goal_stop_started_{false};
  bool result_reported_{false};
  bool armed_seen_{false};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

} // namespace drone_city_nav

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MissionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
