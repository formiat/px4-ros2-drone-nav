#include "drone_city_nav/types.hpp"

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

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
  double height_m{std::numeric_limits<double>::infinity()};
};

[[nodiscard]] double
speed2D(const px4_msgs::msg::VehicleLocalPosition& position) noexcept {
  if (!std::isfinite(position.vx) || !std::isfinite(position.vy)) {
    return std::numeric_limits<double>::infinity();
  }

  return std::hypot(static_cast<double>(position.vx), static_cast<double>(position.vy));
}

[[nodiscard]] double clearanceToFootprint(const Point2 point,
                                          const BuildingFootprint& building) {
  const double half_x = building.size_x_m * 0.5;
  const double half_y = building.size_y_m * 0.5;
  const double dx_inside = half_x - std::abs(point.x - building.center.x);
  const double dy_inside = half_y - std::abs(point.y - building.center.y);
  if (dx_inside >= 0.0 && dy_inside >= 0.0) {
    // Negative clearance means the drone footprint is horizontally inside the
    // building footprint; positive clearance is distance outside it.
    return -std::min(dx_inside, dy_inside);
  }

  const double dx = std::max(std::abs(point.x - building.center.x) - half_x, 0.0);
  const double dy = std::max(std::abs(point.y - building.center.y) - half_y, 0.0);
  return std::hypot(dx, dy);
}

[[nodiscard]] std::vector<BuildingFootprint>
parseBuildings(const std::vector<double>& values) {
  std::vector<BuildingFootprint> buildings;
  buildings.reserve(values.size() / 4U);
  for (std::size_t i = 0U; i + 3U < values.size(); i += 4U) {
    buildings.push_back(BuildingFootprint{Point2{values[i], values[i + 1U]},
                                          values[i + 2U], values[i + 3U]});
  }
  return buildings;
}

[[nodiscard]] std::vector<BuildingFootprint>
parseBuildingVolumes(const std::vector<double>& values) {
  std::vector<BuildingFootprint> buildings;
  buildings.reserve(values.size() / 5U);
  for (std::size_t i = 0U; i + 4U < values.size(); i += 5U) {
    buildings.push_back(BuildingFootprint{Point2{values[i], values[i + 1U]},
                                          values[i + 2U], values[i + 3U],
                                          values[i + 4U]});
  }
  return buildings;
}

[[nodiscard]] double altitudeFromLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition& position) noexcept {
  if (!position.z_valid || !std::isfinite(position.z)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return -static_cast<double>(position.z);
}

void applyUniformBuildingHeight(std::vector<BuildingFootprint>& buildings,
                                const double uniform_height_m) {
  if (!(uniform_height_m > 0.0)) {
    return;
  }

  for (BuildingFootprint& building : buildings) {
    building.height_m = uniform_height_m;
  }
}

} // namespace

class MissionMonitorNode final : public rclcpp::Node {
public:
  MissionMonitorNode()
      : Node{"mission_monitor_node"} {
    start_ = Point2{declare_parameter<double>("start_x_m", 0.0),
                    declare_parameter<double>("start_y_m", 0.0)};
    goal_ = Point2{declare_parameter<double>("goal_x_m", 85.0),
                   declare_parameter<double>("goal_y_m", 0.0)};
    px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 0.0),
                               declare_parameter<double>("px4_local_origin_y_m", 0.0)};
    spawn_tolerance_m_ = declare_parameter<double>("spawn_tolerance_m", 1.0);
    min_movement_distance_m_ =
        declare_parameter<double>("min_movement_distance_m", 5.0);
    goal_radius_m_ = declare_parameter<double>("goal_radius_m", 2.0);
    stop_speed_mps_ = declare_parameter<double>("stop_speed_mps", 0.6);
    stop_hold_s_ = declare_parameter<double>("stop_hold_s", 2.0);
    building_clearance_m_ = declare_parameter<double>("building_clearance_m", 1.0);
    vertical_clearance_m_ = declare_parameter<double>("vertical_clearance_m", 1.0);
    crash_detection_enabled_ = declare_parameter<bool>("crash_detection_enabled", true);
    crash_min_airborne_altitude_m_ =
        declare_parameter<double>("crash_min_airborne_altitude_m", 6.0);
    crash_altitude_m_ = declare_parameter<double>("crash_altitude_m", 2.5);

    const std::vector<double> building_volume_values =
        declare_parameter<std::vector<double>>("building_volumes",
                                               std::vector<double>{});
    const std::vector<BuildingFootprint> volume_buildings =
        parseBuildingVolumes(building_volume_values);
    if (building_volume_values.size() % 5U != 0U) {
      RCLCPP_WARN(get_logger(), "Ignoring trailing building volume values: count=%zu",
                  building_volume_values.size());
    }

    const std::vector<double> building_values = declare_parameter<std::vector<double>>(
        "building_footprints", std::vector<double>{});
    std::string building_source = "none";
    if (!volume_buildings.empty()) {
      buildings_ = volume_buildings;
      building_source = "building_volumes";
      if (!building_values.empty()) {
        RCLCPP_INFO(
            get_logger(),
            "Ignoring building_footprints because building_volumes are configured: "
            "footprint_values=%zu volume_buildings=%zu",
            building_values.size(), buildings_.size());
      }
    } else {
      buildings_ = parseBuildings(building_values);
      building_source = buildings_.empty() ? "none" : "building_footprints";
      if (building_values.size() % 4U != 0U) {
        RCLCPP_WARN(get_logger(),
                    "Ignoring trailing building footprint values: count=%zu",
                    building_values.size());
      }
    }
    uniform_building_height_m_ =
        declare_parameter<double>("uniform_building_height_m", 0.0);
    applyUniformBuildingHeight(buildings_, uniform_building_height_m_);

    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string vehicle_status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status");
    const std::string emergency_stop_topic = declare_parameter<std::string>(
        "emergency_stop_topic", "/drone_city_nav/emergency_stop");
    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    const auto emergency_stop_qos = rclcpp::QoS{1}.reliable().durability_volatile();

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          onVehicleStatus(*msg);
        });
    emergency_stop_pub_ =
        create_publisher<std_msgs::msg::Bool>(emergency_stop_topic, emergency_stop_qos);

    summary_timer_ =
        create_wall_timer(std::chrono::seconds{5}, [this]() { logSummary(); });

    RCLCPP_INFO(get_logger(),
                "Mission monitor ready: start=(%.2f, %.2f) goal=(%.2f, %.2f) "
                "spawn_tolerance=%.2fm goal_radius=%.2fm building_source=%s "
                "buildings=%zu clearance=%.2fm "
                "vertical_clearance=%.2fm uniform_building_height=%.2fm "
                "crash_detection=%s emergency_stop_topic='%s'",
                start_.x, start_.y, goal_.x, goal_.y, spawn_tolerance_m_,
                goal_radius_m_, building_source.c_str(), buildings_.size(),
                building_clearance_m_, vertical_clearance_m_,
                uniform_building_height_m_, crash_detection_enabled_ ? "true" : "false",
                emergency_stop_topic.c_str());
  }

private:
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus& msg) {
    vehicle_status_ = msg;
    vehicle_status_valid_ = true;
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    if (result_reported_ || !msg.xy_valid || !std::isfinite(msg.x) ||
        !std::isfinite(msg.y)) {
      return;
    }

    const Point2 position{static_cast<double>(msg.x) + px4_local_origin_.x,
                          static_cast<double>(msg.y) + px4_local_origin_.y};
    const double current_speed_mps = speed2D(msg);
    const double current_altitude_m = altitudeFromLocalPosition(msg);
    latest_position_ = position;
    latest_speed_mps_ = current_speed_mps;
    latest_altitude_m_ = current_altitude_m;
    latest_position_valid_ = true;
    if (std::isfinite(current_speed_mps)) {
      max_observed_speed_mps_ = std::max(max_observed_speed_mps_, current_speed_mps);
      speed_sum_mps_ += current_speed_mps;
      ++speed_sample_count_;
    }

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
    if (!movement_ok_ && max_distance_from_start_m_ >= min_movement_distance_m_) {
      movement_ok_ = true;
      RCLCPP_INFO(get_logger(),
                  "MISSION_CHECK movement_ok=true max_distance_from_start=%.2f",
                  max_distance_from_start_m_);
    }

    updateBuildingClearance(position, current_altitude_m);
    if (collision_detected_) {
      reportFailure("building clearance violation");
      return;
    }
    updateCrashDetection(current_altitude_m);
    if (crash_detected_) {
      reportFailure("altitude collapse after takeoff");
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

  void updateCrashDetection(const double altitude_m) {
    if (!crash_detection_enabled_ || !std::isfinite(altitude_m)) {
      return;
    }

    max_altitude_m_ = std::max(max_altitude_m_, altitude_m);
    const bool was_airborne = max_altitude_m_ >= crash_min_airborne_altitude_m_;
    if (was_airborne && altitude_m <= crash_altitude_m_) {
      crash_detected_ = true;
      RCLCPP_ERROR(get_logger(),
                   "MISSION_CHECK crash_detected=true altitude=%.2f "
                   "max_altitude=%.2f threshold=%.2f",
                   altitude_m, max_altitude_m_, crash_altitude_m_);
    }
  }

  void updateBuildingClearance(const Point2 position, const double altitude_m) {
    for (const BuildingFootprint& building : buildings_) {
      if (std::isfinite(altitude_m) &&
          altitude_m > building.height_m + vertical_clearance_m_) {
        continue;
      }

      const double clearance_m = clearanceToFootprint(position, building);
      min_building_clearance_m_ = std::min(min_building_clearance_m_, clearance_m);
      if (clearance_m < building_clearance_m_) {
        collision_detected_ = true;
        RCLCPP_ERROR(get_logger(),
                     "MISSION_CHECK collision_risk=true position=(%.2f, %.2f) "
                     "altitude=%.2f building_center=(%.2f, %.2f) "
                     "building_height=%.2f clearance=%.2f required=%.2f",
                     position.x, position.y, altitude_m, building.center.x,
                     building.center.y, building.height_m, clearance_m,
                     building_clearance_m_);
      }
    }
  }

  [[nodiscard]] bool isArmedOrWasArmed() {
    if (vehicle_status_valid_ && vehicle_status_.arming_state ==
                                     px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      armed_seen_ = true;
    }

    return armed_seen_;
  }

  void reportSuccess() {
    result_reported_ = true;
    RCLCPP_INFO(get_logger(),
                "MISSION_RESULT success=true spawn_distance=%.2f "
                "max_distance_from_start=%.2f min_goal_distance=%.2f "
                "min_building_clearance=%.2f final_position=(%.2f, %.2f) "
                "final_altitude=%.2f final_speed=%.2f max_observed_speed=%.2f "
                "mean_observed_speed=%.2f",
                spawn_distance_m_, max_distance_from_start_m_, min_goal_distance_m_,
                min_building_clearance_m_, latest_position_.x, latest_position_.y,
                latest_altitude_m_, latest_speed_mps_, max_observed_speed_mps_,
                meanObservedSpeedMps());
  }

  void reportFailure(const std::string& reason) {
    result_reported_ = true;
    publishEmergencyStop(reason);
    RCLCPP_ERROR(get_logger(),
                 "MISSION_RESULT success=false reason='%s' spawn_distance=%.2f "
                 "max_distance_from_start=%.2f min_goal_distance=%.2f "
                 "min_building_clearance=%.2f latest_position=(%.2f, %.2f) "
                 "latest_altitude=%.2f latest_speed=%.2f max_observed_speed=%.2f "
                 "mean_observed_speed=%.2f",
                 reason.c_str(), spawn_distance_m_, max_distance_from_start_m_,
                 min_goal_distance_m_, min_building_clearance_m_, latest_position_.x,
                 latest_position_.y, latest_altitude_m_, latest_speed_mps_,
                 max_observed_speed_mps_, meanObservedSpeedMps());
  }

  void publishEmergencyStop(const std::string& reason) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    emergency_stop_pub_->publish(msg);
    RCLCPP_ERROR(get_logger(), "MISSION_CHECK emergency_stop=true reason='%s'",
                 reason.c_str());
  }

  void logSummary() {
    if (!latest_position_valid_ || result_reported_) {
      return;
    }
    const bool armed_seen = isArmedOrWasArmed();

    RCLCPP_INFO(
        get_logger(),
        "Mission summary: spawn_ok=%s moved=%s armed_seen=%s "
        "position=(%.2f, %.2f) altitude=%.2f speed=%.2f "
        "max_observed_speed=%.2f mean_observed_speed=%.2f "
        "distance_to_start=%.2f distance_to_goal=%.2f max_distance_from_start=%.2f "
        "min_building_clearance=%.2f",
        spawn_ok_ ? "true" : "false", movement_ok_ ? "true" : "false",
        armed_seen ? "true" : "false", latest_position_.x, latest_position_.y,
        latest_altitude_m_, latest_speed_mps_, max_observed_speed_mps_,
        meanObservedSpeedMps(), distance(latest_position_, start_),
        distance(latest_position_, goal_), max_distance_from_start_m_,
        min_building_clearance_m_);
  }

  [[nodiscard]] double meanObservedSpeedMps() const noexcept {
    if (speed_sample_count_ == 0U) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return speed_sum_mps_ / static_cast<double>(speed_sample_count_);
  }

  std::vector<BuildingFootprint> buildings_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  Point2 start_{};
  Point2 goal_{};
  Point2 px4_local_origin_{};
  Point2 latest_position_{};
  rclcpp::Time goal_stop_start_time_{0, 0, RCL_ROS_TIME};
  double spawn_tolerance_m_{1.0};
  double min_movement_distance_m_{5.0};
  double goal_radius_m_{2.0};
  double stop_speed_mps_{0.6};
  double stop_hold_s_{2.0};
  double building_clearance_m_{1.0};
  double vertical_clearance_m_{1.0};
  double uniform_building_height_m_{0.0};
  double spawn_distance_m_{std::numeric_limits<double>::infinity()};
  double max_distance_from_start_m_{0.0};
  double min_goal_distance_m_{std::numeric_limits<double>::infinity()};
  double min_building_clearance_m_{std::numeric_limits<double>::infinity()};
  double latest_speed_mps_{std::numeric_limits<double>::infinity()};
  double latest_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double max_altitude_m_{0.0};
  double max_observed_speed_mps_{0.0};
  double speed_sum_mps_{0.0};
  std::size_t speed_sample_count_{0U};
  bool latest_position_valid_{false};
  bool vehicle_status_valid_{false};
  bool spawn_checked_{false};
  bool spawn_ok_{false};
  bool movement_ok_{false};
  bool collision_detected_{false};
  bool crash_detected_{false};
  bool goal_stop_started_{false};
  bool result_reported_{false};
  bool armed_seen_{false};
  bool crash_detection_enabled_{true};
  double crash_min_airborne_altitude_m_{6.0};
  double crash_altitude_m_{2.5};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MissionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
