#include "drone_city_nav/msg/crash_state.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/px4_offboard_setpoint_io.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <tf2_ros/transform_broadcaster.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t timeNanoseconds(const builtin_interfaces::msg::Time& time) {
  return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.nanosec);
}

[[nodiscard]] bool finitePoint(const msg::MppiHorizonPoint& point) {
  return std::isfinite(point.time_from_start_s) && std::isfinite(point.velocity.x) &&
         std::isfinite(point.velocity.y) && std::isfinite(point.velocity.z) &&
         std::isfinite(point.acceleration.x) && std::isfinite(point.acceleration.y) &&
         std::isfinite(point.acceleration.z) && std::isfinite(point.yaw_rad) &&
         std::isfinite(point.yaw_rate_radps);
}

[[nodiscard]] double interpolate(const double first, const double second,
                                 const double ratio) {
  return first + (second - first) * ratio;
}

} // namespace

class MppiOffboardNode final : public rclcpp::Node {
public:
  MppiOffboardNode()
      : Node{"mppi_offboard_node"} {
    initial_altitude_m_ = declare_parameter<double>("initial_altitude_m", 18.0);
    takeoff_hover_s_ = declare_parameter<double>("takeoff_hover_s", 1.0);
    horizon_max_receive_age_s_ =
        declare_parameter<double>("mppi_horizon_max_receive_age_s", 0.20);
    control_lookahead_s_ = declare_parameter<double>("mppi_control_lookahead_s", 0.05);
    fallback_braking_acceleration_mps2_ =
        declare_parameter<double>("mppi_fallback_braking_acceleration_mps2", 8.0);
    warmup_setpoints_ =
        static_cast<int>(declare_parameter<std::int64_t>("warmup_setpoints", 20));
    command_resend_period_s_ =
        declare_parameter<double>("command_resend_period_s", 2.0);
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_offboard_ = declare_parameter<bool>("auto_offboard", true);
    rviz_drone_follow_tf_enabled_ =
        declare_parameter<bool>("rviz_drone_follow_tf_enabled", true);
    rviz_drone_follow_parent_frame_ =
        declare_parameter<std::string>("rviz_drone_follow_parent_frame", "gazebo_map");
    rviz_drone_follow_frame_ =
        declare_parameter<std::string>("rviz_drone_follow_frame", "drone_follow");
    if (rviz_drone_follow_tf_enabled_) {
      rviz_drone_follow_tf_broadcaster_ =
          std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    rviz_drone_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        declare_parameter<std::string>("rviz_drone_marker_topic",
                                       "/drone_city_nav/drone_marker"),
        rclcpp::QoS{1}.reliable());
    endpoint_.target_system =
        static_cast<std::uint8_t>(declare_parameter<int>("target_system", 1));
    endpoint_.target_component =
        static_cast<std::uint8_t>(declare_parameter<int>("target_component", 1));
    endpoint_.source_system =
        static_cast<std::uint8_t>(declare_parameter<int>("source_system", 1));
    endpoint_.source_component =
        static_cast<std::uint16_t>(declare_parameter<int>("source_component", 1));

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    horizon_sub_ = create_subscription<msg::MppiTrajectoryHorizon>(
        declare_parameter<std::string>("mppi_execution_horizon_topic",
                                       "/drone_city_nav/mppi/execution_horizon"),
        rclcpp::QoS{2}.reliable(),
        [this](const msg::MppiTrajectoryHorizon::SharedPtr horizon) {
          onHorizon(*horizon);
        });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        declare_parameter<std::string>("px4_local_position_topic",
                                       "/fmu/out/vehicle_local_position_v1"),
        px4_qos, [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr state) {
          onLocalPosition(*state);
        });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        declare_parameter<std::string>("px4_vehicle_status_topic",
                                       "/fmu/out/vehicle_status_v1"),
        px4_qos, [this](const px4_msgs::msg::VehicleStatus::SharedPtr status) {
          vehicle_status_ = *status;
        });
    crash_state_sub_ = create_subscription<msg::CrashState>(
        "/drone_city_nav/crash_state",
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(),
        [this](const msg::CrashState::SharedPtr state) { crashed_ = state->crashed; });
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        declare_parameter<std::string>("offboard_control_mode_topic",
                                       "/fmu/in/offboard_control_mode"),
        px4_qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        declare_parameter<std::string>("trajectory_setpoint_topic",
                                       "/fmu/in/trajectory_setpoint"),
        px4_qos);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        declare_parameter<std::string>("vehicle_command_topic",
                                       "/fmu/in/vehicle_command"),
        px4_qos);
    last_command_time_ =
        now() - rclcpp::Duration::from_seconds(command_resend_period_s_);
    timer_ =
        create_wall_timer(std::chrono::milliseconds{20}, [this]() { controlTick(); });
    RCLCPP_INFO(get_logger(),
                "Production MPPI offboard ready: altitude=%.1f horizon_age=%.2fs "
                "braking=%.1fmps2",
                initial_altitude_m_, horizon_max_receive_age_s_,
                fallback_braking_acceleration_mps2_);
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& state) {
    if (!state.xy_valid || !state.z_valid || !state.v_xy_valid || !state.v_z_valid) {
      position_valid_ = false;
      return;
    }
    local_x_ = state.x;
    local_y_ = state.y;
    altitude_m_ = -static_cast<double>(state.z);
    velocity_x_ = state.vx;
    velocity_y_ = state.vy;
    velocity_up_mps_ = -static_cast<double>(state.vz);
    heading_rad_ = std::isfinite(state.heading) ? state.heading : heading_rad_;
    position_valid_ = true;
    publishRvizDroneFollowTransform();
    publishRvizDroneMarker();
  }

  void publishRvizDroneFollowTransform() {
    if (!rviz_drone_follow_tf_broadcaster_ || !position_valid_) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = rviz_drone_follow_parent_frame_;
    transform.child_frame_id = rviz_drone_follow_frame_;
    // This visualization-only frame follows the Gazebo-aligned RViz convention.
    transform.transform.translation.x = local_y_;
    transform.transform.translation.y = local_x_;
    transform.transform.translation.z = altitude_m_;
    transform.transform.rotation.w = 1.0;
    rviz_drone_follow_tf_broadcaster_->sendTransform(transform);
  }

  void publishRvizDroneMarker() {
    if (!rviz_drone_marker_pub_ || !position_valid_) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = rviz_drone_follow_parent_frame_;
    marker.ns = "drone";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = local_y_;
    marker.pose.position.y = local_x_;
    marker.pose.position.z = altitude_m_;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.45;
    marker.color.r = 0.15F;
    marker.color.g = 0.65F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    rviz_drone_marker_pub_->publish(marker);
  }

  void onHorizon(const msg::MppiTrajectoryHorizon& horizon) {
    if (horizon.sequence <= horizon_sequence_ || horizon.header.frame_id != "map" ||
        horizon.points.size() < 2U ||
        timeNanoseconds(horizon.valid_until) <= timeNanoseconds(horizon.valid_from) ||
        !std::ranges::all_of(horizon.points, finitePoint)) {
      return;
    }
    horizon_ = horizon;
    horizon_sequence_ = horizon.sequence;
    horizon_receive_ns_ = now().nanoseconds();
  }

  [[nodiscard]] bool horizonFresh() const {
    if (!horizon_.has_value()) {
      return false;
    }
    const std::int64_t now_ns = now().nanoseconds();
    const double receive_age_s =
        static_cast<double>(now_ns - horizon_receive_ns_) / 1.0e9;
    return receive_age_s >= 0.0 && receive_age_s <= horizon_max_receive_age_s_ &&
           now_ns >= timeNanoseconds(horizon_->valid_from) &&
           now_ns < timeNanoseconds(horizon_->valid_until);
  }

  void controlTick() {
    if (crashed_) {
      publishCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                     0.0F);
      return;
    }
    const bool navigating =
        position_valid_ && altitude_m_ >= initial_altitude_m_ - 0.5 &&
        takeoff_complete_stamp_.has_value() &&
        (now() - *takeoff_complete_stamp_).seconds() >= takeoff_hover_s_;
    const OffboardSetpointMode mode = navigating ? OffboardSetpointMode::kVelocityCruise
                                                 : OffboardSetpointMode::kPositionHold;
    offboard_mode_pub_->publish(buildOffboardControlMode(nowMicros(), mode));
    if (!navigating) {
      publishTakeoffSetpoint();
      if (position_valid_ && altitude_m_ >= initial_altitude_m_ - 0.5 &&
          !takeoff_complete_stamp_.has_value()) {
        takeoff_complete_stamp_ = now();
      }
    } else if (!publishHorizonSetpoint()) {
      publishBrakingSetpoint();
    }
    if (warmup_count_ < warmup_setpoints_) {
      ++warmup_count_;
      return;
    }
    const rclcpp::Time current = now();
    if ((current - last_command_time_).seconds() < command_resend_period_s_) {
      return;
    }
    if (auto_offboard_ && vehicle_status_.nav_state !=
                              px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      publishCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F,
                     6.0F);
      last_command_time_ = current;
      return;
    }
    if (auto_arm_ && vehicle_status_.arming_state !=
                         px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      publishCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                     1.0F);
      last_command_time_ = current;
    }
  }

  void publishTakeoffSetpoint() {
    setpoint_pub_->publish(buildPositionTrajectorySetpoint(
        nowMicros(), Point2{local_x_, local_y_}, initial_altitude_m_, heading_rad_));
  }

  [[nodiscard]] bool publishHorizonSetpoint() {
    if (!horizon_.has_value() || !horizonFresh()) {
      return false;
    }
    const msg::MppiTrajectoryHorizon& horizon = horizon_.value();
    const double elapsed_s =
        static_cast<double>(now().nanoseconds() - timeNanoseconds(horizon.valid_from)) /
            1.0e9 +
        control_lookahead_s_;
    const auto upper = std::ranges::find_if(
        horizon.points, [elapsed_s](const msg::MppiHorizonPoint& point) {
          return static_cast<double>(point.time_from_start_s) >= elapsed_s;
        });
    const std::size_t upper_index =
        upper == horizon.points.end()
            ? horizon.points.size() - 1U
            : static_cast<std::size_t>(std::distance(horizon.points.begin(), upper));
    const std::size_t lower_index = upper_index > 0U ? upper_index - 1U : 0U;
    const auto& first = horizon.points[lower_index];
    const auto& second = horizon.points[upper_index];
    const double duration =
        static_cast<double>(second.time_from_start_s - first.time_from_start_s);
    const double ratio =
        duration > 1.0e-6
            ? std::clamp((elapsed_s - first.time_from_start_s) / duration, 0.0, 1.0)
            : 0.0;
    setpoint_pub_->publish(buildMppiTrajectorySetpoint(
        nowMicros(),
        Point2{interpolate(first.velocity.x, second.velocity.x, ratio),
               interpolate(first.velocity.y, second.velocity.y, ratio)},
        interpolate(first.velocity.z, second.velocity.z, ratio),
        Point2{interpolate(first.acceleration.x, second.acceleration.x, ratio),
               interpolate(first.acceleration.y, second.acceleration.y, ratio)},
        interpolate(first.acceleration.z, second.acceleration.z, ratio),
        interpolate(first.yaw_rad, second.yaw_rad, ratio),
        interpolate(first.yaw_rate_radps, second.yaw_rate_radps, ratio)));
    return true;
  }

  void publishBrakingSetpoint() {
    const double speed =
        std::hypot(std::hypot(velocity_x_, velocity_y_), velocity_up_mps_);
    if (speed <= 0.15) {
      setpoint_pub_->publish(buildPositionTrajectorySetpoint(
          nowMicros(), Point2{local_x_, local_y_}, altitude_m_, heading_rad_));
      return;
    }
    constexpr double kControlPeriodS{0.02};
    const double scale = std::max(0.0, 1.0 - fallback_braking_acceleration_mps2_ *
                                                 kControlPeriodS / speed);
    setpoint_pub_->publish(buildMppiTrajectorySetpoint(
        nowMicros(), Point2{velocity_x_ * scale, velocity_y_ * scale},
        velocity_up_mps_ * scale,
        Point2{-fallback_braking_acceleration_mps2_ * velocity_x_ / speed,
               -fallback_braking_acceleration_mps2_ * velocity_y_ / speed},
        -fallback_braking_acceleration_mps2_ * velocity_up_mps_ / speed, heading_rad_,
        0.0));
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPPI_HORIZON_DEADLINE_MISSED action=dynamic_braking sequence=%" PRIu64
        " speed=%.2f",
        horizon_sequence_, speed);
  }

  void publishCommand(const std::uint32_t command, const float param1,
                      const float param2 = 0.0F) {
    command_pub_->publish(
        buildVehicleCommand(nowMicros(), command, param1, param2, endpoint_));
  }

  [[nodiscard]] std::uint64_t nowMicros() const {
    return static_cast<std::uint64_t>(std::max<std::int64_t>(0, now().nanoseconds()) /
                                      1000);
  }

  double initial_altitude_m_{18.0};
  double takeoff_hover_s_{1.0};
  double horizon_max_receive_age_s_{0.20};
  double control_lookahead_s_{0.05};
  double fallback_braking_acceleration_mps2_{8.0};
  double command_resend_period_s_{2.0};
  double local_x_{0.0};
  double local_y_{0.0};
  double altitude_m_{0.0};
  double velocity_x_{0.0};
  double velocity_y_{0.0};
  double velocity_up_mps_{0.0};
  double heading_rad_{0.0};
  int warmup_setpoints_{20};
  int warmup_count_{0};
  bool auto_arm_{true};
  bool auto_offboard_{true};
  bool rviz_drone_follow_tf_enabled_{true};
  bool position_valid_{false};
  bool crashed_{false};
  VehicleCommandEndpoint endpoint_{};
  px4_msgs::msg::VehicleStatus vehicle_status_;
  std::optional<msg::MppiTrajectoryHorizon> horizon_;
  std::optional<rclcpp::Time> takeoff_complete_stamp_;
  std::uint64_t horizon_sequence_{0U};
  std::int64_t horizon_receive_ns_{0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::string rviz_drone_follow_parent_frame_{"gazebo_map"};
  std::string rviz_drone_follow_frame_{"drone_follow"};
  std::unique_ptr<tf2_ros::TransformBroadcaster> rviz_drone_follow_tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_drone_marker_pub_;
  rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr horizon_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::CrashState>::SharedPtr crash_state_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MppiOffboardNode>());
  rclcpp::shutdown();
  return 0;
}
