#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/px4_offboard_setpoint_io.hpp"
#include "drone_city_nav/vehicle_destruction_disarm_lifecycle.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <tf2_ros/transform_broadcaster.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t timeNanoseconds(const builtin_interfaces::msg::Time& time) {
  return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.nanosec);
}

[[nodiscard]] bool finitePoint(const msg::MppiHorizonPoint& point) {
  return std::isfinite(point.time_from_start_s) && std::isfinite(point.position.x) &&
         std::isfinite(point.position.y) && std::isfinite(point.position.z) &&
         std::isfinite(point.velocity.x) && std::isfinite(point.velocity.y) &&
         std::isfinite(point.velocity.z) && std::isfinite(point.acceleration.x) &&
         std::isfinite(point.acceleration.y) && std::isfinite(point.acceleration.z) &&
         std::isfinite(point.yaw_rad) && std::isfinite(point.yaw_rate_radps);
}

[[nodiscard]] bool finitePoint(const geometry_msgs::msg::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double interpolate(const double first, const double second,
                                 const double ratio) {
  return first + (second - first) * ratio;
}

[[nodiscard]] const char* executionModeName(const std::uint8_t mode) noexcept {
  switch (mode) {
    case msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED:
      return "planned";
    case msg::MppiTrajectoryHorizon::EXECUTION_MODE_BRAKING:
      return "braking";
    case msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD:
      return "position_hold";
    default:
      return "invalid";
  }
}

[[nodiscard]] const char* executionReasonName(const std::uint8_t reason) noexcept {
  switch (reason) {
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_NONE:
      return "none";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_HORIZON_SAFETY:
      return "horizon_safety";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_GOAL_CAPTURE:
      return "goal_capture";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_GUIDE:
      return "no_guide";
    case msg::MppiTrajectoryHorizon::EXECUTION_REASON_UNAVAILABLE_WORLD:
      return "unavailable_world";
    default:
      return "invalid";
  }
}

[[nodiscard]] const char* vehicleRoleName(const std::uint8_t role) noexcept {
  switch (role) {
    case msg::VehicleDestroyed::ROLE_UNSPECIFIED:
      return "unspecified";
    case msg::VehicleDestroyed::ROLE_INTERCEPTOR:
      return "interceptor";
    case msg::VehicleDestroyed::ROLE_EVADER:
      return "evader";
    default:
      return "invalid";
  }
}

[[nodiscard]] const char* vehicleDeathCauseName(const std::uint8_t cause) noexcept {
  switch (cause) {
    case msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION:
      return "physical_collision";
    case msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT:
      return "proximity_intercept";
    case msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION:
      return "proximity_collision";
    default:
      return "invalid";
  }
}

[[nodiscard]] bool validVehicleDeathCause(const std::uint8_t cause) noexcept {
  return cause == msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION ||
         cause == msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT ||
         cause == msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION;
}

} // namespace

class MppiOffboardNode final : public rclcpp::Node {
public:
  MppiOffboardNode()
      : Node{"mppi_offboard_node"} {
    initial_altitude_m_ = declare_parameter<double>("initial_altitude_m", 18.0);
    flight_envelope_config_.minimum_target_z_m =
        declare_parameter<double>("minimum_target_z_m", 1.0);
    flight_envelope_config_.maximum_target_z_m =
        declare_parameter<double>("maximum_target_z_m", 32.0);
    if (!insideFlightEnvelope(initial_altitude_m_, flight_envelope_config_)) {
      throw std::invalid_argument{"takeoff altitude is outside flight envelope"};
    }
    takeoff_hover_s_ = declare_parameter<double>("takeoff_hover_s", 1.0);
    control_lookahead_s_ = declare_parameter<double>("mppi_control_lookahead_s", 0.05);
    fallback_braking_acceleration_mps2_ =
        declare_parameter<double>("mppi_fallback_braking_acceleration_mps2", 8.0);
    warmup_setpoints_ =
        static_cast<int>(declare_parameter<std::int64_t>("warmup_setpoints", 20));
    command_resend_period_s_ =
        declare_parameter<double>("command_resend_period_s", 2.0);
    destruction_disarm_lifecycle_ = std::make_unique<VehicleDestructionDisarmLifecycle>(
        VehicleDestructionDisarmConfig{.retry_period_s = declare_parameter<double>(
                                           "death_force_disarm_retry_period_s", 0.2)});
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_offboard_ = declare_parameter<bool>("auto_offboard", true);
    expected_vehicle_role_ = static_cast<std::uint8_t>(declare_parameter<std::int64_t>(
        "vehicle_role", msg::VehicleDestroyed::ROLE_UNSPECIFIED));
    expected_vehicle_id_ = declare_parameter<std::string>("vehicle_id", "");
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 0));
    if (expected_vehicle_role_ > msg::VehicleDestroyed::ROLE_EVADER) {
      throw std::invalid_argument{"invalid offboard vehicle role"};
    }
    rviz_drone_follow_tf_enabled_ =
        declare_parameter<bool>("rviz_drone_follow_tf_enabled", true);
    rviz_drone_follow_parent_frame_ =
        declare_parameter<std::string>("rviz_drone_follow_parent_frame", "gazebo_map");
    rviz_drone_follow_frame_ =
        declare_parameter<std::string>("rviz_drone_follow_frame", "drone_follow");
    px4_local_origin_.x = declare_parameter<double>("px4_local_origin_x_m", 54.0);
    px4_local_origin_.y = declare_parameter<double>("px4_local_origin_y_m", 54.0);
    if (rviz_drone_follow_tf_enabled_) {
      rviz_drone_follow_tf_broadcaster_ =
          std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    rviz_drone_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        declare_parameter<std::string>("rviz_drone_marker_topic",
                                       "/drone_city_nav/drone_marker"),
        rclcpp::QoS{1}.reliable());
    rviz_drone_marker_id_ = static_cast<std::int32_t>(
        declare_parameter<std::int64_t>("rviz_drone_marker_id", 0));
    rviz_drone_marker_color_r_ =
        declare_parameter<double>("rviz_drone_marker_color_r", 0.15);
    rviz_drone_marker_color_g_ =
        declare_parameter<double>("rviz_drone_marker_color_g", 0.65);
    rviz_drone_marker_color_b_ =
        declare_parameter<double>("rviz_drone_marker_color_b", 1.0);
    navigation_state_pub_ = create_publisher<msg::VehicleNavigationState>(
        declare_parameter<std::string>("vehicle_navigation_state_topic",
                                       "/drone_city_nav/vehicle_state"),
        rclcpp::QoS{10}.best_effort());
    navigation_readiness_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter<std::string>("navigation_readiness_topic",
                                       "/drone_city_nav/navigation_ready"),
        rclcpp::QoS{1}.reliable().transient_local());
    publishNavigationReadiness(false);
    applied_control_feedback_frame_id_ =
        declare_parameter<std::string>("applied_control_feedback_frame_id", "map");
    applied_control_feedback_pub_ = create_publisher<msg::MppiControlFeedback>(
        declare_parameter<std::string>("applied_control_feedback_topic",
                                       "/drone_city_nav/mppi/applied_control"),
        rclcpp::QoS{10}.reliable());
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
          vehicle_status_seen_ = true;
        });
    vehicle_destroyed_sub_ = create_subscription<msg::VehicleDestroyed>(
        declare_parameter<std::string>("vehicle_destroyed_topic",
                                       "/drone_city_nav/vehicle_destroyed"),
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(),
        [this](const msg::VehicleDestroyed::SharedPtr destroyed) {
          const bool expected_role =
              expected_vehicle_role_ == msg::VehicleDestroyed::ROLE_UNSPECIFIED ||
              destroyed->vehicle_role == expected_vehicle_role_;
          const bool expected_epoch =
              mission_epoch_ == 0U || destroyed->mission_epoch == mission_epoch_;
          const bool expected_id = expected_vehicle_id_.empty() ||
                                   destroyed->vehicle_id == expected_vehicle_id_;
          if (!validVehicleDeathCause(destroyed->death_cause) || !expected_role ||
              !expected_epoch || !expected_id) {
            RCLCPP_ERROR(get_logger(),
                         "VEHICLE_DESTROYED rejected=true reason=invalid_contract "
                         "cause=%u role=%u vehicle_id='%s' mission_epoch=%" PRIu64
                         " expected_role=%u expected_vehicle_id='%s' "
                         "expected_epoch=%" PRIu64,
                         static_cast<unsigned>(destroyed->death_cause),
                         static_cast<unsigned>(destroyed->vehicle_role),
                         destroyed->vehicle_id.c_str(), destroyed->mission_epoch,
                         static_cast<unsigned>(expected_vehicle_role_),
                         expected_vehicle_id_.c_str(), mission_epoch_);
            return;
          }
          if (destruction_disarm_lifecycle_->latched()) {
            return;
          }
          destroyed_role_ = destroyed->vehicle_role;
          destroyed_cause_ = destroyed->death_cause;
          destruction_detail_ = destroyed->detail;
          destruction_mission_epoch_ = destroyed->mission_epoch;
          destruction_disarm_lifecycle_->latch(now().nanoseconds());
          horizon_.reset();
          auto_arm_ = false;
          auto_offboard_ = false;
          RCLCPP_ERROR(
              get_logger(),
              "VEHICLE_DESTROYED latched=true role=%s vehicle_id='%s' cause=%s "
              "mission_epoch=%" PRIu64 " detail='%s' "
              "drone_collision='%s' obstacle_collision='%s'",
              vehicleRoleName(destroyed_role_), destroyed->vehicle_id.c_str(),
              vehicleDeathCauseName(destroyed_cause_), destruction_mission_epoch_,
              destruction_detail_.c_str(), destroyed->drone_collision.c_str(),
              destroyed->obstacle_collision.c_str());
        });
    require_mission_start_signal_ =
        declare_parameter<bool>("require_mission_start_signal", false);
    mission_start_sub_ = create_subscription<std_msgs::msg::Bool>(
        declare_parameter<std::string>("mission_start_topic",
                                       "/drone_city_nav/mission_start"),
        rclcpp::QoS{1}.reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr start) {
          mission_started_ = start->data;
        });
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
                "Production MPPI offboard ready: altitude=%.1f braking=%.1fmps2",
                initial_altitude_m_, fallback_braking_acceleration_mps2_);
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
    const bool finite_heading = std::isfinite(state.heading);
    if (finite_heading) {
      heading_rad_ = state.heading;
    }
    heading_valid_ = state.heading_good_for_control && finite_heading;
    position_valid_ = true;
    publishNavigationState();
    publishRvizDroneFollowTransform();
    publishRvizDroneMarker();
  }

  void publishNavigationState() {
    if (!navigation_state_pub_) {
      return;
    }
    msg::VehicleNavigationState state;
    state.stamp = now();
    state.position.x = local_x_ + px4_local_origin_.x;
    state.position.y = local_y_ + px4_local_origin_.y;
    state.position.z = altitude_m_;
    state.velocity.x = velocity_x_;
    state.velocity.y = velocity_y_;
    state.velocity.z = velocity_up_mps_;
    state.heading_rad = heading_rad_;
    state.position_valid = position_valid_;
    state.velocity_valid = position_valid_;
    state.heading_valid = heading_valid_;
    state.armed =
        vehicle_status_seen_ && vehicle_status_.arming_state ==
                                    px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    state.airborne = state.armed && altitude_m_ >= 1.0;
    state.navigation_ready =
        state.airborne && takeoff_complete_stamp_.has_value() &&
        (now() - *takeoff_complete_stamp_).seconds() >= takeoff_hover_s_;
    navigation_state_pub_->publish(state);
    publishNavigationReadiness(state.navigation_ready);
  }

  void publishNavigationReadiness(const bool ready) {
    if (last_navigation_readiness_.has_value() &&
        *last_navigation_readiness_ == ready) {
      return;
    }
    std_msgs::msg::Bool readiness;
    readiness.data = ready;
    navigation_readiness_pub_->publish(readiness);
    last_navigation_readiness_ = ready;
    RCLCPP_INFO(get_logger(), "NAVIGATION_READINESS ready=%s",
                ready ? "true" : "false");
  }

  void publishRvizDroneFollowTransform() {
    if (!rviz_drone_follow_tf_broadcaster_ || !position_valid_) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = rviz_drone_follow_parent_frame_;
    transform.child_frame_id = rviz_drone_follow_frame_;
    const Point3 position = rvizDronePosition();
    transform.transform.translation.x = position.x;
    transform.transform.translation.y = position.y;
    transform.transform.translation.z = position.z;
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
    marker.id = rviz_drone_marker_id_;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const Point3 position = rvizDronePosition();
    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.45;
    marker.color.r = static_cast<float>(rviz_drone_marker_color_r_);
    marker.color.g = static_cast<float>(rviz_drone_marker_color_g_);
    marker.color.b = static_cast<float>(rviz_drone_marker_color_b_);
    marker.color.a = 1.0F;
    rviz_drone_marker_pub_->publish(marker);
  }

  [[nodiscard]] Point3 rvizDronePosition() const noexcept {
    return gazeboAlignedRvizPositionFromPx4Local(Point2{local_x_, local_y_},
                                                 px4_local_origin_, altitude_m_);
  }

  void onHorizon(const msg::MppiTrajectoryHorizon& horizon) {
    const char* rejection_reason = nullptr;
    if (horizon.sequence <= horizon_sequence_) {
      rejection_reason = "stale_sequence";
    } else if (horizon.header.frame_id != "map") {
      rejection_reason = "invalid_frame";
    } else if (horizon.points.size() < 2U) {
      rejection_reason = "insufficient_points";
    } else if (timeNanoseconds(horizon.valid_until) <=
               timeNanoseconds(horizon.valid_from)) {
      rejection_reason = "invalid_validity_window";
    } else if (!std::ranges::all_of(horizon.points,
                                    [](const msg::MppiHorizonPoint& point) {
                                      return finitePoint(point);
                                    })) {
      rejection_reason = "non_finite_point";
    } else if (horizon.stationary_position_hold &&
               !finitePoint(horizon.stationary_hold_position)) {
      rejection_reason = "non_finite_hold_target";
    } else if (!std::ranges::all_of(horizon.points,
                                    [this](const msg::MppiHorizonPoint& point) {
                                      return insideFlightEnvelope(
                                          point.position.z, flight_envelope_config_);
                                    })) {
      rejection_reason = "point_outside_flight_envelope";
    } else if (horizon.stationary_position_hold &&
               !insideFlightEnvelope(horizon.stationary_hold_position.z,
                                     flight_envelope_config_)) {
      rejection_reason = "hold_target_outside_flight_envelope";
    } else if (horizon.execution_mode >
               msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD) {
      rejection_reason = "invalid_execution_mode";
    } else if (horizon.execution_reason >
               msg::MppiTrajectoryHorizon::EXECUTION_REASON_UNAVAILABLE_WORLD) {
      rejection_reason = "invalid_execution_reason";
    }
    if (rejection_reason != nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "EXECUTION_HORIZON rejected sequence=%" PRIu64
                           " previous=%" PRIu64 " reason=%s",
                           horizon.sequence, horizon_sequence_, rejection_reason);
      return;
    }
    const bool execution_changed =
        !horizon_.has_value() || horizon_->execution_mode != horizon.execution_mode ||
        horizon_->execution_reason != horizon.execution_reason;
    horizon_ = horizon;
    horizon_sequence_ = horizon.sequence;
    if (execution_changed) {
      RCLCPP_INFO(get_logger(),
                  "EXECUTION_HORIZON mode=%s reason=%s sequence=%" PRIu64 " hold=%s"
                  " target=(%.3f,%.3f,%.3f)",
                  executionModeName(horizon.execution_mode),
                  executionReasonName(horizon.execution_reason), horizon.sequence,
                  horizon.stationary_position_hold ? "true" : "false",
                  horizon.stationary_hold_position.x,
                  horizon.stationary_hold_position.y,
                  horizon.stationary_hold_position.z);
    }
  }

  [[nodiscard]] bool horizonFresh() const {
    if (!horizon_.has_value()) {
      return false;
    }
    const std::int64_t now_ns = now().nanoseconds();
    return now_ns >= timeNanoseconds(horizon_->valid_from) &&
           now_ns < timeNanoseconds(horizon_->valid_until);
  }

  [[nodiscard]] bool stationaryPositionHoldActive() const noexcept {
    return horizon_.has_value() && horizon_->stationary_position_hold;
  }

  void controlTick() {
    const bool armed = vehicle_status_.arming_state ==
                       px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const VehicleDestructionDisarmUpdate destruction_disarm =
        destruction_disarm_lifecycle_->update(now().nanoseconds(), vehicle_status_seen_,
                                              armed);
    if (destruction_disarm.latched) {
      if (destruction_disarm.force_disarm_requested) {
        publishCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                       0.0F, kPx4ForceDisarmMagicParam2);
        RCLCPP_INFO(get_logger(),
                    "VEHICLE_DESTROYED force_disarm_sent=true role=%s cause=%s "
                    "armed=%s mission_epoch=%" PRIu64 " detail='%s'",
                    vehicleRoleName(destroyed_role_),
                    vehicleDeathCauseName(destroyed_cause_),
                    armed ? "true" : "unknown_or_false", destruction_mission_epoch_,
                    destruction_detail_.c_str());
      }
      if (destruction_disarm.confirmed && !destruction_disarm_confirmed_logged_) {
        destruction_disarm_confirmed_logged_ = true;
        RCLCPP_INFO(get_logger(),
                    "VEHICLE_DESTROYED disarm_confirmed=true role=%s cause=%s "
                    "mission_epoch=%" PRIu64 " detail='%s'",
                    vehicleRoleName(destroyed_role_),
                    vehicleDeathCauseName(destroyed_cause_), destruction_mission_epoch_,
                    destruction_detail_.c_str());
      }
      return;
    }
    const bool takeoff_ready =
        position_valid_ && takeoff_complete_stamp_.has_value() &&
        (now() - *takeoff_complete_stamp_).seconds() >= takeoff_hover_s_;
    const bool navigating =
        takeoff_ready && (!require_mission_start_signal_ || mission_started_);
    const bool stationary_position_hold = navigating && stationaryPositionHoldActive();
    const OffboardSetpointMode mode = navigating && !stationary_position_hold
                                          ? OffboardSetpointMode::kVelocityCruise
                                          : OffboardSetpointMode::kPositionHold;
    offboard_mode_pub_->publish(buildOffboardControlMode(nowMicros(), mode));
    if (!navigating) {
      publishTakeoffSetpoint();
      if (position_valid_ && altitude_m_ >= initial_altitude_m_ - 0.5 &&
          !takeoff_complete_stamp_.has_value()) {
        takeoff_complete_stamp_ = now();
      }
    } else if (stationary_position_hold) {
      publishStationaryPositionHoldSetpoint();
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

  void publishStationaryPositionHoldSetpoint() {
    if (!horizon_.has_value()) {
      return;
    }
    const geometry_msgs::msg::Point& target = horizon_.value().stationary_hold_position;
    const Point2 local_target{target.x - px4_local_origin_.x,
                              target.y - px4_local_origin_.y};
    setpoint_pub_->publish(buildPositionTrajectorySetpoint(nowMicros(), local_target,
                                                           target.z, heading_rad_));
    publishAppliedControlFeedback(Point2{}, 0.0, 0.0, false);
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
    const Point2 velocity{interpolate(first.velocity.x, second.velocity.x, ratio),
                          interpolate(first.velocity.y, second.velocity.y, ratio)};
    const double vertical_velocity =
        interpolate(first.velocity.z, second.velocity.z, ratio);
    const Point2 acceleration{
        interpolate(first.acceleration.x, second.acceleration.x, ratio),
        interpolate(first.acceleration.y, second.acceleration.y, ratio)};
    const double vertical_acceleration =
        interpolate(first.acceleration.z, second.acceleration.z, ratio);
    const double yaw = interpolate(first.yaw_rad, second.yaw_rad, ratio);
    const double yaw_rate =
        interpolate(first.yaw_rate_radps, second.yaw_rate_radps, ratio);
    setpoint_pub_->publish(buildMppiTrajectorySetpoint(
        nowMicros(), velocity, vertical_velocity, acceleration, vertical_acceleration,
        yaw, yaw_rate));
    publishAppliedControlFeedback(acceleration, vertical_acceleration, yaw_rate,
                                  horizon.emergency_braking);
    return true;
  }

  void publishBrakingSetpoint() {
    const double speed =
        std::hypot(std::hypot(velocity_x_, velocity_y_), velocity_up_mps_);
    if (speed <= 0.15) {
      setpoint_pub_->publish(buildPositionTrajectorySetpoint(
          nowMicros(), Point2{local_x_, local_y_}, altitude_m_, heading_rad_));
      publishAppliedControlFeedback(Point2{}, 0.0, 0.0, true);
      return;
    }
    constexpr double kControlPeriodS{0.02};
    const double scale = std::max(0.0, 1.0 - fallback_braking_acceleration_mps2_ *
                                                 kControlPeriodS / speed);
    const Point2 acceleration{
        -fallback_braking_acceleration_mps2_ * velocity_x_ / speed,
        -fallback_braking_acceleration_mps2_ * velocity_y_ / speed};
    const double vertical_acceleration =
        -fallback_braking_acceleration_mps2_ * velocity_up_mps_ / speed;
    setpoint_pub_->publish(buildMppiTrajectorySetpoint(
        nowMicros(), Point2{velocity_x_ * scale, velocity_y_ * scale},
        velocity_up_mps_ * scale, acceleration, vertical_acceleration, heading_rad_,
        0.0));
    publishAppliedControlFeedback(acceleration, vertical_acceleration, 0.0, true);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPPI_HORIZON_DEADLINE_MISSED action=dynamic_braking sequence=%" PRIu64
        " speed=%.2f",
        horizon_sequence_, speed);
  }

  void publishAppliedControlFeedback(const Point2 acceleration,
                                     const double vertical_acceleration,
                                     const double yaw_rate,
                                     const bool emergency_braking) {
    if (!applied_control_feedback_pub_) {
      return;
    }
    msg::MppiControlFeedback feedback;
    feedback.header.stamp = now();
    feedback.header.frame_id = applied_control_feedback_frame_id_;
    feedback.horizon_sequence = horizon_sequence_;
    feedback.acceleration.x = acceleration.x;
    feedback.acceleration.y = acceleration.y;
    feedback.acceleration.z = vertical_acceleration;
    feedback.yaw_rate_radps = static_cast<float>(yaw_rate);
    feedback.emergency_braking = emergency_braking;
    applied_control_feedback_pub_->publish(feedback);
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
  FlightEnvelopeConfig flight_envelope_config_{};
  double takeoff_hover_s_{1.0};
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
  bool heading_valid_{false};
  double rviz_drone_marker_color_r_{0.15};
  double rviz_drone_marker_color_g_{0.65};
  double rviz_drone_marker_color_b_{1.0};
  int warmup_setpoints_{20};
  int warmup_count_{0};
  std::int32_t rviz_drone_marker_id_{0};
  bool auto_arm_{true};
  bool auto_offboard_{true};
  bool rviz_drone_follow_tf_enabled_{true};
  bool position_valid_{false};
  bool vehicle_status_seen_{false};
  bool destruction_disarm_confirmed_logged_{false};
  bool require_mission_start_signal_{false};
  bool mission_started_{false};
  Point2 px4_local_origin_{54.0, 54.0};
  VehicleCommandEndpoint endpoint_{};
  std::unique_ptr<VehicleDestructionDisarmLifecycle> destruction_disarm_lifecycle_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  std::optional<msg::MppiTrajectoryHorizon> horizon_;
  std::optional<rclcpp::Time> takeoff_complete_stamp_;
  std::optional<bool> last_navigation_readiness_;
  std::uint64_t horizon_sequence_{0U};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::string rviz_drone_follow_parent_frame_{"gazebo_map"};
  std::string rviz_drone_follow_frame_{"drone_follow"};
  std::string applied_control_feedback_frame_id_{"map"};
  std::string destruction_detail_;
  std::string expected_vehicle_id_;
  std::uint64_t destruction_mission_epoch_{0U};
  std::uint64_t mission_epoch_{0U};
  std::uint8_t destroyed_role_{msg::VehicleDestroyed::ROLE_UNSPECIFIED};
  std::uint8_t destroyed_cause_{0U};
  std::uint8_t expected_vehicle_role_{msg::VehicleDestroyed::ROLE_UNSPECIFIED};
  std::unique_ptr<tf2_ros::TransformBroadcaster> rviz_drone_follow_tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_drone_marker_pub_;
  rclcpp::Publisher<msg::MppiControlFeedback>::SharedPtr applied_control_feedback_pub_;
  rclcpp::Publisher<msg::VehicleNavigationState>::SharedPtr navigation_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_readiness_pub_;
  rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr horizon_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_start_sub_;
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
