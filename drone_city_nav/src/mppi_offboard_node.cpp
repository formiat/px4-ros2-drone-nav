#include "drone_city_nav/execution_horizon_admission.hpp"
#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/producer_instance_id.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
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
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <tf2_ros/transform_broadcaster.h>

#include "mppi_offboard_node_names.hpp"

namespace drone_city_nav {

class MppiOffboardNode final : public rclcpp::Node {
public:
  MppiOffboardNode()
      : Node{"mppi_offboard_node"} {
    constexpr std::uint64_t kOffboardFeedbackProducerDomain{0x4f4646424f415244ULL};
    offboard_producer_instance_id_ =
        createProducerInstanceId(kOffboardFeedbackProducerDomain);
    // Takeoff climbs a fixed height above the spawn point in every scenario.
    // The climb has to be a real climb: a vehicle that never leaves its rest
    // never lets the EKF certify a control-grade heading, and the planner has
    // no body axis to seed its proprioceptive footprint from.
    takeoff_climb_m_ = declare_parameter<double>("takeoff_climb_m", 2.0);
    if (!std::isfinite(takeoff_climb_m_) || takeoff_climb_m_ <= 0.0) {
      throw std::invalid_argument{"takeoff climb must be a positive height"};
    }
    flight_envelope_config_.minimum_target_z_m =
        declare_parameter<double>("minimum_target_z_m", 1.0);
    flight_envelope_config_.maximum_target_z_m =
        declare_parameter<double>("maximum_target_z_m", 32.0);
    takeoff_hover_s_ = declare_parameter<double>("takeoff_hover_s", 1.0);
    const double control_lookahead_s =
        declare_parameter<double>("mppi_control_lookahead_s", 0.05);
    const long double control_lookahead_ns =
        static_cast<long double>(control_lookahead_s) * 1'000'000'000.0L;
    if (!std::isfinite(control_lookahead_s) || control_lookahead_s < 0.0 ||
        control_lookahead_ns >
            static_cast<long double>(std::numeric_limits<std::int64_t>::max()) - 0.5L) {
      throw std::invalid_argument{"mppi_control_lookahead_s is invalid"};
    }
    control_lookahead_ns_ =
        static_cast<std::int64_t>(std::floor(control_lookahead_ns + 0.5L));
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
    if (expected_vehicle_role_ > msg::VehicleDestroyed::ROLE_CIVILIAN) {
      throw std::invalid_argument{"invalid offboard vehicle role"};
    }
    rviz_drone_follow_tf_enabled_ =
        declare_parameter<bool>("rviz_drone_follow_tf_enabled", true);
    rviz_drone_follow_parent_frame_ =
        declare_parameter<std::string>("rviz_drone_follow_parent_frame", "gazebo_map");
    rviz_drone_follow_frame_ =
        declare_parameter<std::string>("rviz_drone_follow_frame", "drone_follow");
    px4_map_transform_ = Px4MapFrameTransform{
        .map_origin = Point3{declare_parameter<double>("px4_local_origin_x_m", 54.0),
                             declare_parameter<double>("px4_local_origin_y_m", 54.0),
                             declare_parameter<double>("px4_local_origin_z_m", 0.0)},
        .m00 = declare_parameter<double>("px4_to_map_m00", 1.0),
        .m01 = declare_parameter<double>("px4_to_map_m01", 0.0),
        .m10 = declare_parameter<double>("px4_to_map_m10", 0.0),
        .m11 = declare_parameter<double>("px4_to_map_m11", 1.0),
    };
    if (!insideFlightEnvelope(takeoffAltitudeM(), flight_envelope_config_)) {
      throw std::invalid_argument{"takeoff altitude is outside flight envelope"};
    }
    px4_map_transform_.validate();
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
    require_planner_health_ = declare_parameter<bool>("require_planner_health", true);
    planner_health_timeout_s_ =
        declare_parameter<double>("planner_health_timeout_s", 1.0);
    planner_health_land_after_s_ =
        declare_parameter<double>("planner_health_land_after_s", 5.0);
    planner_health_sub_ = create_subscription<std_msgs::msg::Bool>(
        declare_parameter<std::string>("planner_health_topic",
                                       "/drone_city_nav/mppi/planner_alive"),
        rclcpp::QoS{1}.reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr health) {
          planner_healthy_ = health->data;
          planner_health_received_at_ = std::chrono::steady_clock::now();
        });
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
          onVehicleStatus(*status);
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
                "Production MPPI offboard ready: takeoff_climb_m=%.1f "
                "takeoff_altitude_m=%.1f",
                takeoff_climb_m_, takeoffAltitudeM());
  }

private:
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus& status) {
    const bool was_armed =
        vehicle_status_seen_ && vehicle_status_.arming_state ==
                                    px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool armed =
        status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    vehicle_status_ = status;
    vehicle_status_seen_ = true;
    if (!armed && (was_armed || horizon_.has_value())) {
      execution_horizon_rearm_required_ = true;
      horizon_.reset();
      unavailable_path_hold_target_.reset();
      if (horizon_admission_.current_producer_instance_id != 0U) {
        static_cast<void>(tombstoneExecutionHorizonIdentity(
            horizon_admission_,
            ExecutionHorizonAdmissionCandidate{
                .producer_instance_id = horizon_admission_.current_producer_instance_id,
                .sequence = horizon_admission_.current_sequence,
                .source_stamp_ns = horizon_admission_.latest_source_stamp_ns,
                .valid_from_ns = horizon_admission_.latest_valid_from_ns,
                .content_fingerprint = horizon_admission_.current_content_fingerprint,
            }));
      }
      RCLCPP_WARN(get_logger(),
                  "EXECUTION_HORIZON cleared=true reason=vehicle_disarmed "
                  "action=require_new_identity_after_rearm");
    } else if (!was_armed && armed && execution_horizon_rearm_required_) {
      execution_horizon_rearm_required_ = false;
      RCLCPP_INFO(get_logger(), "EXECUTION_HORIZON rearm_observed=true "
                                "action=wait_for_new_identity");
    }
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& state) {
    if (!state.xy_valid || !state.z_valid || !state.v_xy_valid || !state.v_z_valid) {
      position_valid_ = false;
      return;
    }
    local_x_ = state.x;
    local_y_ = state.y;
    altitude_m_ = -static_cast<double>(state.z);
    const Point2 map_velocity = px4_map_transform_.localVectorToMap(
        Point2{static_cast<double>(state.vx), static_cast<double>(state.vy)});
    velocity_x_ = map_velocity.x;
    velocity_y_ = map_velocity.y;
    velocity_up_mps_ = -static_cast<double>(state.vz);
    const bool finite_heading = std::isfinite(state.heading);
    if (finite_heading) {
      heading_rad_ = px4_map_transform_.px4HeadingToMapYaw(state.heading);
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
    const Point2 map_position =
        px4_map_transform_.localPositionToMap(Point2{local_x_, local_y_});
    state.position.x = map_position.x;
    state.position.y = map_position.y;
    state.position.z = mapAltitudeM();
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
    RCLCPP_INFO(get_logger(), "BOOTSTRAP_TAKEOFF_READINESS ready=%s",
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
    const Point2 map_position =
        px4_map_transform_.localPositionToMap(Point2{local_x_, local_y_});
    return Point3{map_position.y, map_position.x, mapAltitudeM()};
  }

  void onHorizon(const msg::MppiTrajectoryHorizon& horizon) {
    if (horizon.target_offboard_instance_id != offboard_producer_instance_id_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "EXECUTION_HORIZON rejected producer=%" PRIu64 " sequence=%" PRIu64
          " target_offboard=%" PRIu64 " current_offboard=%" PRIu64
          " reason=non_current_offboard_session",
          horizon.producer_instance_id, horizon.sequence,
          horizon.target_offboard_instance_id, offboard_producer_instance_id_);
      return;
    }

    const ExecutionHorizonAdmissionCandidate candidate{
        .producer_instance_id = horizon.producer_instance_id,
        .sequence = horizon.sequence,
        .source_stamp_ns = executionHorizonTimeNanoseconds(horizon.header.stamp),
        .valid_from_ns = executionHorizonTimeNanoseconds(horizon.valid_from),
        .content_fingerprint = executionHorizonContentFingerprint(horizon),
    };
    const std::uint64_t previous_sequence = horizon_admission_.current_sequence;
    const ExecutionHorizonPayloadStatus payload_status = assessExecutionHorizonPayload(
        horizon, ExecutionHorizonPayloadValidationConfig{
                     .expected_frame_id = "map",
                     .flight_envelope = &flight_envelope_config_,
                 });
    const bool revoked =
        horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
    const bool expired =
        !revoked &&
        now().nanoseconds() >= executionHorizonTimeNanoseconds(horizon.valid_until);
    const bool rearm_blocked = execution_horizon_rearm_required_ && !revoked;
    const bool payload_admissible =
        payload_status == ExecutionHorizonPayloadStatus::kValid && !expired &&
        !rearm_blocked;
    const ExecutionHorizonAdmissionResult admission = admitExecutionHorizonIdentity(
        horizon_admission_, candidate, payload_admissible);
    if (admission.state_advanced) {
      horizon_admission_ = admission.next_state;
    }
    if (admission.revoke) {
      // A newer current-producer identity or an ambiguity is authoritative even
      // when its payload is unusable. A rejected prospective producer does not
      // receive authority and therefore cannot clear the resident horizon.
      horizon_.reset();
    }
    if (admission.replay) {
      // An exact replay proves no new lease or evidence. A rejected exact
      // identity remains tombstoned and cannot be repaired in place.
      return;
    }
    if (!admission.accept_identity &&
        (!candidate.valid() || payload_admissible || admission.stale ||
         admission.conflict || admission.retired_capacity_exhausted ||
         admission.prospective_capacity_exhausted)) {
      const char* const reason = admission.conflict ? "identity_content_conflict"
                                 : admission.stale  ? "stale_identity"
                                 : admission.retired_capacity_exhausted
                                     ? "retired_identity_capacity_exhausted"
                                 : admission.prospective_capacity_exhausted
                                     ? "prospective_identity_capacity_exhausted"
                                     : "invalid_identity";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "EXECUTION_HORIZON rejected producer=%" PRIu64
                           " sequence=%" PRIu64 " current_producer=%" PRIu64
                           " current_sequence=%" PRIu64 " reason=%s",
                           horizon.producer_instance_id, horizon.sequence,
                           horizon_admission_.current_producer_instance_id,
                           horizon_admission_.current_sequence, reason);
      return;
    }
    if (payload_status != ExecutionHorizonPayloadStatus::kValid || expired ||
        rearm_blocked) {
      const std::string_view rejection_reason =
          payload_status != ExecutionHorizonPayloadStatus::kValid
              ? executionHorizonPayloadStatusName(payload_status)
          : expired ? std::string_view{"expired_validity_window"}
                    : std::string_view{"vehicle_disarmed_require_new_identity"};
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "EXECUTION_HORIZON rejected sequence=%" PRIu64
                           " previous=%" PRIu64 " reason=%.*s",
                           horizon.sequence, previous_sequence,
                           static_cast<int>(rejection_reason.size()),
                           rejection_reason.data());
      return;
    }
    if (!admission.payload_installable) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "EXECUTION_HORIZON rejected producer=%" PRIu64
                           " sequence=%" PRIu64
                           " reason=non_advancing_timestamp_identity_tombstoned",
                           horizon.producer_instance_id, horizon.sequence);
      return;
    }
    if (revoked) {
      horizon_.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "EXECUTION_HORIZON revoked=true producer=%" PRIu64
                           " sequence=%" PRIu64
                           " reason=%s action=local_non_authoritative_hold",
                           horizon.producer_instance_id, horizon.sequence,
                           executionReasonName(horizon.execution_reason));
      return;
    }
    const bool execution_changed =
        !horizon_.has_value() || horizon_->execution_mode != horizon.execution_mode ||
        horizon_->execution_reason != horizon.execution_reason;
    horizon_ = horizon;
    unavailable_path_hold_target_.reset();
    RCLCPP_INFO(get_logger(),
                "EXECUTION_HORIZON accepted=true producer=%" PRIu64 " sequence=%" PRIu64
                " mode=%s",
                horizon.producer_instance_id, horizon.sequence,
                executionModeName(horizon.execution_mode));
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
    return now_ns >= executionHorizonTimeNanoseconds(horizon_->valid_from) &&
           now_ns < executionHorizonTimeNanoseconds(horizon_->valid_until);
  }

  [[nodiscard]] bool stationaryPositionHoldActive() const noexcept {
    return horizon_.has_value() && horizon_->stationary_position_hold && horizonFresh();
  }

  [[nodiscard]] double currentSpeedMps() const noexcept {
    return std::hypot(std::hypot(velocity_x_, velocity_y_), velocity_up_mps_);
  }

  [[nodiscard]] bool plannedFinitePathFresh() const {
    return horizon_.has_value() &&
           horizon_->execution_mode ==
               msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
           horizonFresh();
  }

  [[nodiscard]] bool plannedFinitePathCompleted() const {
    return horizon_.has_value() &&
           horizon_->execution_mode ==
               msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
           now().nanoseconds() >=
               executionHorizonTimeNanoseconds(horizon_->valid_until);
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
      publishUnavailableControlFeedback();
      return;
    }
    const bool takeoff_ready =
        position_valid_ && takeoff_complete_stamp_.has_value() &&
        (now() - *takeoff_complete_stamp_).seconds() >= takeoff_hover_s_;
    const bool planner_heartbeat_fresh =
        planner_health_received_at_.has_value() &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      *planner_health_received_at_)
                .count() <= planner_health_timeout_s_;
    const bool planner_authorized =
        !require_planner_health_ || (planner_healthy_ && planner_heartbeat_fresh);
    if (takeoff_ready && !planner_authorized) {
      if (!planner_health_loss_started_at_.has_value()) {
        planner_health_loss_started_at_ = std::chrono::steady_clock::now();
        RCLCPP_ERROR(get_logger(), "PLANNER_HEALTH lost=true action=position_hold");
      }
      publishUnavailablePathHoldSetpoint();
      const double loss_s =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        *planner_health_loss_started_at_)
              .count();
      if (!planner_health_land_sent_ && loss_s >= planner_health_land_after_s_) {
        publishCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0F);
        planner_health_land_sent_ = true;
        RCLCPP_ERROR(get_logger(), "PLANNER_HEALTH lost=true action=land");
      }
      publishUnavailableControlFeedback();
      return;
    }
    planner_health_loss_started_at_.reset();
    const bool navigating =
        takeoff_ready && (!require_mission_start_signal_ || mission_started_);
    const bool stationary_position_hold = navigating && stationaryPositionHoldActive();
    const bool planned_path_fresh = navigating && plannedFinitePathFresh();
    const bool planned_path_completed = navigating && plannedFinitePathCompleted();
    OffboardSetpointMode mode{OffboardSetpointMode::kPositionHold};
    if (planned_path_fresh) {
      mode = OffboardSetpointMode::kTrajectoryPositionTracking;
    }
    offboard_mode_pub_->publish(buildOffboardControlMode(nowMicros(), mode));
    bool exact_horizon_feedback_published{false};
    if (!navigating) {
      publishTakeoffSetpoint();
      if (takeoff_ready && require_mission_start_signal_ && !mission_started_) {
        exact_horizon_feedback_published = publishPrestartPlannedHorizonReceipt();
      }
      if (position_valid_ && altitude_m_ >= takeoff_climb_m_ - 0.5 &&
          !takeoff_complete_stamp_.has_value()) {
        takeoff_complete_stamp_ = now();
      }
    } else if (stationary_position_hold) {
      exact_horizon_feedback_published = publishStationaryPositionHoldSetpoint();
    } else if (planned_path_completed) {
      publishCompletedFinitePathHoldSetpoint();
    } else {
      exact_horizon_feedback_published = publishHorizonSetpoint();
      if (!exact_horizon_feedback_published) {
        publishUnavailablePathHoldSetpoint();
      }
    }
    if (!exact_horizon_feedback_published) {
      publishUnavailableControlFeedback();
    }
    if (warmup_count_ < warmup_setpoints_) {
      ++warmup_count_;
      return;
    }
    const rclcpp::Time current = now();
    if ((current - last_command_time_).seconds() < command_resend_period_s_) {
      return;
    }
    if (!planner_authorized) {
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
    // The PX4 local frame rests at zero altitude on the spawn support, so the
    // climb is the local altitude of the takeoff setpoint.
    setpoint_pub_->publish(buildPositionTrajectorySetpoint(
        nowMicros(), Point2{local_x_, local_y_}, takeoff_climb_m_,
        px4_map_transform_.mapYawToPx4Heading(heading_rad_)));
  }

  [[nodiscard]] bool publishPrestartPlannedHorizonReceipt() {
    if (!plannedFinitePathFresh()) {
      return false;
    }
    // The takeoff/hover setpoint was emitted immediately before this receipt.
    // It proves delivery of the exact planned identity for cooperative startup,
    // but remains non-authoritative until mission-start permits trajectory use.
    publishAppliedControlFeedback(Point2{}, 0.0, 0.0, 0.0, false,
                                  msg::MppiControlFeedback::EXECUTION_MODE_PLANNED);
    return true;
  }

  [[nodiscard]] bool publishStationaryPositionHoldSetpoint() {
    if (!horizon_.has_value()) {
      return false;
    }
    const geometry_msgs::msg::Point& target = horizon_.value().stationary_hold_position;
    const Point2 local_target =
        px4_map_transform_.mapPositionToLocal(Point2{target.x, target.y});
    setpoint_pub_->publish(buildPositionTrajectorySetpoint(
        nowMicros(), local_target, target.z - px4_map_transform_.map_origin.z,
        px4_map_transform_.mapYawToPx4Heading(heading_rad_)));
    // This exact non-authoritative witness is valid only because the matching
    // certified stationary setpoint was emitted immediately above.
    publishAppliedControlFeedback(
        Point2{}, 0.0, 0.0, 0.0, false,
        msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD);
    return true;
  }

  void publishCompletedFinitePathHoldSetpoint() {
    if (!horizon_.has_value() || horizon_->points.empty()) {
      return;
    }
    const msg::MppiHorizonPoint& terminal = horizon_->points.back();
    const Point2 local_target = px4_map_transform_.mapPositionToLocal(
        Point2{terminal.position.x, terminal.position.y});
    setpoint_pub_->publish(buildPositionTrajectorySetpoint(
        nowMicros(), local_target,
        terminal.position.z - px4_map_transform_.map_origin.z,
        px4_map_transform_.mapYawToPx4Heading(terminal.yaw_rad)));
    const Point2 map_position =
        px4_map_transform_.localPositionToMap(Point2{local_x_, local_y_});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "FINITE_EXECUTION_PATH terminal_hold=true sequence=%" PRIu64
                         " target=(%.3f,%.3f,%.3f) current=(%.3f,%.3f,%.3f) speed=%.3f",
                         horizon_admission_.current_sequence, terminal.position.x,
                         terminal.position.y, terminal.position.z, map_position.x,
                         map_position.y, mapAltitudeM(), currentSpeedMps());
  }

  [[nodiscard]] bool publishHorizonSetpoint() {
    if (!horizon_.has_value() || !horizonFresh()) {
      return false;
    }
    const msg::MppiTrajectoryHorizon& horizon = horizon_.value();
    const std::int64_t elapsed_without_lookahead_ns =
        now().nanoseconds() - executionHorizonTimeNanoseconds(horizon.valid_from);
    const std::int64_t elapsed_ns =
        control_lookahead_ns_ >
                std::numeric_limits<std::int64_t>::max() - elapsed_without_lookahead_ns
            ? std::numeric_limits<std::int64_t>::max()
            : elapsed_without_lookahead_ns + control_lookahead_ns_;
    const std::optional<ExecutionHorizonBracket> bracket = executionHorizonBracketAt(
        horizon.points.size(), horizon.control_interval_ns, elapsed_ns);
    if (!bracket || !bracket->valid()) {
      return false;
    }
    const auto& first = horizon.points[bracket->lower_index];
    const auto& second = horizon.points[bracket->upper_index];
    const double ratio = bracket->ratio();
    const Point2 map_position{interpolate(first.position.x, second.position.x, ratio),
                              interpolate(first.position.y, second.position.y, ratio)};
    const Point2 local_position = px4_map_transform_.mapPositionToLocal(map_position);
    const double altitude = interpolate(first.position.z, second.position.z, ratio) -
                            px4_map_transform_.map_origin.z;
    const Point2 map_velocity{interpolate(first.velocity.x, second.velocity.x, ratio),
                              interpolate(first.velocity.y, second.velocity.y, ratio)};
    const Point2 velocity = px4_map_transform_.mapVectorToLocal(map_velocity);
    const double vertical_velocity =
        interpolate(first.velocity.z, second.velocity.z, ratio);
    const Point2 map_acceleration{
        interpolate(first.acceleration.x, second.acceleration.x, ratio),
        interpolate(first.acceleration.y, second.acceleration.y, ratio)};
    const Point2 acceleration = px4_map_transform_.mapVectorToLocal(map_acceleration);
    const double vertical_acceleration =
        interpolate(first.acceleration.z, second.acceleration.z, ratio);
    const double yaw = px4_map_transform_.mapYawToPx4Heading(
        interpolateYawShortestPath(first.yaw_rad, second.yaw_rad, ratio));
    const double map_yaw_rate =
        interpolate(first.yaw_rate_radps, second.yaw_rate_radps, ratio);
    const double map_yaw_acceleration = interpolate(
        first.yaw_acceleration_radps2, second.yaw_acceleration_radps2, ratio);
    const double yaw_rate = px4_map_transform_.mapYawRateToPx4(map_yaw_rate);
    setpoint_pub_->publish(buildMppiPathTrajectorySetpoint(
        nowMicros(), local_position, altitude, velocity, vertical_velocity,
        acceleration, vertical_acceleration, yaw, yaw_rate));
    publishAppliedControlFeedback(map_acceleration, vertical_acceleration, map_yaw_rate,
                                  map_yaw_acceleration, true,
                                  msg::MppiControlFeedback::EXECUTION_MODE_PLANNED);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "OFFBOARD_PLANNED_HORIZON_APPLIED producer=%" PRIu64
                         " sequence=%" PRIu64,
                         horizon_admission_.current_producer_instance_id,
                         horizon_admission_.current_sequence);
    return true;
  }

  void publishUnavailablePathHoldSetpoint() {
    if (!unavailable_path_hold_target_.has_value()) {
      unavailable_path_hold_target_ = Point3{
          local_x_,
          local_y_,
          altitude_m_,
      };
      const Point2 map_target = px4_map_transform_.localPositionToMap(
          Point2{unavailable_path_hold_target_->x, unavailable_path_hold_target_->y});
      RCLCPP_WARN(get_logger(),
                  "FINITE_EXECUTION_PATH unavailable=true "
                  "authority=local_non_authoritative action=position_hold "
                  "target=(%.3f,%.3f,%.3f)",
                  map_target.x, map_target.y,
                  unavailable_path_hold_target_->z + px4_map_transform_.map_origin.z);
    }
    setpoint_pub_->publish(buildPositionTrajectorySetpoint(
        nowMicros(),
        Point2{unavailable_path_hold_target_->x, unavailable_path_hold_target_->y},
        unavailable_path_hold_target_->z,
        px4_map_transform_.mapYawToPx4Heading(heading_rad_)));
  }

  void publishAppliedControlFeedback(const Point2 acceleration,
                                     const double vertical_acceleration,
                                     const double yaw_rate,
                                     const double yaw_acceleration,
                                     const bool control_authoritative,
                                     const std::uint8_t execution_mode) {
    if (!applied_control_feedback_pub_) {
      return;
    }
    msg::MppiControlFeedback feedback;
    feedback.header.stamp = now();
    feedback.header.frame_id = applied_control_feedback_frame_id_;
    feedback.producer_instance_id = offboard_producer_instance_id_;
    feedback.horizon_producer_instance_id =
        horizon_admission_.current_producer_instance_id;
    feedback.horizon_sequence = horizon_admission_.current_sequence;
    feedback.execution_mode = execution_mode;
    feedback.control_authoritative = control_authoritative;
    feedback.acceleration.x = acceleration.x;
    feedback.acceleration.y = acceleration.y;
    feedback.acceleration.z = vertical_acceleration;
    feedback.yaw_rate_radps = static_cast<float>(yaw_rate);
    feedback.yaw_acceleration_radps2 = static_cast<float>(yaw_acceleration);
    applied_control_feedback_pub_->publish(feedback);
  }

  void publishOffboardSessionHeartbeat() {
    if (!applied_control_feedback_pub_) {
      return;
    }
    msg::MppiControlFeedback heartbeat;
    heartbeat.header.stamp = now();
    heartbeat.header.frame_id = applied_control_feedback_frame_id_;
    heartbeat.producer_instance_id = offboard_producer_instance_id_;
    heartbeat.horizon_producer_instance_id = 0U;
    heartbeat.horizon_sequence = 0U;
    heartbeat.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD;
    heartbeat.control_authoritative = false;
    applied_control_feedback_pub_->publish(heartbeat);
  }

  void publishUnavailableControlFeedback() {
    // Takeoff, destruction, expired/completed horizons, and local fallback
    // holds are not evidence that any planner horizon was applied.
    publishOffboardSessionHeartbeat();
  }

  [[nodiscard]] double mapAltitudeM() const noexcept {
    return altitude_m_ + px4_map_transform_.map_origin.z;
  }

  [[nodiscard]] double takeoffAltitudeM() const noexcept {
    return px4_map_transform_.map_origin.z + takeoff_climb_m_;
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

  double takeoff_climb_m_{2.0};
  FlightEnvelopeConfig flight_envelope_config_{};
  double takeoff_hover_s_{1.0};
  std::int64_t control_lookahead_ns_{50'000'000};
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
  bool execution_horizon_rearm_required_{false};
  bool destruction_disarm_confirmed_logged_{false};
  bool require_mission_start_signal_{false};
  bool require_planner_health_{true};
  bool mission_started_{false};
  bool planner_healthy_{false};
  double planner_health_timeout_s_{1.0};
  double planner_health_land_after_s_{5.0};
  bool planner_health_land_sent_{false};
  std::optional<std::chrono::steady_clock::time_point> planner_health_received_at_;
  std::optional<std::chrono::steady_clock::time_point> planner_health_loss_started_at_;
  Px4MapFrameTransform px4_map_transform_{};
  VehicleCommandEndpoint endpoint_{};
  std::unique_ptr<VehicleDestructionDisarmLifecycle> destruction_disarm_lifecycle_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  std::optional<msg::MppiTrajectoryHorizon> horizon_;
  std::optional<Point3> unavailable_path_hold_target_;
  std::optional<rclcpp::Time> takeoff_complete_stamp_;
  std::optional<bool> last_navigation_readiness_;
  std::uint64_t offboard_producer_instance_id_{0U};
  ExecutionHorizonAdmissionState horizon_admission_{};
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planner_health_sub_;
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
