#include "mppi_offboard_node.hpp"

namespace drone_city_nav {

MppiOffboardNode::MppiOffboardNode()
    : Node{"mppi_offboard_node"} {
  constexpr std::uint64_t kOffboardFeedbackProducerDomain{0x4f4646424f415244ULL};
  offboard_producer_instance_id_ =
      createProducerInstanceId(kOffboardFeedbackProducerDomain);
  // Takeoff is a real climb above the spawn point: at rest the EKF never
  // certifies a control-grade heading. A spawn below the envelope's floor
  // climbs to that floor plus the capture tolerance, where routes start.
  // The yaw rate's lead is the airframe's: the lag of its autopilot's yaw
  // rate loop (0.26 s for the x500 as tuned here, r825), fitted per vehicle.
  yaw_rate_lead_ns_ = static_cast<std::int64_t>(
      1.0e9 * std::max(0.0, declare_parameter<double>("yaw_rate_lead_s", 0.26)));
  takeoff_climb_m_ = declare_parameter<double>("takeoff_climb_m", 2.0);
  if (!std::isfinite(takeoff_climb_m_) || takeoff_climb_m_ <= 0.0) {
    throw std::invalid_argument{"takeoff climb must be a positive height"};
  }
  flight_envelope_config_.minimum_target_z_m =
      declare_parameter<double>("minimum_target_z_m", 1.0);
  flight_envelope_config_.maximum_target_z_m =
      declare_parameter<double>("maximum_target_z_m", 32.0);
  takeoff_hover_s_ = declare_parameter<double>("takeoff_hover_s", 1.0);
  // The deceleration the hold brakes with while nothing owns the vehicle's
  // motion: the same horizontal acceleration the navigation stack plans its
  // stopping distances against and PX4 is configured to.
  unavailable_path_braking_acceleration_mps2_ =
      declare_parameter<double>("unavailable_path_braking_acceleration_mps2", 4.0);
  if (!std::isfinite(unavailable_path_braking_acceleration_mps2_) ||
      unavailable_path_braking_acceleration_mps2_ <= 0.0) {
    throw std::invalid_argument{
        "unavailable path braking acceleration must be a positive acceleration"};
  }
  // The time the hold brings a residual velocity to zero over: full braking
  // above response times acceleration, proportional below it, so a vehicle
  // that has all but stopped is settled rather than thrown the other way.
  unavailable_path_braking_response_s_ =
      declare_parameter<double>("unavailable_path_braking_response_s", 0.25);
  if (!std::isfinite(unavailable_path_braking_response_s_) ||
      unavailable_path_braking_response_s_ <= 0.0) {
    throw std::invalid_argument{
        "unavailable path braking response must be a positive time"};
  }
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
  command_resend_period_s_ = declare_parameter<double>("command_resend_period_s", 2.0);
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
  gazebo_aligned_rviz_axes_swapped_ = declare_parameter<bool>(
      std::string{kGazeboAlignedRvizAxesSwappedParameter}, true);
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
        planner_health_received_at_ = now();
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
      [this](const msg::MppiTrajectoryHorizon::SharedPtr horizon,
             const rclcpp::MessageInfo& info) {
        horizon_delivery_ms_.add(transportDeliveryLatencyMs(info));
        onHorizon(*horizon);
      });
  autopilot_state_source_ = std::make_unique<AutopilotStateSource>(
      *this, px4_map_transform_,
      AutopilotStateTopics{
          .local_state = declare_parameter<std::string>(
              "px4_local_position_topic", "/fmu/out/vehicle_local_position_v1"),
          .status = declare_parameter<std::string>("px4_vehicle_status_topic",
                                                   "/fmu/out/vehicle_status_v1"),
      },
      px4_qos,
      AutopilotStateCallbacks{
          .local_state =
              [this](const AutopilotLocalState& state) { onLocalState(state); },
          .status =
              [this](const AutopilotStatus& status) { onAutopilotStatus(status); },
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
        RCLCPP_ERROR(get_logger(),
                     "VEHICLE_DESTROYED latched=true role=%s vehicle_id='%s' cause=%s "
                     "mission_epoch=%" PRIu64 " detail='%s' "
                     "drone_collision='%s' obstacle_collision='%s'",
                     vehicleRoleName(destroyed_role_), destroyed->vehicle_id.c_str(),
                     vehicleDeathCauseName(destroyed_cause_),
                     destruction_mission_epoch_, destruction_detail_.c_str(),
                     destroyed->drone_collision.c_str(),
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
  // The position source's poses to the autopilot: quality 0 is dead reckoning.
  visual_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      declare_parameter<std::string>("px4_visual_odometry_topic",
                                     "/fmu/in/vehicle_visual_odometry"),
      px4_qos, [this](const px4_msgs::msg::VehicleOdometry::SharedPtr odometry) {
        dead_reckoning_landing_.observe(now().nanoseconds(), odometry->quality == 0);
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
  last_command_time_ = now() - rclcpp::Duration::from_seconds(command_resend_period_s_);
  timer_ = create_timer(std::chrono::milliseconds{20}, [this]() { controlTick(); });
  RCLCPP_INFO(get_logger(),
              "Production MPPI offboard ready: takeoff_climb_m=%.1f "
              "takeoff_altitude_m=%.1f",
              takeoff_climb_m_, takeoffAltitudeM());
}

void MppiOffboardNode::controlTick() {
  const bool armed = vehicle_status_.armed;
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
  const bool takeoff_hovered =
      takeoff_complete_stamp_.has_value() &&
      (now() - *takeoff_complete_stamp_).seconds() >= takeoff_hover_s_;
  const bool takeoff_ready = position_valid_ && takeoff_hovered;
  const bool planner_heartbeat_fresh =
      planner_health_received_at_.has_value() &&
      (now() - *planner_health_received_at_).seconds() <= planner_health_timeout_s_;
  const bool planner_authorized =
      !require_planner_health_ || (planner_healthy_ && planner_heartbeat_fresh);
  if (takeoff_ready && !planner_authorized) {
    if (!planner_health_loss_started_at_.has_value()) {
      planner_health_loss_started_at_ = now();
      RCLCPP_ERROR(get_logger(), "PLANNER_HEALTH lost=true action=position_hold");
    }
    publishUnavailablePathHoldSetpoint();
    const double loss_s = (now() - *planner_health_loss_started_at_).seconds();
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
  blind_landing_ = takeoff_hovered &&
                   (!require_mission_start_signal_ || mission_started_) &&
                   dead_reckoning_landing_.due(now().nanoseconds());
  OffboardSetpointMode mode{OffboardSetpointMode::kPositionHold};
  if (blind_landing_) {
    mode = OffboardSetpointMode::kVelocityCruise;
  } else if (planned_path_fresh) {
    mode = OffboardSetpointMode::kTrajectoryPositionTracking;
  }
  offboard_mode_pub_->publish(buildOffboardControlMode(nowMicros(), mode));
  bool exact_horizon_feedback_published{false};
  if (blind_landing_) {
    publishDeadReckoningLandingSetpoint();
  } else if (!navigating) {
    publishTakeoffSetpoint();
    if (takeoff_ready && require_mission_start_signal_ && !mission_started_) {
      exact_horizon_feedback_published = publishPrestartPlannedHorizonReceipt();
    }
    if (position_valid_ &&
        mapAltitudeM() >= takeoffAltitudeM() - kTakeoffCaptureToleranceM &&
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
  if (auto_offboard_ && !vehicle_status_.external_control) {
    publishCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
    last_command_time_ = current;
    return;
  }
  if (auto_arm_ && !vehicle_status_.armed) {
    publishCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                   1.0F);
    last_command_time_ = current;
  }
}

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MppiOffboardNode>());
  rclcpp::shutdown();
  return 0;
}
