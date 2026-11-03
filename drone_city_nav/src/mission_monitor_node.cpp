#include "mission_monitor_node.hpp"

#include "mission_monitor_node_internal.hpp"

namespace drone_city_nav {

using namespace mission_monitor_node_internal;

// The mission's judgement: the vehicle's state, the waypoint acknowledgement, the
// result, the landing and the summary (the class in mission_monitor_node.hpp since
// 2026-11-02, the source size contract).

MissionMonitorNode::MissionMonitorNode()
    : Node{"mission_monitor_node"} {
  start_ = Point2{declare_parameter<double>("start_x_m", 54.0),
                  declare_parameter<double>("start_y_m", 54.0)};
  // PX4 reports NED local positions; the launch derives this matrix from the
  // canonical world so the monitor judges the same map position the planner
  // and the offboard controller use.
  px4_map_transform_ = Px4MapFrameTransform{
      .map_origin = Point3{declare_parameter<double>("px4_local_origin_x_m", 54.0),
                           declare_parameter<double>("px4_local_origin_y_m", 54.0),
                           declare_parameter<double>("px4_local_origin_z_m", 0.0)},
      .m00 = declare_parameter<double>("px4_to_map_m00", 1.0),
      .m01 = declare_parameter<double>("px4_to_map_m01", 0.0),
      .m10 = declare_parameter<double>("px4_to_map_m10", 0.0),
      .m11 = declare_parameter<double>("px4_to_map_m11", 1.0),
  };
  px4_map_transform_.validate();
  spawn_tolerance_m_ = declare_parameter<double>("spawn_tolerance_m", 1.0);
  minimum_movement_m_ = declare_parameter<double>("min_movement_distance_m", 5.0);
  acknowledgement_target_tolerance_m_ =
      declare_parameter<double>("mission_waypoint_target_match_tolerance_m", 1.0e-3);
  goal_capture_radius_m_ =
      declare_parameter<double>("mission_goal_capture_radius_m", 2.0);
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  if (!std::isfinite(acknowledgement_target_tolerance_m_) ||
      acknowledgement_target_tolerance_m_ < 0.0 ||
      !std::isfinite(goal_capture_radius_m_) || goal_capture_radius_m_ <= 0.0 ||
      frame_id_.empty()) {
    throw std::invalid_argument{
        "mission acknowledgement frame and tolerance must be valid"};
  }
  shutdown_on_result_ = declare_parameter<bool>("shutdown_on_result", false);
  const std::string mission_type =
      declare_parameter<std::string>("mission_type", "point_to_point");
  if (mission_type != "point_to_point" && mission_type != "survey") {
    throw std::invalid_argument{"mission_type must be point_to_point or survey"};
  }
  survey_ = mission_type == "survey";
  waypoints_ =
      missionWaypointsFromFlatParameters(declare_parameter<std::vector<double>>(
          "mission_goal_sequence_xyz_m", std::vector<double>{}));
  goal_ = Point2{waypoints_.front().x, waypoints_.front().y};

  const auto px4_qos =
      rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
  autopilot_state_source_ = std::make_unique<AutopilotStateSource>(
      *this, px4_map_transform_,
      AutopilotStateTopics{
          .local_state = declare_parameter<std::string>(
              "px4_local_position_topic", "/fmu/out/vehicle_local_position"),
          .status = declare_parameter<std::string>("px4_vehicle_status_topic",
                                                   "/fmu/out/vehicle_status"),
          .battery = declare_parameter<std::string>("px4_battery_status_topic",
                                                    "/fmu/out/battery_status_v1"),
      },
      px4_qos,
      AutopilotStateCallbacks{
          .local_state =
              [this](const AutopilotLocalState& message) { onLocalState(message); },
          .status =
              [this](const AutopilotStatus& status) {
                // A vehicle that disarms before its mission's end has
                // landed: the autopilot's landing when no position is left
                // (roadmap item 17 stage 5) or the operator's landing ends
                // the mission there. After the result, the disarm is the
                // end of the mission's own landing (K30).
                if (armed_seen_ && !status.armed && !result_reported_) {
                  report(false,
                         operator_landing_ ? "operator_landed" : "vehicle_landed");
                } else if (armed_seen_ && !status.armed && landing_after_result_ &&
                           !landed_reported_) {
                  finishLanding(true);
                }
                armed_seen_ = armed_seen_ || status.armed;
              },
          .battery =
              [this](const AutopilotBattery& battery) {
                main_battery_.observe(now().seconds(), battery.remaining,
                                      battery.time_remaining_s);
              },
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
  waypoint_acknowledgement_sub_ =
      create_subscription<msg::MissionWaypointAcknowledgement>(
          declare_parameter<std::string>(
              "mission_waypoint_acknowledgement_topic",
              "/drone_city_nav/mission_waypoint_acknowledgement"),
          rclcpp::QoS{32}.reliable().transient_local(),
          [this](const msg::MissionWaypointAcknowledgement::SharedPtr message) {
            onWaypointAcknowledgement(*message);
          });
  navigation_health_sub_ = create_subscription<msg::NavigationHealth>(
      declare_parameter<std::string>("navigation_health_topic",
                                     "/drone_city_nav/mppi/navigation_health"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::NavigationHealth::SharedPtr message) {
        onNavigationHealth(*message);
      });
  // Roadmap item 19: the goal proven unreachable is given up for the start;
  // the proof floods the memory the navigation publishes, on its own
  // thread, within a voxel budget. Roadmap item 17 stage 5 gives it up as
  // well for a carried light judged unreliable and for a light's battery
  // that will not reach it; the vehicle has no time limit (specification
  // I8).
  const std::int64_t proof_budget = declare_parameter<std::int64_t>(
      "unreachable_goal_proof_voxel_budget", 20'000'000);
  if (proof_budget <= 0) {
    throw std::invalid_argument{"the proof budget must be valid"};
  }
  proof_voxel_budget_ = static_cast<std::size_t>(proof_budget);
  // A proof holds only once it has stood this long (roadmap item 19's decay
  // clause): a closure seen once decays in one confirmation's time
  // (specification K10), so a proof must outlast it.
  proof_hold_s_ = declare_parameter<double>("unreachable_goal_proof_hold_s", 30.0);
  if (!std::isfinite(proof_hold_s_) || proof_hold_s_ < 0.0) {
    throw std::invalid_argument{"the proof's hold must be valid"};
  }
  // The band of heights the planner flies in bounds the proof's flood.
  flight_envelope_ = FlightEnvelopeConfig{
      .minimum_target_z_m = declare_parameter<double>("minimum_target_z_m", 1.0),
      .maximum_target_z_m = declare_parameter<double>("maximum_target_z_m", 32.0),
  };
  if (evaluateFlightEnvelopeAltitude(flight_envelope_.minimum_target_z_m,
                                     flight_envelope_) ==
      FlightEnvelopeStatus::kInvalidConfiguration) {
    throw std::invalid_argument{"the flight envelope must be valid"};
  }
  navigation_progress_sub_ = create_subscription<msg::NavigationProgress>(
      declare_parameter<std::string>("navigation_progress_topic",
                                     "/drone_city_nav/mppi/navigation_progress"),
      rclcpp::QoS{1}.reliable(),
      [this](const msg::NavigationProgress::SharedPtr message) {
        onNavigationProgress(*message);
      });
  // The charge of the carried light's battery, in seconds of light left:
  // published only where the vehicle flies on its light, so the return on
  // the battery switches itself on with the camera set and off without it.
  light_charge_sub_ = create_subscription<std_msgs::msg::Float64>(
      declare_parameter<std::string>("carried_light_charge_topic",
                                     "/carried_light/charge_s"),
      rclcpp::QoS{1}.reliable(),
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        if (std::isfinite(message->data)) {
          light_charge_s_ = message->data;
        }
      });
  // Roadmap item 29: what the light's driver reads of its output, and the
  // share of the time-of-flight surfaces in the camera's field its frame
  // shows dark (specification K12).
  light_reading_sub_ = create_subscription<std_msgs::msg::Float64>(
      declare_parameter<std::string>("carried_light_output_topic",
                                     "/carried_light/output_share"),
      rclcpp::QoS{1}.reliable(),
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        light_reading_ = message->data;
      });
  unlit_surface_sub_ = create_subscription<std_msgs::msg::Float64>(
      declare_parameter<std::string>("unlit_surface_share_topic",
                                     "/stereo_depth/unlit_surface_share"),
      rclcpp::SensorDataQoS{}, [this](const std_msgs::msg::Float64::SharedPtr message) {
        unlit_surface_share_ = message->data;
      });
  objective_pub_ = create_publisher<msg::NavigationObjective>(
      declare_parameter<std::string>("navigation_objective_topic",
                                     "/drone_city_nav/navigation_objective"),
      rclcpp::QoS{1}.reliable().transient_local());
  // The operator's commands (specification K31): stop, land, home, a new
  // goal; the monitor sends "land" itself at the mission's end (K30).
  const std::string operator_topic = declare_parameter<std::string>(
      "operator_command_topic", "/drone_city_nav/operator_command");
  operator_command_pub_ = create_publisher<std_msgs::msg::String>(
      operator_topic, rclcpp::QoS{10}.reliable().transient_local());
  operator_command_sub_ = create_subscription<std_msgs::msg::String>(
      operator_topic, rclcpp::QoS{10}.reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr command) {
        onOperatorCommand(command->data);
      });
  // The mission's events as the vehicle decides them (specification K26):
  // what a ground station would show, and what the evaluation overlay
  // draws in RViz. Kept for late subscribers.
  mission_event_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("mission_event_topic",
                                     "/drone_city_nav/mission_events"),
      rclcpp::QoS{50}.reliable().transient_local());
  memory_snapshot_sub_ = create_subscription<msg::RawObstacleSnapshot3D>(
      declare_parameter<std::string>("raw_obstacle_snapshot_3d_topic",
                                     "/drone_city_nav/raw_obstacle_snapshot_3d"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::RawObstacleSnapshot3D::SharedPtr message) {
        onMemorySnapshot(*message);
      });
  memory_delta_sub_ = create_subscription<msg::RawObstacleDelta3D>(
      declare_parameter<std::string>("raw_obstacle_delta_3d_topic",
                                     "/drone_city_nav/raw_obstacle_delta_3d"),
      rclcpp::QoS{1}.best_effort().transient_local(),
      [this](const msg::RawObstacleDelta3D::SharedPtr message) {
        onMemoryDelta(*message);
      });
  proof_timer_ = create_timer(std::chrono::seconds{10}, [this] { judgeReturnHome(); });
  summary_timer_ = create_timer(std::chrono::seconds{5}, [this] { logSummary(); });
  // Roadmap item 36: the step acknowledgement follows the judgments (one executor).
  step_acknowledger_ = std::make_unique<StepAcknowledger>(*this, "mission_monitor",
                                                          nullptr, [] { return true; });
  RCLCPP_INFO(get_logger(),
              "Mission monitor ready: start=(%.2f, %.2f) waypoint_count=%zu "
              "first_goal=(%.2f, %.2f)",
              start_.x, start_.y, waypoints_.size(), goal_.x, goal_.y);
}

void MissionMonitorNode::onNavigationHealth(const msg::NavigationHealth& health) {
  if (result_reported_ || health.header.frame_id != frame_id_ ||
      health.producer_instance_id == 0U || health.sequence == 0U ||
      health.stage > msg::NavigationHealth::STAGE_TERMINAL_FAILURE ||
      health.failure_reason >
          msg::NavigationHealth::FAILURE_RECOVERY_BUDGET_EXHAUSTED ||
      !std::isfinite(health.stage_age_ms) || health.stage_age_ms < 0.0) {
    return;
  }
  if (health.producer_instance_id == navigation_health_producer_instance_id_ &&
      health.sequence <= navigation_health_sequence_) {
    return;
  }
  if (health.producer_instance_id != navigation_health_producer_instance_id_) {
    navigation_health_producer_instance_id_ = health.producer_instance_id;
    navigation_health_sequence_ = 0U;
  }
  navigation_health_sequence_ = health.sequence;
  navigation_mission_epoch_ = health.mission_epoch;
  if (health.mission_ready && !navigation_mission_ready_) {
    navigation_mission_ready_ = true;
    // The altitude the mission started at: where the return home ends. The
    // configured start is the pad, and r667 sent the vehicle 2.8 m below
    // its surface, to a goal it could only stand 12 m short of.
    if (latest_position_valid_) {
      mission_ready_altitude_z_m_ = latest_map_position_.z;
    }
    RCLCPP_INFO(get_logger(),
                "MISSION_READINESS ready=true mission_epoch=%" PRIu64
                " health_sequence=%" PRIu64,
                health.mission_epoch, health.sequence);
    publishMissionEvent("MISSION_READINESS ready=true");
  }
  if (health.terminal) {
    report(false, navigationFailureReason(health.failure_reason));
  }
}

void MissionMonitorNode::onLocalState(const AutopilotLocalState& message) {
  if (result_reported_ || !message.position_valid || !message.altitude_valid ||
      !message.velocity_valid || !message.vertical_velocity_valid ||
      !std::isfinite(message.position.x) || !std::isfinite(message.position.y) ||
      !std::isfinite(message.position.z) || !std::isfinite(message.velocity.x) ||
      !std::isfinite(message.velocity.y) || !std::isfinite(message.velocity.z)) {
    latest_position_valid_ = false;
    return;
  }
  const Point3 position{message.position.x, message.position.y, message.position.z};
  if (latest_position_valid_) {
    flown_path_m_ += distance3D(latest_map_position_, position);
  } else if (mission_start_ns_ == 0) {
    mission_start_ns_ = now().nanoseconds();
  }
  latest_map_position_ = position;
  latest_position_stamp_ns_ = now().nanoseconds();
  latest_position_ = Point2{message.position.x, message.position.y};
  if (!goal_substituted_) {
    return_trail_.record(position);
  } else if (!operator_hold_) {
    publishReturnObjective();
  }
  // The contract carries the map altitude; the monitor keeps the altitude
  // above the autopilot's origin it has always reported.
  latest_altitude_m_ = message.position.z - px4_map_transform_.map_origin.z;
  latest_speed_mps_ = std::hypot(std::hypot(message.velocity.x, message.velocity.y),
                                 message.velocity.z);
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
  speed_sum_mps_ += latest_speed_mps_;
  ++speed_samples_;
  moved_ = moved_ || maximum_distance_from_start_m_ >= minimum_movement_m_;
}

void MissionMonitorNode::onWaypointAcknowledgement(
    const msg::MissionWaypointAcknowledgement& acknowledgement) {
  if (result_reported_) {
    return;
  }
  const std::int64_t receive_stamp_ns = now().nanoseconds();
  const std::int64_t source_stamp_ns = timeNanoseconds(acknowledgement.header.stamp);
  const Point3 completed_goal = point3(acknowledgement.completed_goal);
  const Point3 route_target = point3(acknowledgement.route_target);
  const Point3 hold_position = point3(acknowledgement.stationary_hold_position);
  const bool waypoint_index_valid =
      acknowledgement.completed_waypoint_index < waypoints_.size();
  const Point3 expected_goal =
      waypoint_index_valid ? waypoints_[acknowledgement.completed_waypoint_index]
                           : Point3{};
  const bool payload_valid =
      acknowledgement.header.frame_id == frame_id_ &&
      finitePoint(acknowledgement.completed_goal) &&
      finitePoint(acknowledgement.route_target) &&
      finitePoint(acknowledgement.stationary_hold_position) &&
      acknowledgement.waypoint_count == waypoints_.size() && waypoint_index_valid &&
      distance3D(completed_goal, expected_goal) <=
          acknowledgement_target_tolerance_m_ &&
      distance3D(route_target, expected_goal) <= acknowledgement_target_tolerance_m_ &&
      distance3D(hold_position, expected_goal) <= goal_capture_radius_m_;
  const MissionWaypointAcknowledgementCandidate candidate{
      .producer_instance_id = acknowledgement.producer_instance_id,
      .acknowledgement_sequence = acknowledgement.acknowledgement_sequence,
      .content_fingerprint = acknowledgementContentFingerprint(acknowledgement),
      .mission_epoch = acknowledgement.mission_epoch,
      .horizon_producer_instance_id = acknowledgement.horizon_producer_instance_id,
      .horizon_sequence = acknowledgement.horizon_sequence,
      .offboard_producer_instance_id = acknowledgement.offboard_producer_instance_id,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .horizon_valid_from_ns = timeNanoseconds(acknowledgement.horizon_valid_from),
      .horizon_valid_until_ns = timeNanoseconds(acknowledgement.horizon_valid_until),
      .witness_stamp_ns = timeNanoseconds(acknowledgement.witness_stamp),
      .completed_waypoint_index = acknowledgement.completed_waypoint_index,
      .completed_waypoint_count = acknowledgement.completed_waypoint_count,
      .waypoint_count = acknowledgement.waypoint_count,
      .active_waypoint_index = acknowledgement.active_waypoint_index,
      .completed_goal = completed_goal,
      .route_target = route_target,
      .stationary_hold_position = hold_position,
      .mission_completed = acknowledgement.mission_completed,
      .payload_valid = payload_valid,
  };
  const MissionWaypointAcknowledgementAdmissionResult admission =
      admitMissionWaypointAcknowledgement(waypoint_acknowledgement_admission_,
                                          candidate);
  if (admission.state_advanced) {
    waypoint_acknowledgement_admission_ = admission.next_state;
  }
  if (!admission.accept) {
    if (!admission.replay && !admission.aggregate_replay) {
      // A rejected terminal acknowledgement leaves the mission without its
      // landing (r2719: a 300 s hold at the goal), so the line names every
      // predicate of the payload and of the identity's time window.
      RCLCPP_WARN(
          get_logger(),
          "MISSION_WAYPOINT_ACK_REJECTED producer=%" PRIu64 " acknowledgement=%" PRIu64
          " reason=%s payload=%s frame=%s count=%s goal_m=%.3f "
          "route_m=%.3f hold_m=%.2f identity=%s "
          "source_after_valid_from_ms=%.1f valid_until_after_source_ms=%.1f "
          "witness_after_valid_from_ms=%.1f source_after_witness_ms=%.1f "
          "receive_after_source_ms=%.1f",
          acknowledgement.producer_instance_id,
          acknowledgement.acknowledgement_sequence,
          acknowledgementRejectionReason(admission),
          payload_valid ? "valid" : "invalid",
          acknowledgement.header.frame_id == frame_id_ ? "ok" : "other",
          acknowledgement.waypoint_count == waypoints_.size() && waypoint_index_valid
              ? "ok"
              : "other",
          distance3D(completed_goal, expected_goal),
          distance3D(route_target, expected_goal),
          distance3D(hold_position, expected_goal),
          candidate.identityValid() ? "valid" : "invalid",
          static_cast<double>(source_stamp_ns - candidate.horizon_valid_from_ns) *
              1.0e-6,
          static_cast<double>(candidate.horizon_valid_until_ns - source_stamp_ns) *
              1.0e-6,
          static_cast<double>(candidate.witness_stamp_ns -
                              candidate.horizon_valid_from_ns) *
              1.0e-6,
          static_cast<double>(source_stamp_ns - candidate.witness_stamp_ns) * 1.0e-6,
          static_cast<double>(receive_stamp_ns - source_stamp_ns) * 1.0e-6);
    }
    return;
  }

  completed_waypoint_count_ = acknowledgement.completed_waypoint_count;
  active_waypoint_index_ = acknowledgement.active_waypoint_index;
  if (!acknowledgement.mission_completed) {
    goal_ = Point2{waypoints_[active_waypoint_index_].x,
                   waypoints_[active_waypoint_index_].y};
    minimum_goal_distance_m_ = std::numeric_limits<double>::infinity();
    RCLCPP_INFO(
        get_logger(),
        "MISSION_WAYPOINT_REACHED completed_index=%u waypoint_count=%zu "
        "planner=%" PRIu64 " acknowledgement=%" PRIu64 " horizon=%" PRIu64
        " offboard=%" PRIu64 " next_goal=(%.2f,%.2f,%.2f)",
        acknowledgement.completed_waypoint_index, waypoints_.size(),
        acknowledgement.producer_instance_id, acknowledgement.acknowledgement_sequence,
        acknowledgement.horizon_sequence, acknowledgement.offboard_producer_instance_id,
        waypoints_[active_waypoint_index_].x, waypoints_[active_waypoint_index_].y,
        waypoints_[active_waypoint_index_].z);
    return;
  }
  RCLCPP_INFO(get_logger(),
              "MISSION_WAYPOINT_REACHED completed_index=%u waypoint_count=%zu "
              "planner=%" PRIu64 " acknowledgement=%" PRIu64 " terminal=true",
              acknowledgement.completed_waypoint_index, waypoints_.size(),
              acknowledgement.producer_instance_id,
              acknowledgement.acknowledgement_sequence);
  if (survey_ && !goal_substituted_) {
    // A survey goal reached (roadmap item 34): the vehicle holds there for
    // the survey's next one; the mission ends at home.
    const Point3 reached =
        waypoints_[std::min(active_waypoint_index_, waypoints_.size() - 1U)];
    RCLCPP_INFO(get_logger(), "SURVEY_GOAL_REACHED goal=(%.2f,%.2f,%.2f)", reached.x,
                reached.y, reached.z);
    publishMissionEvent("SURVEY_GOAL_REACHED");
    if (!navigation_mission_ready_) {
      // The survey's first goal is the takeoff's point: captured before the
      // planner declares the mission ready, the planner never does, and the
      // survey's goals would wait on readiness for good (r2813: 320 s of
      // commands unanswered). A goal flown to and acknowledged is the
      // navigation ready.
      navigation_mission_ready_ = true;
      if (latest_position_valid_) {
        mission_ready_altitude_z_m_ = latest_map_position_.z;
      }
      RCLCPP_INFO(get_logger(),
                  "MISSION_READINESS ready=true reason=survey_first_goal");
      publishMissionEvent("MISSION_READINESS ready=true");
    }
    return;
  }
  if (goal_substituted_ && !survey_) {
    // The start was reached in place of the goal. A return is not the
    // mission's success: the check counts it only where the manifest says
    // the unreachability was injected.
    report(false, "goal_unreachable_returned");
    return;
  }
  // The goal, or the survey's home (its return is the mission's end).
  report(spawn_ok_ && moved_,
         spawn_ok_ && moved_ ? (survey_ ? "survey_home" : "none") : "mission_contract");
}

void MissionMonitorNode::publishObjective(const Point3& position,
                                          const std::uint64_t epoch,
                                          const std::uint64_t sample, const bool hold) {
  msg::NavigationObjective objective;
  objective.stamp = now();
  objective.mission_epoch = epoch;
  objective.sample_sequence = sample;
  objective.position.x = position.x;
  objective.position.y = position.y;
  objective.position.z = position.z;
  objective.objective_type = msg::NavigationObjective::OBJECTIVE_TYPE_POSITION;
  objective.terminal_policy =
      hold ? msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD
           : msg::NavigationObjective::TERMINAL_POLICY_POSITION_HOLD;
  objective_pub_->publish(objective);
}

[[nodiscard]] double MissionMonitorNode::meanSpeed() const noexcept {
  return speed_samples_ == 0U ? std::numeric_limits<double>::quiet_NaN()
                              : speed_sum_mps_ / static_cast<double>(speed_samples_);
}

void MissionMonitorNode::publishMissionEvent(const std::string& event) {
  std_msgs::msg::String message;
  message.data = event;
  mission_event_pub_->publish(message);
}

void MissionMonitorNode::report(const bool success, const std::string& reason) {
  result_reported_ = true;
  publishMissionEvent(std::string{"MISSION_RESULT success="} +
                      (success ? "true" : "false") + " reason=" + reason);
  const char* format =
      "MISSION_RESULT success=%s reason='%s' spawn_distance=%.2f "
      "max_distance_from_start=%.2f min_goal_distance=%.2f waypoint_count=%zu "
      "completed_waypoints=%zu final_position=(%.2f, %.2f) final_altitude=%.2f "
      "final_speed=%.2f max_observed_speed=%.2f mean_observed_speed=%.2f";
  if (success) {
    RCLCPP_INFO(get_logger(), format, "true", reason.c_str(), spawn_distance_m_,
                maximum_distance_from_start_m_, minimum_goal_distance_m_,
                waypoints_.size(), completed_waypoint_count_, latest_position_.x,
                latest_position_.y, latest_altitude_m_, latest_speed_mps_,
                maximum_speed_mps_, meanSpeed());
  } else {
    RCLCPP_ERROR(get_logger(), format, "false", reason.c_str(), spawn_distance_m_,
                 maximum_distance_from_start_m_, minimum_goal_distance_m_,
                 waypoints_.size(), completed_waypoint_count_, latest_position_.x,
                 latest_position_.y, latest_altitude_m_, latest_speed_mps_,
                 maximum_speed_mps_, meanSpeed());
  }
  // Specification K30: a mission that reached its goal or came home ends
  // with a landing there and the autopilot's disarm; the monitor waits for
  // it (90 s at most) before it ends the run.
  if (success || reason == "goal_unreachable_returned") {
    landing_after_result_ = true;
    landing_requested_ns_ = now().nanoseconds();
    std_msgs::msg::String land;
    land.data = "land";
    operator_command_pub_->publish(land);
    RCLCPP_INFO(get_logger(), "MISSION_LANDING requested=true");
    landing_timer_ = create_timer(std::chrono::seconds{1}, [this] {
      if (!landed_reported_ &&
          static_cast<double>(now().nanoseconds() - landing_requested_ns_) * 1.0e-9 >
              kLandingTimeoutS) {
        finishLanding(false);
      }
    });
    return;
  }
  scheduleShutdown();
}

void MissionMonitorNode::finishLanding(const bool disarmed) {
  landed_reported_ = true;
  landing_timer_->cancel();
  const double after_s =
      static_cast<double>(now().nanoseconds() - landing_requested_ns_) * 1.0e-9;
  publishMissionEvent(std::string{"MISSION_LANDED disarmed="} +
                      (disarmed ? "true" : "false"));
  RCLCPP_WARN(
      get_logger(),
      "MISSION_LANDED disarmed=%s after_s=%.1f final_position=(%.2f, %.2f, %.2f)",
      disarmed ? "true" : "false", after_s, latest_map_position_.x,
      latest_map_position_.y, latest_map_position_.z);
  scheduleShutdown();
}

void MissionMonitorNode::scheduleShutdown() {
  if (shutdown_on_result_) {
    shutdown_timer_ = create_wall_timer(std::chrono::milliseconds{100}, [this] {
      shutdown_timer_->cancel();
      rclcpp::shutdown();
    });
  }
}

void MissionMonitorNode::logSummary() {
  if (!latest_position_valid_ || result_reported_) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Mission summary: spawn_ok=%s moved=%s armed_seen=%s "
              "position=(%.2f, %.2f) altitude=%.2f speed=%.2f "
              "active_waypoint=%zu/%zu distance_to_goal=%.2f "
              "min_goal_distance=%.2f",
              spawn_ok_ ? "true" : "false", moved_ ? "true" : "false",
              armed_seen_ ? "true" : "false", latest_position_.x, latest_position_.y,
              latest_altitude_m_, latest_speed_mps_, active_waypoint_index_ + 1U,
              waypoints_.size(), distance(latest_position_, goal_),
              minimum_goal_distance_m_);
}

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MissionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
