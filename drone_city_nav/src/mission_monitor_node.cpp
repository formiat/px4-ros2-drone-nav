#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/carried_light_judgment.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/goal_reachability_proof_3d.hpp"
#include "drone_city_nav/main_battery_estimate.hpp"
#include "drone_city_nav/mission_waypoint_acknowledgement_admission.hpp"
#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/msg/mission_waypoint_acknowledgement.hpp"
#include "drone_city_nav/msg/navigation_health.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/navigation_progress.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/operator_command.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/return_trail.hpp"
#include "drone_city_nav/smoke_ladder.hpp"
#include "drone_city_nav/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mission_monitor_node_internal.hpp"

namespace drone_city_nav {

using namespace mission_monitor_node_internal;

class MissionMonitorNode final : public rclcpp::Node {
public:
  MissionMonitorNode()
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
    // Roadmap item 18 stage 4: the share of the ring's zones returning smoke
    // (K38); the ladder of K40 reads it with the frame's range.
    smoke_share_sub_ = create_subscription<std_msgs::msg::Float64>(
        declare_parameter<std::string>("smoke_share_topic",
                                       "/stereo_depth/smoke_share"),
        rclcpp::SensorDataQoS{},
        [this](const std_msgs::msg::Float64::SharedPtr message) {
          smoke_share_ = message->data;
        });
    unlit_surface_sub_ = create_subscription<std_msgs::msg::Float64>(
        declare_parameter<std::string>("unlit_surface_share_topic",
                                       "/stereo_depth/unlit_surface_share"),
        rclcpp::SensorDataQoS{},
        [this](const std_msgs::msg::Float64::SharedPtr message) {
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
    proof_timer_ =
        create_timer(std::chrono::seconds{10}, [this] { judgeReturnHome(); });
    summary_timer_ = create_timer(std::chrono::seconds{5}, [this] { logSummary(); });
    RCLCPP_INFO(get_logger(),
                "Mission monitor ready: start=(%.2f, %.2f) waypoint_count=%zu "
                "first_goal=(%.2f, %.2f)",
                start_.x, start_.y, waypoints_.size(), goal_.x, goal_.y);
  }

private:
  void onNavigationHealth(const msg::NavigationHealth& health) {
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

  void onLocalState(const AutopilotLocalState& message) {
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

  void onWaypointAcknowledgement(
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
        distance3D(route_target, expected_goal) <=
            acknowledgement_target_tolerance_m_ &&
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
        RCLCPP_WARN(get_logger(),
                    "MISSION_WAYPOINT_ACK_REJECTED producer=%" PRIu64
                    " acknowledgement=%" PRIu64 " reason=%s",
                    acknowledgement.producer_instance_id,
                    acknowledgement.acknowledgement_sequence,
                    acknowledgementRejectionReason(admission));
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
          acknowledgement.producer_instance_id,
          acknowledgement.acknowledgement_sequence, acknowledgement.horizon_sequence,
          acknowledgement.offboard_producer_instance_id,
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
    if (goal_substituted_) {
      // The start was reached in place of the goal. A return is not the
      // mission's success: the check counts it only where the manifest says
      // the unreachability was injected.
      report(false, "goal_unreachable_returned");
      return;
    }
    report(spawn_ok_ && moved_, spawn_ok_ && moved_ ? "none" : "mission_contract");
  }

  void onMemorySnapshot(const msg::RawObstacleSnapshot3D& snapshot) {
    const GridBounds3D bounds{
        .origin_x = snapshot.origin_x_m,
        .origin_y = snapshot.origin_y_m,
        .origin_z = snapshot.origin_z_m,
        .resolution_m = snapshot.resolution_m,
        .width_cells = static_cast<int>(snapshot.width_cells),
        .height_cells = static_cast<int>(snapshot.height_cells),
        .depth_cells = static_cast<int>(snapshot.depth_cells),
    };
    if (!std::isfinite(bounds.resolution_m) || bounds.resolution_m <= 0.0 ||
        bounds.width_cells <= 0 || bounds.height_cells <= 0 ||
        bounds.depth_cells <= 0 ||
        static_cast<int>(snapshot.chunk_size_cells) !=
            ObservedOccupancyGrid3D::kChunkSize) {
      return;
    }
    auto grid = std::make_shared<ObservedOccupancyGrid3D>(bounds);
    applyMemoryChunks(*grid, snapshot.chunks);
    memory_ = std::move(grid);
    memory_producer_instance_id_ = snapshot.producer_instance_id;
    memory_base_revision_ = snapshot.obstacle_snapshot_revision;
  }

  void onMemoryDelta(const msg::RawObstacleDelta3D& delta) {
    if (memory_ == nullptr ||
        delta.producer_instance_id != memory_producer_instance_id_ ||
        delta.base_snapshot_revision != memory_base_revision_) {
      return;
    }
    // The proof reads a grid of its own on another thread: a delta writes a
    // fresh grid object, whose untouched chunks share the old storage.
    auto grid = std::make_shared<ObservedOccupancyGrid3D>(*memory_);
    applyMemoryChunks(*grid, delta.chunks);
    memory_ = std::move(grid);
  }

  static void
  applyMemoryChunks(ObservedOccupancyGrid3D& grid,
                    const std::vector<msg::ObservedObstacleChunk3D>& chunks) {
    for (const msg::ObservedObstacleChunk3D& message : chunks) {
      ObservedOccupancyGrid3D::Chunk chunk;
      chunk.observed = message.observed_words;
      chunk.occupied = message.occupied_words;
      static_cast<void>(grid.replaceChunk(
          OccupancyChunkIndex3D{message.x, message.y, message.z}, chunk));
    }
  }

  // After a return the proof runs to the start: a start proven unreachable as
  // well is logged, and the vehicle holds where it is, because the planner
  // has no route; a proof that opens again (the memory's closures decay) is
  // logged too, and the planner flies on by itself.
  void judgeReturnHome() {
    if (result_reported_ || !navigation_mission_ready_ ||
        active_waypoint_index_ >= waypoints_.size()) {
      return;
    }
    const std::int64_t now_ns = now().nanoseconds();
    if (proof_future_.valid() &&
        proof_future_.wait_for(std::chrono::seconds{0}) == std::future_status::ready) {
      const GoalReachabilityProof3D proof = proof_future_.get();
      const Point3 goal = waypoints_[active_waypoint_index_];
      // A proof started for the goal the return gave up says nothing of the
      // start.
      if (proof_goal_.x == goal.x && proof_goal_.y == goal.y &&
          proof_goal_.z == goal.z) {
        latest_proof_ = proof;
        if (!latest_proof_.provenUnreachable()) {
          proven_since_ns_ = 0;
        } else if (proven_since_ns_ == 0) {
          proven_since_ns_ = now_ns;
        }
        const bool proof_held =
            proven_since_ns_ > 0 &&
            static_cast<double>(now_ns - proven_since_ns_) * 1.0e-9 >= proof_hold_s_;
        RCLCPP_INFO(get_logger(),
                    "GOAL_REACHABILITY_PROOF verdict=%s component_voxels=%zu "
                    "goal_inside=%s budget_exhausted=%s goal_region_voxels=%zu "
                    "held=%s returning=%s",
                    goalReachabilityProofVerdict(latest_proof_),
                    latest_proof_.component_voxels,
                    latest_proof_.goal_inside ? "true" : "false",
                    latest_proof_.budget_exhausted ? "true" : "false",
                    latest_proof_.goal_region_voxels, proof_held ? "true" : "false",
                    goal_substituted_ ? "true" : "false");
        if (!goal_substituted_ && proof_held &&
            substituteGoalWithStart("topological", now_ns)) {
          proven_since_ns_ = 0;
          return;
        }
        if (goal_substituted_ && proof_held != start_unreachable_) {
          start_unreachable_ = proof_held;
          RCLCPP_WARN(
              get_logger(),
              "%s position=(%.3f,%.3f,%.3f) start=(%.3f,%.3f,%.3f) "
              "component_voxels=%zu",
              start_unreachable_ ? "START_UNREACHABLE_HELD" : "START_REACHABLE_AGAIN",
              latest_map_position_.x, latest_map_position_.y, latest_map_position_.z,
              goal.x, goal.y, goal.z, latest_proof_.component_voxels);
        }
      }
    }
    if (!proof_future_.valid() && memory_ != nullptr && latest_position_valid_) {
      const std::shared_ptr<const ObservedOccupancyGrid3D> grid = memory_;
      const Point3 vehicle = latest_map_position_;
      const Point3 goal = waypoints_[active_waypoint_index_];
      const FlightEnvelopeConfig envelope = flight_envelope_;
      const double capture_radius_m = goal_capture_radius_m_;
      const std::size_t budget = proof_voxel_budget_;
      proof_goal_ = goal;
      proof_future_ = std::async(std::launch::async, [grid, vehicle, goal, envelope,
                                                      capture_radius_m, budget] {
        return proveGoalUnreachable3D(*grid, vehicle, goal, envelope, capture_radius_m,
                                      budget);
      });
    }
  }

  // The way back is known and observed: the path flown so far at the
  // flight's mean speed, floored at the start's 0.5 m/s, errs on the long
  // side; the planner routes anew through the observed space, and over
  // twelve returns of item 19 the way back took 0.72 to 1.65 times the flight
  // out, so it is doubled, with a 20 s reserve. Logged beside the battery's
  // charge when a goal is given up.
  [[nodiscard]] double homeEstimateS() const noexcept {
    const double mean = meanSpeed();
    return kReturnEstimateMargin * flown_path_m_ /
               std::max(std::isfinite(mean) ? mean : 0.0, 0.5) +
           kReturnReserveS;
  }

  // The way home against the way out, which it retraces: 1.5 (the owner,
  // 2026-10-09; at 2.0 the low-battery flight came home with 90 of its 240 s
  // unspent, the way home having taken 62 s for 60 s out, r1185).
  static constexpr double kReturnEstimateMargin{1.5};
  static constexpr double kReturnReserveS{20.0};
  // The charge kept over what the way home needs when the vehicle turns
  // (specification K11, the owner's rule of 2026-10-05).
  static constexpr double kReturnChargeMargin{1.1};

  // Roadmap item 17 stage 5: the light judged from what the navigation sees,
  // and the battery weighed against the way home.
  void onNavigationProgress(const msg::NavigationProgress& progress) {
    if (result_reported_ || goal_substituted_ || !navigation_mission_ready_ ||
        progress.header.frame_id != frame_id_ ||
        active_waypoint_index_ >= waypoints_.size()) {
      return;
    }
    const std::int64_t now_ns = now().nanoseconds();
    // The share of its configured range the braking contract grants.
    light_judgment_.observe(static_cast<double>(now_ns) * 1.0e-9,
                            progress.sensor_guaranteed_range_m > 0.0
                                ? progress.sensor_measured_range_m /
                                      progress.sensor_guaranteed_range_m
                                : -1.0,
                            light_reading_, unlit_surface_share_);
    if (std::isfinite(progress.route_remaining_m)) {
      route_remaining_m_ = progress.route_remaining_m;
    }
    if (light_judgment_.unreliable()) {
      static_cast<void>(substituteGoalWithStart("unreliable_light", now_ns));
      return;
    }
    if (smokeLadder(progress, now_ns)) {
      return;
    }
    // The way to B is not known until it is flown, the way home is: the
    // vehicle flies on while its charge covers the way home with a margin,
    // and turns when it no longer does. A flight with no battery (no charge
    // published) never turns for it.
    if (std::isfinite(light_charge_s_) &&
        light_charge_s_ < kReturnChargeMargin * homeEstimateS()) {
      static_cast<void>(substituteGoalWithStart("battery", now_ns));
    }
    // The vehicle's own battery by the same rule (specification K32).
    if (main_battery_.timeLeftS() < kReturnChargeMargin * homeEstimateS()) {
      static_cast<void>(substituteGoalWithStart("main_battery", now_ns));
    }
  }

  // Specification K31: the operator's commands. A new goal is the mission's
  // only waypoint from then on; a stop holds where the vehicle is until the
  // next command; home gives the goal up for the start (K20); land is flown
  // by the offboard, and the disarm ends the mission.
  void onOperatorCommand(const std::string& text) {
    const std::optional<OperatorCommand> command = parseOperatorCommand(text);
    if (!command.has_value() || result_reported_ || !navigation_mission_ready_) {
      return;
    }
    RCLCPP_WARN(get_logger(), "OPERATOR_COMMAND accepted=true command='%s'",
                text.c_str());
    publishMissionEvent("OPERATOR_COMMAND " + text);
    switch (command->kind) {
      case OperatorCommandKind::kStop:
        operator_hold_ = true;
        publishObjective(latest_map_position_, nextObjectiveEpoch(), 1U, true);
        break;
      case OperatorCommandKind::kGoal:
        operator_hold_ = false;
        goal_substituted_ = false;
        waypoints_ = {command->goal};
        goal_ = Point2{command->goal.x, command->goal.y};
        active_waypoint_index_ = 0U;
        completed_waypoint_count_ = 0U;
        minimum_goal_distance_m_ = std::numeric_limits<double>::infinity();
        publishObjective(command->goal, nextObjectiveEpoch(), 1U, false);
        break;
      case OperatorCommandKind::kHome:
        operator_hold_ = false;
        static_cast<void>(substituteGoalWithStart("operator", now().nanoseconds()));
        break;
      case OperatorCommandKind::kLand:
        operator_landing_ = true;
        break;
    }
  }

  // The next mission epoch: one past the planner's and past every one the
  // monitor published.
  [[nodiscard]] std::uint64_t nextObjectiveEpoch() noexcept {
    last_objective_epoch_ =
        std::max(navigation_mission_epoch_, last_objective_epoch_) + 1U;
    return last_objective_epoch_;
  }

  // A position objective on the channel every goal enters by: the return's
  // trail points (K20) and the operator's goals and holds (K31).
  void publishObjective(const Point3& position, const std::uint64_t epoch,
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

  // The return to home (RTH; PX4's own is RTL, which flies no map): the goal
  // is given up for the start, through the channel any objective enters the
  // navigation by; the mission's waypoint list becomes the start
  // alone, so the arrival there is judged as any goal's. A return needs a
  // position source: without a fresh position nothing is substituted.
  [[nodiscard]] bool substituteGoalWithStart(const char* trigger,
                                             const std::int64_t now_ns) {
    if (!returnHomePositionSourceFresh(latest_position_valid_,
                                       latest_position_stamp_ns_, now_ns)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "GOAL_UNREACHABLE_HELD trigger=%s reason=no_position_source",
                           trigger);
      return false;
    }
    const Point3 original_goal = waypoints_[active_waypoint_index_];
    const Point3 start{start_.x, start_.y,
                       std::isfinite(mission_ready_altitude_z_m_)
                           ? mission_ready_altitude_z_m_
                           : waypoints_.front().z};
    waypoints_ = {start};
    goal_ = start_;
    active_waypoint_index_ = 0U;
    completed_waypoint_count_ = 0U;
    minimum_goal_distance_m_ = std::numeric_limits<double>::infinity();
    goal_substituted_ = true;
    publishMissionEvent(std::string{"GOAL_UNREACHABLE trigger="} + trigger);
    // The return is one mission epoch (K20), opened here; its objective then
    // moves along the trail a sample at a time.
    return_epoch_ = nextObjectiveEpoch();
    return_sample_ = 0U;
    return_target_ = std::numeric_limits<std::size_t>::max();
    publishReturnObjective();
    const double elapsed_s =
        mission_start_ns_ > 0 ? static_cast<double>(now_ns - mission_start_ns_) * 1.0e-9
                              : 0.0;
    RCLCPP_WARN(get_logger(),
                "GOAL_UNREACHABLE trigger=%s goal=(%.3f,%.3f,%.3f) "
                "substituted_goal=(%.3f,%.3f,%.3f) mission_epoch=%" PRIu64
                " elapsed_s=%.1f flown_path_m=%.1f home_estimate_s=%.1f "
                "light_charge_s=%.1f route_remaining_m=%.1f "
                "light_outage_s=%.1f light_outage_share=%.3f proof=%s "
                "component_voxels=%zu",
                trigger, original_goal.x, original_goal.y, original_goal.z, start.x,
                start.y, start.z, return_epoch_, elapsed_s, flown_path_m_,
                homeEstimateS(), light_charge_s_, route_remaining_m_,
                light_judgment_.currentOutageS(), light_judgment_.outageShare(),
                goalReachabilityProofVerdict(latest_proof_),
                latest_proof_.component_voxels);
    return true;
  }

  // Roadmap item 18 stage 4 (specification K40): the ladder in smoke; true
  // when the goal was given up here.
  [[nodiscard]] bool smokeLadder(const msg::NavigationProgress& progress,
                                 const std::int64_t now_ns) {
    const SmokeLadderAction action = smoke_ladder_.observe(SmokeLadderInput{
        .blind = progress.sensor_guaranteed_range_m > 0.0 &&
                 progress.sensor_measured_range_m / progress.sensor_guaranteed_range_m <
                     LightReliabilityJudgment::kBlindSensorShare,
        .smoke_share = smoke_share_,
        .position_fresh = returnHomePositionSourceFresh(
            latest_position_valid_, latest_position_stamp_ns_, now_ns),
        .position = latest_map_position_,
        .now_ns = now_ns});
    switch (action) {
      case SmokeLadderAction::kNone:
        return false;
      case SmokeLadderAction::kHold:
        publishObjective(latest_map_position_, nextObjectiveEpoch(), 1U, true);
        publishMissionEvent("SMOKE_HOLD");
        RCLCPP_WARN(
            get_logger(), "SMOKE_HOLD out of plume %zu at (%.3f,%.3f,%.3f) for %.0f s",
            smoke_ladder_.entries(), latest_map_position_.x, latest_map_position_.y,
            latest_map_position_.z, SmokeLadder::kHoldS);
        return false;
      case SmokeLadderAction::kRestore:
        publishObjective(waypoints_[active_waypoint_index_], nextObjectiveEpoch(), 1U,
                         false);
        publishMissionEvent("SMOKE_CLEARED");
        RCLCPP_WARN(get_logger(), "SMOKE_CLEARED goal restored");
        return false;
      case SmokeLadderAction::kGiveUp:
        return substituteGoalWithStart("smoke", now_ns);
    }
    return false;
  }

  SmokeLadder smoke_ladder_;
  double smoke_share_{0.0};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr smoke_share_sub_;

  // The way home is the way the vehicle came (specification K20): the
  // objective is the trail point 25 m back from the one nearest the vehicle,
  // the start at the end.
  void publishReturnObjective() {
    const std::size_t target = return_trail_.next(latest_map_position_);
    if (target == return_target_) {
      return;
    }
    return_target_ = target;
    const Point3 position =
        target == 0U ? waypoints_.front() : return_trail_.points()[target];
    // Each point is a sample of the return's one objective (K20): the planner
    // routes to the moved target afresh and its lifecycle replaces the route
    // to the old one (r908 stood 540 s at the first point with the objective
    // 20 m on, when it kept that route as equivalent).
    publishObjective(position, return_epoch_, ++return_sample_, false);
    RCLCPP_INFO(get_logger(),
                "RETURN_TRAIL objective=(%.3f,%.3f,%.3f) index=%zu of %zu "
                "mission_epoch=%" PRIu64 " sample=%" PRIu64,
                position.x, position.y, position.z, target,
                return_trail_.points().size(), return_epoch_, return_sample_);
  }

  [[nodiscard]] double meanSpeed() const noexcept {
    return speed_samples_ == 0U ? std::numeric_limits<double>::quiet_NaN()
                                : speed_sum_mps_ / static_cast<double>(speed_samples_);
  }

  void publishMissionEvent(const std::string& event) {
    std_msgs::msg::String message;
    message.data = event;
    mission_event_pub_->publish(message);
  }

  void report(const bool success, const std::string& reason) {
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

  void finishLanding(const bool disarmed) {
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

  void scheduleShutdown() {
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
                "active_waypoint=%zu/%zu distance_to_goal=%.2f "
                "min_goal_distance=%.2f",
                spawn_ok_ ? "true" : "false", moved_ ? "true" : "false",
                armed_seen_ ? "true" : "false", latest_position_.x, latest_position_.y,
                latest_altitude_m_, latest_speed_mps_, active_waypoint_index_ + 1U,
                waypoints_.size(), distance(latest_position_, goal_),
                minimum_goal_distance_m_);
  }

  Point2 start_{};
  Point2 goal_{};
  Px4MapFrameTransform px4_map_transform_{};
  Point2 latest_position_{};
  std::vector<Point3> waypoints_;
  std::string frame_id_{"map"};
  double spawn_tolerance_m_{1.0};
  double minimum_movement_m_{5.0};
  double acknowledgement_target_tolerance_m_{1.0e-3};
  double goal_capture_radius_m_{2.0};
  double latest_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double latest_speed_mps_{std::numeric_limits<double>::infinity()};
  double spawn_distance_m_{std::numeric_limits<double>::infinity()};
  double maximum_distance_from_start_m_{0.0};
  double minimum_goal_distance_m_{std::numeric_limits<double>::infinity()};
  double maximum_speed_mps_{0.0};
  double speed_sum_mps_{0.0};
  std::size_t speed_samples_{0U};
  std::size_t active_waypoint_index_{0U};
  std::size_t completed_waypoint_count_{0U};
  bool first_position_seen_{false};
  bool latest_position_valid_{false};
  bool spawn_ok_{false};
  bool moved_{false};
  bool armed_seen_{false};
  bool result_reported_{false};
  bool navigation_mission_ready_{false};
  bool shutdown_on_result_{false};
  std::uint64_t navigation_health_producer_instance_id_{0U};
  std::uint64_t navigation_health_sequence_{0U};
  std::uint64_t navigation_mission_epoch_{0U};
  std::size_t proof_voxel_budget_{20'000'000U};
  FlightEnvelopeConfig flight_envelope_{};
  double proof_hold_s_{30.0};
  std::int64_t proven_since_ns_{0};
  Point3 proof_goal_{};
  bool start_unreachable_{false};
  Point3 latest_map_position_{};
  std::int64_t latest_position_stamp_ns_{0};
  std::int64_t mission_start_ns_{0};
  LightReliabilityJudgment light_judgment_;
  double light_charge_s_{std::numeric_limits<double>::quiet_NaN()};
  double route_remaining_m_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Subscription<msg::NavigationProgress>::SharedPtr navigation_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_charge_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_reading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr unlit_surface_sub_;
  double light_reading_{std::numeric_limits<double>::quiet_NaN()};
  double unlit_surface_share_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
  double mission_ready_altitude_z_m_{std::numeric_limits<double>::quiet_NaN()};
  double flown_path_m_{0.0};
  bool goal_substituted_{false};
  ReturnTrail return_trail_;
  std::uint64_t return_epoch_{0U};
  std::uint64_t return_sample_{0U};
  std::size_t return_target_{std::numeric_limits<std::size_t>::max()};
  GoalReachabilityProof3D latest_proof_{};
  std::future<GoalReachabilityProof3D> proof_future_;
  std::shared_ptr<const ObservedOccupancyGrid3D> memory_;
  std::uint64_t memory_producer_instance_id_{0U};
  std::uint64_t memory_base_revision_{0U};
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr objective_pub_;
  rclcpp::Subscription<msg::RawObstacleSnapshot3D>::SharedPtr memory_snapshot_sub_;
  rclcpp::Subscription<msg::RawObstacleDelta3D>::SharedPtr memory_delta_sub_;
  rclcpp::TimerBase::SharedPtr proof_timer_;
  MissionWaypointAcknowledgementAdmissionState waypoint_acknowledgement_admission_{};
  std::unique_ptr<AutopilotStateSource> autopilot_state_source_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
  rclcpp::Subscription<msg::MissionWaypointAcknowledgement>::SharedPtr
      waypoint_acknowledgement_sub_;
  rclcpp::Subscription<msg::NavigationHealth>::SharedPtr navigation_health_sub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  static constexpr double kLandingTimeoutS{90.0};
  MainBatteryEstimate main_battery_;
  bool operator_hold_{false};
  bool operator_landing_{false};
  bool landing_after_result_{false};
  bool landed_reported_{false};
  std::int64_t landing_requested_ns_{0};
  std::uint64_t last_objective_epoch_{0U};
  rclcpp::TimerBase::SharedPtr landing_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operator_command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operator_command_sub_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MissionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
