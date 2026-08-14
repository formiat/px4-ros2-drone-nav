#include "intercept_mission_referee_node.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "intercept_referee_support.hpp"
#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

void requireUniqueIds(const std::vector<std::string>& ids, const std::string& label) {
  std::unordered_set<std::string> unique;
  for (const std::string& id : ids) {
    if (id.empty() || !unique.insert(id).second) {
      throw std::invalid_argument{label + " must be non-empty and unique"};
    }
  }
}

} // namespace

InterceptMissionRefereeNode::InterceptMissionRefereeNode()
    : Node{"intercept_mission_referee_node"} {
  const std::int64_t mission_epoch =
      declare_parameter<std::int64_t>("mission_epoch", 1);
  if (mission_epoch <= 0) {
    throw std::invalid_argument{"mission epoch must be positive"};
  }
  mission_epoch_ = static_cast<std::uint64_t>(mission_epoch);
  mission_name_ = declare_parameter<std::string>("mission_name", "intercept");
  if (mission_name_.empty()) {
    throw std::invalid_argument{"mission name must be non-empty"};
  }
  capture_radius_m_ = declare_parameter<double>("capture_radius_m", 5.0);
  target_goal_radius_m_ = declare_parameter<double>("evader_goal_radius_m", 2.0);
  if (!(capture_radius_m_ > 0.0) || !(target_goal_radius_m_ > 0.0) ||
      !std::isfinite(capture_radius_m_) || !std::isfinite(target_goal_radius_m_)) {
    throw std::invalid_argument{"mission radii must be finite and positive"};
  }
  state_config_.maximum_state_age_s =
      declare_parameter<double>("maximum_state_age_s", 1.0);
  state_config_.maximum_degraded_duration_s =
      declare_parameter<double>("maximum_degraded_state_duration_s", 5.0);
  hold_config_.position_tolerance_m =
      declare_parameter<double>("interceptor_hold_position_tolerance_m", 2.0);
  hold_config_.maximum_speed_mps =
      declare_parameter<double>("interceptor_hold_maximum_speed_mps", 0.8);
  hold_config_.confirmation_duration_s =
      declare_parameter<double>("interceptor_hold_confirmation_duration_s", 1.0);
  destruction_settlement_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("destruction_settlement_timeout_s", 5.0));
  hold_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("interceptor_hold_timeout_s", 20.0));
  boundary_startup_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("ground_truth_boundary_startup_timeout_s", 10.0));
  mission_readiness_timeout_ns_ = missionTimeoutNanoseconds(
      declare_parameter<double>("mission_readiness_timeout_s", 30.0));
  shutdown_on_terminal_outcome_ =
      declare_parameter<bool>("shutdown_on_terminal_outcome", true);

  target_status_pub_ = create_publisher<msg::InterceptTargetStatus>(
      declare_parameter<std::string>("target_status_topic", "/intercept/target_status"),
      rclcpp::QoS{100}.reliable().transient_local());
  const std::vector<std::string> target_ids =
      declare_parameter<std::vector<std::string>>("target_ids", {"evader"});
  const std::vector<std::int64_t> target_detection_ids =
      declare_parameter<std::vector<std::int64_t>>("target_detection_ids", {1});
  const std::vector<double> target_goals =
      declare_parameter<std::vector<double>>("target_goals_xyz_m", {54.0, 378.0, 18.0});
  configureTargets(target_ids, target_detection_ids, target_goals);
  configureInterceptors(declare_parameter<std::vector<std::string>>(
      "interceptor_ids", {"interceptor_0", "interceptor_1", "interceptor_2"}));

  truth_alignment_sub_ = create_subscription<msg::SimulationTruthAlignment>(
      declare_parameter<std::string>("truth_alignment_status_topic",
                                     "/simulation_truth/alignment"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::SimulationTruthAlignment::SharedPtr status) {
        onTruthAlignmentStatus(*status);
      });
  configureGroundTruthBoundary();
  for (std::size_t index = 0U; index < targets_.size(); ++index) {
    publishTargetObjective(index);
  }
  timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
  RCLCPP_INFO(get_logger(),
              "Intercept mission referee ready: mission='%s' epoch=%" PRIu64
              " interceptors=%zu targets=%zu capture_radius_m=%.2f",
              mission_name_.c_str(), mission_epoch_, interceptors_.size(),
              targets_.size(), capture_radius_m_);
}

void InterceptMissionRefereeNode::configureInterceptors(
    const std::vector<std::string>& ids) {
  if (ids.empty()) {
    throw std::invalid_argument{"at least one interceptor is required"};
  }
  requireUniqueIds(ids, "interceptor ids");
  const InterceptorTopicConfig topics = declareInterceptorTopicConfig(*this, ids);
  const auto state_qos = rclcpp::QoS{10}.best_effort();
  const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
  interceptors_.resize(ids.size());
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    InterceptorRuntime& runtime = interceptors_[index];
    runtime.id = ids[index];
    runtime.radar_simulator_fqn = topics.radar_simulator_fqn[index];
    runtime.truth_state_topic = topics.physical_truth_state[index];
    runtime.target_adjudications.reserve(targets_.size());
    for (std::size_t target_index = 0U; target_index < targets_.size();
         ++target_index) {
      runtime.target_adjudications.push_back(
          std::make_unique<InterceptStateAdjudicationLifecycle>(state_config_));
    }
    runtime.state_sub = create_subscription<msg::VehicleNavigationState>(
        topics.navigation_state[index], state_qos,
        [this, index](const msg::VehicleNavigationState::SharedPtr state) {
          interceptors_[index].state = detail::vehicleState(*state);
        });
    runtime.truth_state_sub = create_subscription<msg::SimulationTruthState>(
        topics.physical_truth_state[index], state_qos,
        [this, index](const msg::SimulationTruthState::SharedPtr state) {
          if (state->vehicle_id == interceptors_[index].id) {
            interceptors_[index].truth_state = detail::physicalTruthState(*state);
          }
        });
    runtime.horizon_sub = create_subscription<msg::MppiTrajectoryHorizon>(
        topics.execution_horizon[index], rclcpp::QoS{10}.best_effort(),
        [this, index](const msg::MppiTrajectoryHorizon::SharedPtr horizon) {
          InterceptorRuntime& interceptor = interceptors_[index];
          interceptor.hold_horizon = HoldHorizon{
              .position = Point3{horizon->stationary_hold_position.x,
                                 horizon->stationary_hold_position.y,
                                 horizon->stationary_hold_position.z},
              .sequence = horizon->sequence,
              .active = horizon->stationary_position_hold &&
                        horizon->execution_mode ==
                            msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD,
          };
          interceptor.executable_horizon_ready =
              interceptor.executable_horizon_ready ||
              (horizon->sequence > 0U &&
               horizon->execution_mode ==
                   msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
               !horizon->emergency_braking && horizon->points.size() >= 2U);
        });
    runtime.world_ready_sub = create_subscription<std_msgs::msg::Bool>(
        topics.world_readiness[index], latched_qos,
        [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
          interceptors_[index].world_ready = ready->data;
        });
    runtime.track_ready_sub = create_subscription<std_msgs::msg::Bool>(
        topics.track_readiness[index], latched_qos,
        [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
          interceptors_[index].track_ready = ready->data;
        });
    runtime.destroyed_sub = create_subscription<msg::VehicleDestroyed>(
        topics.destroyed[index], latched_qos,
        [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
          onVehicleDestroyed(*destroyed, true, index);
        });
    runtime.start_pub =
        create_publisher<std_msgs::msg::Bool>(topics.mission_start[index], latched_qos);
    runtime.destroyed_pub =
        create_publisher<msg::VehicleDestroyed>(topics.destroyed[index], latched_qos);
    runtime.command_pub = create_publisher<msg::InterceptMissionCommand>(
        topics.mission_command[index], latched_qos);
  }
}

void InterceptMissionRefereeNode::configureTargets(
    const std::vector<std::string>& ids, const std::vector<std::int64_t>& detection_ids,
    const std::vector<double>& goals_xyz) {
  if (ids.empty() || detection_ids.size() != ids.size() ||
      goals_xyz.size() != ids.size() * 3U) {
    throw std::invalid_argument{
        "target ids, detection ids, and flattened goals must have matching sizes"};
  }
  requireUniqueIds(ids, "target ids");
  const TargetTopicConfig topics = declareTargetTopicConfig(*this, ids);
  const auto state_qos = rclcpp::QoS{10}.best_effort();
  const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
  std::unordered_set<std::uint64_t> unique_detection_ids;
  targets_.resize(ids.size());
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    if (detection_ids[index] <= 0) {
      throw std::invalid_argument{"target detection ids must be positive"};
    }
    const auto detection_id = static_cast<std::uint64_t>(detection_ids[index]);
    if (!unique_detection_ids.insert(detection_id).second) {
      throw std::invalid_argument{"target detection ids must be unique"};
    }
    const Point3 goal{goals_xyz[index * 3U], goals_xyz[index * 3U + 1U],
                      goals_xyz[index * 3U + 2U]};
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.z)) {
      throw std::invalid_argument{"target goals must be finite"};
    }
    TargetRuntime& runtime = targets_[index];
    runtime.id = ids[index];
    runtime.avoidance_radar_simulator_fqn = topics.avoidance_radar_simulator_fqn[index];
    runtime.avoidance_tracker_fqn = topics.avoidance_tracker_fqn[index];
    runtime.state_topic = topics.navigation_state[index];
    runtime.truth_state_topic = topics.physical_truth_state[index];
    runtime.goal = goal;
    runtime.detection_id = detection_id;
    runtime.state_sub = create_subscription<msg::VehicleNavigationState>(
        topics.navigation_state[index], state_qos,
        [this, index](const msg::VehicleNavigationState::SharedPtr state) {
          targets_[index].state = detail::vehicleState(*state);
        });
    runtime.truth_state_sub = create_subscription<msg::SimulationTruthState>(
        topics.physical_truth_state[index], state_qos,
        [this, index](const msg::SimulationTruthState::SharedPtr state) {
          if (state->vehicle_id == targets_[index].id) {
            targets_[index].truth_state = detail::physicalTruthState(*state);
          }
        });
    runtime.horizon_sub = create_subscription<msg::MppiTrajectoryHorizon>(
        topics.execution_horizon[index], rclcpp::QoS{10}.best_effort(),
        [this, index](const msg::MppiTrajectoryHorizon::SharedPtr horizon) {
          TargetRuntime& target = targets_[index];
          target.executable_horizon_ready =
              target.executable_horizon_ready ||
              (horizon->sequence > 0U &&
               horizon->execution_mode ==
                   msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
               !horizon->emergency_braking && horizon->points.size() >= 2U);
        });
    runtime.world_ready_sub = create_subscription<std_msgs::msg::Bool>(
        topics.world_readiness[index], latched_qos,
        [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
          targets_[index].world_ready = ready->data;
        });
    runtime.destroyed_sub = create_subscription<msg::VehicleDestroyed>(
        topics.destroyed[index], latched_qos,
        [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
          onVehicleDestroyed(*destroyed, false, index);
        });
    runtime.objective_pub = create_publisher<msg::NavigationObjective>(
        topics.objective[index], latched_qos);
    runtime.start_pub =
        create_publisher<std_msgs::msg::Bool>(topics.mission_start[index], latched_qos);
    runtime.destroyed_pub =
        create_publisher<msg::VehicleDestroyed>(topics.destroyed[index], latched_qos);
  }
}

void InterceptMissionRefereeNode::configureGroundTruthBoundary() {
  std::vector<std::string> avoidance_radar_simulator_fqns;
  avoidance_radar_simulator_fqns.reserve(targets_.size());
  for (const TargetRuntime& target : targets_) {
    avoidance_radar_simulator_fqns.push_back(target.avoidance_radar_simulator_fqn);
  }
  std::vector<InterceptorTruthEndpoint> interceptor_endpoints;
  interceptor_endpoints.reserve(interceptors_.size());
  for (const InterceptorRuntime& interceptor : interceptors_) {
    interceptor_endpoints.push_back(InterceptorTruthEndpoint{
        .physical_truth_topic = interceptor.truth_state_topic,
        .radar_simulator_fqn = interceptor.radar_simulator_fqn,
    });
  }
  std::vector<TargetTruthEndpoint> target_endpoints;
  target_endpoints.reserve(targets_.size());
  for (const TargetRuntime& target : targets_) {
    target_endpoints.push_back(TargetTruthEndpoint{
        .navigation_topic = target.state_topic,
        .physical_truth_topic = target.truth_state_topic,
        .own_radar_simulator_fqn = target.avoidance_radar_simulator_fqn,
        .own_tracker_fqn = target.avoidance_tracker_fqn,
    });
  }
  ground_truth_boundary_ = makeInterceptGroundTruthBoundary(
      get_fully_qualified_name(),
      declare_parameter<std::string>("simulation_truth_adapter_node_fqn",
                                     "/simulation_truth_adapter_node"),
      target_endpoints, interceptor_endpoints, avoidance_radar_simulator_fqns,
      declare_parameter<std::vector<std::string>>("target_navigation_observer_fqns",
                                                  std::vector<std::string>{}));
}

void InterceptMissionRefereeNode::onTruthAlignmentStatus(
    const msg::SimulationTruthAlignment& status) {
  truth_alignment_reason_ = status.reason;
  truth_alignment_vehicle_id_ = status.vehicle_id;
  truth_alignment_maximum_error_m_ = status.maximum_position_error_m;
  truth_alignment_mission_update_ =
      truth_alignment_lifecycle_.update(SimulationTruthAlignmentObservation{
          .ready = status.ready,
          .sample_aligned = status.reason == "aligned",
          .failure_confirmed = status.failure_confirmed,
      });
  logRuntimeTruthAlignmentTransition(get_logger(), status,
                                     truth_alignment_mission_update_);
}

void InterceptMissionRefereeNode::onVehicleDestroyed(
    const msg::VehicleDestroyed& destroyed, const bool interceptor,
    const std::size_t index) {
  const std::uint8_t expected_role = interceptor
                                         ? msg::VehicleDestroyed::ROLE_INTERCEPTOR
                                         : msg::VehicleDestroyed::ROLE_EVADER;
  const std::string& expected_id =
      interceptor ? interceptors_[index].id : targets_[index].id;
  if (!validateVehicleDestroyedEvent(get_logger(), destroyed, expected_role,
                                     expected_id, mission_epoch_)) {
    return;
  }
  bool& already_destroyed =
      interceptor ? interceptors_[index].destroyed : targets_[index].destroyed;
  if (already_destroyed) {
    return;
  }
  bool& destruction_requested = interceptor ? interceptors_[index].destruction_requested
                                            : targets_[index].destruction_requested;
  std::int64_t& requested_ns = interceptor
                                   ? interceptors_[index].destruction_requested_ns
                                   : targets_[index].destruction_requested_ns;
  const bool expected_proximity_event = destruction_requested;
  already_destroyed = true;
  destruction_requested = true;
  if (requested_ns <= 0) {
    requested_ns = now().nanoseconds();
  }
  if (destruction_requested_ns_ <= 0) {
    destruction_requested_ns_ = now().nanoseconds();
  }
  RCLCPP_ERROR(get_logger(),
               "VEHICLE_DESTROYED referee_observed=true role=%s vehicle_id='%s' "
               "cause=%s mission_epoch=%" PRIu64 " detail='%s'",
               detail::vehicleRoleName(destroyed.vehicle_role),
               destroyed.vehicle_id.c_str(),
               detail::deathCauseName(destroyed.death_cause), mission_epoch_,
               destroyed.detail.c_str());

  if (!interceptor &&
      destroyed.death_cause == msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION &&
      targets_[index].outcome == TargetOutcome::kActive) {
    markTargetOutcome(index, TargetOutcome::kDestroyed, "physical_collision");
  }
  if (destroyed.death_cause == msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION) {
    if (!system_failure_reason_.has_value()) {
      system_failure_reason_ = "physical_collision:" + expected_id;
      requestHoldsForSurvivors(*system_failure_reason_);
    }
    return;
  }
  if (!expected_proximity_event && !system_failure_reason_.has_value()) {
    system_failure_reason_ = "unexpected_proximity_destruction:" + expected_id;
    requestHoldsForSurvivors(*system_failure_reason_);
  }
}

bool InterceptMissionRefereeNode::verifyGroundTruthBoundary(const std::int64_t now_ns) {
  if (last_boundary_check_ns_ > 0 && now_ns - last_boundary_check_ns_ < 500'000'000LL) {
    return boundary_verified_;
  }
  last_boundary_check_ns_ = now_ns;
  const GroundTruthBoundaryUpdate update = ground_truth_boundary_->update(*this);
  if (!update.violating_subscriber.empty()) {
    failMission("ground_truth_boundary_violation:" + update.violating_topic + ":" +
                update.violating_subscriber);
    return false;
  }
  boundary_verified_ = update.verified;
  if (update.newly_verified) {
    RCLCPP_INFO(get_logger(),
                "RADAR_DATA_BOUNDARY verified=true physical_truth=true "
                "interceptors=%zu targets=%zu",
                interceptors_.size(), targets_.size());
  }
  return boundary_verified_;
}

bool InterceptMissionRefereeNode::missionReady() const {
  if (!truth_alignment_mission_update_.startup_ready) {
    return false;
  }
  const bool targets_ready =
      std::ranges::all_of(targets_, [](const TargetRuntime& target) {
        return !target.destroyed && target.state && target.truth_state &&
               target.state->navigation_ready && target.world_ready &&
               target.executable_horizon_ready;
      });
  const bool interceptors_ready =
      std::ranges::all_of(interceptors_, [](const InterceptorRuntime& interceptor) {
        return !interceptor.destroyed && interceptor.state && interceptor.truth_state &&
               interceptor.state->navigation_ready && interceptor.world_ready &&
               interceptor.track_ready && interceptor.executable_horizon_ready;
      });
  return targets_ready && interceptors_ready;
}

std::optional<TimedVehicleState> InterceptMissionRefereeNode::interceptorPhysicalState(
    const std::size_t index) const noexcept {
  return detail::physicalState(interceptors_[index].state,
                               interceptors_[index].truth_state);
}

std::optional<TimedVehicleState> InterceptMissionRefereeNode::targetPhysicalState(
    const std::size_t index) const noexcept {
  return detail::physicalState(targets_[index].state, targets_[index].truth_state);
}

void InterceptMissionRefereeNode::publishTargetObjective(const std::size_t index) {
  TargetRuntime& target = targets_[index];
  target.objective_pub->publish(makePositionHoldObjective(
      now(), mission_epoch_, ++target.objective_sequence, target.goal));
}

void InterceptMissionRefereeNode::publishTargetStatus(const std::size_t index,
                                                      const std::string& detail_text) {
  const TargetRuntime& target = targets_[index];
  msg::InterceptTargetStatus status;
  status.header.stamp = now();
  status.header.frame_id = "map";
  status.mission_epoch = mission_epoch_;
  status.target_id = target.id;
  status.target_detection_id = target.detection_id;
  status.capturing_interceptor_id = target.capturing_interceptor_id;
  switch (target.outcome) {
    case TargetOutcome::kActive:
      status.status = msg::InterceptTargetStatus::STATUS_ACTIVE;
      break;
    case TargetOutcome::kIntercepted:
      status.status = msg::InterceptTargetStatus::STATUS_INTERCEPTED;
      break;
    case TargetOutcome::kReachedGoal:
      status.status = msg::InterceptTargetStatus::STATUS_REACHED_GOAL;
      break;
    case TargetOutcome::kDestroyed:
      status.status = msg::InterceptTargetStatus::STATUS_DESTROYED;
      break;
  }
  status.detail = detail_text;
  target_status_pub_->publish(status);
}

void InterceptMissionRefereeNode::publishMissionStart() {
  if (!truth_alignment_lifecycle_.latchStartupContract()) {
    failMission("truth_alignment_contract_not_ready");
    return;
  }
  std_msgs::msg::Bool start;
  start.data = true;
  for (InterceptorRuntime& interceptor : interceptors_) {
    interceptor.start_pub->publish(start);
  }
  for (std::size_t index = 0U; index < targets_.size(); ++index) {
    targets_[index].start_pub->publish(start);
    publishTargetStatus(index, "mission_started");
  }
  mission_started_ = true;
  RCLCPP_INFO(get_logger(),
              "INTERCEPT_MISSION state=running mission='%s' epoch=%" PRIu64
              " interceptor_count=%zu target_count=%zu "
              "startup_coordinate_contract_latched=true "
              "all_executable_horizons_ready=true",
              mission_name_.c_str(), mission_epoch_, interceptors_.size(),
              targets_.size());
}

std::size_t InterceptMissionRefereeNode::operationalInterceptorCount() const noexcept {
  return static_cast<std::size_t>(
      std::ranges::count_if(interceptors_, [](const InterceptorRuntime& runtime) {
        return !runtime.destroyed && !runtime.destruction_requested &&
               !runtime.disabled;
      }));
}

std::size_t InterceptMissionRefereeNode::survivingInterceptorCount() const noexcept {
  return static_cast<std::size_t>(
      std::ranges::count_if(interceptors_, [](const InterceptorRuntime& runtime) {
        return !runtime.destroyed && !runtime.destruction_requested;
      }));
}

std::size_t InterceptMissionRefereeNode::activeTargetCount() const noexcept {
  return static_cast<std::size_t>(
      std::ranges::count(targets_, TargetOutcome::kActive, &TargetRuntime::outcome));
}

void InterceptMissionRefereeNode::requestHold(const std::size_t index,
                                              const std::string& reason) {
  InterceptorRuntime& interceptor = interceptors_[index];
  if (interceptor.destroyed || interceptor.destruction_requested ||
      interceptor.hold_confirmation) {
    return;
  }
  if (!interceptor.state || !interceptor.state->position_valid) {
    if (mission_started_) {
      system_failure_reason_ =
          "interceptor_hold_position_unavailable:" + interceptor.id;
    }
    return;
  }
  interceptor.disabled = true;
  interceptor.hold_confirmation =
      std::make_unique<InterceptorHoldConfirmation>(hold_config_);
  interceptor.hold_request_horizon_sequence =
      interceptor.hold_horizon ? interceptor.hold_horizon->sequence : 0U;
  interceptor.hold_requested_ns = now().nanoseconds();
  msg::InterceptMissionCommand command;
  command.stamp = now();
  command.mission_epoch = mission_epoch_;
  command.command = msg::InterceptMissionCommand::COMMAND_HOLD_CURRENT_POSITION;
  command.reason = reason;
  interceptor.command_pub->publish(command);
  RCLCPP_INFO(get_logger(),
              "INTERCEPTOR_HOLD requested=true vehicle_id='%s' reason='%s' "
              "position=(%.3f,%.3f,%.3f) mission_epoch=%" PRIu64,
              interceptor.id.c_str(), reason.c_str(), interceptor.state->position.x,
              interceptor.state->position.y, interceptor.state->position.z,
              mission_epoch_);
}

void InterceptMissionRefereeNode::requestHoldsForSurvivors(
    const std::string& reason, const std::optional<std::size_t> excluded) {
  for (std::size_t index = 0U; index < interceptors_.size(); ++index) {
    if (!excluded.has_value() || *excluded != index) {
      requestHold(index, reason);
    }
  }
}

void InterceptMissionRefereeNode::requestTargetHold(const std::size_t index,
                                                    const std::string& reason) {
  TargetRuntime& target = targets_[index];
  if (target.hold_requested || target.destroyed || !target.state ||
      !target.state->position_valid) {
    return;
  }
  target.hold_requested = true;
  msg::NavigationObjective objective;
  objective.stamp = now();
  objective.mission_epoch = mission_epoch_;
  objective.sample_sequence = ++target.objective_sequence;
  objective.position.x = target.state->position.x;
  objective.position.y = target.state->position.y;
  objective.position.z = target.state->position.z;
  objective.objective_type = msg::NavigationObjective::OBJECTIVE_TYPE_POSITION;
  objective.guidance_mode = msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
  objective.terminal_policy = msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD;
  target.objective_pub->publish(objective);
  RCLCPP_INFO(get_logger(),
              "TARGET_HOLD requested=true target_id='%s' reason='%s' "
              "position=(%.3f,%.3f,%.3f) mission_epoch=%" PRIu64,
              target.id.c_str(), reason.c_str(), target.state->position.x,
              target.state->position.y, target.state->position.z, mission_epoch_);
}

void InterceptMissionRefereeNode::handleStartupCoordinateAlignmentFailure() {
  if (system_failure_reason_.has_value()) {
    return;
  }
  system_failure_reason_ =
      "coordinate_alignment_mismatch:" + truth_alignment_vehicle_id_ + ":" +
      truth_alignment_reason_;
  RCLCPP_ERROR(get_logger(),
               "SIMULATION_TRUTH_ALIGNMENT startup_mission_blocked=true reason='%s' "
               "vehicle_id='%s' max_error_m=%.3f mission_started=%s",
               truth_alignment_reason_.c_str(), truth_alignment_vehicle_id_.c_str(),
               truth_alignment_maximum_error_m_, mission_started_ ? "true" : "false");
  requestHoldsForSurvivors(*system_failure_reason_);
  for (std::size_t index = 0U; index < targets_.size(); ++index) {
    requestTargetHold(index, *system_failure_reason_);
  }
}

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::InterceptMissionRefereeNode>());
  rclcpp::shutdown();
  return 0;
}
