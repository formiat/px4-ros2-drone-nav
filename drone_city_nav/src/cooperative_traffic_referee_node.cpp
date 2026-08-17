#include "cooperative_traffic_referee_node.hpp"

#include "drone_city_nav/cooperative_traffic_ros.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t timeoutNanoseconds(const double seconds,
                                              const std::string& label) {
  if (!(seconds > 0.0) || !std::isfinite(seconds)) {
    throw std::invalid_argument{label + " must be finite and positive"};
  }
  return static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
}

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] std::vector<std::string>
vehicleTopics(const std::vector<std::string>& ids, const std::string& prefix,
              const std::string& suffix) {
  std::vector<std::string> result;
  result.reserve(ids.size());
  for (const std::string& id : ids) {
    std::string topic;
    topic.reserve(prefix.size() + id.size() + suffix.size());
    topic.append(prefix).append(id).append(suffix);
    result.push_back(std::move(topic));
  }
  return result;
}

void requireCount(const std::vector<std::string>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    throw std::invalid_argument{parameter_name + " must contain " +
                                std::to_string(count) + " entries"};
  }
}

[[nodiscard]] msg::NavigationObjective
positionObjective(const rclcpp::Time& stamp, const std::uint64_t mission_epoch,
                  const std::uint64_t sequence, const Point3& position,
                  const std::uint8_t terminal_policy) {
  msg::NavigationObjective objective;
  objective.stamp = stamp;
  objective.mission_epoch = mission_epoch;
  objective.sample_sequence = sequence;
  objective.position.x = position.x;
  objective.position.y = position.y;
  objective.position.z = position.z;
  objective.objective_type = msg::NavigationObjective::OBJECTIVE_TYPE_POSITION;
  objective.guidance_mode = msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
  objective.terminal_policy = terminal_policy;
  return objective;
}

} // namespace

CooperativeTrafficRefereeNode::CooperativeTrafficRefereeNode()
    : Node{"cooperative_traffic_referee_node"} {
  const std::int64_t mission_epoch =
      declare_parameter<std::int64_t>("mission_epoch", 1);
  if (mission_epoch <= 0) {
    throw std::invalid_argument{"mission epoch must be positive"};
  }
  mission_epoch_ = static_cast<std::uint64_t>(mission_epoch);
  goal_hold_config_ = CooperativeGoalHoldConfig{
      .goal_tolerance_m = declare_parameter<double>("goal_tolerance_m", 2.0),
      .hold_position_tolerance_m =
          declare_parameter<double>("hold_position_tolerance_m", 2.0),
      .maximum_speed_mps = declare_parameter<double>("hold_maximum_speed_mps", 0.8),
      .confirmation_duration_s =
          declare_parameter<double>("hold_confirmation_duration_s", 1.0),
  };
  separation_config_ = CooperativeSeparationConfig{
      .desired_minimum_separation_m =
          declare_parameter<double>("desired_minimum_separation_m", 5.0),
      .release_separation_m =
          declare_parameter<double>("separation_release_distance_m", 7.0),
      .maximum_continuity_gap_s =
          declare_parameter<double>("maximum_separation_continuity_gap_s", 0.25),
  };
  maximum_input_age_ns_ = timeoutNanoseconds(
      declare_parameter<double>("maximum_state_age_s", 1.0), "maximum state age");
  const double maximum_intent_age_s =
      declare_parameter<double>("maximum_intent_age_s", 0.5);
  maximum_intent_age_ns_ =
      timeoutNanoseconds(maximum_intent_age_s, "maximum intent age");
  maximum_degraded_duration_ns_ =
      timeoutNanoseconds(declare_parameter<double>("maximum_degraded_duration_s", 5.0),
                         "maximum degraded duration");
  readiness_timeout_ns_ =
      timeoutNanoseconds(declare_parameter<double>("mission_readiness_timeout_s", 60.0),
                         "mission readiness timeout");
  mission_timeout_ns_ = timeoutNanoseconds(
      declare_parameter<double>("mission_timeout_s", 240.0), "mission timeout");
  hold_timeout_ns_ =
      timeoutNanoseconds(declare_parameter<double>("hold_confirmation_timeout_s", 20.0),
                         "hold confirmation timeout");
  destruction_settlement_timeout_ns_ = timeoutNanoseconds(
      declare_parameter<double>("destruction_settlement_timeout_s", 5.0),
      "destruction settlement timeout");
  boundary_startup_timeout_ns_ = timeoutNanoseconds(
      declare_parameter<double>("ground_truth_boundary_startup_timeout_s", 10.0),
      "ground truth boundary startup timeout");
  shutdown_on_terminal_outcome_ =
      declare_parameter<bool>("shutdown_on_terminal_outcome", true);
  outcome_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      declare_parameter<std::string>("mission_outcome_marker_topic",
                                     "/cooperative_traffic/mission_outcome_marker"),
      rclcpp::QoS{1}.reliable().transient_local());

  const std::vector<std::string> vehicle_ids =
      declare_parameter<std::vector<std::string>>(
          "vehicle_ids", {"civilian_0", "civilian_1", "civilian_2", "civilian_3"});
  const std::vector<double> goals_xyz = declare_parameter<std::vector<double>>(
      "vehicle_goals_xyz_m",
      {270.0, 378.0, 18.0, 54.0, 378.0, 18.0, 270.0, 54.0, 18.0, 54.0, 54.0, 18.0});
  configureVehicles(vehicle_ids, goals_xyz);
  separation_monitor_ = std::make_unique<CooperativeSeparationMonitor>(
      vehicles_.size(), separation_config_);
  intent_validator_ = std::make_unique<CooperativePeerStore>(
      "__cooperative_traffic_referee__",
      CooperativePeerStoreConfig{.maximum_publication_age_s = maximum_intent_age_s,
                                 .maximum_peers = vehicles_.size()});

  truth_alignment_sub_ = create_subscription<msg::SimulationTruthAlignment>(
      declare_parameter<std::string>("truth_alignment_status_topic",
                                     "/simulation_truth/alignment"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::SimulationTruthAlignment::SharedPtr status) {
        onTruthAlignmentStatus(*status);
      });
  intent_sub_ = create_subscription<msg::CooperativeFlightIntent>(
      declare_parameter<std::string>("flight_intent_topic",
                                     "/cooperative_traffic/flight_intents"),
      cooperativeFlightIntentQos(),
      [this](const msg::CooperativeFlightIntent::SharedPtr intent) {
        onFlightIntent(*intent);
      });
  configureGroundTruthBoundary();
  for (std::size_t index = 0U; index < vehicles_.size(); ++index) {
    publishGoalObjective(index);
  }
  readiness_started_ns_ = now().nanoseconds();
  timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
  RCLCPP_INFO(get_logger(),
              "COOPERATIVE_TRAFFIC_REFEREE_READY mission_epoch=%" PRIu64
              " vehicles=%zu desired_separation_m=%.2f release_m=%.2f",
              mission_epoch_, vehicles_.size(),
              separation_config_.desired_minimum_separation_m,
              separation_config_.release_separation_m);
}

void CooperativeTrafficRefereeNode::configureVehicles(
    const std::vector<std::string>& ids, const std::vector<double>& goals_xyz) {
  if (ids.size() < 2U || goals_xyz.size() != ids.size() * 3U) {
    throw std::invalid_argument{
        "cooperative vehicle ids and flattened goals have incompatible sizes"};
  }
  std::unordered_set<std::string> unique_ids;
  for (const std::string& id : ids) {
    if (id.empty() || !unique_ids.insert(id).second) {
      throw std::invalid_argument{
          "cooperative vehicle ids must be non-empty and unique"};
    }
  }
  const auto defaults = [&](const std::string& suffix) {
    return vehicleTopics(ids, "/vehicles/", suffix);
  };
  const std::vector<std::string> state_topics =
      declare_parameter<std::vector<std::string>>("vehicle_state_topics",
                                                  defaults("/state"));
  const std::vector<std::string> truth_topics =
      declare_parameter<std::vector<std::string>>(
          "vehicle_truth_state_topics",
          vehicleTopics(ids, "/simulation_truth/vehicles/", "/state"));
  const std::vector<std::string> horizon_topics =
      declare_parameter<std::vector<std::string>>("vehicle_execution_horizon_topics",
                                                  defaults("/mppi/execution_horizon"));
  const std::vector<std::string> world_topics =
      declare_parameter<std::vector<std::string>>("vehicle_world_readiness_topics",
                                                  defaults("/mppi/world_ready"));
  const std::vector<std::string> destroyed_topics =
      declare_parameter<std::vector<std::string>>("vehicle_destroyed_topics",
                                                  defaults("/vehicle_destroyed"));
  const std::vector<std::string> objective_topics =
      declare_parameter<std::vector<std::string>>("vehicle_objective_topics",
                                                  defaults("/navigation_objective"));
  const std::vector<std::string> start_topics =
      declare_parameter<std::vector<std::string>>("vehicle_start_topics",
                                                  defaults("/mission_start"));
  for (const auto& [values, name] :
       std::vector<std::pair<const std::vector<std::string>*, std::string>>{
           {&state_topics, "vehicle_state_topics"},
           {&truth_topics, "vehicle_truth_state_topics"},
           {&horizon_topics, "vehicle_execution_horizon_topics"},
           {&world_topics, "vehicle_world_readiness_topics"},
           {&destroyed_topics, "vehicle_destroyed_topics"},
           {&objective_topics, "vehicle_objective_topics"},
           {&start_topics, "vehicle_start_topics"}}) {
    requireCount(*values, ids.size(), name);
  }

  const auto state_qos = rclcpp::QoS{10}.best_effort();
  const auto horizon_qos = rclcpp::QoS{4}.reliable();
  const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
  vehicles_.resize(ids.size());
  for (std::size_t index = 0U; index < ids.size(); ++index) {
    const Point3 goal{goals_xyz[index * 3U], goals_xyz[index * 3U + 1U],
                      goals_xyz[index * 3U + 2U]};
    if (!finite(goal) || goal.z < 1.0 || goal.z >= 32.0) {
      throw std::invalid_argument{"cooperative goal is outside the flight envelope"};
    }
    vehicle_indices_.emplace(ids[index], index);
    VehicleRuntime& runtime = vehicles_[index];
    runtime.id = ids[index];
    runtime.truth_state_topic = truth_topics[index];
    runtime.goal = goal;
    runtime.goal_hold_confirmation =
        std::make_unique<CooperativeGoalHoldConfirmation>(goal_hold_config_);
    runtime.state_sub = create_subscription<msg::VehicleNavigationState>(
        state_topics[index], state_qos,
        [this, index](const msg::VehicleNavigationState::SharedPtr state) {
          vehicles_[index].navigation_state = detail::vehicleState(*state);
        });
    runtime.truth_state_sub = create_subscription<msg::SimulationTruthState>(
        truth_topics[index], state_qos,
        [this, index](const msg::SimulationTruthState::SharedPtr state) {
          if (state->vehicle_id == vehicles_[index].id) {
            vehicles_[index].truth_state = detail::physicalTruthState(*state);
          }
        });
    runtime.horizon_sub = create_subscription<msg::MppiTrajectoryHorizon>(
        horizon_topics[index], horizon_qos,
        [this, index](const msg::MppiTrajectoryHorizon::SharedPtr horizon) {
          VehicleRuntime& vehicle = vehicles_[index];
          vehicle.hold_horizon = HoldHorizon{
              .position = Point3{horizon->stationary_hold_position.x,
                                 horizon->stationary_hold_position.y,
                                 horizon->stationary_hold_position.z},
              .sequence = horizon->sequence,
              .active = horizon->stationary_position_hold &&
                        horizon->execution_mode ==
                            msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD,
          };
          if (!vehicle.executable_horizon_ready && horizon->sequence > 0U &&
              horizon->execution_mode ==
                  msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED &&
              horizon->points.size() >= 2U) {
            vehicle.executable_horizon_ready = true;
            vehicle.first_executable_horizon_sequence = horizon->sequence;
            RCLCPP_INFO(get_logger(),
                        "COOPERATIVE_EXECUTABLE_HORIZON_READY vehicle_id='%s' "
                        "sequence=%" PRIu64,
                        vehicle.id.c_str(), horizon->sequence);
          }
        });
    runtime.world_ready_sub = create_subscription<std_msgs::msg::Bool>(
        world_topics[index], latched_qos,
        [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
          vehicles_[index].world_ready = ready->data;
        });
    runtime.destroyed_sub = create_subscription<msg::VehicleDestroyed>(
        destroyed_topics[index], latched_qos,
        [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
          onVehicleDestroyed(*destroyed, index);
        });
    runtime.objective_pub = create_publisher<msg::NavigationObjective>(
        objective_topics[index], latched_qos);
    runtime.start_pub =
        create_publisher<std_msgs::msg::Bool>(start_topics[index], latched_qos);
  }
}

void CooperativeTrafficRefereeNode::configureGroundTruthBoundary() {
  std::vector<std::string> truth_topics;
  truth_topics.reserve(vehicles_.size());
  for (const VehicleRuntime& vehicle : vehicles_) {
    truth_topics.push_back(vehicle.truth_state_topic);
  }
  ground_truth_boundary_ =
      makeExclusiveGroundTruthBoundary(get_fully_qualified_name(), truth_topics);
}

void CooperativeTrafficRefereeNode::onTruthAlignmentStatus(
    const msg::SimulationTruthAlignment& status) {
  truth_alignment_reason_ = status.reason;
  truth_alignment_vehicle_id_ = status.vehicle_id;
  truth_alignment_maximum_error_m_ = status.maximum_position_error_m;
  truth_alignment_update_ =
      truth_alignment_lifecycle_.update(SimulationTruthAlignmentObservation{
          .ready = status.ready,
          .sample_aligned = status.reason == "aligned",
          .failure_confirmed = status.failure_confirmed,
      });
}

void CooperativeTrafficRefereeNode::onFlightIntent(
    const msg::CooperativeFlightIntent& intent_message) {
  CooperativeFlightIntentData intent = cooperativeFlightIntentData(intent_message);
  const auto vehicle = vehicle_indices_.find(intent.vehicle_id);
  if (vehicle == vehicle_indices_.end()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "COOPERATIVE_REFEREE_INTENT rejected=true reason=unknown_vehicle "
        "vehicle_id='%s'",
        intent.vehicle_id.c_str());
    return;
  }
  const std::int64_t now_ns = now().nanoseconds();
  const CooperativePeerUpdateStatus status = intent_validator_->update(intent, now_ns);
  if (status != CooperativePeerUpdateStatus::kAccepted) {
    if (status == CooperativePeerUpdateStatus::kInvalid ||
        status == CooperativePeerUpdateStatus::kStale) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "COOPERATIVE_REFEREE_INTENT rejected=true status=%u vehicle_id='%s'",
          static_cast<unsigned>(status), intent.vehicle_id.c_str());
    }
    return;
  }
  VehicleRuntime& runtime = vehicles_[vehicle->second];
  if (!runtime.intent_ready) {
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_INTENT_READY vehicle_id='%s' generation=%" PRIu64
                " valid_for_ms=%.1f",
                runtime.id.c_str(), intent.intent_generation,
                static_cast<double>(intent.valid_until_ns - now_ns) * 1.0e-6);
  }
  runtime.intent_ready = true;
  runtime.latest_intent_generation = intent.intent_generation;
  runtime.latest_intent_receive_ns = now_ns;
  runtime.latest_intent_valid_until_ns = intent.valid_until_ns;
}

void CooperativeTrafficRefereeNode::onVehicleDestroyed(
    const msg::VehicleDestroyed& destroyed, const std::size_t vehicle_index) {
  VehicleRuntime& vehicle = vehicles_[vehicle_index];
  const bool valid =
      destroyed.vehicle_role == msg::VehicleDestroyed::ROLE_CIVILIAN &&
      destroyed.vehicle_id == vehicle.id &&
      detail::validDeathCause(destroyed.death_cause) &&
      (destroyed.mission_epoch == 0U || destroyed.mission_epoch == mission_epoch_);
  if (!valid) {
    RCLCPP_ERROR(get_logger(),
                 "COOPERATIVE_VEHICLE_DESTROYED rejected=true expected_id='%s' "
                 "actual_id='%s' role=%u cause=%u event_epoch=%" PRIu64,
                 vehicle.id.c_str(), destroyed.vehicle_id.c_str(),
                 static_cast<unsigned>(destroyed.vehicle_role),
                 static_cast<unsigned>(destroyed.death_cause), destroyed.mission_epoch);
    return;
  }
  if (vehicle.destroyed) {
    return;
  }
  vehicle.destroyed = true;
  const std::int64_t observed_ns = now().nanoseconds();
  vehicle.destroyed_observed_ns = observed_ns;
  RCLCPP_ERROR(get_logger(),
               "COOPERATIVE_VEHICLE_DESTROYED referee_observed=true vehicle_id='%s' "
               "cause=%s drone_collision='%s' obstacle_collision='%s' detail='%s' "
               "mission_epoch=%" PRIu64,
               vehicle.id.c_str(), detail::deathCauseName(destroyed.death_cause),
               destroyed.drone_collision.c_str(), destroyed.obstacle_collision.c_str(),
               destroyed.detail.c_str(), mission_epoch_);
  if (vehicle.navigation_state && vehicle.truth_state) {
    const double position_error_m =
        distance3D(vehicle.navigation_state->position, vehicle.truth_state->position);
    const double navigation_age_ms =
        static_cast<double>(observed_ns - vehicle.navigation_state->stamp_ns) * 1.0e-6;
    const double truth_age_ms =
        static_cast<double>(observed_ns - vehicle.truth_state->stamp_ns) * 1.0e-6;
    RCLCPP_ERROR(
        get_logger(),
        "COOPERATIVE_VEHICLE_DESTRUCTION_STATE vehicle_id='%s' "
        "navigation_position=(%.3f,%.3f,%.3f) truth_position=(%.3f,%.3f,%.3f) "
        "navigation_truth_error_m=%.3f navigation_age_ms=%.1f truth_age_ms=%.1f "
        "mission_epoch=%" PRIu64,
        vehicle.id.c_str(), vehicle.navigation_state->position.x,
        vehicle.navigation_state->position.y, vehicle.navigation_state->position.z,
        vehicle.truth_state->position.x, vehicle.truth_state->position.y,
        vehicle.truth_state->position.z, position_error_m, navigation_age_ms,
        truth_age_ms, mission_epoch_);
  }
  beginFailure("vehicle_destroyed:" + vehicle.id + ":" +
               detail::deathCauseName(destroyed.death_cause));
}

std::optional<TimedVehicleState>
CooperativeTrafficRefereeNode::physicalState(const std::size_t index) const noexcept {
  return detail::physicalState(vehicles_[index].navigation_state,
                               vehicles_[index].truth_state);
}

void CooperativeTrafficRefereeNode::publishGoalObjective(const std::size_t index) {
  VehicleRuntime& vehicle = vehicles_[index];
  vehicle.objective_pub->publish(positionObjective(
      now(), mission_epoch_, ++vehicle.objective_sequence, vehicle.goal,
      msg::NavigationObjective::TERMINAL_POLICY_POSITION_HOLD));
  RCLCPP_INFO(get_logger(),
              "COOPERATIVE_OBJECTIVE vehicle_id='%s' goal=(%.3f,%.3f,%.3f) "
              "sequence=%" PRIu64 " mission_epoch=%" PRIu64,
              vehicle.id.c_str(), vehicle.goal.x, vehicle.goal.y, vehicle.goal.z,
              vehicle.objective_sequence, mission_epoch_);
}

bool CooperativeTrafficRefereeNode::verifyGroundTruthBoundary(
    const std::int64_t now_ns) {
  if (last_boundary_check_ns_ > 0 && now_ns - last_boundary_check_ns_ < 500'000'000LL) {
    return boundary_verified_;
  }
  last_boundary_check_ns_ = now_ns;
  const GroundTruthBoundaryUpdate update = ground_truth_boundary_->update(*this);
  if (!update.violating_subscriber.empty()) {
    beginFailure("ground_truth_boundary_violation:" + update.violating_topic + ":" +
                 update.violating_subscriber);
    return false;
  }
  boundary_verified_ = update.verified;
  if (update.newly_verified) {
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_GROUND_TRUTH_BOUNDARY verified=true vehicles=%zu "
                "referee_only=true",
                vehicles_.size());
  }
  return boundary_verified_;
}

bool CooperativeTrafficRefereeNode::missionReady(const std::int64_t now_ns) const {
  if (!truth_alignment_update_.startup_ready || !boundary_verified_) {
    return false;
  }
  return std::ranges::all_of(vehicles_, [this, now_ns](const VehicleRuntime& vehicle) {
    return !vehicle.destroyed && vehicle.navigation_state && vehicle.truth_state &&
           vehicle.navigation_state->navigation_ready && vehicle.world_ready &&
           vehicle.executable_horizon_ready && vehicle.intent_ready &&
           vehicle.latest_intent_receive_ns > 0 &&
           now_ns >= vehicle.latest_intent_receive_ns &&
           now_ns - vehicle.latest_intent_receive_ns <= maximum_intent_age_ns_ &&
           now_ns <= vehicle.latest_intent_valid_until_ns;
  });
}

void CooperativeTrafficRefereeNode::logMissionReadiness(
    const std::int64_t now_ns) const {
  RCLCPP_ERROR(get_logger(), "COOPERATIVE_READINESS_STATUS alignment=%s boundary=%s",
               truth_alignment_update_.startup_ready ? "ready" : "not_ready",
               boundary_verified_ ? "ready" : "not_ready");
  for (const VehicleRuntime& vehicle : vehicles_) {
    const double intent_age_ms =
        vehicle.latest_intent_receive_ns > 0 &&
                now_ns >= vehicle.latest_intent_receive_ns
            ? static_cast<double>(now_ns - vehicle.latest_intent_receive_ns) * 1.0e-6
            : std::numeric_limits<double>::infinity();
    RCLCPP_ERROR(
        get_logger(),
        "COOPERATIVE_READINESS_STATUS vehicle_id='%s' destroyed=%s navigation=%s "
        "world=%s horizon=%s intent=%s intent_age_ms=%.1f intent_valid=%s",
        vehicle.id.c_str(), vehicle.destroyed ? "true" : "false",
        vehicle.navigation_state && vehicle.navigation_state->navigation_ready
            ? "ready"
            : "not_ready",
        vehicle.world_ready ? "ready" : "not_ready",
        vehicle.executable_horizon_ready ? "ready" : "not_ready",
        vehicle.intent_ready ? "ready" : "not_ready", intent_age_ms,
        now_ns <= vehicle.latest_intent_valid_until_ns ? "true" : "false");
  }
}

void CooperativeTrafficRefereeNode::publishMissionStart() {
  if (!truth_alignment_lifecycle_.latchStartupContract()) {
    beginFailure("truth_alignment_contract_not_ready");
    return;
  }
  std_msgs::msg::Bool start;
  start.data = true;
  for (VehicleRuntime& vehicle : vehicles_) {
    vehicle.start_pub->publish(start);
  }
  mission_started_ = true;
  mission_started_ns_ = now().nanoseconds();
  RCLCPP_INFO(get_logger(),
              "COOPERATIVE_TRAFFIC_MISSION state=running vehicle_count=%zu "
              "mission_epoch=%" PRIu64
              " startup_coordinate_contract_latched=true all_intents_ready=true "
              "all_executable_horizons_ready=true",
              vehicles_.size(), mission_epoch_);
}

} // namespace drone_city_nav
