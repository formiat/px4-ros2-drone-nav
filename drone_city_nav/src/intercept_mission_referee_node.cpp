#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/msg/intercept_mission_command.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::string
fullyQualifiedNodeName(const rclcpp::TopicEndpointInfo& endpoint) {
  const std::string& node_namespace = endpoint.node_namespace();
  return node_namespace == "/" ? "/" + endpoint.node_name()
                               : node_namespace + "/" + endpoint.node_name();
}

[[nodiscard]] bool
endpointIdentityKnown(const rclcpp::TopicEndpointInfo& endpoint) noexcept {
  return !endpoint.node_name().empty() &&
         endpoint.node_name() != "_NODE_NAME_UNKNOWN_" &&
         !endpoint.node_namespace().empty() &&
         endpoint.node_namespace() != "_NODE_NAMESPACE_UNKNOWN_";
}

[[nodiscard]] const char* vehicleRoleName(const std::uint8_t role) noexcept {
  switch (role) {
    case msg::VehicleDestroyed::ROLE_INTERCEPTOR:
      return "interceptor";
    case msg::VehicleDestroyed::ROLE_EVADER:
      return "evader";
    default:
      return "unspecified";
  }
}

[[nodiscard]] const char* deathCauseName(const std::uint8_t cause) noexcept {
  switch (cause) {
    case msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION:
      return "physical_collision";
    case msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT:
      return "proximity_intercept";
    default:
      return "invalid";
  }
}

[[nodiscard]] double speed(const TimedVehicleState& state) noexcept {
  return state.velocity_valid ? std::sqrt(state.velocity.x * state.velocity.x +
                                          state.velocity.y * state.velocity.y +
                                          state.velocity.z * state.velocity.z)
                              : 0.0;
}

} // namespace

class InterceptMissionRefereeNode final : public rclcpp::Node {
public:
  InterceptMissionRefereeNode()
      : Node{"intercept_mission_referee_node"} {
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 1));
    const Point3 target_goal{
        declare_parameter<double>("evader_goal_x_m", 54.0),
        declare_parameter<double>("evader_goal_y_m", 378.0),
        declare_parameter<double>("evader_goal_z_m", 18.0),
    };
    evaluator_capture_radius_m_ = declare_parameter<double>("capture_radius_m", 5.0);
    evaluator_ = std::make_unique<InterceptMissionEvaluator>(
        target_goal,
        InterceptMissionConfig{
            .capture_radius_m = evaluator_capture_radius_m_,
            .evader_goal_radius_m =
                declare_parameter<double>("evader_goal_radius_m", 2.0),
            .evader_goal_stop_speed_mps =
                declare_parameter<double>("evader_goal_stop_speed_mps", 0.8),
            .evader_goal_hold_s = declare_parameter<double>("evader_goal_hold_s", 2.0),
        });
    target_goal_ = target_goal;
    maximum_state_age_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("maximum_state_age_s", 1.0) * 1.0e9);
    destruction_settlement_timeout_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("destruction_settlement_timeout_s", 5.0) * 1.0e9);
    hold_config_.position_tolerance_m =
        declare_parameter<double>("interceptor_hold_position_tolerance_m", 2.0);
    hold_config_.maximum_speed_mps =
        declare_parameter<double>("interceptor_hold_maximum_speed_mps", 0.8);
    hold_config_.confirmation_duration_s =
        declare_parameter<double>("interceptor_hold_confirmation_duration_s", 1.0);
    hold_timeout_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("interceptor_hold_timeout_s", 20.0) * 1.0e9);
    boundary_startup_timeout_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("ground_truth_boundary_startup_timeout_s", 10.0) *
        1.0e9);
    shutdown_on_terminal_outcome_ =
        declare_parameter<bool>("shutdown_on_terminal_outcome", true);
    if (!(hold_config_.position_tolerance_m > 0.0) ||
        !(hold_config_.maximum_speed_mps >= 0.0) ||
        !(hold_config_.confirmation_duration_s >= 0.0) || !(hold_timeout_ns_ > 0) ||
        !(boundary_startup_timeout_ns_ > 0) ||
        !(destruction_settlement_timeout_ns_ > 0)) {
      throw std::invalid_argument{"invalid intercept referee configuration"};
    }

    ownship_state_topic_ = declare_parameter<std::string>(
        "interceptor_state_topic", "/vehicles/interceptor/state");
    target_state_topic_ =
        declare_parameter<std::string>("evader_state_topic", "/vehicles/evader/state");
    radar_simulator_node_fqn_ = declare_parameter<std::string>(
        "radar_simulator_node_fqn", "/radar_simulator_node");
    const auto state_qos = rclcpp::QoS{10}.best_effort();
    ownship_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        ownship_state_topic_, state_qos,
        [this](const msg::VehicleNavigationState::SharedPtr state) {
          ownship_state_ = detail::vehicleState(*state);
        });
    target_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        target_state_topic_, state_qos,
        [this](const msg::VehicleNavigationState::SharedPtr state) {
          target_state_ = detail::vehicleState(*state);
        });
    ownship_destroyed_topic_ = declare_parameter<std::string>(
        "interceptor_destroyed_topic", "/vehicles/interceptor/vehicle_destroyed");
    target_destroyed_topic_ = declare_parameter<std::string>(
        "evader_destroyed_topic", "/vehicles/evader/vehicle_destroyed");
    ownship_destroyed_sub_ = makeDestroyedSubscription(
        ownship_destroyed_topic_, msg::VehicleDestroyed::ROLE_INTERCEPTOR);
    target_destroyed_sub_ = makeDestroyedSubscription(
        target_destroyed_topic_, msg::VehicleDestroyed::ROLE_EVADER);

    target_objective_pub_ = create_publisher<msg::NavigationObjective>(
        declare_parameter<std::string>("evader_objective_topic",
                                       "/vehicles/evader/navigation_objective"),
        rclcpp::QoS{1}.reliable().transient_local());
    ownship_start_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter<std::string>("interceptor_start_topic",
                                       "/vehicles/interceptor/mission_start"),
        rclcpp::QoS{1}.reliable().transient_local());
    target_start_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter<std::string>("evader_start_topic",
                                       "/vehicles/evader/mission_start"),
        rclcpp::QoS{1}.reliable().transient_local());
    ownship_destroyed_pub_ = create_publisher<msg::VehicleDestroyed>(
        ownship_destroyed_topic_, rclcpp::QoS{1}.reliable().transient_local());
    target_destroyed_pub_ = create_publisher<msg::VehicleDestroyed>(
        target_destroyed_topic_, rclcpp::QoS{1}.reliable().transient_local());
    mission_command_pub_ = create_publisher<msg::InterceptMissionCommand>(
        declare_parameter<std::string>("interceptor_mission_command_topic",
                                       "/vehicles/interceptor/mission_command"),
        rclcpp::QoS{1}.reliable().transient_local());

    publishTargetObjective();
    timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Intercept mission referee ready: epoch=%" PRIu64
                " target_goal=(%.2f,%.2f,%.2f) truth_boundary_topic='%s'",
                mission_epoch_, target_goal_.x, target_goal_.y, target_goal_.z,
                target_state_topic_.c_str());
  }

private:
  [[nodiscard]] rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr
  makeDestroyedSubscription(const std::string& topic,
                            const std::uint8_t expected_role) {
    return create_subscription<msg::VehicleDestroyed>(
        topic, rclcpp::QoS{1}.reliable().transient_local(),
        [this, expected_role](const msg::VehicleDestroyed::SharedPtr destroyed) {
          if (destroyed->vehicle_role != expected_role ||
              (destroyed->mission_epoch != 0U &&
               destroyed->mission_epoch != mission_epoch_)) {
            RCLCPP_ERROR(get_logger(),
                         "VEHICLE_DESTROYED referee_rejected=true expected_role=%s "
                         "actual_role=%u event_epoch=%" PRIu64
                         " mission_epoch=%" PRIu64,
                         vehicleRoleName(expected_role),
                         static_cast<unsigned>(destroyed->vehicle_role),
                         destroyed->mission_epoch, mission_epoch_);
            return;
          }
          onVehicleDestroyed(*destroyed);
        });
  }

  void onVehicleDestroyed(const msg::VehicleDestroyed& destroyed) {
    if (destroyed.death_cause != msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION &&
        destroyed.death_cause != msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT) {
      RCLCPP_ERROR(get_logger(),
                   "VEHICLE_DESTROYED referee_rejected=true role=%s cause=%u "
                   "reason=invalid_cause mission_epoch=%" PRIu64,
                   vehicleRoleName(destroyed.vehicle_role),
                   static_cast<unsigned>(destroyed.death_cause), mission_epoch_);
      return;
    }
    const bool ownship =
        destroyed.vehicle_role == msg::VehicleDestroyed::ROLE_INTERCEPTOR;
    bool& role_destroyed = ownship ? ownship_destroyed_ : target_destroyed_;
    std::uint8_t& role_cause = ownship ? ownship_death_cause_ : target_death_cause_;
    if (role_destroyed) {
      return;
    }
    role_destroyed = true;
    role_cause = destroyed.death_cause;
    RCLCPP_ERROR(get_logger(),
                 "VEHICLE_DESTROYED referee_observed=true role=%s cause=%s "
                 "mission_epoch=%" PRIu64 " detail='%s'",
                 vehicleRoleName(destroyed.vehicle_role),
                 deathCauseName(destroyed.death_cause), mission_epoch_,
                 destroyed.detail.c_str());

    if (destroyed.death_cause != msg::VehicleDestroyed::CAUSE_PHYSICAL_COLLISION ||
        terminal_outcome_.has_value() || physical_death_pending_) {
      return;
    }
    physical_death_pending_ = true;
    physical_death_role_ = destroyed.vehicle_role;
    physical_death_reason_ =
        std::string{"physical_collision_"} + vehicleRoleName(destroyed.vehicle_role);
    destruction_requested_ns_ = now().nanoseconds();
    if (!ownship) {
      requestHold("evader_destroyed");
    }
  }

  void publishTargetObjective() {
    msg::NavigationObjective objective;
    objective.stamp = now();
    objective.mission_epoch = mission_epoch_;
    objective.sample_sequence = 1U;
    objective.position.x = target_goal_.x;
    objective.position.y = target_goal_.y;
    objective.position.z = target_goal_.z;
    objective.objective_type = msg::NavigationObjective::OBJECTIVE_TYPE_POSITION;
    objective.guidance_mode = msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
    objective.terminal_policy = msg::NavigationObjective::TERMINAL_POLICY_POSITION_HOLD;
    target_objective_pub_->publish(objective);
  }

  [[nodiscard]] bool verifyGroundTruthBoundary(const std::int64_t now_ns) {
    if (last_boundary_check_ns_ > 0 &&
        now_ns - last_boundary_check_ns_ < 500'000'000LL) {
      return boundary_verified_;
    }
    last_boundary_check_ns_ = now_ns;
    const std::unordered_set<std::string> allowed{get_fully_qualified_name(),
                                                  radar_simulator_node_fqn_};
    std::unordered_set<std::string> observed;
    bool identity_pending = false;
    for (const rclcpp::TopicEndpointInfo& endpoint :
         get_subscriptions_info_by_topic(target_state_topic_)) {
      if (!endpointIdentityKnown(endpoint)) {
        identity_pending = true;
        continue;
      }
      const std::string subscriber = fullyQualifiedNodeName(endpoint);
      observed.insert(subscriber);
      if (!allowed.contains(subscriber)) {
        failMission("ground_truth_boundary_violation:" + subscriber);
        return false;
      }
    }
    if (identity_pending) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "RADAR_DATA_BOUNDARY verified=%s identity_pending=true action=%s",
          boundary_verified_ ? "true" : "false",
          boundary_verified_ ? "retain_verified_boundary" : "delay_mission_start");
      return boundary_verified_;
    }
    const bool complete = observed.contains(get_fully_qualified_name()) &&
                          observed.contains(radar_simulator_node_fqn_);
    if (complete && !boundary_verified_) {
      boundary_verified_ = true;
      RCLCPP_INFO(get_logger(),
                  "RADAR_DATA_BOUNDARY verified=true truth_topic='%s' "
                  "allowed_subscribers=referee,radar_simulator",
                  target_state_topic_.c_str());
    }
    return boundary_verified_;
  }

  void publishMissionStart() {
    std_msgs::msg::Bool start;
    start.data = true;
    ownship_start_pub_->publish(start);
    target_start_pub_->publish(start);
    mission_started_ = true;
    RCLCPP_INFO(get_logger(), "INTERCEPT_MISSION state=running epoch=%" PRIu64,
                mission_epoch_);
  }

  void requestHold(const std::string& reason) {
    if (ownship_destroyed_ || hold_confirmation_) {
      return;
    }
    if (!ownship_state_.has_value() || !ownship_state_->position_valid) {
      failMission("interceptor_hold_position_unavailable");
      return;
    }
    hold_position_ = ownship_state_->position;
    hold_confirmation_ =
        std::make_unique<InterceptorHoldConfirmation>(hold_position_, hold_config_);
    hold_requested_ns_ = now().nanoseconds();
    msg::InterceptMissionCommand command;
    command.stamp = now();
    command.mission_epoch = mission_epoch_;
    command.command = msg::InterceptMissionCommand::COMMAND_HOLD_CURRENT_POSITION;
    command.reason = reason;
    mission_command_pub_->publish(command);
    RCLCPP_INFO(get_logger(),
                "INTERCEPTOR_HOLD requested=true source=mission_command "
                "position=(%.3f,%.3f,%.3f) mission_epoch=%" PRIu64,
                hold_position_.x, hold_position_.y, hold_position_.z, mission_epoch_);
  }

  [[nodiscard]] msg::VehicleDestroyed
  makeProximityDestruction(const TimedVehicleState& state, const std::uint8_t role,
                           const std::string& detail) const {
    msg::VehicleDestroyed destroyed;
    destroyed.stamp = now();
    destroyed.mission_epoch = mission_epoch_;
    destroyed.vehicle_role = role;
    destroyed.death_cause = msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT;
    destroyed.detail = detail;
    destroyed.event_position.x = state.position.x;
    destroyed.event_position.y = state.position.y;
    destroyed.event_position.z = state.position.z;
    destroyed.altitude_m = state.position.z;
    destroyed.speed_mps = speed(state);
    return destroyed;
  }

  void requestProximityDestruction(const std::string& detail = "intercepted") {
    if (proximity_destruction_requested_ || !ownship_state_ || !target_state_) {
      return;
    }
    proximity_destruction_requested_ = true;
    ownship_destroyed_pub_->publish(makeProximityDestruction(
        *ownship_state_, msg::VehicleDestroyed::ROLE_INTERCEPTOR, detail));
    target_destroyed_pub_->publish(makeProximityDestruction(
        *target_state_, msg::VehicleDestroyed::ROLE_EVADER, detail));
    destruction_requested_ns_ = now().nanoseconds();
    RCLCPP_ERROR(get_logger(),
                 "PROXIMITY_INTERCEPT destruction_requested=true "
                 "separation_threshold_m=%.3f mission_epoch=%" PRIu64 " detail='%s'",
                 evaluator_capture_radius_m_, mission_epoch_, detail.c_str());
  }

  void handleLateCapture(const InterceptMissionUpdate& update) {
    if (!terminal_outcome_.has_value() ||
        *terminal_outcome_ != InterceptMissionOutcome::kEvaderReachedGoal ||
        !update.newly_captured || late_capture_after_goal_) {
      return;
    }
    late_capture_after_goal_ = true;
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_LATE_CAPTURE outcome_preserved=evader_reached_goal "
                "separation_m=%.3f mission_epoch=%" PRIu64,
                update.separation_m, mission_epoch_);
    RCLCPP_INFO(get_logger(),
                "INTERCEPTOR_HOLD_ABORTED reason=late_capture mission_epoch=%" PRIu64,
                mission_epoch_);
    requestProximityDestruction("late_intercept_after_evader_goal");
  }

  [[nodiscard]] bool updateHoldConfirmation(const std::int64_t now_ns) {
    if (!hold_confirmation_ || !ownship_state_.has_value()) {
      failMission("interceptor_hold_not_initialized");
      return false;
    }
    const InterceptorHoldUpdate hold = hold_confirmation_->update(*ownship_state_);
    if (hold.newly_confirmed) {
      RCLCPP_INFO(get_logger(),
                  "INTERCEPTOR_HOLD_CONFIRMED position_error_m=%.3f speed_mps=%.3f "
                  "mission_epoch=%" PRIu64,
                  hold.position_error_m, hold.speed_mps, mission_epoch_);
    }
    if (hold_requested_ns_ > 0 && now_ns - hold_requested_ns_ > hold_timeout_ns_) {
      failMission("interceptor_hold_not_confirmed");
      return false;
    }
    return hold.confirmed;
  }

  void settleHold(const std::int64_t now_ns) {
    if (updateHoldConfirmation(now_ns)) {
      finishMission();
    }
  }

  void settleProximityDestruction(const std::int64_t now_ns) {
    const bool events_confirmed =
        ownship_destroyed_ && target_destroyed_ &&
        ownship_death_cause_ == msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT &&
        target_death_cause_ == msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT;
    const bool disarm_confirmed = ownship_state_ && target_state_ &&
                                  !ownship_state_->armed && !target_state_->armed;
    if (events_confirmed && disarm_confirmed) {
      RCLCPP_INFO(get_logger(),
                  "INTERCEPT_SETTLEMENT destruction_events=2 disarm_confirmations=2 "
                  "mission_epoch=%" PRIu64,
                  mission_epoch_);
      finishMission();
      return;
    }
    if (destruction_requested_ns_ > 0 &&
        now_ns - destruction_requested_ns_ > destruction_settlement_timeout_ns_) {
      failMission("proximity_destruction_not_confirmed");
    }
  }

  void settlePhysicalDeath(const std::int64_t now_ns) {
    if (!physical_death_pending_ || !ownship_state_ || !target_state_) {
      return;
    }
    const bool evader_destroyed =
        physical_death_role_ == msg::VehicleDestroyed::ROLE_EVADER;
    const bool destroyed_vehicle_disarmed =
        evader_destroyed ? !target_state_->armed : !ownship_state_->armed;
    const bool survivor_settled =
        !evader_destroyed || ownship_destroyed_ || updateHoldConfirmation(now_ns);
    if (destroyed_vehicle_disarmed && survivor_settled) {
      failMission(physical_death_reason_);
      return;
    }
    if (destruction_requested_ns_ > 0 &&
        now_ns - destruction_requested_ns_ > destruction_settlement_timeout_ns_) {
      failMission(physical_death_reason_ + ":settlement_not_confirmed");
    }
  }

  void finishMission() {
    if (result_reported_ || !terminal_outcome_.has_value()) {
      return;
    }
    result_reported_ = true;
    const bool intercepted =
        *terminal_outcome_ == InterceptMissionOutcome::kIntercepted;
    RCLCPP_INFO(get_logger(),
                "MISSION_RESULT success=true mission=intercept outcome=%s "
                "intercept_success=%s mission_epoch=%" PRIu64,
                interceptMissionOutcomeName(*terminal_outcome_),
                intercepted ? "true" : "false", mission_epoch_);
    completeResultLifecycle();
  }

  void failMission(const std::string& reason) {
    if (result_reported_) {
      return;
    }
    result_reported_ = true;
    RCLCPP_ERROR(get_logger(),
                 "MISSION_RESULT success=false mission=intercept "
                 "outcome=system_failure reason='%s' mission_epoch=%" PRIu64
                 " disarm_requested=false",
                 reason.c_str(), mission_epoch_);
    completeResultLifecycle();
  }

  void completeResultLifecycle() {
    if (shutdown_on_terminal_outcome_) {
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_MISSION state=terminal_observation "
                "simulation_shutdown_requested=false epoch=%" PRIu64,
                mission_epoch_);
  }

  [[nodiscard]] bool stateFresh(const TimedVehicleState& state,
                                const std::int64_t now_ns) const noexcept {
    return state.stamp_ns > 0 && now_ns >= state.stamp_ns &&
           now_ns - state.stamp_ns <= maximum_state_age_ns_;
  }

  void tick() {
    if (result_reported_) {
      return;
    }
    const std::int64_t now_ns = now().nanoseconds();
    if (boundary_check_started_ns_ <= 0) {
      boundary_check_started_ns_ = now_ns;
    }
    if (!verifyGroundTruthBoundary(now_ns)) {
      if (!result_reported_ && boundary_check_started_ns_ > 0 &&
          now_ns - boundary_check_started_ns_ > boundary_startup_timeout_ns_) {
        failMission("ground_truth_boundary_not_ready");
      }
      return;
    }
    if (!ownship_state_.has_value() || !target_state_.has_value()) {
      return;
    }
    if (mission_started_ && physical_death_pending_) {
      settlePhysicalDeath(now_ns);
      return;
    }
    if (mission_started_ &&
        (!stateFresh(*ownship_state_, now_ns) || !stateFresh(*target_state_, now_ns))) {
      failMission("stale_vehicle_state");
      return;
    }
    if (!mission_started_) {
      if (ownship_state_->navigation_ready && target_state_->navigation_ready) {
        publishMissionStart();
      }
      return;
    }
    const InterceptMissionUpdate update =
        evaluator_->update(*ownship_state_, *target_state_);
    if (!terminal_outcome_.has_value()) {
      if (update.newly_terminal) {
        terminal_outcome_ = update.outcome;
        RCLCPP_INFO(get_logger(),
                    "INTERCEPT_OUTCOME outcome=%s first_terminal_event=true "
                    "separation_m=%.3f epoch=%" PRIu64,
                    interceptMissionOutcomeName(update.outcome), update.separation_m,
                    mission_epoch_);
        if (update.outcome == InterceptMissionOutcome::kIntercepted) {
          requestProximityDestruction();
        } else {
          requestHold("evader_reached_goal");
        }
      }
      return;
    }
    handleLateCapture(update);
    if (*terminal_outcome_ == InterceptMissionOutcome::kIntercepted ||
        late_capture_after_goal_) {
      settleProximityDestruction(now_ns);
    } else {
      settleHold(now_ns);
    }
  }

  std::unique_ptr<InterceptMissionEvaluator> evaluator_;
  std::unique_ptr<InterceptorHoldConfirmation> hold_confirmation_;
  InterceptorHoldConfig hold_config_{};
  Point3 target_goal_{};
  Point3 hold_position_{};
  std::optional<TimedVehicleState> ownship_state_;
  std::optional<TimedVehicleState> target_state_;
  std::optional<InterceptMissionOutcome> terminal_outcome_;
  std::string ownship_state_topic_;
  std::string target_state_topic_;
  std::string ownship_destroyed_topic_;
  std::string target_destroyed_topic_;
  std::string radar_simulator_node_fqn_;
  std::string physical_death_reason_;
  double evaluator_capture_radius_m_{5.0};
  std::uint64_t mission_epoch_{1U};
  std::int64_t maximum_state_age_ns_{1'000'000'000LL};
  std::int64_t destruction_settlement_timeout_ns_{5'000'000'000LL};
  std::int64_t hold_timeout_ns_{20'000'000'000LL};
  std::int64_t boundary_startup_timeout_ns_{10'000'000'000LL};
  std::int64_t destruction_requested_ns_{0};
  std::int64_t hold_requested_ns_{0};
  std::int64_t boundary_check_started_ns_{0};
  std::int64_t last_boundary_check_ns_{0};
  bool mission_started_{false};
  bool result_reported_{false};
  bool late_capture_after_goal_{false};
  bool proximity_destruction_requested_{false};
  bool ownship_destroyed_{false};
  bool target_destroyed_{false};
  bool physical_death_pending_{false};
  bool shutdown_on_terminal_outcome_{true};
  bool boundary_verified_{false};
  std::uint8_t ownship_death_cause_{0U};
  std::uint8_t target_death_cause_{0U};
  std::uint8_t physical_death_role_{msg::VehicleDestroyed::ROLE_UNSPECIFIED};
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr ownship_state_sub_;
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr target_state_sub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr ownship_destroyed_sub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr target_destroyed_sub_;
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr target_objective_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ownship_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_start_pub_;
  rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr ownship_destroyed_pub_;
  rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr target_destroyed_pub_;
  rclcpp::Publisher<msg::InterceptMissionCommand>::SharedPtr mission_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::InterceptMissionRefereeNode>());
  rclcpp::shutdown();
  return 0;
}
