#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/msg/crash_state.hpp"
#include "drone_city_nav/msg/intercept_mission_command.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/msg/vehicle_termination.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cinttypes>
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
    evaluator_ = std::make_unique<InterceptMissionEvaluator>(
        target_goal,
        InterceptMissionConfig{
            .capture_radius_m = declare_parameter<double>("capture_radius_m", 5.0),
            .evader_goal_radius_m =
                declare_parameter<double>("evader_goal_radius_m", 2.0),
            .evader_goal_stop_speed_mps =
                declare_parameter<double>("evader_goal_stop_speed_mps", 0.8),
            .evader_goal_hold_s = declare_parameter<double>("evader_goal_hold_s", 2.0),
        });
    target_goal_ = target_goal;
    maximum_state_age_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("maximum_state_age_s", 1.0) * 1.0e9);
    termination_timeout_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("termination_timeout_s", 3.0) * 1.0e9);
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
        !(boundary_startup_timeout_ns_ > 0)) {
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
    ownship_crash_sub_ = makeCrashSubscription(
        declare_parameter<std::string>("interceptor_crash_topic",
                                       "/vehicles/interceptor/crash_state"),
        "interceptor");
    target_crash_sub_ =
        makeCrashSubscription(declare_parameter<std::string>(
                                  "evader_crash_topic", "/vehicles/evader/crash_state"),
                              "evader");

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
    ownship_termination_pub_ = create_publisher<msg::VehicleTermination>(
        declare_parameter<std::string>("interceptor_termination_topic",
                                       "/vehicles/interceptor/termination"),
        rclcpp::QoS{1}.reliable().transient_local());
    target_termination_pub_ = create_publisher<msg::VehicleTermination>(
        declare_parameter<std::string>("evader_termination_topic",
                                       "/vehicles/evader/termination"),
        rclcpp::QoS{1}.reliable().transient_local());
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
  [[nodiscard]] rclcpp::Subscription<msg::CrashState>::SharedPtr
  makeCrashSubscription(const std::string& topic, const std::string& role) {
    return create_subscription<msg::CrashState>(
        topic, rclcpp::QoS{1}.reliable().transient_local(),
        [this, role](const msg::CrashState::SharedPtr crash) {
          if (crash->crashed && !terminal_outcome_) {
            failMission("physical_collision_" + role);
          }
        });
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

  void requestHold() {
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
    command.reason = "evader_reached_goal";
    mission_command_pub_->publish(command);
    RCLCPP_INFO(get_logger(),
                "INTERCEPTOR_HOLD requested=true source=mission_command "
                "position=(%.3f,%.3f,%.3f) mission_epoch=%" PRIu64,
                hold_position_.x, hold_position_.y, hold_position_.z, mission_epoch_);
  }

  void requestDisarm(const std::string& detail = "intercepted") {
    msg::VehicleTermination termination;
    termination.stamp = now();
    termination.mission_epoch = mission_epoch_;
    termination.reason = msg::VehicleTermination::REASON_INTERCEPT;
    termination.detail = detail;
    ownship_termination_pub_->publish(termination);
    target_termination_pub_->publish(termination);
    termination_requested_ns_ = now().nanoseconds();
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
    requestDisarm("late_intercept_after_evader_goal");
  }

  void settleHold(const std::int64_t now_ns) {
    if (!hold_confirmation_ || !ownship_state_.has_value()) {
      failMission("interceptor_hold_not_initialized");
      return;
    }
    const InterceptorHoldUpdate hold = hold_confirmation_->update(*ownship_state_);
    if (hold.newly_confirmed) {
      RCLCPP_INFO(get_logger(),
                  "INTERCEPTOR_HOLD_CONFIRMED position_error_m=%.3f speed_mps=%.3f "
                  "mission_epoch=%" PRIu64,
                  hold.position_error_m, hold.speed_mps, mission_epoch_);
      finishMission();
      return;
    }
    if (hold_requested_ns_ > 0 && now_ns - hold_requested_ns_ > hold_timeout_ns_) {
      failMission("interceptor_hold_not_confirmed");
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
    msg::VehicleTermination termination;
    termination.stamp = now();
    termination.mission_epoch = mission_epoch_;
    termination.reason = msg::VehicleTermination::REASON_UNSPECIFIED;
    termination.detail = reason;
    ownship_termination_pub_->publish(termination);
    target_termination_pub_->publish(termination);
    result_reported_ = true;
    RCLCPP_ERROR(get_logger(),
                 "MISSION_RESULT success=false mission=intercept "
                 "outcome=system_failure reason='%s' mission_epoch=%" PRIu64,
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
          requestDisarm();
        } else {
          requestHold();
        }
      }
      return;
    }
    handleLateCapture(update);
    if (*terminal_outcome_ == InterceptMissionOutcome::kIntercepted) {
      if (!ownship_state_->armed && !target_state_->armed) {
        finishMission();
      } else if (termination_requested_ns_ > 0 &&
                 now_ns - termination_requested_ns_ > termination_timeout_ns_) {
        failMission("termination_not_confirmed");
      }
    } else if (late_capture_after_goal_) {
      if (!ownship_state_->armed && !target_state_->armed) {
        finishMission();
      } else if (termination_requested_ns_ > 0 &&
                 now_ns - termination_requested_ns_ > termination_timeout_ns_) {
        failMission("late_intercept_termination_not_confirmed");
      }
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
  std::string radar_simulator_node_fqn_;
  std::uint64_t mission_epoch_{1U};
  std::int64_t maximum_state_age_ns_{1'000'000'000LL};
  std::int64_t termination_timeout_ns_{3'000'000'000LL};
  std::int64_t hold_timeout_ns_{20'000'000'000LL};
  std::int64_t boundary_startup_timeout_ns_{10'000'000'000LL};
  std::int64_t termination_requested_ns_{0};
  std::int64_t hold_requested_ns_{0};
  std::int64_t boundary_check_started_ns_{0};
  std::int64_t last_boundary_check_ns_{0};
  bool mission_started_{false};
  bool result_reported_{false};
  bool late_capture_after_goal_{false};
  bool shutdown_on_terminal_outcome_{true};
  bool boundary_verified_{false};
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr ownship_state_sub_;
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr target_state_sub_;
  rclcpp::Subscription<msg::CrashState>::SharedPtr ownship_crash_sub_;
  rclcpp::Subscription<msg::CrashState>::SharedPtr target_crash_sub_;
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr target_objective_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ownship_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_start_pub_;
  rclcpp::Publisher<msg::VehicleTermination>::SharedPtr ownship_termination_pub_;
  rclcpp::Publisher<msg::VehicleTermination>::SharedPtr target_termination_pub_;
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
