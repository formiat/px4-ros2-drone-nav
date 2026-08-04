#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/msg/crash_state.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/msg/vehicle_termination.hpp"

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

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] TimedVehicleState
convertState(const msg::VehicleNavigationState& message) noexcept {
  return TimedVehicleState{
      .position = Point3{message.position.x, message.position.y, message.position.z},
      .velocity = Vec3{message.velocity.x, message.velocity.y, message.velocity.z},
      .stamp_ns = timeNanoseconds(message.stamp),
      .position_valid = message.position_valid,
      .velocity_valid = message.velocity_valid,
      .armed = message.armed,
      .airborne = message.airborne,
      .navigation_ready = message.navigation_ready,
  };
}

} // namespace

class InterceptMissionNode final : public rclcpp::Node {
public:
  InterceptMissionNode()
      : Node{"intercept_mission_node"} {
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 1));
    const Point3 evader_goal{
        declare_parameter<double>("evader_goal_x_m", 216.0),
        declare_parameter<double>("evader_goal_y_m", 378.0),
        declare_parameter<double>("evader_goal_z_m", 18.0),
    };
    evaluator_ = std::make_unique<InterceptMissionEvaluator>(
        evader_goal,
        InterceptMissionConfig{
            .capture_radius_m = declare_parameter<double>("capture_radius_m", 5.0),
            .evader_goal_radius_m =
                declare_parameter<double>("evader_goal_radius_m", 2.0),
            .evader_goal_stop_speed_mps =
                declare_parameter<double>("evader_goal_stop_speed_mps", 0.8),
            .evader_goal_hold_s = declare_parameter<double>("evader_goal_hold_s", 2.0),
        });
    evader_goal_ = evader_goal;
    maximum_state_age_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("maximum_state_age_s", 1.0) * 1.0e9);
    termination_timeout_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("termination_timeout_s", 3.0) * 1.0e9);
    interceptor_hold_config_.position_tolerance_m =
        declare_parameter<double>("interceptor_hold_position_tolerance_m", 2.0);
    interceptor_hold_config_.maximum_speed_mps =
        declare_parameter<double>("interceptor_hold_maximum_speed_mps", 0.8);
    interceptor_hold_config_.confirmation_duration_s =
        declare_parameter<double>("interceptor_hold_confirmation_duration_s", 1.0);
    interceptor_hold_timeout_ns_ = static_cast<std::int64_t>(
        declare_parameter<double>("interceptor_hold_timeout_s", 20.0) * 1.0e9);
    if (!(interceptor_hold_config_.position_tolerance_m > 0.0) ||
        !(interceptor_hold_config_.maximum_speed_mps >= 0.0) ||
        !(interceptor_hold_config_.confirmation_duration_s >= 0.0) ||
        !(interceptor_hold_timeout_ns_ > 0)) {
      throw std::invalid_argument{"invalid interceptor hold parameters"};
    }
    shutdown_on_terminal_outcome_ =
        declare_parameter<bool>("shutdown_on_terminal_outcome", true);

    const auto state_qos = rclcpp::QoS{10}.best_effort();
    interceptor_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        declare_parameter<std::string>("interceptor_state_topic",
                                       "/vehicles/interceptor/state"),
        state_qos, [this](const msg::VehicleNavigationState::SharedPtr state) {
          interceptor_state_ = convertState(*state);
        });
    evader_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        declare_parameter<std::string>("evader_state_topic", "/vehicles/evader/state"),
        state_qos, [this](const msg::VehicleNavigationState::SharedPtr state) {
          evader_state_ = convertState(*state);
        });
    interceptor_crash_sub_ = makeCrashSubscription(
        declare_parameter<std::string>("interceptor_crash_topic",
                                       "/vehicles/interceptor/crash_state"),
        "interceptor");
    evader_crash_sub_ =
        makeCrashSubscription(declare_parameter<std::string>(
                                  "evader_crash_topic", "/vehicles/evader/crash_state"),
                              "evader");

    interceptor_objective_pub_ = create_publisher<msg::NavigationObjective>(
        declare_parameter<std::string>("interceptor_objective_topic",
                                       "/vehicles/interceptor/navigation_objective"),
        rclcpp::QoS{1}.reliable().transient_local());
    evader_objective_pub_ = create_publisher<msg::NavigationObjective>(
        declare_parameter<std::string>("evader_objective_topic",
                                       "/vehicles/evader/navigation_objective"),
        rclcpp::QoS{1}.reliable().transient_local());
    interceptor_start_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter<std::string>("interceptor_start_topic",
                                       "/vehicles/interceptor/mission_start"),
        rclcpp::QoS{1}.reliable().transient_local());
    evader_start_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter<std::string>("evader_start_topic",
                                       "/vehicles/evader/mission_start"),
        rclcpp::QoS{1}.reliable().transient_local());
    interceptor_termination_pub_ = create_publisher<msg::VehicleTermination>(
        declare_parameter<std::string>("interceptor_termination_topic",
                                       "/vehicles/interceptor/termination"),
        rclcpp::QoS{1}.reliable().transient_local());
    evader_termination_pub_ = create_publisher<msg::VehicleTermination>(
        declare_parameter<std::string>("evader_termination_topic",
                                       "/vehicles/evader/termination"),
        rclcpp::QoS{1}.reliable().transient_local());

    publishEvaderObjective();
    timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Intercept mission ready: epoch=%" PRIu64
                " evader_goal=(%.2f,%.2f,%.2f) shutdown_on_terminal_outcome=%s "
                "hold[position_tolerance_m=%.2f maximum_speed_mps=%.2f "
                "confirmation_s=%.2f timeout_s=%.2f]",
                mission_epoch_, evader_goal_.x, evader_goal_.y, evader_goal_.z,
                shutdown_on_terminal_outcome_ ? "true" : "false",
                interceptor_hold_config_.position_tolerance_m,
                interceptor_hold_config_.maximum_speed_mps,
                interceptor_hold_config_.confirmation_duration_s,
                static_cast<double>(interceptor_hold_timeout_ns_) * 1.0e-9);
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

  void publishEvaderObjective() {
    msg::NavigationObjective objective;
    objective.stamp = now();
    objective.mission_epoch = mission_epoch_;
    objective.sample_sequence = 1U;
    objective.position.x = evader_goal_.x;
    objective.position.y = evader_goal_.y;
    objective.position.z = evader_goal_.z;
    objective.terminal_policy = msg::NavigationObjective::TERMINAL_POLICY_POSITION_HOLD;
    evader_objective_pub_->publish(objective);
  }

  void publishInterceptorObjective() {
    if (!evader_state_ || !evader_state_->position_valid) {
      return;
    }
    msg::NavigationObjective objective;
    objective.stamp = now();
    objective.mission_epoch = mission_epoch_;
    objective.sample_sequence = ++interceptor_objective_sequence_;
    objective.position.x = evader_state_->position.x;
    objective.position.y = evader_state_->position.y;
    objective.position.z = evader_state_->position.z;
    objective.terminal_policy =
        msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING;
    interceptor_objective_pub_->publish(objective);
  }

  void requestInterceptorHold() {
    if (!interceptor_state_ || !interceptor_state_->position_valid) {
      failMission("interceptor_hold_position_unavailable");
      return;
    }
    interceptor_hold_position_ = interceptor_state_->position;
    interceptor_hold_confirmation_ = std::make_unique<InterceptorHoldConfirmation>(
        interceptor_hold_position_, interceptor_hold_config_);
    interceptor_hold_requested_ns_ = now().nanoseconds();

    msg::NavigationObjective objective;
    objective.stamp = now();
    objective.mission_epoch = mission_epoch_;
    objective.sample_sequence = ++interceptor_objective_sequence_;
    objective.position.x = interceptor_hold_position_.x;
    objective.position.y = interceptor_hold_position_.y;
    objective.position.z = interceptor_hold_position_.z;
    objective.terminal_policy = msg::NavigationObjective::TERMINAL_POLICY_POSITION_HOLD;
    interceptor_objective_pub_->publish(objective);
    RCLCPP_INFO(get_logger(),
                "INTERCEPTOR_HOLD requested=true position=(%.3f,%.3f,%.3f) "
                "mission_epoch=%" PRIu64,
                interceptor_hold_position_.x, interceptor_hold_position_.y,
                interceptor_hold_position_.z, mission_epoch_);
  }

  void publishMissionStart() {
    std_msgs::msg::Bool start;
    start.data = true;
    interceptor_start_pub_->publish(start);
    evader_start_pub_->publish(start);
    mission_started_ = true;
    RCLCPP_INFO(get_logger(), "INTERCEPT_MISSION state=running epoch=%" PRIu64,
                mission_epoch_);
  }

  void requestInterceptDisarm(const std::string& detail = "intercepted") {
    msg::VehicleTermination termination;
    termination.stamp = now();
    termination.mission_epoch = mission_epoch_;
    termination.reason = msg::VehicleTermination::REASON_INTERCEPT;
    termination.detail = detail;
    interceptor_termination_pub_->publish(termination);
    evader_termination_pub_->publish(termination);
    termination_requested_ns_ = now().nanoseconds();
  }

  void handleLateCapture(const InterceptMissionUpdate& update) {
    if (!terminal_outcome_ ||
        *terminal_outcome_ != InterceptMissionOutcome::kEvaderReachedGoal ||
        !update.newly_captured || late_capture_after_evader_goal_) {
      return;
    }
    late_capture_after_evader_goal_ = true;
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_LATE_CAPTURE outcome_preserved=evader_reached_goal "
                "separation_m=%.3f mission_epoch=%" PRIu64,
                update.separation_m, mission_epoch_);
    RCLCPP_INFO(get_logger(),
                "INTERCEPTOR_HOLD_ABORTED reason=late_capture mission_epoch=%" PRIu64,
                mission_epoch_);
    requestInterceptDisarm("late_intercept_after_evader_goal");
  }

  void settleInterceptorHold(const std::int64_t now_ns) {
    if (!interceptor_hold_confirmation_ || !interceptor_state_) {
      failMission("interceptor_hold_not_initialized");
      return;
    }
    const InterceptorHoldUpdate hold =
        interceptor_hold_confirmation_->update(*interceptor_state_);
    if (hold.newly_confirmed) {
      RCLCPP_INFO(get_logger(),
                  "INTERCEPTOR_HOLD_CONFIRMED position_error_m=%.3f speed_mps=%.3f "
                  "mission_epoch=%" PRIu64,
                  hold.position_error_m, hold.speed_mps, mission_epoch_);
      finishMission();
      return;
    }
    if (interceptor_hold_requested_ns_ > 0 &&
        now_ns - interceptor_hold_requested_ns_ > interceptor_hold_timeout_ns_) {
      failMission("interceptor_hold_not_confirmed");
    }
  }

  void finishMission() {
    if (result_reported_ || !terminal_outcome_) {
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
    interceptor_termination_pub_->publish(termination);
    evader_termination_pub_->publish(termination);
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
    if (!interceptor_state_ || !evader_state_) {
      return;
    }
    if (mission_started_ && (!stateFresh(*interceptor_state_, now_ns) ||
                             !stateFresh(*evader_state_, now_ns))) {
      failMission("stale_vehicle_state");
      return;
    }
    if (!terminal_outcome_) {
      publishInterceptorObjective();
    }
    if (!mission_started_) {
      if (interceptor_state_->navigation_ready && evader_state_->navigation_ready) {
        publishMissionStart();
      }
      return;
    }
    const InterceptMissionUpdate update =
        evaluator_->update(*interceptor_state_, *evader_state_);
    if (!terminal_outcome_) {
      if (update.newly_terminal) {
        terminal_outcome_ = update.outcome;
        RCLCPP_INFO(get_logger(),
                    "INTERCEPT_OUTCOME outcome=%s first_terminal_event=true "
                    "separation_m=%.3f epoch=%" PRIu64,
                    interceptMissionOutcomeName(update.outcome), update.separation_m,
                    mission_epoch_);
        if (update.outcome == InterceptMissionOutcome::kIntercepted) {
          requestInterceptDisarm();
        } else {
          requestInterceptorHold();
        }
      }
      return;
    }
    handleLateCapture(update);
    if (*terminal_outcome_ == InterceptMissionOutcome::kIntercepted) {
      if (!interceptor_state_->armed && !evader_state_->armed) {
        finishMission();
      } else if (termination_requested_ns_ > 0 &&
                 now_ns - termination_requested_ns_ > termination_timeout_ns_) {
        failMission("termination_not_confirmed");
      }
    } else if (late_capture_after_evader_goal_) {
      if (!interceptor_state_->armed && !evader_state_->armed) {
        finishMission();
      } else if (termination_requested_ns_ > 0 &&
                 now_ns - termination_requested_ns_ > termination_timeout_ns_) {
        failMission("late_intercept_termination_not_confirmed");
      }
    } else {
      settleInterceptorHold(now_ns);
    }
  }

  std::unique_ptr<InterceptMissionEvaluator> evaluator_;
  Point3 evader_goal_{};
  std::optional<TimedVehicleState> interceptor_state_;
  std::optional<TimedVehicleState> evader_state_;
  std::optional<InterceptMissionOutcome> terminal_outcome_;
  std::unique_ptr<InterceptorHoldConfirmation> interceptor_hold_confirmation_;
  InterceptorHoldConfig interceptor_hold_config_{};
  Point3 interceptor_hold_position_{};
  std::uint64_t mission_epoch_{1U};
  std::uint64_t interceptor_objective_sequence_{0U};
  std::int64_t maximum_state_age_ns_{1'000'000'000LL};
  std::int64_t termination_timeout_ns_{3'000'000'000LL};
  std::int64_t termination_requested_ns_{0};
  std::int64_t interceptor_hold_requested_ns_{0};
  std::int64_t interceptor_hold_timeout_ns_{20'000'000'000LL};
  bool mission_started_{false};
  bool result_reported_{false};
  bool late_capture_after_evader_goal_{false};
  bool shutdown_on_terminal_outcome_{true};

  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr interceptor_state_sub_;
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr evader_state_sub_;
  rclcpp::Subscription<msg::CrashState>::SharedPtr interceptor_crash_sub_;
  rclcpp::Subscription<msg::CrashState>::SharedPtr evader_crash_sub_;
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr interceptor_objective_pub_;
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr evader_objective_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr interceptor_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr evader_start_pub_;
  rclcpp::Publisher<msg::VehicleTermination>::SharedPtr interceptor_termination_pub_;
  rclcpp::Publisher<msg::VehicleTermination>::SharedPtr evader_termination_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::InterceptMissionNode>());
  rclcpp::shutdown();
  return 0;
}
