#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/msg/intercept_mission_command.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

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

void requireCount(const std::vector<std::string>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    throw std::invalid_argument{parameter_name + " must contain " +
                                std::to_string(count) + " entries"};
  }
}

} // namespace

class InterceptMissionRefereeNode final : public rclcpp::Node {
public:
  InterceptMissionRefereeNode()
      : Node{"intercept_mission_referee_node"} {
    mission_epoch_ =
        static_cast<std::uint64_t>(declare_parameter<std::int64_t>("mission_epoch", 1));
    target_goal_ = Point3{
        declare_parameter<double>("evader_goal_x_m", 54.0),
        declare_parameter<double>("evader_goal_y_m", 378.0),
        declare_parameter<double>("evader_goal_z_m", 18.0),
    };
    capture_radius_m_ = declare_parameter<double>("capture_radius_m", 5.0);
    const InterceptMissionConfig mission_config{
        .capture_radius_m = capture_radius_m_,
        .evader_goal_radius_m = declare_parameter<double>("evader_goal_radius_m", 2.0),
    };
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
    destruction_settlement_timeout_ns_ = secondsToNanoseconds(
        declare_parameter<double>("destruction_settlement_timeout_s", 5.0));
    hold_timeout_ns_ = secondsToNanoseconds(
        declare_parameter<double>("interceptor_hold_timeout_s", 20.0));
    boundary_startup_timeout_ns_ = secondsToNanoseconds(
        declare_parameter<double>("ground_truth_boundary_startup_timeout_s", 10.0));
    mission_readiness_timeout_ns_ = secondsToNanoseconds(
        declare_parameter<double>("mission_readiness_timeout_s", 30.0));
    shutdown_on_terminal_outcome_ =
        declare_parameter<bool>("shutdown_on_terminal_outcome", true);

    const std::vector<std::string> ids = declare_parameter<std::vector<std::string>>(
        "interceptor_ids", {"interceptor_0", "interceptor_1", "interceptor_2"});
    configureInterceptors(ids);
    evaluator_ = std::make_unique<MultiInterceptMissionEvaluator>(
        target_goal_, interceptors_.size(), mission_config);
    previous_collision_states_.resize(interceptors_.size());
    configureEvader();
    publishTargetObjective();

    timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Intercept mission referee ready: epoch=%" PRIu64
                " interceptors=%zu target_goal=(%.2f,%.2f,%.2f) "
                "truth_boundary_topic='%s'",
                mission_epoch_, interceptors_.size(), target_goal_.x, target_goal_.y,
                target_goal_.z, target_state_topic_.c_str());
  }

private:
  struct HoldHorizon {
    Point3 position{};
    std::uint64_t sequence{0U};
    bool active{false};
  };

  struct InterceptorRuntime {
    std::string id;
    std::string radar_simulator_fqn;
    std::optional<TimedVehicleState> state;
    std::optional<HoldHorizon> hold_horizon;
    std::unique_ptr<InterceptStateAdjudicationLifecycle> adjudication;
    std::unique_ptr<InterceptorHoldConfirmation> hold_confirmation;
    bool world_ready{false};
    bool track_ready{false};
    bool destroyed{false};
    bool destruction_requested{false};
    bool disabled{false};
    std::int64_t hold_requested_ns{0};
    std::uint64_t hold_request_horizon_sequence{0U};
    rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr state_sub;
    rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr horizon_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr world_ready_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr track_ready_sub;
    rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr destroyed_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub;
    rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr destroyed_pub;
    rclcpp::Publisher<msg::InterceptMissionCommand>::SharedPtr command_pub;
  };

  [[nodiscard]] static std::int64_t secondsToNanoseconds(const double seconds) {
    if (!(seconds > 0.0) || !std::isfinite(seconds)) {
      throw std::invalid_argument{"mission timeout must be finite and positive"};
    }
    return static_cast<std::int64_t>(seconds * 1.0e9);
  }

  void configureInterceptors(const std::vector<std::string>& ids) {
    if (ids.empty()) {
      throw std::invalid_argument{"at least one interceptor is required"};
    }
    const auto defaultTopics = [&ids](const std::string& suffix) {
      std::vector<std::string> topics;
      topics.reserve(ids.size());
      for (const std::string& id : ids) {
        std::string topic{"/vehicles/"};
        topic.append(id);
        topic.append(suffix);
        topics.push_back(std::move(topic));
      }
      return topics;
    };
    const std::vector<std::string> state_topics =
        declare_parameter<std::vector<std::string>>("interceptor_state_topics",
                                                    defaultTopics("/state"));
    const std::vector<std::string> horizon_topics =
        declare_parameter<std::vector<std::string>>(
            "interceptor_execution_horizon_topics",
            defaultTopics("/mppi/execution_horizon"));
    const std::vector<std::string> world_topics =
        declare_parameter<std::vector<std::string>>(
            "interceptor_world_readiness_topics", defaultTopics("/mppi/world_ready"));
    const std::vector<std::string> track_topics =
        declare_parameter<std::vector<std::string>>(
            "target_track_readiness_topics", defaultTopics("/target_track_ready"));
    const std::vector<std::string> destroyed_topics =
        declare_parameter<std::vector<std::string>>(
            "interceptor_destroyed_topics", defaultTopics("/vehicle_destroyed"));
    const std::vector<std::string> start_topics =
        declare_parameter<std::vector<std::string>>("interceptor_start_topics",
                                                    defaultTopics("/mission_start"));
    const std::vector<std::string> command_topics =
        declare_parameter<std::vector<std::string>>(
            "interceptor_mission_command_topics", defaultTopics("/mission_command"));
    std::vector<std::string> default_radar_fqns;
    default_radar_fqns.reserve(ids.size());
    for (const std::string& id : ids) {
      std::string fqn{"/vehicles/"};
      fqn.append(id);
      fqn.append("/radar_simulator_node");
      default_radar_fqns.push_back(std::move(fqn));
    }
    const std::vector<std::string> radar_fqns =
        declare_parameter<std::vector<std::string>>("radar_simulator_node_fqns",
                                                    default_radar_fqns);
    for (const auto& [values, name] :
         std::vector<std::pair<const std::vector<std::string>*, std::string>>{
             {&state_topics, "interceptor_state_topics"},
             {&horizon_topics, "interceptor_execution_horizon_topics"},
             {&world_topics, "interceptor_world_readiness_topics"},
             {&track_topics, "target_track_readiness_topics"},
             {&destroyed_topics, "interceptor_destroyed_topics"},
             {&start_topics, "interceptor_start_topics"},
             {&command_topics, "interceptor_mission_command_topics"},
             {&radar_fqns, "radar_simulator_node_fqns"}}) {
      requireCount(*values, ids.size(), name);
    }

    interceptors_.resize(ids.size());
    const auto state_qos = rclcpp::QoS{10}.best_effort();
    const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
    for (std::size_t index = 0; index < ids.size(); ++index) {
      InterceptorRuntime& runtime = interceptors_[index];
      runtime.id = ids[index];
      runtime.radar_simulator_fqn = radar_fqns[index];
      runtime.adjudication =
          std::make_unique<InterceptStateAdjudicationLifecycle>(state_config_);
      runtime.state_sub = create_subscription<msg::VehicleNavigationState>(
          state_topics[index], state_qos,
          [this, index](const msg::VehicleNavigationState::SharedPtr state) {
            interceptors_[index].state = detail::vehicleState(*state);
          });
      runtime.horizon_sub = create_subscription<msg::MppiTrajectoryHorizon>(
          horizon_topics[index], rclcpp::QoS{10}.best_effort(),
          [this, index](const msg::MppiTrajectoryHorizon::SharedPtr horizon) {
            interceptors_[index].hold_horizon = HoldHorizon{
                .position = Point3{horizon->stationary_hold_position.x,
                                   horizon->stationary_hold_position.y,
                                   horizon->stationary_hold_position.z},
                .sequence = horizon->sequence,
                .active = horizon->stationary_position_hold &&
                          horizon->execution_mode ==
                              msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD,
            };
          });
      runtime.world_ready_sub = create_subscription<std_msgs::msg::Bool>(
          world_topics[index], latched_qos,
          [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
            interceptors_[index].world_ready = ready->data;
          });
      runtime.track_ready_sub = create_subscription<std_msgs::msg::Bool>(
          track_topics[index], latched_qos,
          [this, index](const std_msgs::msg::Bool::SharedPtr ready) {
            interceptors_[index].track_ready = ready->data;
          });
      runtime.destroyed_sub = create_subscription<msg::VehicleDestroyed>(
          destroyed_topics[index], latched_qos,
          [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
            onVehicleDestroyed(*destroyed, index);
          });
      runtime.start_pub =
          create_publisher<std_msgs::msg::Bool>(start_topics[index], latched_qos);
      runtime.destroyed_pub =
          create_publisher<msg::VehicleDestroyed>(destroyed_topics[index], latched_qos);
      runtime.command_pub = create_publisher<msg::InterceptMissionCommand>(
          command_topics[index], latched_qos);
    }
  }

  void configureEvader() {
    evader_id_ = declare_parameter<std::string>("evader_id", "evader");
    target_state_topic_ =
        declare_parameter<std::string>("evader_state_topic", "/vehicles/evader/state");
    target_destroyed_topic_ = declare_parameter<std::string>(
        "evader_destroyed_topic", "/vehicles/evader/vehicle_destroyed");
    const auto state_qos = rclcpp::QoS{10}.best_effort();
    const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
    target_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        target_state_topic_, state_qos,
        [this](const msg::VehicleNavigationState::SharedPtr state) {
          target_state_ = detail::vehicleState(*state);
        });
    target_world_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        declare_parameter<std::string>("evader_world_readiness_topic",
                                       "/vehicles/evader/mppi/world_ready"),
        latched_qos, [this](const std_msgs::msg::Bool::SharedPtr ready) {
          target_world_ready_ = ready->data;
        });
    target_destroyed_sub_ = create_subscription<msg::VehicleDestroyed>(
        target_destroyed_topic_, latched_qos,
        [this](const msg::VehicleDestroyed::SharedPtr destroyed) {
          onVehicleDestroyed(*destroyed, std::nullopt);
        });
    target_objective_pub_ = create_publisher<msg::NavigationObjective>(
        declare_parameter<std::string>("evader_objective_topic",
                                       "/vehicles/evader/navigation_objective"),
        latched_qos);
    target_start_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter<std::string>("evader_start_topic",
                                       "/vehicles/evader/mission_start"),
        latched_qos);
    target_destroyed_pub_ =
        create_publisher<msg::VehicleDestroyed>(target_destroyed_topic_, latched_qos);
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

  void onVehicleDestroyed(const msg::VehicleDestroyed& destroyed,
                          const std::optional<std::size_t> interceptor_index) {
    const std::uint8_t expected_role = interceptor_index.has_value()
                                           ? msg::VehicleDestroyed::ROLE_INTERCEPTOR
                                           : msg::VehicleDestroyed::ROLE_EVADER;
    const std::string& expected_id = interceptor_index.has_value()
                                         ? interceptors_[*interceptor_index].id
                                         : evader_id_;
    if (destroyed.vehicle_role != expected_role ||
        destroyed.vehicle_id != expected_id ||
        !detail::validDeathCause(destroyed.death_cause) ||
        (destroyed.mission_epoch != 0U && destroyed.mission_epoch != mission_epoch_)) {
      RCLCPP_ERROR(get_logger(),
                   "VEHICLE_DESTROYED referee_rejected=true expected_role=%s "
                   "expected_vehicle_id='%s' actual_role=%u actual_vehicle_id='%s' "
                   "cause=%u event_epoch=%" PRIu64 " mission_epoch=%" PRIu64,
                   detail::vehicleRoleName(expected_role), expected_id.c_str(),
                   static_cast<unsigned>(destroyed.vehicle_role),
                   destroyed.vehicle_id.c_str(),
                   static_cast<unsigned>(destroyed.death_cause),
                   destroyed.mission_epoch, mission_epoch_);
      return;
    }

    if (interceptor_index.has_value()) {
      InterceptorRuntime& interceptor = interceptors_[*interceptor_index];
      if (interceptor.destroyed) {
        return;
      }
      interceptor.destroyed = true;
      interceptor.destruction_requested = true;
    } else {
      if (target_destroyed_) {
        return;
      }
      target_destroyed_ = true;
      target_destruction_requested_ = true;
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

    if (!mission_started_ || terminal_outcome_.has_value()) {
      return;
    }
    if (!interceptor_index.has_value()) {
      beginTerminal(InterceptMissionOutcome::kEvaderCrashed,
                    "physical_collision_evader");
      return;
    }
    if (operationalInterceptorCount() == 0U) {
      beginTerminal(InterceptMissionOutcome::kNoInterceptorsRemaining,
                    "no_interceptors_remaining");
    }
  }

  [[nodiscard]] bool verifyGroundTruthBoundary(const std::int64_t now_ns) {
    if (last_boundary_check_ns_ > 0 &&
        now_ns - last_boundary_check_ns_ < 500'000'000LL) {
      return boundary_verified_;
    }
    last_boundary_check_ns_ = now_ns;
    std::unordered_set<std::string> allowed{get_fully_qualified_name()};
    for (const InterceptorRuntime& interceptor : interceptors_) {
      allowed.insert(interceptor.radar_simulator_fqn);
    }
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
      return boundary_verified_;
    }
    bool complete = observed.contains(get_fully_qualified_name());
    for (const InterceptorRuntime& interceptor : interceptors_) {
      complete = complete && observed.contains(interceptor.radar_simulator_fqn);
    }
    if (complete && !boundary_verified_) {
      boundary_verified_ = true;
      RCLCPP_INFO(get_logger(),
                  "RADAR_DATA_BOUNDARY verified=true truth_topic='%s' "
                  "allowed_radar_subscribers=%zu",
                  target_state_topic_.c_str(), interceptors_.size());
    }
    return boundary_verified_;
  }

  [[nodiscard]] bool missionReady() const {
    if (!target_state_ || !target_state_->navigation_ready || !target_world_ready_) {
      return false;
    }
    return !target_destroyed_ &&
           std::ranges::all_of(interceptors_, [](const InterceptorRuntime& runtime) {
             return !runtime.destroyed && runtime.state &&
                    runtime.state->navigation_ready && runtime.world_ready &&
                    runtime.track_ready;
           });
  }

  void publishMissionStart() {
    std_msgs::msg::Bool start;
    start.data = true;
    for (InterceptorRuntime& interceptor : interceptors_) {
      interceptor.start_pub->publish(start);
    }
    target_start_pub_->publish(start);
    mission_started_ = true;
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_MISSION state=running epoch=%" PRIu64
                " interceptor_count=%zu",
                mission_epoch_, interceptors_.size());
  }

  [[nodiscard]] std::size_t operationalInterceptorCount() const noexcept {
    return static_cast<std::size_t>(
        std::ranges::count_if(interceptors_, [](const InterceptorRuntime& runtime) {
          return !runtime.destroyed && !runtime.destruction_requested &&
                 !runtime.disabled;
        }));
  }

  [[nodiscard]] std::size_t survivingInterceptorCount() const noexcept {
    return static_cast<std::size_t>(
        std::ranges::count_if(interceptors_, [](const InterceptorRuntime& runtime) {
          return !runtime.destroyed && !runtime.destruction_requested;
        }));
  }

  void requestHold(const std::size_t index, const std::string& reason) {
    InterceptorRuntime& interceptor = interceptors_[index];
    if (interceptor.destroyed || interceptor.destruction_requested ||
        interceptor.hold_confirmation) {
      return;
    }
    if (!interceptor.state || !interceptor.state->position_valid) {
      failMission("interceptor_hold_position_unavailable:" + interceptor.id);
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

  void
  requestHoldsForSurvivors(const std::string& reason,
                           const std::optional<std::size_t> excluded = std::nullopt) {
    for (std::size_t index = 0; index < interceptors_.size(); ++index) {
      if (!excluded.has_value() || index != *excluded) {
        requestHold(index, reason);
      }
    }
  }

  [[nodiscard]] msg::VehicleDestroyed
  makeProximityDestruction(const TimedVehicleState& state, const std::uint8_t role,
                           const std::string& vehicle_id, const std::uint8_t cause,
                           const std::string& detail) const {
    msg::VehicleDestroyed destroyed;
    destroyed.stamp = now();
    destroyed.mission_epoch = mission_epoch_;
    destroyed.vehicle_role = role;
    destroyed.vehicle_id = vehicle_id;
    destroyed.death_cause = cause;
    destroyed.detail = detail;
    destroyed.event_position.x = state.position.x;
    destroyed.event_position.y = state.position.y;
    destroyed.event_position.z = state.position.z;
    destroyed.altitude_m = state.position.z;
    destroyed.speed_mps = detail::speed(state);
    return destroyed;
  }

  void requestCaptureDestruction(const std::size_t interceptor_index,
                                 const std::string& detail) {
    if (interceptor_index >= interceptors_.size()) {
      failMission("capturing_interceptor_index_out_of_range");
      return;
    }
    const std::optional<TimedVehicleState> target_state = target_state_;
    const std::optional<TimedVehicleState> interceptor_state =
        interceptors_[interceptor_index].state;
    if (target_destruction_requested_ || !target_state || !interceptor_state) {
      return;
    }
    InterceptorRuntime& interceptor = interceptors_[interceptor_index];
    interceptor.destruction_requested = true;
    target_destruction_requested_ = true;
    capturing_interceptor_index_ = interceptor_index;
    interceptor.destroyed_pub->publish(makeProximityDestruction(
        interceptor_state.value(), msg::VehicleDestroyed::ROLE_INTERCEPTOR,
        interceptor.id, msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT, detail));
    target_destroyed_pub_->publish(makeProximityDestruction(
        target_state.value(), msg::VehicleDestroyed::ROLE_EVADER, evader_id_,
        msg::VehicleDestroyed::CAUSE_PROXIMITY_INTERCEPT, detail));
    destruction_requested_ns_ = now().nanoseconds();
    RCLCPP_ERROR(get_logger(),
                 "PROXIMITY_INTERCEPT destruction_requested=true "
                 "interceptor_id='%s' separation_threshold_m=%.3f "
                 "mission_epoch=%" PRIu64,
                 interceptor.id.c_str(), capture_radius_m_, mission_epoch_);
  }

  void requestInterceptorCollision(const std::size_t first_index,
                                   const std::size_t second_index,
                                   const double separation_m) {
    InterceptorRuntime& first = interceptors_[first_index];
    InterceptorRuntime& second = interceptors_[second_index];
    if (!first.state || !second.state || first.destruction_requested ||
        second.destruction_requested) {
      return;
    }
    first.destruction_requested = true;
    second.destruction_requested = true;
    const std::string detail = "interceptor_collision:" + first.id + ":" + second.id;
    first.destroyed_pub->publish(makeProximityDestruction(
        *first.state, msg::VehicleDestroyed::ROLE_INTERCEPTOR, first.id,
        msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION, detail));
    second.destroyed_pub->publish(makeProximityDestruction(
        *second.state, msg::VehicleDestroyed::ROLE_INTERCEPTOR, second.id,
        msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION, detail));
    destruction_requested_ns_ = now().nanoseconds();
    RCLCPP_ERROR(get_logger(),
                 "INTERCEPTOR_PROXIMITY_COLLISION first='%s' second='%s' "
                 "separation_m=%.3f threshold_m=%.3f mission_epoch=%" PRIu64,
                 first.id.c_str(), second.id.c_str(), separation_m, capture_radius_m_,
                 mission_epoch_);
  }

  void beginTerminal(const InterceptMissionOutcome outcome, const std::string& reason) {
    if (terminal_outcome_.has_value()) {
      return;
    }
    terminal_outcome_ = outcome;
    if (outcome == InterceptMissionOutcome::kEvaderCrashed ||
        outcome == InterceptMissionOutcome::kNoInterceptorsRemaining) {
      destruction_requested_ns_ = now().nanoseconds();
    }
    if (outcome == InterceptMissionOutcome::kEvaderReachedGoal ||
        outcome == InterceptMissionOutcome::kEvaderCrashed) {
      requestHoldsForSurvivors(reason);
    }
    RCLCPP_INFO(get_logger(),
                "INTERCEPT_OUTCOME outcome=%s first_terminal_event=true reason='%s' "
                "live_interceptors=%zu epoch=%" PRIu64,
                interceptMissionOutcomeName(outcome), reason.c_str(),
                operationalInterceptorCount(), mission_epoch_);
  }

  [[nodiscard]] std::vector<TimedVehicleState>
  evaluatorStates(const bool include_disabled = false) const {
    std::vector<TimedVehicleState> states(interceptors_.size());
    for (std::size_t index = 0; index < interceptors_.size(); ++index) {
      const InterceptorRuntime& interceptor = interceptors_[index];
      if (interceptor.state) {
        states[index] = *interceptor.state;
      }
      if (interceptor.destroyed || interceptor.destruction_requested ||
          (interceptor.disabled && !include_disabled)) {
        states[index].armed = false;
        states[index].airborne = false;
      }
    }
    return states;
  }

  [[nodiscard]] bool updateStateAdjudication(const std::int64_t now_ns) {
    if (!target_state_) {
      return false;
    }
    const TimedVehicleState& target_state = target_state_.value();
    bool target_fresh = true;
    bool continuity_reset = false;
    for (std::size_t index = 0; index < interceptors_.size(); ++index) {
      InterceptorRuntime& interceptor = interceptors_[index];
      if (!interceptor.state || interceptor.destroyed ||
          interceptor.destruction_requested || interceptor.disabled) {
        continue;
      }
      const InterceptStateAdjudicationUpdate update = interceptor.adjudication->update(
          now_ns, interceptor.state.value(), target_state);
      continuity_reset = continuity_reset || update.newly_recovered;
      target_fresh = target_fresh && update.evader_fresh;
      if (update.status == InterceptStateAdjudicationStatus::kHealthy) {
        continue;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "INTERCEPT_ADJUDICATION vehicle_id='%s' state=%s "
          "interceptor_age_ms=%.1f evader_age_ms=%.1f degraded_duration_s=%.2f",
          interceptor.id.c_str(),
          update.status == InterceptStateAdjudicationStatus::kProlongedFailure
              ? "prolonged_failure"
              : "degraded",
          update.interceptor_age_s * 1000.0, update.evader_age_s * 1000.0,
          update.degraded_duration_s);
      if (update.newly_prolonged_failure && !update.evader_fresh) {
        system_failure_reason_ = "prolonged_stale_evader_state";
        requestHoldsForSurvivors(*system_failure_reason_);
        return false;
      }
      if (update.newly_prolonged_failure && !update.interceptor_fresh) {
        interceptor.disabled = true;
        requestHold(index, "prolonged_stale_interceptor_state");
      }
    }
    if (continuity_reset) {
      evaluator_->resetTemporalContinuity();
      std::fill(previous_collision_states_.begin(), previous_collision_states_.end(),
                std::nullopt);
    }
    return target_fresh;
  }

  void detectInterceptorCollisions() {
    for (std::size_t first = 0; first < interceptors_.size(); ++first) {
      const std::optional<TimedVehicleState> first_state = interceptors_[first].state;
      const TimedVehicleState first_value = first_state.value_or(TimedVehicleState{});
      if (!first_state || interceptors_[first].destroyed ||
          interceptors_[first].destruction_requested || !first_value.armed ||
          !first_value.airborne) {
        continue;
      }
      for (std::size_t second = first + 1; second < interceptors_.size(); ++second) {
        const std::optional<TimedVehicleState> second_state =
            interceptors_[second].state;
        const TimedVehicleState second_value =
            second_state.value_or(TimedVehicleState{});
        if (!second_state || interceptors_[second].destroyed ||
            interceptors_[second].destruction_requested || !second_value.armed ||
            !second_value.airborne) {
          continue;
        }
        const double separation = minimumSweptVehicleSeparation(
            first_value, second_value, previous_collision_states_[first],
            previous_collision_states_[second]);
        if (separation <= capture_radius_m_) {
          requestInterceptorCollision(first, second, separation);
          if (operationalInterceptorCount() == 0U) {
            beginTerminal(InterceptMissionOutcome::kNoInterceptorsRemaining,
                          "no_interceptors_remaining");
          }
        }
      }
    }
    for (std::size_t index = 0; index < interceptors_.size(); ++index) {
      previous_collision_states_[index] = interceptors_[index].state;
    }
  }

  [[nodiscard]] bool allRequestedDeathsSettled() const {
    for (const InterceptorRuntime& interceptor : interceptors_) {
      if (interceptor.destruction_requested &&
          (!interceptor.destroyed || !interceptor.state || interceptor.state->armed)) {
        return false;
      }
    }
    return !target_destruction_requested_ ||
           (target_destroyed_ && target_state_ && !target_state_->armed);
  }

  [[nodiscard]] bool allSurvivorsHeld(const std::int64_t now_ns) {
    for (InterceptorRuntime& interceptor : interceptors_) {
      if (interceptor.destroyed || interceptor.destruction_requested) {
        continue;
      }
      if (!interceptor.hold_confirmation || !interceptor.state) {
        return false;
      }
      std::optional<Point3> active_hold_position;
      if (interceptor.hold_horizon && interceptor.hold_horizon->active &&
          interceptor.hold_horizon->sequence >
              interceptor.hold_request_horizon_sequence) {
        active_hold_position = interceptor.hold_horizon->position;
      }
      const InterceptorHoldUpdate update = interceptor.hold_confirmation->update(
          *interceptor.state, active_hold_position);
      if (update.newly_confirmed) {
        RCLCPP_INFO(get_logger(),
                    "INTERCEPTOR_HOLD_CONFIRMED vehicle_id='%s' "
                    "position_error_m=%.3f speed_mps=%.3f mission_epoch=%" PRIu64,
                    interceptor.id.c_str(), update.position_error_m, update.speed_mps,
                    mission_epoch_);
      }
      if (!update.confirmed) {
        if (interceptor.hold_requested_ns > 0 &&
            now_ns - interceptor.hold_requested_ns > hold_timeout_ns_) {
          failMission("interceptor_hold_not_confirmed:" + interceptor.id);
        }
        return false;
      }
    }
    return true;
  }

  void settleTerminal(const std::int64_t now_ns) {
    if (system_failure_reason_.has_value()) {
      if (allSurvivorsHeld(now_ns)) {
        failMission(*system_failure_reason_);
      }
      return;
    }
    if (!terminal_outcome_.has_value()) {
      return;
    }
    const bool deaths_settled = allRequestedDeathsSettled();
    const bool holds_required =
        *terminal_outcome_ == InterceptMissionOutcome::kEvaderReachedGoal ||
        *terminal_outcome_ == InterceptMissionOutcome::kEvaderCrashed ||
        (*terminal_outcome_ == InterceptMissionOutcome::kIntercepted &&
         survivingInterceptorCount() > 0U);
    const bool holds_settled = !holds_required || allSurvivorsHeld(now_ns);
    if (deaths_settled && holds_settled) {
      finishMission();
      return;
    }
    if (!deaths_settled && destruction_requested_ns_ > 0 &&
        now_ns - destruction_requested_ns_ > destruction_settlement_timeout_ns_) {
      failMission("vehicle_destruction_not_confirmed");
    }
  }

  void handleMissionEvaluation(const std::int64_t now_ns) {
    if (!target_state_) {
      return;
    }
    const bool detect_late_capture =
        terminal_outcome_ == InterceptMissionOutcome::kEvaderReachedGoal;
    std::vector<TimedVehicleState> states = evaluatorStates(detect_late_capture);
    const MultiInterceptMissionUpdate update =
        evaluator_->update(states, target_state_.value());
    if (!terminal_outcome_.has_value() && update.newly_terminal) {
      if (update.outcome == InterceptMissionOutcome::kIntercepted) {
        if (!update.capturing_interceptor_index) {
          failMission("intercept_outcome_without_capturing_interceptor");
          return;
        }
        const std::size_t interceptor_index =
            update.capturing_interceptor_index.value();
        beginTerminal(update.outcome, "intercepted");
        requestCaptureDestruction(interceptor_index, "intercepted");
        requestHoldsForSurvivors("evader_intercepted", interceptor_index);
      } else {
        beginTerminal(update.outcome, "evader_reached_goal");
      }
      return;
    }
    if (terminal_outcome_ == InterceptMissionOutcome::kEvaderReachedGoal &&
        update.newly_captured && !late_capture_after_goal_) {
      if (!update.capturing_interceptor_index) {
        failMission("late_capture_without_capturing_interceptor");
        return;
      }
      late_capture_after_goal_ = true;
      const std::size_t interceptor_index = update.capturing_interceptor_index.value();
      RCLCPP_INFO(get_logger(),
                  "INTERCEPT_LATE_CAPTURE outcome_preserved=evader_reached_goal "
                  "interceptor_id='%s' separation_m=%.3f mission_epoch=%" PRIu64,
                  interceptors_[interceptor_index].id.c_str(), update.separation_m,
                  mission_epoch_);
      RCLCPP_INFO(get_logger(),
                  "INTERCEPTOR_HOLD_ABORTED vehicle_id='%s' reason=late_capture "
                  "mission_epoch=%" PRIu64,
                  interceptors_[interceptor_index].id.c_str(), mission_epoch_);
      requestCaptureDestruction(interceptor_index, "late_intercept_after_evader_goal");
    }
    if (!terminal_outcome_.has_value()) {
      detectInterceptorCollisions();
    }
    (void)now_ns;
  }

  void finishMission() {
    if (result_reported_ || !terminal_outcome_.has_value()) {
      return;
    }
    result_reported_ = true;
    const bool intercepted =
        *terminal_outcome_ == InterceptMissionOutcome::kIntercepted;
    const bool technical_success =
        intercepted ||
        *terminal_outcome_ == InterceptMissionOutcome::kEvaderReachedGoal;
    const std::string capturing_id =
        capturing_interceptor_index_.has_value()
            ? interceptors_[*capturing_interceptor_index_].id
            : "none";
    if (technical_success) {
      RCLCPP_INFO(get_logger(),
                  "MISSION_RESULT success=true mission=intercept outcome=%s "
                  "intercept_success=%s capturing_interceptor_id='%s' "
                  "mission_epoch=%" PRIu64,
                  interceptMissionOutcomeName(*terminal_outcome_),
                  intercepted ? "true" : "false", capturing_id.c_str(), mission_epoch_);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "MISSION_RESULT success=false mission=intercept outcome=%s "
                   "intercept_success=false capturing_interceptor_id='%s' "
                   "mission_epoch=%" PRIu64,
                   interceptMissionOutcomeName(*terminal_outcome_),
                   capturing_id.c_str(), mission_epoch_);
    }
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

  void tick() {
    if (result_reported_) {
      return;
    }
    const std::int64_t now_ns = now().nanoseconds();
    if (boundary_check_started_ns_ <= 0) {
      boundary_check_started_ns_ = now_ns;
    }
    if (!verifyGroundTruthBoundary(now_ns)) {
      if (!result_reported_ &&
          now_ns - boundary_check_started_ns_ > boundary_startup_timeout_ns_) {
        failMission("ground_truth_boundary_not_ready");
      }
      return;
    }
    if (!target_state_ ||
        std::ranges::any_of(interceptors_, [](const InterceptorRuntime& runtime) {
          return !runtime.state.has_value();
        })) {
      return;
    }
    if (!mission_started_) {
      const bool vehicles_ready =
          target_state_->navigation_ready &&
          std::ranges::all_of(interceptors_, [](const InterceptorRuntime& runtime) {
            return runtime.state->navigation_ready;
          });
      if (vehicles_ready && mission_readiness_started_ns_ <= 0) {
        mission_readiness_started_ns_ = now_ns;
      }
      if (missionReady()) {
        publishMissionStart();
      } else if (vehicles_ready && now_ns - mission_readiness_started_ns_ >
                                       mission_readiness_timeout_ns_) {
        failMission("mission_readiness_timeout");
      }
      return;
    }
    if (terminal_outcome_.has_value() || system_failure_reason_.has_value()) {
      if (terminal_outcome_ == InterceptMissionOutcome::kEvaderReachedGoal &&
          !target_destroyed_) {
        handleMissionEvaluation(now_ns);
      }
      settleTerminal(now_ns);
      return;
    }
    if (!updateStateAdjudication(now_ns)) {
      return;
    }
    if (operationalInterceptorCount() == 0U) {
      if (survivingInterceptorCount() == 0U) {
        beginTerminal(InterceptMissionOutcome::kNoInterceptorsRemaining,
                      "no_interceptors_remaining");
      } else {
        system_failure_reason_ = "no_operational_interceptors";
        requestHoldsForSurvivors(*system_failure_reason_);
      }
      return;
    }
    handleMissionEvaluation(now_ns);
  }

  std::vector<InterceptorRuntime> interceptors_;
  std::vector<std::optional<TimedVehicleState>> previous_collision_states_;
  std::unique_ptr<MultiInterceptMissionEvaluator> evaluator_;
  InterceptStateAdjudicationConfig state_config_{};
  InterceptorHoldConfig hold_config_{};
  Point3 target_goal_{};
  std::optional<TimedVehicleState> target_state_;
  std::optional<InterceptMissionOutcome> terminal_outcome_;
  std::optional<std::size_t> capturing_interceptor_index_;
  std::optional<std::string> system_failure_reason_;
  std::string evader_id_;
  std::string target_state_topic_;
  std::string target_destroyed_topic_;
  double capture_radius_m_{5.0};
  std::uint64_t mission_epoch_{1U};
  std::int64_t destruction_settlement_timeout_ns_{5'000'000'000LL};
  std::int64_t hold_timeout_ns_{20'000'000'000LL};
  std::int64_t boundary_startup_timeout_ns_{10'000'000'000LL};
  std::int64_t mission_readiness_timeout_ns_{30'000'000'000LL};
  std::int64_t destruction_requested_ns_{0};
  std::int64_t boundary_check_started_ns_{0};
  std::int64_t last_boundary_check_ns_{0};
  std::int64_t mission_readiness_started_ns_{0};
  bool mission_started_{false};
  bool result_reported_{false};
  bool late_capture_after_goal_{false};
  bool target_destroyed_{false};
  bool target_destruction_requested_{false};
  bool target_world_ready_{false};
  bool shutdown_on_terminal_outcome_{true};
  bool boundary_verified_{false};
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr target_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_world_ready_sub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr target_destroyed_sub_;
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr target_objective_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_start_pub_;
  rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr target_destroyed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::InterceptMissionRefereeNode>());
  rclcpp::shutdown();
  return 0;
}
