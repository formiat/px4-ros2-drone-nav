#include "drone_city_nav/target_assignment_coordinator_node.hpp"

#include "drone_city_nav/msg/intercept_target_status.hpp"
#include "drone_city_nav/msg/target_assignment.hpp"
#include "drone_city_nav/msg/target_track.hpp"
#include "drone_city_nav/msg/target_track_array.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/target_assignment.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

void requireCount(const std::vector<std::string>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    std::string message = parameter_name;
    message.append(" must contain ");
    message.append(std::to_string(count));
    message.append(" entries");
    throw std::invalid_argument{message};
  }
}

[[nodiscard]] std::vector<std::string>
vehicleTopics(const std::vector<std::string>& ids, const std::string& suffix) {
  std::vector<std::string> topics;
  topics.reserve(ids.size());
  for (const std::string& id : ids) {
    std::string topic{"/vehicles/"};
    topic.append(id);
    topic.append(suffix);
    topics.push_back(std::move(topic));
  }
  return topics;
}

[[nodiscard]] std::uint8_t
assignmentReasonMessage(const TargetAssignmentReason reason) noexcept {
  switch (reason) {
    case TargetAssignmentReason::kInitial:
      return msg::TargetAssignment::REASON_INITIAL;
    case TargetAssignmentReason::kTargetSetChanged:
      return msg::TargetAssignment::REASON_TARGET_SET_CHANGED;
    case TargetAssignmentReason::kCostImprovement:
      return msg::TargetAssignment::REASON_COST_IMPROVEMENT;
    case TargetAssignmentReason::kRefresh:
      return msg::TargetAssignment::REASON_REFRESH;
  }
  return msg::TargetAssignment::REASON_REFRESH;
}

} // namespace

class TargetAssignmentCoordinatorNode final : public rclcpp::Node {
public:
  explicit TargetAssignmentCoordinatorNode(const rclcpp::NodeOptions& options)
      : Node{"target_assignment_coordinator_node", options},
        assignment_{TargetAssignmentConfig{
            .interceptor_speed_mps =
                declare_parameter<double>("interceptor_speed_mps", 20.0),
            .maximum_track_age_s =
                declare_parameter<double>("maximum_track_age_s", 3.5),
            .switch_penalty_s = declare_parameter<double>("switch_penalty_s", 2.0),
            .minimum_switch_improvement_s =
                declare_parameter<double>("minimum_switch_improvement_s", 1.0),
            .minimum_switch_improvement_ratio =
                declare_parameter<double>("minimum_switch_improvement_ratio", 0.10),
            .minimum_assignment_hold_s =
                declare_parameter<double>("minimum_assignment_hold_s", 1.0),
            .switch_confirmation_s =
                declare_parameter<double>("switch_confirmation_s", 0.5),
            .no_intercept_solution_penalty_s =
                declare_parameter<double>("no_intercept_solution_penalty_s", 30.0),
        }} {
    const std::int64_t mission_epoch =
        declare_parameter<std::int64_t>("mission_epoch", 1);
    if (mission_epoch <= 0) {
      throw std::invalid_argument{"mission_epoch must be positive"};
    }
    mission_epoch_ = static_cast<std::uint64_t>(mission_epoch);
    const std::vector<std::string> ids = declare_parameter<std::vector<std::string>>(
        "interceptor_ids", {"interceptor_0"});
    if (ids.empty() ||
        std::ranges::any_of(ids, [](const std::string& id) { return id.empty(); })) {
      throw std::invalid_argument{"interceptor ids must be non-empty"};
    }
    if (std::unordered_set<std::string>{ids.begin(), ids.end()}.size() != ids.size()) {
      throw std::invalid_argument{"interceptor ids must be unique"};
    }
    const std::vector<std::int64_t> expected_detection_ids =
        declare_parameter<std::vector<std::int64_t>>("target_detection_ids", {1});
    if (expected_detection_ids.empty()) {
      throw std::invalid_argument{"target detection ids must be non-empty"};
    }
    for (const std::int64_t id : expected_detection_ids) {
      if (id <= 0 ||
          !expected_detection_ids_.insert(static_cast<std::uint64_t>(id)).second) {
        throw std::invalid_argument{"target detection ids must be positive and unique"};
      }
    }

    const std::vector<std::string> state_topics =
        declare_parameter<std::vector<std::string>>("interceptor_state_topics",
                                                    vehicleTopics(ids, "/state"));
    const std::vector<std::string> track_array_topics =
        declare_parameter<std::vector<std::string>>(
            "target_track_array_topics", vehicleTopics(ids, "/target_tracks"));
    const std::vector<std::string> selected_track_topics =
        declare_parameter<std::vector<std::string>>(
            "selected_target_track_topics", vehicleTopics(ids, "/target_track"));
    const std::vector<std::string> assignment_topics =
        declare_parameter<std::vector<std::string>>(
            "target_assignment_topics", vehicleTopics(ids, "/target_assignment"));
    const std::vector<std::string> readiness_topics =
        declare_parameter<std::vector<std::string>>(
            "target_track_readiness_topics", vehicleTopics(ids, "/target_track_ready"));
    const std::vector<std::string> destroyed_topics =
        declare_parameter<std::vector<std::string>>(
            "interceptor_destroyed_topics", vehicleTopics(ids, "/vehicle_destroyed"));
    for (const auto& [values, name] :
         std::vector<std::pair<const std::vector<std::string>*, std::string>>{
             {&state_topics, "interceptor_state_topics"},
             {&track_array_topics, "target_track_array_topics"},
             {&selected_track_topics, "selected_target_track_topics"},
             {&assignment_topics, "target_assignment_topics"},
             {&readiness_topics, "target_track_readiness_topics"},
             {&destroyed_topics, "interceptor_destroyed_topics"}}) {
      requireCount(*values, ids.size(), name);
    }

    const auto state_qos = rclcpp::QoS{10}.best_effort();
    runtimes_.resize(ids.size());
    for (std::size_t index = 0U; index < ids.size(); ++index) {
      Runtime& runtime = runtimes_[index];
      runtime.id = ids[index];
      runtime.state_sub = create_subscription<msg::VehicleNavigationState>(
          state_topics[index], state_qos,
          [this, index](const msg::VehicleNavigationState::SharedPtr state) {
            runtimes_[index].ownship = detail::vehicleState(*state);
          });
      runtime.track_array_sub = create_subscription<msg::TargetTrackArray>(
          track_array_topics[index], rclcpp::QoS{1}.reliable().transient_local(),
          [this, index](const msg::TargetTrackArray::SharedPtr tracks) {
            runtimes_[index].tracks = *tracks;
          });
      runtime.destroyed_sub = create_subscription<msg::VehicleDestroyed>(
          destroyed_topics[index], rclcpp::QoS{10}.reliable().transient_local(),
          [this, index](const msg::VehicleDestroyed::SharedPtr destroyed) {
            onInterceptorDestroyed(index, *destroyed);
          });
      runtime.selected_track_pub = create_publisher<msg::TargetTrack>(
          selected_track_topics[index], rclcpp::QoS{1}.reliable().transient_local());
      runtime.assignment_pub = create_publisher<msg::TargetAssignment>(
          assignment_topics[index], rclcpp::QoS{1}.reliable().transient_local());
      runtime.readiness_pub = create_publisher<std_msgs::msg::Bool>(
          readiness_topics[index], rclcpp::QoS{1}.reliable().transient_local());
      publishReadiness(runtime, false);
    }
    target_status_sub_ = create_subscription<msg::InterceptTargetStatus>(
        declare_parameter<std::string>("target_status_topic",
                                       "/intercept/target_status"),
        rclcpp::QoS{100}.reliable().transient_local(),
        [this](const msg::InterceptTargetStatus::SharedPtr status) {
          onTargetStatus(*status);
        });
    timer_ = create_wall_timer(std::chrono::milliseconds{50}, [this] { tick(); });
    RCLCPP_INFO(get_logger(),
                "Target assignment coordinator ready: interceptors=%zu targets=%zu",
                runtimes_.size(), expected_detection_ids_.size());
  }

private:
  struct Runtime {
    std::string id;
    std::optional<TimedVehicleState> ownship;
    std::optional<msg::TargetTrackArray> tracks;
    std::uint64_t last_published_scan_sequence{0U};
    std::uint64_t assigned_detection_id{0U};
    bool ready{false};
    bool readiness_published{false};
    bool active{true};
    rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr state_sub;
    rclcpp::Subscription<msg::TargetTrackArray>::SharedPtr track_array_sub;
    rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr destroyed_sub;
    rclcpp::Publisher<msg::TargetTrack>::SharedPtr selected_track_pub;
    rclcpp::Publisher<msg::TargetAssignment>::SharedPtr assignment_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr readiness_pub;
  };

  void publishReadiness(Runtime& runtime, const bool ready) {
    if (runtime.ready == ready && runtime.readiness_published) {
      return;
    }
    runtime.ready = ready;
    runtime.readiness_published = true;
    std_msgs::msg::Bool message;
    message.data = ready;
    runtime.readiness_pub->publish(message);
    RCLCPP_INFO(get_logger(), "TARGET_ASSIGNMENT_READY interceptor_id='%s' ready=%s",
                runtime.id.c_str(), ready ? "true" : "false");
  }

  void onTargetStatus(const msg::InterceptTargetStatus& status) {
    if (status.mission_epoch != mission_epoch_ || status.target_detection_id == 0U ||
        status.status > msg::InterceptTargetStatus::STATUS_DESTROYED) {
      return;
    }
    if (status.status == msg::InterceptTargetStatus::STATUS_ACTIVE) {
      inactive_detection_ids_.erase(status.target_detection_id);
    } else {
      inactive_detection_ids_.insert(status.target_detection_id);
    }
    if (status.status == msg::InterceptTargetStatus::STATUS_INTERCEPTED &&
        !status.capturing_interceptor_id.empty()) {
      deactivateInterceptor(status.capturing_interceptor_id, "target_intercepted");
    }
  }

  void onInterceptorDestroyed(const std::size_t index,
                              const msg::VehicleDestroyed& destroyed) {
    if (destroyed.mission_epoch != mission_epoch_ ||
        destroyed.vehicle_role != msg::VehicleDestroyed::ROLE_INTERCEPTOR ||
        destroyed.vehicle_id != runtimes_[index].id) {
      return;
    }
    deactivateInterceptor(destroyed.vehicle_id, "vehicle_destroyed");
  }

  void deactivateInterceptor(const std::string& interceptor_id, const char* reason) {
    const auto runtime = std::ranges::find(runtimes_, interceptor_id, &Runtime::id);
    if (runtime == runtimes_.end() || !runtime->active) {
      return;
    }
    runtime->active = false;
    runtime->assigned_detection_id = 0U;
    publishReadiness(*runtime, false);
    RCLCPP_INFO(get_logger(),
                "TARGET_ASSIGNMENT_INTERCEPTOR_INACTIVE interceptor_id='%s' "
                "reason=%s mission_epoch=%" PRIu64,
                interceptor_id.c_str(), reason, mission_epoch_);
  }

  [[nodiscard]] bool allActiveTargetsObserved(const Runtime& runtime) const {
    if (!runtime.tracks.has_value()) {
      return false;
    }
    const msg::TargetTrackArray& tracks = runtime.tracks.value();
    for (const std::uint64_t detection_id : expected_detection_ids_) {
      if (inactive_detection_ids_.contains(detection_id)) {
        continue;
      }
      if (std::ranges::none_of(tracks.tracks, [detection_id](
                                                  const msg::TargetTrack& track) {
            return track.source_detection_id == detection_id && track.position_valid;
          })) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] std::vector<TargetAssignmentAgent> assignmentAgents() const {
    std::vector<TargetAssignmentAgent> agents;
    agents.reserve(runtimes_.size());
    for (const Runtime& runtime : runtimes_) {
      if (!runtime.active || !runtime.ownship.has_value() ||
          !runtime.tracks.has_value() || !allActiveTargetsObserved(runtime)) {
        continue;
      }
      const msg::TargetTrackArray& tracks = runtime.tracks.value();
      TargetAssignmentAgent agent{
          .interceptor_id = runtime.id,
          .ownship = runtime.ownship.value(),
          .tracks = {},
      };
      for (const msg::TargetTrack& track : tracks.tracks) {
        if (!track.position_valid || track.source_detection_id == 0U ||
            inactive_detection_ids_.contains(track.source_detection_id)) {
          continue;
        }
        agent.tracks.push_back(TargetAssignmentTrack{
            .state =
                TimedVehicleState{
                    .position =
                        Point3{track.position.x, track.position.y, track.position.z},
                    .velocity =
                        Vec3{track.velocity.x, track.velocity.y, track.velocity.z},
                    .stamp_ns = detail::timeNanoseconds(track.header.stamp),
                    .position_valid = track.position_valid,
                    .velocity_valid = track.velocity_valid,
                    .navigation_ready = true,
                },
            .detection_id = track.source_detection_id,
            .track_id = track.track_id,
        });
      }
      agents.push_back(std::move(agent));
    }
    return agents;
  }

  void publishDecision(Runtime& runtime, const TargetAssignmentDecision& decision,
                       const TargetAssignmentUpdate& update) {
    if (!runtime.tracks.has_value()) {
      publishReadiness(runtime, false);
      return;
    }
    const msg::TargetTrackArray& tracks = runtime.tracks.value();
    const auto selected = std::ranges::find(tracks.tracks, decision.detection_id,
                                            &msg::TargetTrack::source_detection_id);
    if (selected == tracks.tracks.end()) {
      publishReadiness(runtime, false);
      return;
    }
    const bool assignment_changed =
        runtime.assigned_detection_id != decision.detection_id;
    if (assignment_changed ||
        runtime.last_published_scan_sequence != selected->source_scan_sequence) {
      runtime.selected_track_pub->publish(*selected);
      runtime.last_published_scan_sequence = selected->source_scan_sequence;
    }
    runtime.assigned_detection_id = decision.detection_id;
    publishReadiness(runtime, true);
    if (!update.changed && !assignment_changed) {
      return;
    }
    msg::TargetAssignment message;
    message.header.stamp = now();
    message.header.frame_id = selected->header.frame_id;
    message.mission_epoch = mission_epoch_;
    message.assignment_generation = update.generation;
    message.interceptor_id = runtime.id;
    message.target_detection_id = decision.detection_id;
    message.target_track_id = decision.track_id;
    message.estimated_intercept_time_s = decision.estimated_intercept_time_s;
    message.reason = assignmentReasonMessage(update.reason);
    message.active = true;
    runtime.assignment_pub->publish(message);
    RCLCPP_INFO(get_logger(),
                "TARGET_ASSIGNMENT interceptor_id='%s' detection_id=%" PRIu64
                " track_id=%" PRIu64 " estimated_intercept_time_s=%.3f "
                "generation=%" PRIu64 " reason=%s",
                runtime.id.c_str(), decision.detection_id, decision.track_id,
                decision.estimated_intercept_time_s, update.generation,
                targetAssignmentReasonName(update.reason));
  }

  void tick() {
    const std::vector<TargetAssignmentAgent> agents = assignmentAgents();
    const TargetAssignmentUpdate update =
        assignment_.update(now().nanoseconds(), agents);
    for (Runtime& runtime : runtimes_) {
      const auto decision = std::ranges::find(
          update.decisions, runtime.id, &TargetAssignmentDecision::interceptor_id);
      if (decision == update.decisions.end()) {
        runtime.assigned_detection_id = 0U;
        publishReadiness(runtime, false);
        continue;
      }
      publishDecision(runtime, *decision, update);
    }
  }

  AdaptiveTargetAssignment assignment_;
  std::vector<Runtime> runtimes_;
  std::unordered_set<std::uint64_t> expected_detection_ids_;
  std::unordered_set<std::uint64_t> inactive_detection_ids_;
  std::uint64_t mission_epoch_{1U};
  rclcpp::Subscription<msg::InterceptTargetStatus>::SharedPtr target_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node>
makeTargetAssignmentCoordinatorNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<TargetAssignmentCoordinatorNode>(options);
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::TargetAssignmentCoordinatorNode)
