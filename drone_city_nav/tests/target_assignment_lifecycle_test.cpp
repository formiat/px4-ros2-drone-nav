#include "drone_city_nav/interceptor_guidance_node.hpp"
#include "drone_city_nav/msg/intercept_target_status.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/target_assignment.hpp"
#include "drone_city_nav/msg/target_track_array.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/target_assignment_coordinator_node.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace drone_city_nav {
namespace {

using namespace std::chrono_literals;

const std::string kInterceptorId{"interceptor_0"};
const std::string kStateTopic{"/test/assignment_lifecycle/state"};
const std::string kTrackArrayTopic{"/test/assignment_lifecycle/target_tracks"};
const std::string kSelectedTrackTopic{"/test/assignment_lifecycle/target_track"};
const std::string kAssignmentTopic{"/test/assignment_lifecycle/assignment"};
const std::string kReadinessTopic{"/test/assignment_lifecycle/readiness"};
const std::string kDestroyedTopic{"/test/assignment_lifecycle/destroyed"};
const std::string kTargetStatusTopic{"/test/assignment_lifecycle/target_status"};
const std::string kMissionCommandTopic{"/test/assignment_lifecycle/mission_command"};
const std::string kRadarModeTopic{"/test/assignment_lifecycle/radar_mode"};
const std::string kObjectiveTopic{"/test/assignment_lifecycle/objective"};

class TargetAssignmentLifecycleTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions coordinator_options;
    coordinator_options.parameter_overrides(
        {rclcpp::Parameter{"mission_epoch", std::int64_t{1}},
         rclcpp::Parameter{"interceptor_ids", std::vector<std::string>{kInterceptorId}},
         rclcpp::Parameter{"interceptor_state_topics",
                           std::vector<std::string>{kStateTopic}},
         rclcpp::Parameter{"target_track_array_topics",
                           std::vector<std::string>{kTrackArrayTopic}},
         rclcpp::Parameter{"selected_target_track_topics",
                           std::vector<std::string>{kSelectedTrackTopic}},
         rclcpp::Parameter{"target_assignment_topics",
                           std::vector<std::string>{kAssignmentTopic}},
         rclcpp::Parameter{"target_track_readiness_topics",
                           std::vector<std::string>{kReadinessTopic}},
         rclcpp::Parameter{"interceptor_destroyed_topics",
                           std::vector<std::string>{kDestroyedTopic}},
         rclcpp::Parameter{"target_detection_ids", std::vector<std::int64_t>{1}},
         rclcpp::Parameter{"target_status_topic", kTargetStatusTopic},
         rclcpp::Parameter{"maximum_track_age_s", 10.0}});
    coordinator_ = makeTargetAssignmentCoordinatorNode(coordinator_options);

    rclcpp::NodeOptions guidance_options;
    guidance_options.parameter_overrides(
        {rclcpp::Parameter{"mission_epoch", std::int64_t{1}},
         rclcpp::Parameter{"interceptor_id", kInterceptorId},
         rclcpp::Parameter{"ownship_state_topic", kStateTopic},
         rclcpp::Parameter{"target_track_topic", kSelectedTrackTopic},
         rclcpp::Parameter{"target_assignment_topic", kAssignmentTopic},
         rclcpp::Parameter{"mission_command_topic", kMissionCommandTopic},
         rclcpp::Parameter{"radar_track_mode_command_topic", kRadarModeTopic},
         rclcpp::Parameter{"navigation_objective_topic", kObjectiveTopic}});
    guidance_ = makeInterceptorGuidanceNode(guidance_options);
    driver_ = std::make_shared<rclcpp::Node>("target_assignment_lifecycle_driver");

    const auto transient_qos = rclcpp::QoS{1}.reliable().transient_local();
    state_pub_ = driver_->create_publisher<msg::VehicleNavigationState>(
        kStateTopic, rclcpp::QoS{10}.best_effort());
    tracks_pub_ = driver_->create_publisher<msg::TargetTrackArray>(kTrackArrayTopic,
                                                                   transient_qos);
    target_status_pub_ = driver_->create_publisher<msg::InterceptTargetStatus>(
        kTargetStatusTopic, rclcpp::QoS{100}.reliable().transient_local());
    assignment_sub_ = driver_->create_subscription<msg::TargetAssignment>(
        kAssignmentTopic, transient_qos,
        [this](const msg::TargetAssignment::SharedPtr assignment) {
          assignments_.push_back(*assignment);
        });
    objective_sub_ = driver_->create_subscription<msg::NavigationObjective>(
        kObjectiveTopic, transient_qos,
        [this](const msg::NavigationObjective::SharedPtr objective) {
          objectives_.push_back(*objective);
        });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(coordinator_);
    executor_->add_node(guidance_);
    executor_->add_node(driver_);
  }

  void TearDown() override {
    executor_->remove_node(driver_);
    executor_->remove_node(guidance_);
    executor_->remove_node(coordinator_);
    executor_.reset();
    driver_.reset();
    guidance_.reset();
    coordinator_.reset();
  }

  [[nodiscard]] msg::VehicleNavigationState ownshipState() const {
    msg::VehicleNavigationState state;
    state.stamp = driver_->now();
    state.position.x = 10.0;
    state.position.y = 20.0;
    state.position.z = 18.0;
    state.position_valid = true;
    state.velocity_valid = true;
    state.navigation_ready = true;
    return state;
  }

  [[nodiscard]] msg::TargetTrackArray targetTracks() const {
    msg::TargetTrack track;
    track.header.stamp = driver_->now();
    track.header.frame_id = "map";
    track.track_id = 101U;
    track.source_scan_sequence = 7U;
    track.source_detection_id = 1U;
    track.position.x = 80.0;
    track.position.y = 20.0;
    track.position.z = 18.0;
    track.velocity.x = 5.0;
    track.position_valid = true;
    track.velocity_valid = true;
    track.status = msg::TargetTrack::STATUS_TRACKING;
    msg::TargetTrackArray tracks;
    tracks.header = track.header;
    tracks.source_scan_sequence = track.source_scan_sequence;
    tracks.tracks.push_back(track);
    return tracks;
  }

  void publishInputs() {
    state_pub_->publish(ownshipState());
    tracks_pub_->publish(targetTracks());
  }

  [[nodiscard]] bool spinUntil(const std::function<bool()>& predicate,
                               const std::function<void()>& publish,
                               const std::chrono::milliseconds timeout = 3s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      publish();
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    executor_->spin_some();
    return predicate();
  }

  [[nodiscard]] bool activeLifecycleObserved() const {
    const bool active_assignment =
        std::ranges::any_of(assignments_, [](const msg::TargetAssignment& assignment) {
          return assignment.active && assignment.target_track_id == 101U;
        });
    const bool tracking_objective =
        std::ranges::any_of(objectives_, [](const msg::NavigationObjective& objective) {
          return objective.objective_type ==
                     msg::NavigationObjective::OBJECTIVE_TYPE_TRACKING_PREDICTION &&
                 objective.assignment_generation != 0U &&
                 objective.target_detection_id == 1U &&
                 objective.target_track_id == 101U;
        });
    return active_assignment && tracking_objective;
  }

  void publishTerminalInputs() {
    state_pub_->publish(ownshipState());
    msg::InterceptTargetStatus terminal;
    terminal.header.stamp = driver_->now();
    terminal.mission_epoch = 1U;
    terminal.target_id = "evader";
    terminal.target_detection_id = 1U;
    terminal.status = msg::InterceptTargetStatus::STATUS_INTERCEPTED;
    terminal.capturing_interceptor_id = kInterceptorId;
    target_status_pub_->publish(terminal);
  }

  [[nodiscard]] std::optional<std::uint64_t> clearObjectiveSequence() const {
    const auto clear = std::ranges::find_if(
        objectives_, [](const msg::NavigationObjective& objective) {
          return objective.objective_type ==
                     msg::NavigationObjective::OBJECTIVE_TYPE_POSITION &&
                 objective.target_track_id == 0U &&
                 objective.terminal_policy ==
                     msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD;
        });
    if (clear == objectives_.end()) {
      return std::nullopt;
    }
    return clear->sample_sequence;
  }

  [[nodiscard]] bool clearedLifecycleObserved() const {
    const bool inactive_assignment =
        std::ranges::any_of(assignments_, [](const msg::TargetAssignment& assignment) {
          return !assignment.active && assignment.target_track_id == 101U;
        });
    return inactive_assignment && clearObjectiveSequence().has_value();
  }

  [[nodiscard]] bool
  trackingObjectivePublishedAfter(const std::uint64_t sample_sequence) const {
    return std::ranges::any_of(
        objectives_, [sample_sequence](const msg::NavigationObjective& objective) {
          return objective.sample_sequence > sample_sequence &&
                 objective.objective_type ==
                     msg::NavigationObjective::OBJECTIVE_TYPE_TRACKING_PREDICTION;
        });
  }

private:
  std::shared_ptr<rclcpp::Node> coordinator_;
  std::shared_ptr<rclcpp::Node> guidance_;
  std::shared_ptr<rclcpp::Node> driver_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<msg::VehicleNavigationState>::SharedPtr state_pub_;
  rclcpp::Publisher<msg::TargetTrackArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<msg::InterceptTargetStatus>::SharedPtr target_status_pub_;
  rclcpp::Subscription<msg::TargetAssignment>::SharedPtr assignment_sub_;
  rclcpp::Subscription<msg::NavigationObjective>::SharedPtr objective_sub_;
  std::vector<msg::TargetAssignment> assignments_;
  std::vector<msg::NavigationObjective> objectives_;
};

TEST_F(TargetAssignmentLifecycleTest,
       ClearsTrackingObjectiveWhenAssignedTargetBecomesTerminal) {
  ASSERT_TRUE(spinUntil([this] { return activeLifecycleObserved(); },
                        [this] { publishInputs(); }));
  ASSERT_TRUE(spinUntil([this] { return clearedLifecycleObserved(); },
                        [this] { publishTerminalInputs(); }));

  const std::optional<std::uint64_t> clear_sequence = clearObjectiveSequence();
  ASSERT_TRUE(clear_sequence.has_value());
  const std::uint64_t resolved_clear_sequence = clear_sequence.value_or(0U);
  (void)spinUntil([] { return false; }, [this] { publishInputs(); }, 250ms);
  EXPECT_FALSE(trackingObjectivePublishedAfter(resolved_clear_sequence));
}

} // namespace
} // namespace drone_city_nav
