#include "drone_city_nav/mission_waypoint_acknowledgement_admission.hpp"
#include "drone_city_nav/msg/mission_waypoint_acknowledgement.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

namespace drone_city_nav {
namespace {

using namespace std::chrono_literals;

constexpr const char* kAcknowledgementTopic{
    "/test/mission_waypoint_acknowledgement_transport"};

[[nodiscard]] builtin_interfaces::msg::Time
timeFromNanoseconds(const std::int64_t nanoseconds) noexcept {
  builtin_interfaces::msg::Time time;
  if (nanoseconds <= 0) {
    return time;
  }
  time.sec = static_cast<std::int32_t>(nanoseconds / 1'000'000'000LL);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % 1'000'000'000LL);
  return time;
}

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& time) noexcept {
  return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.nanosec);
}

[[nodiscard]] msg::MissionWaypointAcknowledgement
acknowledgement(const std::uint64_t producer_instance_id,
                const std::uint64_t acknowledgement_sequence,
                const std::uint32_t completed_waypoint_count,
                const std::uint64_t mission_epoch, const std::int64_t source_stamp_ns) {
  msg::MissionWaypointAcknowledgement message;
  message.header.stamp = timeFromNanoseconds(source_stamp_ns);
  message.header.frame_id = "map";
  message.producer_instance_id = producer_instance_id;
  message.acknowledgement_sequence = acknowledgement_sequence;
  message.mission_epoch = mission_epoch;
  message.completed_waypoint_index = completed_waypoint_count - 1U;
  message.completed_waypoint_count = completed_waypoint_count;
  message.waypoint_count = 3U;
  message.mission_completed = completed_waypoint_count == message.waypoint_count;
  message.active_waypoint_index = message.mission_completed
                                      ? message.waypoint_count - 1U
                                      : completed_waypoint_count;
  message.completed_goal.x = 10.0 * static_cast<double>(completed_waypoint_count);
  message.completed_goal.y = 20.0;
  message.completed_goal.z = 18.0;
  message.horizon_producer_instance_id = producer_instance_id;
  message.horizon_sequence = acknowledgement_sequence + 100U;
  message.offboard_producer_instance_id = 31U;
  message.horizon_valid_from = timeFromNanoseconds(source_stamp_ns - 3'000'000'000LL);
  message.horizon_valid_until = timeFromNanoseconds(source_stamp_ns + 1'000'000'000LL);
  message.witness_stamp = timeFromNanoseconds(source_stamp_ns - 10'000'000LL);
  message.route_target = message.completed_goal;
  message.stationary_hold_position = message.completed_goal;
  return message;
}

class MissionWaypointAcknowledgementTransportTest : public ::testing::Test {
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
    publisher_node_ =
        std::make_shared<rclcpp::Node>("mission_waypoint_ack_transport_publisher");
    monitor_node_ =
        std::make_shared<rclcpp::Node>("mission_waypoint_ack_transport_monitor");
    publisher_ = publisher_node_->create_publisher<msg::MissionWaypointAcknowledgement>(
        kAcknowledgementTopic, rclcpp::QoS{32}.reliable().transient_local());
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(publisher_node_);
    executor_->add_node(monitor_node_);
  }

  void TearDown() override {
    acknowledgement_sub_.reset();
    publisher_.reset();
    executor_->remove_node(monitor_node_);
    executor_->remove_node(publisher_node_);
    executor_.reset();
    monitor_node_.reset();
    publisher_node_.reset();
  }

  void subscribe() {
    acknowledgement_sub_ =
        monitor_node_->create_subscription<msg::MissionWaypointAcknowledgement>(
            kAcknowledgementTopic, rclcpp::QoS{32}.reliable().transient_local(),
            [this](const msg::MissionWaypointAcknowledgement::SharedPtr message) {
              const std::int64_t receive_stamp_ns = monitor_node_->now().nanoseconds();
              MissionWaypointAcknowledgementCandidate candidate{
                  .producer_instance_id = message->producer_instance_id,
                  .acknowledgement_sequence = message->acknowledgement_sequence,
                  .content_fingerprint = (message->producer_instance_id << 32U) ^
                                         (message->acknowledgement_sequence << 16U) ^
                                         message->completed_waypoint_count ^
                                         message->mission_epoch ^
                                         static_cast<std::uint64_t>(
                                             timeNanoseconds(message->header.stamp)) ^
                                         1U,
                  .mission_epoch = message->mission_epoch,
                  .horizon_producer_instance_id = message->horizon_producer_instance_id,
                  .horizon_sequence = message->horizon_sequence,
                  .offboard_producer_instance_id =
                      message->offboard_producer_instance_id,
                  .source_stamp_ns = timeNanoseconds(message->header.stamp),
                  .receive_stamp_ns = receive_stamp_ns,
                  .horizon_valid_from_ns = timeNanoseconds(message->horizon_valid_from),
                  .horizon_valid_until_ns =
                      timeNanoseconds(message->horizon_valid_until),
                  .witness_stamp_ns = timeNanoseconds(message->witness_stamp),
                  .completed_waypoint_index = message->completed_waypoint_index,
                  .completed_waypoint_count = message->completed_waypoint_count,
                  .waypoint_count = message->waypoint_count,
                  .active_waypoint_index = message->active_waypoint_index,
                  .completed_goal =
                      Point3{message->completed_goal.x, message->completed_goal.y,
                             message->completed_goal.z},
                  .route_target =
                      Point3{message->route_target.x, message->route_target.y,
                             message->route_target.z},
                  .stationary_hold_position =
                      Point3{message->stationary_hold_position.x,
                             message->stationary_hold_position.y,
                             message->stationary_hold_position.z},
                  .mission_completed = message->mission_completed,
                  .payload_valid = true,
              };
              if (candidate.content_fingerprint == 0U) {
                candidate.content_fingerprint = 1U;
              }
              const MissionWaypointAcknowledgementAdmissionResult admission =
                  admitMissionWaypointAcknowledgement(admission_state_, candidate);
              if (admission.state_advanced) {
                admission_state_ = admission.next_state;
              }
              accepted_count_ += admission.accept ? 1U : 0U;
              replay_count_ += admission.replay ? 1U : 0U;
              aggregate_replay_count_ += admission.aggregate_replay ? 1U : 0U;
            });
  }

  [[nodiscard]] bool spinUntil(
      const std::function<bool()>& predicate,
      const std::function<void()>& publish = [] {},
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

  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::Node> monitor_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<msg::MissionWaypointAcknowledgement>::SharedPtr publisher_;
  rclcpp::Subscription<msg::MissionWaypointAcknowledgement>::SharedPtr
      acknowledgement_sub_;
  MissionWaypointAcknowledgementAdmissionState admission_state_{};
  std::size_t accepted_count_{0U};
  std::size_t replay_count_{0U};
  std::size_t aggregate_replay_count_{0U};
};

TEST_F(MissionWaypointAcknowledgementTransportTest,
       LateJoinDropRecoveryAndMonotonicProducerHandoffAreExactlyOnce) {
  const std::int64_t base_stamp_ns =
      publisher_node_->now().nanoseconds() - 10'000'000'000LL;
  publisher_->publish(acknowledgement(11U, 1U, 1U, 0U, base_stamp_ns));
  publisher_->publish(
      acknowledgement(11U, 3U, 2U, 1U, base_stamp_ns + 1'000'000'000LL));

  subscribe();
  ASSERT_TRUE(spinUntil(
      [this] { return admission_state_.current.completed_waypoint_count == 2U; }));
  EXPECT_EQ(admission_state_.current.acknowledgement_sequence, 3U);

  executor_->remove_node(publisher_node_);
  publisher_.reset();
  publisher_node_.reset();
  publisher_node_ = std::make_shared<rclcpp::Node>(
      "mission_waypoint_ack_transport_replacement_publisher");
  publisher_ = publisher_node_->create_publisher<msg::MissionWaypointAcknowledgement>(
      kAcknowledgementTopic, rclcpp::QoS{32}.reliable().transient_local());
  executor_->add_node(publisher_node_);

  const msg::MissionWaypointAcknowledgement restarted =
      acknowledgement(22U, 1U, 2U, 1U, base_stamp_ns + 2'000'000'000LL);
  const std::size_t accepted_before_restart = accepted_count_;
  ASSERT_TRUE(
      spinUntil([this] { return admission_state_.current.producer_instance_id == 22U; },
                [this, &restarted] { publisher_->publish(restarted); }));
  EXPECT_EQ(accepted_count_, accepted_before_restart);
  EXPECT_GT(aggregate_replay_count_, 0U);
  EXPECT_TRUE(admission_state_.producerRetired(11U));

  ASSERT_TRUE(spinUntil([this] { return replay_count_ > 0U; },
                        [this, &restarted] { publisher_->publish(restarted); }));
  EXPECT_EQ(accepted_count_, accepted_before_restart);

  const msg::MissionWaypointAcknowledgement resumed =
      acknowledgement(22U, 2U, 3U, 2U, base_stamp_ns + 3'000'000'000LL);
  ASSERT_TRUE(spinUntil(
      [this] { return admission_state_.current.completed_waypoint_count == 3U; },
      [this, &resumed] { publisher_->publish(resumed); }));
  EXPECT_EQ(accepted_count_, accepted_before_restart + 1U);
}

} // namespace
} // namespace drone_city_nav
