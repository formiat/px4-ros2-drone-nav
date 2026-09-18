#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/multi_vehicle_spectator_node.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace drone_city_nav {
namespace {

using namespace std::chrono_literals;

const std::vector<std::string> kVehicleIds{"civilian_0", "civilian_1", "civilian_2"};
const std::vector<std::string> kDestroyedTopics{
    "/test/spectator_reselection/civilian_0_destroyed",
    "/test/spectator_reselection/civilian_1_destroyed",
    "/test/spectator_reselection/civilian_2_destroyed"};
const std::string kTargetTopic{"/test/spectator_reselection/target"};

class SpectatorReselectionLifecycleTest : public ::testing::Test {
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
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter{"mission_epoch", std::int64_t{1}},
         rclcpp::Parameter{"vehicle_ids", kVehicleIds},
         rclcpp::Parameter{
             "vehicle_state_topics",
             std::vector<std::string>{"/test/spectator_reselection/civilian_0_state",
                                      "/test/spectator_reselection/civilian_1_state",
                                      "/test/spectator_reselection/civilian_2_state"}},
         rclcpp::Parameter{"vehicle_destroyed_topics", kDestroyedTopics},
         rclcpp::Parameter{
             "vehicle_roles",
             std::vector<std::int64_t>{msg::VehicleDestroyed::ROLE_CIVILIAN,
                                       msg::VehicleDestroyed::ROLE_CIVILIAN,
                                       msg::VehicleDestroyed::ROLE_CIVILIAN}},
         rclcpp::Parameter{"gazebo_models",
                           std::vector<std::string>{"civilian_0_model",
                                                    "civilian_1_model",
                                                    "civilian_2_model"}},
         rclcpp::Parameter{"initial_vehicle_id", kVehicleIds.front()},
         rclcpp::Parameter{"reselection_policy", "first_living"},
         rclcpp::Parameter{"reselection_delay_s", 0.25},
         rclcpp::Parameter{"spectator_target_topic", kTargetTopic}});
    spectator_ = makeMultiVehicleSpectatorNode(options);
    driver_ = std::make_shared<rclcpp::Node>("spectator_reselection_driver");

    const auto latched_qos = rclcpp::QoS{1}.reliable().transient_local();
    for (const std::string& topic : kDestroyedTopics) {
      destroyed_publishers_.push_back(
          driver_->create_publisher<msg::VehicleDestroyed>(topic, latched_qos));
    }
    target_subscription_ = driver_->create_subscription<msg::SpectatorTarget>(
        kTargetTopic, latched_qos,
        [this](const msg::SpectatorTarget::SharedPtr target) {
          targets_.push_back(*target);
          latest_target_received_at_ = std::chrono::steady_clock::now();
        });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(spectator_);
    executor_->add_node(driver_);
  }

  void TearDown() override {
    executor_->remove_node(driver_);
    executor_->remove_node(spectator_);
    executor_.reset();
    driver_.reset();
    spectator_.reset();
  }

  [[nodiscard]] bool spinUntil(const std::function<bool()>& predicate,
                               const std::chrono::milliseconds timeout = 1s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    executor_->spin_some();
    return predicate();
  }

  void spinFor(const std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor_->spin_some();
  }

  void publishDestroyed(const std::size_t index) {
    msg::VehicleDestroyed destroyed;
    destroyed.stamp = driver_->now();
    destroyed.mission_epoch = 1U;
    destroyed.vehicle_id = kVehicleIds[index];
    destroyed.vehicle_role = msg::VehicleDestroyed::ROLE_CIVILIAN;
    destroyed.death_cause = msg::VehicleDestroyed::CAUSE_PROXIMITY_COLLISION;
    destroyed_publishers_[index]->publish(destroyed);
  }

  [[nodiscard]] bool destroyedSubscriptionsReady() const {
    return destroyed_publishers_[0]->get_subscription_count() == 1U &&
           destroyed_publishers_[1]->get_subscription_count() == 1U;
  }

  [[nodiscard]] std::size_t targetCount() const noexcept {
    return targets_.size();
  }

  [[nodiscard]] const std::string& latestTargetId() const {
    return targets_.back().vehicle_id;
  }

  [[nodiscard]] std::chrono::steady_clock::time_point
  latestTargetReceivedAt() const noexcept {
    return latest_target_received_at_;
  }

private:
  std::shared_ptr<rclcpp::Node> spectator_;
  std::shared_ptr<rclcpp::Node> driver_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr>
      destroyed_publishers_;
  rclcpp::Subscription<msg::SpectatorTarget>::SharedPtr target_subscription_;
  std::vector<msg::SpectatorTarget> targets_;
  std::chrono::steady_clock::time_point latest_target_received_at_;
};

TEST_F(SpectatorReselectionLifecycleTest,
       DelaysHandoffAndSkipsCandidateDestroyedDuringDelay) {
  ASSERT_TRUE(spinUntil([this] { return targetCount() != 0U; }));
  ASSERT_EQ(latestTargetId(), "civilian_0");
  ASSERT_TRUE(spinUntil([this] { return destroyedSubscriptionsReady(); }));

  const auto observed_destroyed_at = std::chrono::steady_clock::now();
  publishDestroyed(0U);
  spinFor(80ms);
  EXPECT_EQ(targetCount(), 1U);
  publishDestroyed(1U);
  spinFor(80ms);
  EXPECT_EQ(targetCount(), 1U);

  ASSERT_TRUE(spinUntil([this] { return targetCount() >= 2U; }));
  EXPECT_EQ(latestTargetId(), "civilian_2");
  EXPECT_GE(latestTargetReceivedAt() - observed_destroyed_at, 200ms);
}

} // namespace
} // namespace drone_city_nav
