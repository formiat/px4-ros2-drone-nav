#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace drone_city_nav {

struct GroundTruthTopicContract {
  std::string topic;
  std::unordered_set<std::string> allowed_subscribers;
  std::unordered_set<std::string> required_subscribers;
};

struct GroundTruthBoundaryUpdate {
  bool verified{false};
  bool newly_verified{false};
  bool identity_pending{false};
  std::string violating_subscriber;
  std::string violating_topic;
};

class SimulationGroundTruthBoundary final {
public:
  explicit SimulationGroundTruthBoundary(
      std::vector<GroundTruthTopicContract> contracts);

  [[nodiscard]] GroundTruthBoundaryUpdate update(const rclcpp::Node& node);

private:
  std::vector<GroundTruthTopicContract> contracts_;
  bool verified_{false};
};

[[nodiscard]] std::unique_ptr<SimulationGroundTruthBoundary>
makeExclusiveGroundTruthBoundary(const std::string& required_subscriber_fqn,
                                 const std::vector<std::string>& truth_topics);

} // namespace drone_city_nav
