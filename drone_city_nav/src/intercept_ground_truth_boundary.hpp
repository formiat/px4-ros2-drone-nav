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

struct InterceptorTruthEndpoint {
  std::string physical_truth_topic;
  std::string radar_simulator_fqn;
};

class InterceptGroundTruthBoundary final {
public:
  explicit InterceptGroundTruthBoundary(
      std::vector<GroundTruthTopicContract> contracts);

  [[nodiscard]] GroundTruthBoundaryUpdate update(const rclcpp::Node& node);

private:
  std::vector<GroundTruthTopicContract> contracts_;
  bool verified_{false};
};

[[nodiscard]] std::unique_ptr<InterceptGroundTruthBoundary>
makeInterceptGroundTruthBoundary(
    const std::string& referee_fqn, const std::string& adapter_fqn,
    const std::string& evader_navigation_topic,
    const std::string& evader_physical_truth_topic,
    const std::vector<InterceptorTruthEndpoint>& interceptor_endpoints);

} // namespace drone_city_nav
