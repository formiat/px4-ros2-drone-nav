#include "intercept_ground_truth_boundary.hpp"

#include <stdexcept>
#include <utility>

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

InterceptGroundTruthBoundary::InterceptGroundTruthBoundary(
    std::vector<GroundTruthTopicContract> contracts)
    : contracts_{std::move(contracts)} {
  if (contracts_.empty()) {
    throw std::invalid_argument{"ground truth boundary requires contracts"};
  }
  for (const GroundTruthTopicContract& contract : contracts_) {
    if (contract.topic.empty() || contract.allowed_subscribers.empty() ||
        contract.required_subscribers.empty()) {
      throw std::invalid_argument{"invalid ground truth topic contract"};
    }
  }
}

GroundTruthBoundaryUpdate
InterceptGroundTruthBoundary::update(const rclcpp::Node& node) {
  GroundTruthBoundaryUpdate result{
      .verified = verified_,
      .newly_verified = false,
      .identity_pending = false,
      .violating_subscriber = {},
      .violating_topic = {},
  };
  bool complete = true;
  for (const GroundTruthTopicContract& contract : contracts_) {
    std::unordered_set<std::string> observed;
    for (const rclcpp::TopicEndpointInfo& endpoint :
         node.get_subscriptions_info_by_topic(contract.topic)) {
      if (!endpointIdentityKnown(endpoint)) {
        result.identity_pending = true;
        complete = false;
        continue;
      }
      const std::string subscriber = fullyQualifiedNodeName(endpoint);
      observed.insert(subscriber);
      if (!contract.allowed_subscribers.contains(subscriber)) {
        result.violating_subscriber = subscriber;
        result.violating_topic = contract.topic;
        return result;
      }
    }
    for (const std::string& required : contract.required_subscribers) {
      complete = complete && observed.contains(required);
    }
  }
  if (complete && !result.identity_pending && !verified_) {
    verified_ = true;
    result.verified = true;
    result.newly_verified = true;
  }
  return result;
}

std::unique_ptr<InterceptGroundTruthBoundary> makeInterceptGroundTruthBoundary(
    const std::string& referee_fqn, const std::string& adapter_fqn,
    const std::vector<TargetTruthEndpoint>& target_endpoints,
    const std::vector<InterceptorTruthEndpoint>& interceptor_endpoints,
    const std::vector<std::string>& target_navigation_observer_fqns) {
  std::vector<GroundTruthTopicContract> contracts;
  std::unordered_set<std::string> target_truth_subscribers{referee_fqn};
  for (const InterceptorTruthEndpoint& endpoint : interceptor_endpoints) {
    contracts.push_back(GroundTruthTopicContract{
        .topic = endpoint.physical_truth_topic,
        .allowed_subscribers = {referee_fqn, endpoint.radar_simulator_fqn},
        .required_subscribers = {referee_fqn, endpoint.radar_simulator_fqn},
    });
    target_truth_subscribers.insert(endpoint.radar_simulator_fqn);
  }
  for (const TargetTruthEndpoint& endpoint : target_endpoints) {
    std::unordered_set<std::string> navigation_subscribers{referee_fqn, adapter_fqn};
    navigation_subscribers.insert(target_navigation_observer_fqns.begin(),
                                  target_navigation_observer_fqns.end());
    contracts.push_back(GroundTruthTopicContract{
        .topic = endpoint.navigation_topic,
        .allowed_subscribers = std::move(navigation_subscribers),
        .required_subscribers = {referee_fqn, adapter_fqn},
    });
    contracts.push_back(GroundTruthTopicContract{
        .topic = endpoint.physical_truth_topic,
        .allowed_subscribers = target_truth_subscribers,
        .required_subscribers = target_truth_subscribers,
    });
  }
  return std::make_unique<InterceptGroundTruthBoundary>(std::move(contracts));
}

std::unique_ptr<InterceptGroundTruthBoundary> makeInterceptGroundTruthBoundary(
    const std::string& referee_fqn, const std::string& adapter_fqn,
    const std::string& evader_navigation_topic,
    const std::string& evader_physical_truth_topic,
    const std::vector<InterceptorTruthEndpoint>& interceptor_endpoints,
    const std::vector<std::string>& target_navigation_observer_fqns) {
  return makeInterceptGroundTruthBoundary(
      referee_fqn, adapter_fqn,
      std::vector<TargetTruthEndpoint>{{
          .navigation_topic = evader_navigation_topic,
          .physical_truth_topic = evader_physical_truth_topic,
      }},
      interceptor_endpoints, target_navigation_observer_fqns);
}

std::unique_ptr<InterceptGroundTruthBoundary>
makeExclusiveGroundTruthBoundary(const std::string& required_subscriber_fqn,
                                 const std::vector<std::string>& truth_topics) {
  if (required_subscriber_fqn.empty() || truth_topics.empty()) {
    throw std::invalid_argument{"exclusive ground truth boundary requires endpoints"};
  }
  std::vector<GroundTruthTopicContract> contracts;
  contracts.reserve(truth_topics.size());
  for (const std::string& topic : truth_topics) {
    contracts.push_back(GroundTruthTopicContract{
        .topic = topic,
        .allowed_subscribers = {required_subscriber_fqn},
        .required_subscribers = {required_subscriber_fqn},
    });
  }
  return std::make_unique<InterceptGroundTruthBoundary>(std::move(contracts));
}

} // namespace drone_city_nav
