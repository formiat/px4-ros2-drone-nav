#include "simulation_truth_boundary.hpp"

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

SimulationGroundTruthBoundary::SimulationGroundTruthBoundary(
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
SimulationGroundTruthBoundary::update(const rclcpp::Node& node) {
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

std::unique_ptr<SimulationGroundTruthBoundary>
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
  return std::make_unique<SimulationGroundTruthBoundary>(std::move(contracts));
}

} // namespace drone_city_nav
