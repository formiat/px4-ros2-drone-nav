// The onboard budget as a relay (roadmap item 36, specification K53), an
// evaluation process nobody tells the vehicle about. Each output of the
// onboard loop passes through it and is held until it is as old, on the
// simulation clock, as the host of record measured that output to be at its
// consumer (`budget_relay.hpp`): an estimate, a memory update or a horizon
// reaches its consumer as late as it would on board (K25), whatever the host
// does to the nodes. At a real-time factor of 1.0 the outputs are already
// that old when they arrive and the relay changes nothing; under a simulator
// that waits for the loop it is what keeps the budget honest. The bytes pass
// untouched through generic subscriptions and publishers. `mode` measure
// listens on the producers' topics and records every output's age; `mode` on
// relays (the consumers read the `/budgeted` topics, the estimators publish
// their odometry to the relay: `launch/budget_relay.py`). The ages are never
// turned down by hand.
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "budget_relay.hpp"

namespace {

using drone_city_nav::budget_relay::kRelayed;
using drone_city_nav::budget_relay::Qos;
using drone_city_nav::budget_relay::Relayed;

rclcpp::QoS qosOf(const Qos kind) {
  switch (kind) {
    case Qos::kSensor:
      return rclcpp::SensorDataQoS{};
    case Qos::kBestEffortTransient:
      return rclcpp::QoS{1}.best_effort().transient_local();
    case Qos::kReliableTransient:
      return rclcpp::QoS{1}.reliable().transient_local();
    case Qos::kReliable2:
      return rclcpp::QoS{2}.reliable();
    case Qos::kBestEffort10:
      return rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
  }
  return rclcpp::QoS{1};
}

struct Held {
  std::int64_t due_ns;
  std::shared_ptr<rclcpp::SerializedMessage> message;
  std::int64_t received_ns;
};

struct Output {
  const Relayed* relayed{nullptr};
  double age_ms{0.0};
  std::shared_ptr<rclcpp::GenericSubscription> subscription;
  std::shared_ptr<rclcpp::GenericPublisher> publisher;
  std::deque<Held> held;
  std::vector<double> ages_ms;
  std::vector<double> holds_ms;
};

struct Percentiles {
  double p50;
  double p95;
  double max;
};

Percentiles percentilesOf(std::vector<double>& values) {
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return {
      values[n / 2],
      values[std::min(n - 1, static_cast<std::size_t>(0.95 * static_cast<double>(n)))],
      values[n - 1]};
}

class BudgetRelayNode final : public rclcpp::Node {
public:
  BudgetRelayNode()
      : rclcpp::Node("budget_relay") {
    mode_ = declare_parameter<std::string>("mode", "measure");
    const auto profile = declare_parameter<std::string>("profile", "stereo_tof");
    const auto output_path =
        declare_parameter<std::string>("output", "budget_relay.csv");
    const double summary_period_s = declare_parameter<double>("summary_period_s", 10.0);
    summary_period_ns_ = static_cast<std::int64_t>(summary_period_s * 1.0e9);
    if (mode_ != "on" && mode_ != "measure") {
      throw std::invalid_argument("budget_relay: mode is 'on' or 'measure'");
    }
    log_.open(output_path);
    log_ << "sim_s,output,samples,age_p50_ms,age_p95_ms,age_max_ms,held_p50_ms\n";
    std::string outputs;
    for (const Relayed& relayed : kRelayed) {
      const double age_ms =
          profile == "lidar" ? relayed.lidar_age_ms : relayed.stereo_age_ms;
      if (age_ms <= 0.0) {
        continue;
      }
      auto output = std::make_unique<Output>();
      output->relayed = &relayed;
      output->age_ms = age_ms;
      // In measure mode the consumers read the producers directly: listen there,
      // publish nothing.
      const std::string source{mode_ == "on" || relayed.name != "odometry"
                                   ? relayed.source
                                   : relayed.target};
      Output* raw = output.get();
      output->subscription = create_generic_subscription(
          source, std::string{relayed.type}, qosOf(relayed.qos),
          [this, raw](std::shared_ptr<rclcpp::SerializedMessage> message) {
            receive(*raw, message);
          });
      if (mode_ == "on") {
        output->publisher = create_generic_publisher(
            std::string{relayed.target}, std::string{relayed.type}, qosOf(relayed.qos));
      }
      outputs += std::string{relayed.name} + "=" + std::to_string(age_ms) + " ";
      outputs_.push_back(std::move(output));
    }
    // The simulation clock moves in the world's 4 ms steps: a finer timer only costs
    // cores.
    timer_ = create_wall_timer(std::chrono::milliseconds(4), [this] { release(); });
    RCLCPP_INFO(get_logger(), "BUDGET_RELAY mode=%s profile=%s outputs: %s",
                mode_.c_str(), profile.c_str(), outputs.c_str());
  }

  BudgetRelayNode(const BudgetRelayNode&) = delete;
  BudgetRelayNode(BudgetRelayNode&&) = delete;
  BudgetRelayNode& operator=(const BudgetRelayNode&) = delete;
  BudgetRelayNode& operator=(BudgetRelayNode&&) = delete;

  ~BudgetRelayNode() override {
    const std::int64_t now_ns = now().nanoseconds();
    for (auto& output : outputs_) {
      writeRow(*output, now_ns);
    }
    log_.flush();
  }

private:
  void receive(Output& output,
               const std::shared_ptr<rclcpp::SerializedMessage>& message) {
    const std::int64_t received_ns = now().nanoseconds();
    const auto& serialized = message->get_rcl_serialized_message();
    const auto stamp_ns = drone_city_nav::budget_relay::stampNsOf(
        serialized.buffer, serialized.buffer_length, output.relayed->header_first);
    if (!stamp_ns.has_value()) {
      return;
    }
    output.ages_ms.push_back(static_cast<double>(received_ns - *stamp_ns) * 1.0e-6);
    if (mode_ == "on") {
      const std::int64_t due_ns = std::max(
          received_ns, *stamp_ns + static_cast<std::int64_t>(output.age_ms * 1.0e6));
      output.held.push_back(Held{due_ns, message, received_ns});
      release();
    }
  }

  void release() {
    const std::int64_t now_ns = now().nanoseconds();
    for (auto& output : outputs_) {
      while (!output->held.empty() && output->held.front().due_ns <= now_ns) {
        const Held held = std::move(output->held.front());
        output->held.pop_front();
        output->publisher->publish(*held.message);
        output->holds_ms.push_back(static_cast<double>(now_ns - held.received_ns) *
                                   1.0e-6);
      }
    }
    if (now_ns - last_summary_ns_ >= summary_period_ns_) {
      last_summary_ns_ = now_ns;
      for (auto& output : outputs_) {
        writeRow(*output, now_ns);
      }
      log_.flush();
    }
  }

  void writeRow(Output& output, const std::int64_t now_ns) {
    if (output.ages_ms.empty()) {
      return;
    }
    const Percentiles ages = percentilesOf(output.ages_ms);
    const double held_p50 =
        output.holds_ms.empty() ? 0.0 : percentilesOf(output.holds_ms).p50;
    log_ << static_cast<double>(now_ns) * 1.0e-9 << ',' << output.relayed->name << ','
         << output.ages_ms.size() << ',' << ages.p50 << ',' << ages.p95 << ','
         << ages.max << ',' << held_p50 << '\n';
    output.ages_ms.clear();
    output.holds_ms.clear();
  }

  std::string mode_;
  std::int64_t summary_period_ns_{10'000'000'000LL};
  std::int64_t last_summary_ns_{0};
  std::ofstream log_;
  std::vector<std::unique_ptr<Output>> outputs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BudgetRelayNode>());
  rclcpp::shutdown();
  return 0;
}
