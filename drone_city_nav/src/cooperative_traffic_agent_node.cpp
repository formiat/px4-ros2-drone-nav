#include "drone_city_nav/cooperative_traffic_agent_node.hpp"

#include "drone_city_nav/cooperative_passage_coordination.hpp"
#include "drone_city_nav/cooperative_space_time.hpp"
#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/cooperative_traffic_ros.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/cooperative_maneuver_command.hpp"
#include "drone_city_nav/msg/cooperative_passage_intent.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 point(const geometry_msgs::msg::Point& value) noexcept {
  return Point3{value.x, value.y, value.z};
}

[[nodiscard]] Vec3 vector(const geometry_msgs::msg::Vector3& value) noexcept {
  return Vec3{value.x, value.y, value.z};
}

[[nodiscard]] std::int64_t secondsToNanoseconds(const double seconds) noexcept {
  return static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
}

} // namespace

class CooperativeTrafficAgentNode final : public rclcpp::Node {
public:
  explicit CooperativeTrafficAgentNode(const rclcpp::NodeOptions& options)
      : Node{"cooperative_traffic_agent_node", options},
        vehicle_id_{declare_parameter<std::string>("vehicle_id", "civilian_0")},
        frame_id_{declare_parameter<std::string>("frame_id", "map")},
        publication_rate_hz_{
            declare_parameter<double>("intent_publication_rate_hz", 20.0)},
        maximum_input_age_s_{declare_parameter<double>("maximum_input_age_s", 0.5)},
        maximum_intent_horizon_s_{
            declare_parameter<double>("maximum_intent_horizon_s", 5.0)},
        command_validity_s_{declare_parameter<double>("command_validity_s", 0.25)},
        footprint_radius_m_{declare_parameter<double>("footprint_radius_m", 0.82)},
        footprint_lower_extent_m_{
            declare_parameter<double>("footprint_lower_extent_m", 0.23)},
        footprint_upper_extent_m_{
            declare_parameter<double>("footprint_upper_extent_m", 0.35)},
        peer_store_{vehicle_id_,
                    CooperativePeerStoreConfig{
                        .maximum_publication_age_s = declare_parameter<double>(
                            "maximum_peer_publication_age_s", 0.5),
                        .maximum_peers = static_cast<std::size_t>(
                            declare_parameter<int>("maximum_peer_count", 32)),
                    }},
        conflict_lifecycle_{CooperativeConflictConfig{
            .prediction_horizon_s =
                declare_parameter<double>("conflict_prediction_horizon_s", 5.0),
            .desired_minimum_separation_m =
                declare_parameter<double>("desired_minimum_separation_m", 5.0),
            .release_separation_m =
                declare_parameter<double>("release_separation_m", 7.0),
            .minimum_maneuver_latch_s =
                declare_parameter<double>("minimum_maneuver_latch_s", 1.0),
            .release_confirmation_s =
                declare_parameter<double>("release_confirmation_s", 0.5),
        }} {
    if (vehicle_id_.empty() || frame_id_.empty() || !(publication_rate_hz_ > 0.0) ||
        !(maximum_input_age_s_ > 0.0) || !(maximum_intent_horizon_s_ > 0.0) ||
        !(command_validity_s_ > 0.0) || !(footprint_radius_m_ > 0.0) ||
        !(footprint_lower_extent_m_ >= 0.0) || !(footprint_upper_extent_m_ >= 0.0)) {
      throw std::invalid_argument{"invalid cooperative traffic agent configuration"};
    }
    passage_config_.reservation_time_margin_s =
        declare_parameter<double>("passage_reservation_time_margin_s", 0.5);
    passage_config_.same_path_entry_headway_s =
        declare_parameter<double>("passage_same_path_entry_headway_s", 1.0);
    passage_config_.lateral_separation_tolerance_m =
        declare_parameter<double>("passage_lateral_separation_tolerance_m", 0.1);
    passage_config_.conflict = CooperativeConflictConfig{
        .prediction_horizon_s =
            declare_parameter<double>("passage_conflict_prediction_horizon_s", 5.0),
        .desired_minimum_separation_m =
            declare_parameter<double>("passage_desired_minimum_separation_m", 5.0),
        .release_separation_m =
            declare_parameter<double>("passage_release_separation_m", 7.0),
        .minimum_maneuver_latch_s = 0.0,
        .release_confirmation_s = 0.0,
    };
    space_time_config_ = CooperativeSpaceTimeConfig{
        .prediction_horizon_s =
            declare_parameter<double>("space_time_prediction_horizon_s", 5.0),
        .desired_minimum_separation_m =
            declare_parameter<double>("space_time_desired_minimum_separation_m", 5.0),
        .spatial_transition_s =
            declare_parameter<double>("space_time_spatial_transition_s", 1.5),
        .minimum_spatial_offset_m =
            declare_parameter<double>("space_time_minimum_spatial_offset_m", 0.5),
        .maximum_spatial_offset_m =
            declare_parameter<double>("space_time_maximum_spatial_offset_m", 5.0),
        .minimum_time_shift_s =
            declare_parameter<double>("space_time_minimum_shift_s", 0.25),
        .maximum_time_shift_s =
            declare_parameter<double>("space_time_maximum_shift_s", 2.0),
        .sample_period_s = declare_parameter<double>("space_time_sample_period_s", 0.1),
        .spatial_margin_m =
            declare_parameter<double>("space_time_spatial_margin_m", 0.25),
        .incumbent_hysteresis_m =
            declare_parameter<double>("space_time_incumbent_hysteresis_m", 0.25),
    };
    if (!cooperativeSpaceTimeConfigIsValid(space_time_config_)) {
      throw std::invalid_argument{"invalid cooperative space-time configuration"};
    }

    const auto state_qos = rclcpp::QoS{10}.best_effort();
    const auto horizon_qos = rclcpp::QoS{4}.reliable();
    const auto intent_qos = cooperativeFlightIntentQos();
    state_sub_ = create_subscription<msg::VehicleNavigationState>(
        declare_parameter<std::string>("navigation_state_topic",
                                       "/vehicles/civilian_0/state"),
        state_qos, [this](const msg::VehicleNavigationState::SharedPtr message) {
          navigation_state_ = *message;
          navigation_state_receive_ns_ = now().nanoseconds();
        });
    horizon_sub_ = create_subscription<msg::MppiTrajectoryHorizon>(
        declare_parameter<std::string>("execution_horizon_topic",
                                       "/vehicles/civilian_0/mppi/execution_horizon"),
        horizon_qos, [this](const msg::MppiTrajectoryHorizon::SharedPtr message) {
          execution_horizon_ = *message;
          execution_horizon_receive_ns_ = now().nanoseconds();
        });
    passage_sub_ = create_subscription<msg::CooperativePassageIntent>(
        declare_parameter<std::string>(
            "passage_state_topic", "/vehicles/civilian_0/cooperative/passage_state"),
        horizon_qos, [this](const msg::CooperativePassageIntent::SharedPtr message) {
          passage_state_ = cooperativePassageUseData(*message);
          passage_state_receive_ns_ = now().nanoseconds();
        });
    intent_sub_ = create_subscription<msg::CooperativeFlightIntent>(
        declare_parameter<std::string>("flight_intent_topic",
                                       "/cooperative_traffic/flight_intents"),
        intent_qos, [this](const msg::CooperativeFlightIntent::SharedPtr message) {
          const std::int64_t now_ns = now().nanoseconds();
          const CooperativePeerUpdateStatus status =
              peer_store_.update(cooperativeFlightIntentData(*message), now_ns);
          if (status == CooperativePeerUpdateStatus::kInvalid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "COOPERATIVE_INTENT_REJECTED reason=invalid");
          }
        });
    intent_pub_ = create_publisher<msg::CooperativeFlightIntent>(
        declare_parameter<std::string>("flight_intent_publish_topic",
                                       "/cooperative_traffic/flight_intents"),
        intent_qos);
    command_pub_ = create_publisher<msg::CooperativeManeuverCommand>(
        declare_parameter<std::string>("maneuver_command_topic",
                                       "/vehicles/civilian_0/cooperative/command"),
        horizon_qos);
    timer_ =
        create_wall_timer(std::chrono::duration<double>{1.0 / publication_rate_hz_},
                          [this] { publishIntentAndCommand(); });
    RCLCPP_INFO(get_logger(),
                "COOPERATIVE_AGENT_READY vehicle_id='%s' rate_hz=%.1f "
                "prediction_horizon_s=%.1f desired_separation_m=%.1f",
                vehicle_id_.c_str(), publication_rate_hz_, maximum_intent_horizon_s_,
                passage_config_.conflict.desired_minimum_separation_m);
  }

private:
  [[nodiscard]] bool inputFresh(const std::int64_t receive_ns,
                                const std::int64_t now_ns) const noexcept {
    return receive_ns > 0 && now_ns >= receive_ns &&
           now_ns - receive_ns <= secondsToNanoseconds(maximum_input_age_s_);
  }

  [[nodiscard]] std::optional<CooperativeFlightIntentData>
  makeOwnIntent(const std::int64_t now_ns) {
    if (!navigation_state_.has_value() || !execution_horizon_.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "COOPERATIVE_INTENT_UNAVAILABLE vehicle_id='%s' reason=missing_input "
          "navigation_available=%s horizon_available=%s",
          vehicle_id_.c_str(), navigation_state_.has_value() ? "true" : "false",
          execution_horizon_.has_value() ? "true" : "false");
      return std::nullopt;
    }
    if (!inputFresh(navigation_state_receive_ns_, now_ns) ||
        !navigation_state_->position_valid || !navigation_state_->velocity_valid ||
        !navigation_state_->navigation_ready) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "COOPERATIVE_INTENT_UNAVAILABLE vehicle_id='%s' "
                           "reason=navigation_unavailable receive_age_ms=%.1f "
                           "position_valid=%s velocity_valid=%s navigation_ready=%s",
                           vehicle_id_.c_str(),
                           static_cast<double>(now_ns - navigation_state_receive_ns_) *
                               1.0e-6,
                           navigation_state_->position_valid ? "true" : "false",
                           navigation_state_->velocity_valid ? "true" : "false",
                           navigation_state_->navigation_ready ? "true" : "false");
      return std::nullopt;
    }
    const msg::MppiTrajectoryHorizon& horizon = *execution_horizon_;
    const std::int64_t valid_from_ns = cooperativeTimeNanoseconds(horizon.valid_from);
    const std::int64_t horizon_valid_until_ns =
        cooperativeTimeNanoseconds(horizon.valid_until);
    const std::int64_t valid_until_ns =
        std::min(horizon_valid_until_ns,
                 valid_from_ns + secondsToNanoseconds(maximum_intent_horizon_s_));
    const bool stationary_hold =
        horizon.stationary_position_hold &&
        horizon.execution_mode ==
            msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
    // An execution horizon is a time-indexed contract and remains current until
    // its own validity boundary. The planner intentionally does not republish an
    // unchanged finite path on every control tick.
    if (valid_from_ns <= 0 || execution_horizon_receive_ns_ <= 0 ||
        execution_horizon_receive_ns_ > now_ns || valid_until_ns <= now_ns ||
        (horizon.points.empty() && !stationary_hold)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "COOPERATIVE_INTENT_UNAVAILABLE vehicle_id='%s' "
          "reason=horizon_invalid receive_age_ms=%.1f valid_from_ns=%" PRId64
          " valid_until_ns=%" PRId64 " now_ns=%" PRId64
          " point_count=%zu stationary_hold=%s",
          vehicle_id_.c_str(),
          static_cast<double>(now_ns - execution_horizon_receive_ns_) * 1.0e-6,
          valid_from_ns, valid_until_ns, now_ns, horizon.points.size(),
          stationary_hold ? "true" : "false");
      return std::nullopt;
    }
    CooperativeFlightIntentData intent{
        .vehicle_id = vehicle_id_,
        .frame_id = frame_id_,
        .stamp_ns = now_ns,
        .intent_generation = ++intent_generation_,
        .valid_from_ns = valid_from_ns,
        .valid_until_ns = valid_until_ns,
        .footprint_radius_m = footprint_radius_m_,
        .footprint_lower_extent_m = footprint_lower_extent_m_,
        .footprint_upper_extent_m = footprint_upper_extent_m_,
        .current_position = point(navigation_state_->position),
        .current_velocity = vector(navigation_state_->velocity),
        .maneuver_state = last_maneuver_,
        .conflict_generation = last_conflict_generation_,
        .conflicting_vehicle_ids = {},
        .passage = {},
        .trajectory = {},
    };
    if (passage_state_.has_value() && inputFresh(passage_state_receive_ns_, now_ns)) {
      intent.passage = *passage_state_;
    }
    if (stationary_hold) {
      intent.trajectory =
          makeStationaryCooperativeTrajectory(point(horizon.stationary_hold_position),
                                              valid_from_ns, valid_until_ns, now_ns);
      return intent;
    }
    intent.trajectory.reserve(horizon.points.size());
    std::int64_t previous_time_ns = 0;
    for (const msg::MppiHorizonPoint& point_message : horizon.points) {
      const std::int64_t sample_time_ns =
          valid_from_ns + secondsToNanoseconds(point_message.time_from_start_s);
      if (sample_time_ns < valid_from_ns || sample_time_ns > valid_until_ns ||
          sample_time_ns <= previous_time_ns) {
        continue;
      }
      intent.trajectory.push_back(CooperativeTrajectorySample{
          .time_ns = sample_time_ns,
          .position = point(point_message.position),
          .velocity = vector(point_message.velocity),
      });
      previous_time_ns = sample_time_ns;
    }
    if (intent.trajectory.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "COOPERATIVE_INTENT_UNAVAILABLE vehicle_id='%s' "
          "reason=no_valid_trajectory_samples point_count=%zu valid_from_ns=%" PRId64
          " valid_until_ns=%" PRId64,
          vehicle_id_.c_str(), horizon.points.size(), valid_from_ns, valid_until_ns);
      return std::nullopt;
    }
    return intent;
  }

  void publishIntentAndCommand() {
    const std::int64_t now_ns = now().nanoseconds();
    std::optional<CooperativeFlightIntentData> ownship = makeOwnIntent(now_ns);
    if (!ownship.has_value()) {
      return;
    }
    const std::vector<CooperativeFlightIntentData> peers =
        peer_store_.activeIntents(now_ns);
    const CooperativeAvoidanceDecision avoidance =
        conflict_lifecycle_.update(now_ns, *ownship, peers);
    if (!avoidance.active ||
        avoidance.conflict_generation != last_space_time_conflict_generation_) {
      last_space_time_maneuver_.reset();
    }
    const CooperativeSpaceTimeDecision space_time =
        optimizeCooperativeSpaceTime(*ownship, avoidance.peers, now_ns,
                                     space_time_config_, last_space_time_maneuver_);
    CooperativeAvoidanceDecision coordinated_avoidance = avoidance;
    if (space_time.valid) {
      coordinated_avoidance.preferred_maneuver = space_time.maneuver;
      coordinated_avoidance.preferred_acceleration_direction =
          space_time.preferred_acceleration_direction;
      coordinated_avoidance.predicted_minimum_separation_m =
          space_time.predicted_minimum_separation_m;
      coordinated_avoidance.time_to_minimum_s = space_time.time_to_minimum_s;
      last_space_time_maneuver_ = space_time.maneuver;
      last_space_time_conflict_generation_ = avoidance.conflict_generation;
    } else if (!avoidance.active) {
      last_space_time_conflict_generation_ = 0U;
    }
    const CooperativePassageDecision passage =
        coordinateCooperativePassage(*ownship, peers, passage_config_);
    ownship->maneuver_state = passage.yield_before_entry
                                  ? CooperativeManeuver::kSlow
                                  : coordinated_avoidance.preferred_maneuver;
    ownship->conflict_generation = avoidance.conflict_generation;
    ownship->conflicting_vehicle_ids.reserve(avoidance.peers.size());
    for (const CooperativeConflictPeer& peer : avoidance.peers) {
      ownship->conflicting_vehicle_ids.push_back(peer.intent.vehicle_id);
    }
    last_maneuver_ = ownship->maneuver_state;
    last_conflict_generation_ = ownship->conflict_generation;
    intent_pub_->publish(cooperativeFlightIntentMessage(*ownship));
    publishCommand(now_ns, *ownship, coordinated_avoidance, space_time, passage);
    if (avoidance.changed || passage.yield_before_entry != last_passage_yield_ ||
        avoidance.primary_peer_id != last_primary_peer_id_ || space_time.changed) {
      RCLCPP_INFO(get_logger(),
                  "COOPERATIVE_CONFLICT vehicle_id='%s' active=%s generation=%" PRIu64
                  " maneuver=%s primary_peer='%s' predicted_minimum_m=%.3f "
                  "time_to_minimum_s=%.3f passage_yield=%s yield_to='%s' "
                  "passage_offset_m=%.3f entry_not_before_ns=%" PRId64
                  " space_time_active=%s space_time_lateral_m=%.3f "
                  "space_time_vertical_m=%.3f space_time_shift_s=%.3f "
                  "space_time_candidates=%zu space_time_shortfall_m2_s=%.3f",
                  vehicle_id_.c_str(), avoidance.active ? "true" : "false",
                  avoidance.conflict_generation,
                  cooperativeManeuverName(ownship->maneuver_state).data(),
                  avoidance.primary_peer_id.c_str(),
                  coordinated_avoidance.predicted_minimum_separation_m,
                  coordinated_avoidance.time_to_minimum_s,
                  passage.yield_before_entry ? "true" : "false",
                  passage.yield_to_vehicle_id.c_str(), passage.lateral_offset_m,
                  passage.entry_not_before_ns, space_time.active ? "true" : "false",
                  space_time.lateral_offset_m, space_time.vertical_offset_m,
                  space_time.time_shift_s, space_time.evaluated_candidate_count,
                  space_time.integrated_separation_shortfall_m2_s);
    }
    last_passage_yield_ = passage.yield_before_entry;
    last_primary_peer_id_ = avoidance.primary_peer_id;
  }

  void publishCommand(const std::int64_t now_ns,
                      const CooperativeFlightIntentData& ownship,
                      const CooperativeAvoidanceDecision& avoidance,
                      const CooperativeSpaceTimeDecision& space_time,
                      const CooperativePassageDecision& passage) {
    msg::CooperativeManeuverCommand command;
    command.header.stamp = cooperativeTimeMessage(now_ns);
    command.header.frame_id = frame_id_;
    command.vehicle_id = vehicle_id_;
    command.command_generation = ++command_generation_;
    command.valid_until =
        cooperativeTimeMessage(now_ns + secondsToNanoseconds(command_validity_s_));
    command.avoidance_active = avoidance.active;
    command.preferred_maneuver = static_cast<std::uint8_t>(ownship.maneuver_state);
    command.preferred_acceleration_direction.x =
        avoidance.preferred_acceleration_direction.x;
    command.preferred_acceleration_direction.y =
        avoidance.preferred_acceleration_direction.y;
    command.preferred_acceleration_direction.z =
        avoidance.preferred_acceleration_direction.z;
    command.conflict_generation = avoidance.conflict_generation;
    command.predicted_minimum_separation_m =
        static_cast<float>(avoidance.predicted_minimum_separation_m);
    command.time_to_closest_approach_s =
        static_cast<float>(avoidance.time_to_minimum_s);
    command.conflicting_vehicle_ids = ownship.conflicting_vehicle_ids;
    command.space_time_plan_active = space_time.valid;
    command.space_time_lateral_offset_m = space_time.lateral_offset_m;
    command.space_time_vertical_offset_m = space_time.vertical_offset_m;
    command.space_time_shift_s = space_time.time_shift_s;
    command.space_time_predicted_minimum_separation_m =
        space_time.predicted_minimum_separation_m;
    command.space_time_integrated_shortfall_m2_s =
        space_time.integrated_separation_shortfall_m2_s;
    command.space_time_evaluated_candidate_count = static_cast<std::uint32_t>(
        std::min<std::size_t>(space_time.evaluated_candidate_count,
                              std::numeric_limits<std::uint32_t>::max()));
    command.passage_yield_required = passage.yield_before_entry;
    command.passage_yield_to_vehicle_id = passage.yield_to_vehicle_id;
    command.passage_traversal_id = ownship.passage.passage_traversal_id.value();
    command.passage_conflict_resource_id = passage.conflict_resource_id.value();
    command.passage_route_generation = ownship.passage.route_generation;
    command.passage_lateral_offset_m = ownship.passage.lateral_offset_m;
    command.passage_minimum_lateral_offset_m = ownship.passage.minimum_lateral_offset_m;
    command.passage_maximum_lateral_offset_m = ownship.passage.maximum_lateral_offset_m;
    command.passage_entry_not_before =
        cooperativeTimeMessage(passage.entry_not_before_ns);
    command.passage_queue_hold_station_valid = passage.queue_hold_station_valid;
    command.passage_queue_hold_station_m = passage.queue_hold_station_m;
    command.conflicting_peers.reserve(avoidance.peers.size());
    for (const CooperativeConflictPeer& peer : avoidance.peers) {
      command.conflicting_peers.push_back(
          cooperativePeerTrajectoryMessage(peer.intent));
    }
    command_pub_->publish(command);
  }

  std::string vehicle_id_;
  std::string frame_id_;
  double publication_rate_hz_{20.0};
  double maximum_input_age_s_{0.5};
  double maximum_intent_horizon_s_{5.0};
  double command_validity_s_{0.25};
  double footprint_radius_m_{0.82};
  double footprint_lower_extent_m_{0.23};
  double footprint_upper_extent_m_{0.35};
  CooperativePeerStore peer_store_;
  CooperativeConflictLifecycle conflict_lifecycle_;
  CooperativePassageCoordinationConfig passage_config_{};
  CooperativeSpaceTimeConfig space_time_config_{};
  std::optional<msg::VehicleNavigationState> navigation_state_;
  std::optional<msg::MppiTrajectoryHorizon> execution_horizon_;
  std::optional<CooperativePassageUse> passage_state_;
  std::int64_t navigation_state_receive_ns_{0};
  std::int64_t execution_horizon_receive_ns_{0};
  std::int64_t passage_state_receive_ns_{0};
  std::uint64_t intent_generation_{0U};
  std::uint64_t command_generation_{0U};
  std::uint64_t last_conflict_generation_{0U};
  CooperativeManeuver last_maneuver_{CooperativeManeuver::kKeep};
  std::optional<CooperativeManeuver> last_space_time_maneuver_;
  std::uint64_t last_space_time_conflict_generation_{0U};
  bool last_passage_yield_{false};
  std::string last_primary_peer_id_;
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr state_sub_;
  rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr horizon_sub_;
  rclcpp::Subscription<msg::CooperativePassageIntent>::SharedPtr passage_sub_;
  rclcpp::Subscription<msg::CooperativeFlightIntent>::SharedPtr intent_sub_;
  rclcpp::Publisher<msg::CooperativeFlightIntent>::SharedPtr intent_pub_;
  rclcpp::Publisher<msg::CooperativeManeuverCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node>
makeCooperativeTrafficAgentNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<CooperativeTrafficAgentNode>(options);
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::CooperativeTrafficAgentNode)
