#include "drone_city_nav/cooperative_traffic_ros.hpp"

#include <cinttypes>
#include <cmath>
#include <stdexcept>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::configureCooperativeTraffic() {
  cooperative_traffic_enabled_ =
      declare_parameter<bool>("cooperative_traffic_enabled", false);
  vehicle_id_ = declare_parameter<std::string>("vehicle_id", "");
  cooperative_channel_route_config_.desired_center_separation_m =
      declare_parameter<double>("cooperative_channel_desired_center_separation_m", 5.0);
  cooperative_passage_volume_config_.minimum_wall_clearance_m =
      declare_parameter<double>("cooperative_channel_minimum_wall_clearance_m", 1.0);
  cooperative_passage_volume_config_.lateral_probe_step_m =
      declare_parameter<double>("cooperative_channel_lateral_probe_step_m", 0.5);
  cooperative_passage_volume_config_.cross_section_spacing_m =
      declare_parameter<double>("cooperative_passage_cross_section_spacing_m", 1.0);
  cooperative_passage_volume_config_.secondary_probe_step_m =
      declare_parameter<double>("cooperative_passage_secondary_probe_step_m", 0.5);
  cooperative_passage_volume_config_.maximum_cross_section_probe_m =
      declare_parameter<double>("cooperative_passage_maximum_probe_m", 30.0);
  cooperative_passage_volume_config_.flight_envelope = flight_envelope_config_;
  cooperative_passage_volume_config_.footprint = SweptFootprintConfig{
      .radius_m = lattice_3d_config_.physical_footprint_radius_m,
      .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
      .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
      .perimeter_samples = safety_config_.physical_footprint_samples,
      .radial_rings = safety_config_.physical_footprint_radial_rings,
      .axial_samples = safety_config_.physical_footprint_axial_samples,
      .sweep_step_m = safety_config_.swept_validation_step_m,
  };
  cooperative_channel_route_config_.preferred_transition_length_m =
      declare_parameter<double>("cooperative_channel_preferred_transition_m", 10.0);
  cooperative_channel_route_config_.minimum_transition_length_m =
      declare_parameter<double>("cooperative_channel_minimum_transition_m", 3.0);
  cooperative_channel_route_config_.directional_offset_fraction =
      declare_parameter<double>("cooperative_channel_directional_offset_fraction", 0.5);
  cooperative_channel_route_config_.footprint =
      cooperative_passage_volume_config_.footprint;
  cooperative_channel_timing_config_.minimum_prediction_speed_mps =
      declare_parameter<double>("cooperative_channel_minimum_prediction_speed_mps",
                                1.0);
  cooperative_channel_timing_config_.maximum_prediction_horizon_s =
      declare_parameter<double>("cooperative_channel_maximum_prediction_horizon_s",
                                30.0);
  cooperative_channel_yield_config_.stopping_buffer_m =
      declare_parameter<double>("cooperative_channel_stopping_buffer_m", 2.0);
  cooperative_channel_yield_config_.reaction_latency_s =
      declare_parameter<double>("cooperative_channel_reaction_latency_s", 0.1);
  cooperative_channel_yield_config_.maximum_braking_acceleration_mps2 =
      declare_parameter<double>("cooperative_channel_maximum_braking_mps2", 8.0);
  mppi_config_.cooperative.desired_minimum_separation_m = static_cast<float>(
      declare_parameter<double>("cooperative_desired_minimum_separation_m", 5.0));
  mppi_config_.cooperative.candidate_acceleration_fraction = static_cast<float>(
      declare_parameter<double>("cooperative_candidate_acceleration_fraction", 0.75));
  mppi_config_.cooperative.candidate_duration_s = static_cast<float>(
      declare_parameter<double>("cooperative_candidate_duration_s", 1.5));
  mppi_config_.costs.peer_separation_weight = static_cast<float>(
      declare_parameter<double>("cooperative_peer_separation_weight", 80.0));
  mppi_config_.costs.cooperative_maneuver_preference_weight = static_cast<float>(
      declare_parameter<double>("cooperative_maneuver_preference_weight", 1.5));

  if ((cooperative_traffic_enabled_ && vehicle_id_.empty()) ||
      !passageVolumeConfigIsValid(cooperative_passage_volume_config_) ||
      !(cooperative_channel_route_config_.desired_center_separation_m > 0.0) ||
      !(cooperative_channel_route_config_.directional_offset_fraction >= 0.0) ||
      !(cooperative_channel_route_config_.directional_offset_fraction <= 1.0) ||
      !(cooperative_channel_route_config_.preferred_transition_length_m > 0.0) ||
      !(cooperative_channel_route_config_.minimum_transition_length_m > 0.0) ||
      cooperative_channel_route_config_.minimum_transition_length_m >
          cooperative_channel_route_config_.preferred_transition_length_m ||
      !(cooperative_channel_timing_config_.minimum_prediction_speed_mps > 0.0) ||
      !(cooperative_channel_timing_config_.maximum_prediction_horizon_s > 0.0) ||
      !(cooperative_channel_yield_config_.stopping_buffer_m >= 0.0) ||
      !(cooperative_channel_yield_config_.reaction_latency_s >= 0.0) ||
      !(cooperative_channel_yield_config_.maximum_braking_acceleration_mps2 > 0.0)) {
    throw std::invalid_argument{"invalid cooperative planner configuration"};
  }
}

void ProductionMppiNode::createCooperativeTrafficInterfaces(
    const rclcpp::SubscriptionOptions& subscription_options) {
  if (!cooperative_traffic_enabled_) {
    return;
  }
  const auto command_qos = rclcpp::QoS{4}.reliable();
  cooperative_command_sub_ = create_subscription<msg::CooperativeManeuverCommand>(
      declare_parameter<std::string>("cooperative_maneuver_command_topic",
                                     "/drone_city_nav/cooperative/command"),
      command_qos,
      [this](const msg::CooperativeManeuverCommand::SharedPtr message) {
        onCooperativeManeuverCommand(*message);
      },
      subscription_options);
  cooperative_channel_state_pub_ = create_publisher<msg::CooperativeChannelIntent>(
      declare_parameter<std::string>("cooperative_channel_state_topic",
                                     "/drone_city_nav/cooperative/channel_state"),
      command_qos);
}

void ProductionMppiNode::onCooperativeManeuverCommand(
    const msg::CooperativeManeuverCommand& message) {
  const CooperativeManeuverCommandData command =
      cooperativeManeuverCommandData(message);
  if (message.header.frame_id != frame_id_ || command.vehicle_id != vehicle_id_ ||
      command.command_generation == 0U || command.stamp_ns <= 0 ||
      command.valid_until_ns < command.stamp_ns) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "COOPERATIVE_COMMAND_REJECTED vehicle_id='%s' source_vehicle_id='%s' "
        "generation=%" PRIu64 " reason=invalid_contract",
        vehicle_id_.c_str(), command.vehicle_id.c_str(), command.command_generation);
    return;
  }
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const std::scoped_lock lock{input_mutex_};
  if (cooperative_command_.has_value()) {
    const CooperativeManeuverCommandData& previous = cooperative_command_->data;
    if (command.stamp_ns < previous.stamp_ns ||
        (command.stamp_ns == previous.stamp_ns &&
         command.command_generation <= previous.command_generation)) {
      return;
    }
  }
  cooperative_command_ = ProductionMppiCooperativeCommand{
      .data = command,
      .receive_stamp_ns = receive_stamp_ns,
  };
}

ProductionMppiCooperativeUpdate ProductionMppiNode::prepareCooperativeTick(
    const ProductionMppiPreparedEsdf& esdf,
    const ConstrainedRouteObservation& route_observation,
    const std::optional<ProductionMppiCooperativeCommand>& command,
    const std::int64_t now_ns, const double planned_speed_mps) {
  ProductionMppiCooperativeUpdate result;
  if (!cooperative_traffic_enabled_) {
    return result;
  }

  const CooperativeChannelAssignment* assignment = nullptr;
  if (route_observation.span_available && esdf.cooperative_channel_assignments &&
      route_observation.span_index < esdf.cooperative_channel_assignments->size()) {
    const CooperativeChannelAssignment& candidate =
        (*esdf.cooperative_channel_assignments)[route_observation.span_index];
    if (candidate.span_index == route_observation.span_index &&
        candidate.route_generation == route_observation.route_generation &&
        candidate.channel_id == route_observation.channel_id) {
      assignment = &candidate;
    }
  }
  if (assignment != nullptr) {
    result.channel = makeCooperativeChannelUse(route_observation, *assignment, now_ns,
                                               planned_speed_mps,
                                               cooperative_channel_timing_config_);
  }
  if (cooperative_channel_state_pub_) {
    cooperative_channel_state_pub_->publish(
        cooperativeChannelIntentMessage(result.channel));
  }
  if (!command.has_value()) {
    return result;
  }

  result.command_generation = command->data.command_generation;
  result.command_age_ms =
      now_ns >= command->data.stamp_ns
          ? static_cast<double>(now_ns - command->data.stamp_ns) / 1.0e6
          : -1.0;
  result.mppi =
      adaptCooperativeMppiCommand(command->data, vehicle_id_, now_ns,
                                  mppi_config_.steps, mppi_config_.dynamics.dt_s);
  result.yield = evaluateCooperativeChannelYield(
      command->data, result.channel, route_observation, vehicle_id_, now_ns,
      route_observation.actual_horizontal_speed_mps, cooperative_channel_yield_config_);
  return result;
}

} // namespace drone_city_nav
