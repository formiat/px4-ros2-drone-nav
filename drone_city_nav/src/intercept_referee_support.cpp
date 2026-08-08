#include "intercept_referee_support.hpp"

#include <cinttypes>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::vector<std::string>
vehicleTopics(const std::vector<std::string>& ids, const std::string& prefix,
              const std::string& suffix) {
  std::vector<std::string> topics;
  topics.reserve(ids.size());
  for (const std::string& id : ids) {
    std::string topic;
    topic.reserve(prefix.size() + id.size() + suffix.size());
    topic.append(prefix).append(id).append(suffix);
    topics.push_back(std::move(topic));
  }
  return topics;
}

void requireCount(const std::vector<std::string>& values, const std::size_t count,
                  const std::string& parameter_name) {
  if (values.size() != count) {
    throw std::invalid_argument{parameter_name + " must contain " +
                                std::to_string(count) + " entries"};
  }
}

} // namespace

InterceptorTopicConfig
declareInterceptorTopicConfig(rclcpp::Node& node,
                              const std::vector<std::string>& vehicle_ids) {
  const auto vehicleDefault = [&](const std::string& suffix) {
    return vehicleTopics(vehicle_ids, "/vehicles/", suffix);
  };
  InterceptorTopicConfig config{
      .navigation_state = node.declare_parameter<std::vector<std::string>>(
          "interceptor_state_topics", vehicleDefault("/state")),
      .physical_truth_state = node.declare_parameter<std::vector<std::string>>(
          "interceptor_truth_state_topics",
          vehicleTopics(vehicle_ids, "/simulation_truth/vehicles/", "/state")),
      .execution_horizon = node.declare_parameter<std::vector<std::string>>(
          "interceptor_execution_horizon_topics",
          vehicleDefault("/mppi/execution_horizon")),
      .world_readiness = node.declare_parameter<std::vector<std::string>>(
          "interceptor_world_readiness_topics", vehicleDefault("/mppi/world_ready")),
      .track_readiness = node.declare_parameter<std::vector<std::string>>(
          "target_track_readiness_topics", vehicleDefault("/target_track_ready")),
      .destroyed = node.declare_parameter<std::vector<std::string>>(
          "interceptor_destroyed_topics", vehicleDefault("/vehicle_destroyed")),
      .mission_start = node.declare_parameter<std::vector<std::string>>(
          "interceptor_start_topics", vehicleDefault("/mission_start")),
      .mission_command = node.declare_parameter<std::vector<std::string>>(
          "interceptor_mission_command_topics", vehicleDefault("/mission_command")),
      .radar_simulator_fqn = node.declare_parameter<std::vector<std::string>>(
          "radar_simulator_node_fqns", vehicleDefault("/radar_simulator_node")),
  };
  for (const auto& [values, parameter_name] :
       std::vector<std::pair<const std::vector<std::string>*, std::string>>{
           {&config.navigation_state, "interceptor_state_topics"},
           {&config.physical_truth_state, "interceptor_truth_state_topics"},
           {&config.execution_horizon, "interceptor_execution_horizon_topics"},
           {&config.world_readiness, "interceptor_world_readiness_topics"},
           {&config.track_readiness, "target_track_readiness_topics"},
           {&config.destroyed, "interceptor_destroyed_topics"},
           {&config.mission_start, "interceptor_start_topics"},
           {&config.mission_command, "interceptor_mission_command_topics"},
           {&config.radar_simulator_fqn, "radar_simulator_node_fqns"}}) {
    requireCount(*values, vehicle_ids.size(), parameter_name);
  }
  return config;
}

std::int64_t missionTimeoutNanoseconds(const double seconds) {
  if (!(seconds > 0.0) || !std::isfinite(seconds)) {
    throw std::invalid_argument{"mission timeout must be finite and positive"};
  }
  return static_cast<std::int64_t>(seconds * 1.0e9);
}

bool validateVehicleDestroyedEvent(const rclcpp::Logger& logger,
                                   const msg::VehicleDestroyed& destroyed,
                                   const std::uint8_t expected_role,
                                   const std::string& expected_id,
                                   const std::uint64_t mission_epoch) {
  const bool valid =
      destroyed.vehicle_role == expected_role && destroyed.vehicle_id == expected_id &&
      detail::validDeathCause(destroyed.death_cause) &&
      (destroyed.mission_epoch == 0U || destroyed.mission_epoch == mission_epoch);
  if (!valid) {
    RCLCPP_ERROR(logger,
                 "VEHICLE_DESTROYED referee_rejected=true expected_role=%s "
                 "expected_vehicle_id='%s' actual_role=%u actual_vehicle_id='%s' "
                 "cause=%u event_epoch=%" PRIu64 " mission_epoch=%" PRIu64,
                 detail::vehicleRoleName(expected_role), expected_id.c_str(),
                 static_cast<unsigned>(destroyed.vehicle_role),
                 destroyed.vehicle_id.c_str(),
                 static_cast<unsigned>(destroyed.death_cause), destroyed.mission_epoch,
                 mission_epoch);
  }
  return valid;
}

msg::NavigationObjective makePositionHoldObjective(const rclcpp::Time& stamp,
                                                   const std::uint64_t mission_epoch,
                                                   const std::uint64_t sample_sequence,
                                                   const Point3& position) {
  msg::NavigationObjective objective;
  objective.stamp = stamp;
  objective.mission_epoch = mission_epoch;
  objective.sample_sequence = sample_sequence;
  objective.position.x = position.x;
  objective.position.y = position.y;
  objective.position.z = position.z;
  objective.objective_type = msg::NavigationObjective::OBJECTIVE_TYPE_POSITION;
  objective.guidance_mode = msg::NavigationObjective::GUIDANCE_MODE_DIRECT;
  objective.terminal_policy = msg::NavigationObjective::TERMINAL_POLICY_POSITION_HOLD;
  return objective;
}

void logPhysicalProximityIntercept(const rclcpp::Logger& logger,
                                   const std::string& interceptor_id,
                                   const MultiInterceptMissionUpdate& update,
                                   const TimedVehicleState& interceptor_state,
                                   const TimedVehicleState& evader_state,
                                   const double capture_radius_m,
                                   const std::int64_t event_stamp_ns,
                                   const std::uint64_t mission_epoch) {
  const double interceptor_age_ms =
      static_cast<double>(event_stamp_ns - interceptor_state.stamp_ns) * 1.0e-6;
  const double evader_age_ms =
      static_cast<double>(event_stamp_ns - evader_state.stamp_ns) * 1.0e-6;
  RCLCPP_ERROR(logger,
               "PROXIMITY_INTERCEPT destruction_requested=true physical_truth=true "
               "interceptor_id='%s' measured_swept_separation_m=%.3f "
               "current_separation_m=%.3f separation_threshold_m=%.3f "
               "interpolation_fraction=%.6f interceptor_position=(%.3f,%.3f,%.3f) "
               "evader_position=(%.3f,%.3f,%.3f) interceptor_truth_age_ms=%.1f "
               "evader_truth_age_ms=%.1f mission_epoch=%" PRIu64,
               interceptor_id.c_str(), update.separation_m, update.current_separation_m,
               capture_radius_m, update.interpolation_fraction,
               interceptor_state.position.x, interceptor_state.position.y,
               interceptor_state.position.z, evader_state.position.x,
               evader_state.position.y, evader_state.position.z, interceptor_age_ms,
               evader_age_ms, mission_epoch);
}

} // namespace drone_city_nav
