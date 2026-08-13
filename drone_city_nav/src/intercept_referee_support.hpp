#pragma once

#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/simulation_truth_alignment.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/simulation_truth_alignment.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace drone_city_nav {

struct InterceptorTopicConfig {
  std::vector<std::string> navigation_state;
  std::vector<std::string> physical_truth_state;
  std::vector<std::string> execution_horizon;
  std::vector<std::string> world_readiness;
  std::vector<std::string> track_readiness;
  std::vector<std::string> destroyed;
  std::vector<std::string> mission_start;
  std::vector<std::string> mission_command;
  std::vector<std::string> radar_simulator_fqn;
};

struct TargetTopicConfig {
  std::vector<std::string> navigation_state;
  std::vector<std::string> physical_truth_state;
  std::vector<std::string> execution_horizon;
  std::vector<std::string> world_readiness;
  std::vector<std::string> destroyed;
  std::vector<std::string> objective;
  std::vector<std::string> mission_start;
};

[[nodiscard]] InterceptorTopicConfig
declareInterceptorTopicConfig(rclcpp::Node& node,
                              const std::vector<std::string>& vehicle_ids);

[[nodiscard]] TargetTopicConfig
declareTargetTopicConfig(rclcpp::Node& node,
                         const std::vector<std::string>& vehicle_ids);

[[nodiscard]] std::int64_t missionTimeoutNanoseconds(double seconds);

[[nodiscard]] bool validateVehicleDestroyedEvent(const rclcpp::Logger& logger,
                                                 const msg::VehicleDestroyed& destroyed,
                                                 std::uint8_t expected_role,
                                                 const std::string& expected_id,
                                                 std::uint64_t mission_epoch);

[[nodiscard]] msg::NavigationObjective
makePositionHoldObjective(const rclcpp::Time& stamp, std::uint64_t mission_epoch,
                          std::uint64_t sample_sequence, const Point3& position);

[[nodiscard]] msg::VehicleDestroyed
makeProximityDestructionEvent(const rclcpp::Time& stamp, std::uint64_t mission_epoch,
                              const TimedVehicleState& state, std::uint8_t role,
                              const std::string& vehicle_id, std::uint8_t cause,
                              const std::string& event_detail);

void logRuntimeTruthAlignmentTransition(
    const rclcpp::Logger& logger, const msg::SimulationTruthAlignment& status,
    const SimulationTruthAlignmentMissionUpdate& update);

void logPhysicalProximityIntercept(
    const rclcpp::Logger& logger, const std::string& interceptor_id,
    const std::string& target_id, const SweptVehicleSeparation& separation,
    const TimedVehicleState& interceptor_state, const TimedVehicleState& evader_state,
    double capture_radius_m, std::int64_t event_stamp_ns, std::uint64_t mission_epoch);

} // namespace drone_city_nav
