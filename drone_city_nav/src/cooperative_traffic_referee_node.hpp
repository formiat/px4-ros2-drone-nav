#pragma once

#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/cooperative_traffic_mission.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/simulation_truth_alignment.hpp"
#include "drone_city_nav/msg/simulation_truth_state.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/simulation_truth_alignment.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "intercept_ground_truth_boundary.hpp"

namespace drone_city_nav {

class CooperativeTrafficRefereeNode final : public rclcpp::Node {
public:
  CooperativeTrafficRefereeNode();

private:
  struct HoldHorizon {
    Point3 position{};
    std::uint64_t sequence{0U};
    bool active{false};
  };

  struct VehicleRuntime {
    std::string id;
    std::string truth_state_topic;
    Point3 goal{};
    std::optional<TimedVehicleState> navigation_state;
    std::optional<TimedVehicleState> truth_state;
    std::optional<HoldHorizon> hold_horizon;
    std::unique_ptr<CooperativeGoalHoldConfirmation> goal_hold_confirmation;
    std::unique_ptr<CooperativeGoalHoldConfirmation> failure_hold_confirmation;
    std::optional<Point3> requested_hold_position;
    bool world_ready{false};
    bool intent_ready{false};
    bool executable_horizon_ready{false};
    bool destroyed{false};
    bool goal_hold_confirmed{false};
    bool hold_requested{false};
    std::uint64_t objective_sequence{0U};
    std::uint64_t latest_intent_generation{0U};
    std::uint64_t first_executable_horizon_sequence{0U};
    std::uint64_t hold_request_horizon_sequence{0U};
    std::int64_t latest_intent_receive_ns{0};
    std::int64_t latest_intent_valid_until_ns{0};
    std::int64_t degraded_since_ns{0};
    std::int64_t hold_requested_ns{0};
    std::int64_t destroyed_observed_ns{0};
    rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr state_sub;
    rclcpp::Subscription<msg::SimulationTruthState>::SharedPtr truth_state_sub;
    rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr horizon_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr world_ready_sub;
    rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr destroyed_sub;
    rclcpp::Publisher<msg::NavigationObjective>::SharedPtr objective_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub;
  };

  void configureVehicles(const std::vector<std::string>& ids,
                         const std::vector<double>& goals_xyz);
  void configureGroundTruthBoundary();
  void onTruthAlignmentStatus(const msg::SimulationTruthAlignment& status);
  void onFlightIntent(const msg::CooperativeFlightIntent& intent);
  void onVehicleDestroyed(const msg::VehicleDestroyed& destroyed,
                          std::size_t vehicle_index);

  [[nodiscard]] std::optional<TimedVehicleState>
  physicalState(std::size_t index) const noexcept;
  void publishGoalObjective(std::size_t index);
  void publishMissionStart();
  [[nodiscard]] bool verifyGroundTruthBoundary(std::int64_t now_ns);
  [[nodiscard]] bool missionReady(std::int64_t now_ns) const;
  [[nodiscard]] bool runtimeInputsHealthy(std::int64_t now_ns);
  void updateSeparationMetrics();
  void updateGoalHolds();
  [[nodiscard]] bool allGoalHoldsConfirmed() const noexcept;

  void beginFailure(const std::string& reason);
  void requestHold(std::size_t index, const std::string& reason);
  void requestHoldsForSurvivors(const std::string& reason);
  [[nodiscard]] bool allDestroyedVehiclesSettled(std::int64_t now_ns);
  [[nodiscard]] bool allSurvivorsHeld(std::int64_t now_ns);
  void settleFailure(std::int64_t now_ns);
  void finishSuccess();
  void finishFailure(const std::string& reason);
  void completeResultLifecycle();
  void tick();

  std::vector<VehicleRuntime> vehicles_;
  std::unordered_map<std::string, std::size_t> vehicle_indices_;
  std::unique_ptr<CooperativePeerStore> intent_validator_;
  std::unique_ptr<CooperativeSeparationMonitor> separation_monitor_;
  std::unique_ptr<InterceptGroundTruthBoundary> ground_truth_boundary_;
  SimulationTruthAlignmentMissionLifecycle truth_alignment_lifecycle_;
  SimulationTruthAlignmentMissionUpdate truth_alignment_update_{};
  CooperativeGoalHoldConfig goal_hold_config_{};
  CooperativeSeparationConfig separation_config_{};
  std::optional<std::string> failure_reason_;
  std::string truth_alignment_reason_;
  std::string truth_alignment_vehicle_id_;
  std::string minimum_pair_first_id_;
  std::string minimum_pair_second_id_;
  double truth_alignment_maximum_error_m_{0.0};
  std::uint64_t mission_epoch_{1U};
  std::int64_t maximum_input_age_ns_{1'000'000'000LL};
  std::int64_t maximum_intent_age_ns_{500'000'000LL};
  std::int64_t maximum_degraded_duration_ns_{5'000'000'000LL};
  std::int64_t readiness_timeout_ns_{60'000'000'000LL};
  std::int64_t mission_timeout_ns_{240'000'000'000LL};
  std::int64_t hold_timeout_ns_{20'000'000'000LL};
  std::int64_t destruction_settlement_timeout_ns_{5'000'000'000LL};
  std::int64_t boundary_startup_timeout_ns_{10'000'000'000LL};
  std::int64_t readiness_started_ns_{0};
  std::int64_t mission_started_ns_{0};
  std::int64_t failure_latched_ns_{0};
  std::int64_t boundary_check_started_ns_{0};
  std::int64_t last_boundary_check_ns_{0};
  std::size_t active_desired_violation_count_{0U};
  bool mission_started_{false};
  bool result_reported_{false};
  bool shutdown_on_terminal_outcome_{true};
  bool boundary_verified_{false};
  rclcpp::Subscription<msg::SimulationTruthAlignment>::SharedPtr truth_alignment_sub_;
  rclcpp::Subscription<msg::CooperativeFlightIntent>::SharedPtr intent_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav
