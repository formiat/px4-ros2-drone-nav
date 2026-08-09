#pragma once

#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/msg/intercept_mission_command.hpp"
#include "drone_city_nav/msg/intercept_target_status.hpp"
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
#include <vector>

#include "intercept_ground_truth_boundary.hpp"

namespace drone_city_nav {

class InterceptMissionRefereeNode final : public rclcpp::Node {
public:
  InterceptMissionRefereeNode();

private:
  enum class TargetOutcome : std::uint8_t {
    kActive,
    kIntercepted,
    kReachedGoal,
    kDestroyed,
  };

  struct HoldHorizon {
    Point3 position{};
    std::uint64_t sequence{0U};
    bool active{false};
  };

  struct InterceptorRuntime {
    std::string id;
    std::string radar_simulator_fqn;
    std::string truth_state_topic;
    std::optional<TimedVehicleState> state;
    std::optional<TimedVehicleState> truth_state;
    std::optional<TimedVehicleState> previous_physical_state;
    std::optional<HoldHorizon> hold_horizon;
    std::vector<std::unique_ptr<InterceptStateAdjudicationLifecycle>>
        target_adjudications;
    std::unique_ptr<InterceptorHoldConfirmation> hold_confirmation;
    bool world_ready{false};
    bool track_ready{false};
    bool destroyed{false};
    bool destruction_requested{false};
    bool disabled{false};
    std::int64_t destruction_requested_ns{0};
    std::int64_t hold_requested_ns{0};
    std::uint64_t hold_request_horizon_sequence{0U};
    rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr state_sub;
    rclcpp::Subscription<msg::SimulationTruthState>::SharedPtr truth_state_sub;
    rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr horizon_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr world_ready_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr track_ready_sub;
    rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr destroyed_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub;
    rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr destroyed_pub;
    rclcpp::Publisher<msg::InterceptMissionCommand>::SharedPtr command_pub;
  };

  struct TargetRuntime {
    std::string id;
    std::string state_topic;
    std::string truth_state_topic;
    Point3 goal{};
    std::uint64_t detection_id{0U};
    std::optional<TimedVehicleState> state;
    std::optional<TimedVehicleState> truth_state;
    std::optional<TimedVehicleState> previous_physical_state;
    std::string capturing_interceptor_id;
    TargetOutcome outcome{TargetOutcome::kActive};
    bool world_ready{false};
    bool destroyed{false};
    bool destruction_requested{false};
    bool hold_requested{false};
    std::int64_t destruction_requested_ns{0};
    std::uint64_t objective_sequence{0U};
    rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr state_sub;
    rclcpp::Subscription<msg::SimulationTruthState>::SharedPtr truth_state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr world_ready_sub;
    rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr destroyed_sub;
    rclcpp::Publisher<msg::NavigationObjective>::SharedPtr objective_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub;
    rclcpp::Publisher<msg::VehicleDestroyed>::SharedPtr destroyed_pub;
  };

  struct ContactCandidate {
    std::size_t interceptor_index{0U};
    std::size_t target_index{0U};
    SweptVehicleSeparation separation{};
  };

  void configureInterceptors(const std::vector<std::string>& ids);
  void configureTargets(const std::vector<std::string>& ids,
                        const std::vector<std::int64_t>& detection_ids,
                        const std::vector<double>& goals_xyz);
  void configureGroundTruthBoundary();
  void onTruthAlignmentStatus(const msg::SimulationTruthAlignment& status);
  void onVehicleDestroyed(const msg::VehicleDestroyed& destroyed, bool interceptor,
                          std::size_t index);
  [[nodiscard]] bool verifyGroundTruthBoundary(std::int64_t now_ns);
  [[nodiscard]] bool missionReady() const;
  [[nodiscard]] std::optional<TimedVehicleState>
  interceptorPhysicalState(std::size_t index) const noexcept;
  [[nodiscard]] std::optional<TimedVehicleState>
  targetPhysicalState(std::size_t index) const noexcept;
  void publishMissionStart();
  void publishTargetObjective(std::size_t index);
  void publishTargetStatus(std::size_t index, const std::string& detail);
  void requestHold(std::size_t index, const std::string& reason);
  void requestHoldsForSurvivors(const std::string& reason,
                                std::optional<std::size_t> excluded = std::nullopt);
  void requestTargetHold(std::size_t index, const std::string& reason);
  void handleStartupCoordinateAlignmentFailure();

  [[nodiscard]] std::size_t operationalInterceptorCount() const noexcept;
  [[nodiscard]] std::size_t survivingInterceptorCount() const noexcept;
  [[nodiscard]] std::size_t activeTargetCount() const noexcept;
  [[nodiscard]] bool updateStateAdjudication(std::int64_t now_ns);
  void resetPhysicalContinuity();
  [[nodiscard]] std::vector<ContactCandidate> contactCandidates() const;
  void requestCaptureDestruction(const ContactCandidate& candidate,
                                 const std::string& detail);
  void requestSameRoleCollision(bool interceptor, std::size_t first_index,
                                std::size_t second_index, double separation_m);
  void detectPhysicalContacts();
  void evaluateTargetGoals();
  void updatePreviousPhysicalStates();
  void markTargetOutcome(std::size_t index, TargetOutcome outcome,
                         const std::string& detail,
                         const std::string& capturing_interceptor_id = {});
  [[nodiscard]] static const char* targetOutcomeName(TargetOutcome outcome) noexcept;
  void updateAggregateTerminal();
  [[nodiscard]] bool allRequestedDeathsSettled() const;
  [[nodiscard]] bool destructionSettlementTimedOut(std::int64_t now_ns) const;
  [[nodiscard]] bool allSurvivorsHeld(std::int64_t now_ns);
  void settleTerminal(std::int64_t now_ns);
  void finishMission();
  void failMission(const std::string& reason);
  void completeResultLifecycle();
  void tick();

  std::vector<InterceptorRuntime> interceptors_;
  std::vector<TargetRuntime> targets_;
  std::unique_ptr<InterceptGroundTruthBoundary> ground_truth_boundary_;
  SimulationTruthAlignmentMissionLifecycle truth_alignment_lifecycle_;
  SimulationTruthAlignmentMissionUpdate truth_alignment_mission_update_{};
  InterceptStateAdjudicationConfig state_config_{};
  InterceptorHoldConfig hold_config_{};
  std::optional<std::string> aggregate_outcome_;
  std::optional<std::string> system_failure_reason_;
  std::string mission_name_{"intercept"};
  std::string truth_alignment_reason_;
  std::string truth_alignment_vehicle_id_;
  double capture_radius_m_{5.0};
  double target_goal_radius_m_{2.0};
  double truth_alignment_maximum_error_m_{0.0};
  std::uint64_t mission_epoch_{1U};
  std::int64_t destruction_settlement_timeout_ns_{5'000'000'000LL};
  std::int64_t hold_timeout_ns_{20'000'000'000LL};
  std::int64_t boundary_startup_timeout_ns_{10'000'000'000LL};
  std::int64_t mission_readiness_timeout_ns_{30'000'000'000LL};
  std::int64_t destruction_requested_ns_{0};
  std::int64_t boundary_check_started_ns_{0};
  std::int64_t last_boundary_check_ns_{0};
  std::int64_t mission_readiness_started_ns_{0};
  bool mission_started_{false};
  bool result_reported_{false};
  bool shutdown_on_terminal_outcome_{true};
  bool boundary_verified_{false};
  rclcpp::Subscription<msg::SimulationTruthAlignment>::SharedPtr truth_alignment_sub_;
  rclcpp::Publisher<msg::InterceptTargetStatus>::SharedPtr target_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav
