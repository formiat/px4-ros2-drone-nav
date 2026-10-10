#pragma once

#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/dead_reckoning_landing.hpp"
#include "drone_city_nav/execution_horizon_admission.hpp"
#include "drone_city_nav/execution_horizon_contract_ros.hpp"
#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/local_hold_pin.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/producer_instance_id.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/px4_offboard_setpoint_io.hpp"
#include "drone_city_nav/transport_latency_ros.hpp"
#include "drone_city_nav/vehicle_destruction_disarm_lifecycle.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <tf2_ros/transform_broadcaster.h>

#include "mppi_offboard_node_names.hpp"

namespace drone_city_nav {

// The offboard: the vehicle's autopilot driven from the planner's execution
// horizons (mppi_offboard_node.cpp: the session, the parameters and the
// control tick; mppi_offboard_node_horizon.cpp: the horizons and the
// setpoints; mppi_offboard_node_state.cpp: the autopilot's state and what
// the node tells of it).
class MppiOffboardNode final : public rclcpp::Node {
public:
  MppiOffboardNode();

private:
  void controlTick();
  void onAutopilotStatus(const AutopilotStatus& status);
  void onLocalState(const AutopilotLocalState& state);
  void publishNavigationState();
  void publishNavigationReadiness(bool ready);
  void publishRvizDrone();
  void onHorizon(const msg::MppiTrajectoryHorizon& horizon);
  [[nodiscard]] bool horizonFresh() const;
  [[nodiscard]] bool stationaryPositionHoldActive() const noexcept;
  [[nodiscard]] double currentSpeedMps() const noexcept;
  [[nodiscard]] bool plannedFinitePathFresh() const;
  [[nodiscard]] bool plannedFinitePathCompleted() const;
  void publishDeadReckoningLandingSetpoint();
  void publishTakeoffSetpoint();
  [[nodiscard]] bool publishPrestartPlannedHorizonReceipt();
  [[nodiscard]] bool publishStationaryPositionHoldSetpoint();
  void publishCompletedFinitePathHoldSetpoint();
  [[nodiscard]] bool publishHorizonSetpoint();
  void publishUnavailablePathHoldSetpoint();
  void publishAppliedControlFeedback(Point2 acceleration, double vertical_acceleration,
                                     double yaw_rate, double yaw_acceleration,
                                     bool control_authoritative,
                                     std::uint8_t execution_mode);
  void publishOffboardSessionHeartbeat();
  void publishUnavailableControlFeedback();
  [[nodiscard]] double mapAltitudeM() const noexcept;
  [[nodiscard]] double takeoffAltitudeM() const noexcept;
  void publishCommand(std::uint32_t command, float param1, float param2 = 0.0F);
  [[nodiscard]] std::uint64_t nowMicros() const;

  static constexpr double kTakeoffCaptureToleranceM{0.5};
  double takeoff_climb_m_{2.0};
  FlightEnvelopeConfig flight_envelope_config_{};
  double takeoff_hover_s_{1.0};
  std::int64_t control_lookahead_ns_{50'000'000};
  double command_resend_period_s_{2.0};
  double local_x_{0.0};
  double local_y_{0.0};
  double altitude_m_{0.0};
  double unavailable_path_braking_acceleration_mps2_{4.0};
  double unavailable_path_braking_response_s_{0.25};
  double velocity_x_{0.0};
  double velocity_y_{0.0};
  double velocity_up_mps_{0.0};
  double heading_rad_{0.0};
  bool heading_valid_{false};
  double rviz_drone_marker_color_r_{0.15};
  double rviz_drone_marker_color_g_{0.65};
  double rviz_drone_marker_color_b_{1.0};
  int warmup_setpoints_{20};
  int warmup_count_{0};
  std::int32_t rviz_drone_marker_id_{0};
  bool auto_arm_{true};
  bool auto_offboard_{true};
  bool rviz_drone_follow_tf_enabled_{true};
  bool position_valid_{false};
  bool vehicle_status_seen_{false};
  bool execution_horizon_rearm_required_{false};
  bool destruction_disarm_confirmed_logged_{false};
  bool require_mission_start_signal_{false};
  bool require_planner_health_{true};
  bool mission_started_{false};
  bool planner_healthy_{false};
  double planner_health_timeout_s_{1.0};
  double planner_health_land_after_s_{5.0};
  bool planner_health_land_sent_{false};
  std::optional<rclcpp::Time> planner_health_received_at_;
  std::optional<rclcpp::Time> planner_health_loss_started_at_;
  Px4MapFrameTransform px4_map_transform_{};
  VehicleCommandEndpoint endpoint_{};
  std::unique_ptr<VehicleDestructionDisarmLifecycle> destruction_disarm_lifecycle_;
  AutopilotStatus vehicle_status_;
  std::optional<msg::MppiTrajectoryHorizon> horizon_;
  LocalHoldPin unavailable_path_hold_pin_;
  std::optional<rclcpp::Time> takeoff_complete_stamp_;
  std::optional<bool> last_navigation_readiness_;
  std::uint64_t offboard_producer_instance_id_{0U};
  ExecutionHorizonAdmissionState horizon_admission_{};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  DeadReckoningLanding dead_reckoning_landing_;
  std::int64_t yaw_rate_lead_ns_{0};
  std::string rviz_drone_follow_parent_frame_{"gazebo_map"};
  bool gazebo_aligned_rviz_axes_swapped_{true};
  std::string rviz_drone_follow_frame_{"drone_follow"};
  std::string applied_control_feedback_frame_id_{"map"};
  std::string destruction_detail_;
  std::string expected_vehicle_id_;
  std::uint64_t destruction_mission_epoch_{0U};
  std::uint64_t mission_epoch_{0U};
  std::uint8_t destroyed_role_{msg::VehicleDestroyed::ROLE_UNSPECIFIED};
  std::uint8_t destroyed_cause_{0U};
  std::uint8_t expected_vehicle_role_{msg::VehicleDestroyed::ROLE_UNSPECIFIED};
  std::unique_ptr<tf2_ros::TransformBroadcaster> rviz_drone_follow_tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_drone_marker_pub_;
  rclcpp::Publisher<msg::MppiControlFeedback>::SharedPtr applied_control_feedback_pub_;
  rclcpp::Publisher<msg::VehicleNavigationState>::SharedPtr navigation_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_readiness_pub_;
  // The blind descent of K19 under way, carried by the vehicle's state (A9).
  bool blind_landing_{false};
  rclcpp::Subscription<msg::MppiTrajectoryHorizon>::SharedPtr horizon_sub_;
  // The hop from the controller: what the transport took to deliver each
  // horizon, reported with the applied-horizon diagnostic.
  TransportLatencySamples horizon_delivery_ms_;
  std::unique_ptr<AutopilotStateSource> autopilot_state_source_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planner_health_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav
