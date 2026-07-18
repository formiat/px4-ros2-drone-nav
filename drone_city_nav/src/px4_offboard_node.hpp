#pragma once

#include "drone_city_nav/final_trajectory_debug_io.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/offboard_blackbox.hpp"
#include "drone_city_nav/offboard_debug_markers.hpp"
#include "drone_city_nav/offboard_path_follower.hpp"
#include "drone_city_nav/offboard_trajectory_state.hpp"
#include "drone_city_nav/offboard_velocity_follower.hpp"
#include "drone_city_nav/offboard_vertical_follower.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/px4_offboard_node_config.hpp"
#include "drone_city_nav/px4_offboard_setpoint_io.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/route_diagnostics.hpp"
#include "drone_city_nav/terminal_capture_state_machine.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_diagnostics_io.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/trajectory_update_continuity.hpp"
#include "drone_city_nav/types.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

namespace drone_city_nav {

inline constexpr auto kControllerPeriod = std::chrono::milliseconds{50};
inline constexpr double kTinyDistanceM = 1.0e-6;
inline constexpr double kProhibitedGridClearanceDiagnosticRadiusM = 12.0;
inline constexpr std::int8_t kInflatedOccupancyValue = 80;
inline constexpr double kRvizGroundZ = 0.08;

[[nodiscard]] inline double radiansToDegrees(const double radians) noexcept {
  return radians * 180.0 / std::numbers::pi;
}

[[nodiscard]] inline double radiansFromDegrees(const double degrees) noexcept {
  return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] inline double normalizeAngle(const double angle_rad) noexcept {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

inline void writeJsonBool(std::ostream& stream, const bool value) {
  writeBlackboxJsonBool(stream, value);
}

inline void writeJsonNumberOrNull(std::ostream& stream, const double value) {
  writeBlackboxJsonNumberOrNull(stream, value);
}

struct PathTrackingDiagnostics {
  bool valid{false};
  std::size_t segment_start_index{0U};
  double segment_t{0.0};
  double cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double signed_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double path_heading_rad{std::numeric_limits<double>::quiet_NaN()};
  double heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
  Point2 projection{};
  double projection_z_m{std::numeric_limits<double>::quiet_NaN()};
};

struct NearestProhibitedCellDiagnostic {
  bool valid{false};
  double clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double bearing_map_rad{std::numeric_limits<double>::quiet_NaN()};
  double bearing_body_rad{std::numeric_limits<double>::quiet_NaN()};
  double bearing_body_deg{std::numeric_limits<double>::quiet_NaN()};
  Point2 point{};
};

class Px4OffboardNode final : public rclcpp::Node {
public:
  Px4OffboardNode();

  ~Px4OffboardNode() override = default;

  Px4OffboardNode(const Px4OffboardNode&) = delete;
  Px4OffboardNode& operator=(const Px4OffboardNode&) = delete;
  Px4OffboardNode(Px4OffboardNode&&) = delete;
  Px4OffboardNode& operator=(Px4OffboardNode&&) = delete;

private:
  [[nodiscard]] OffboardPathFollowerConfig pathFollowerConfig() const;

  void applyConfig(const Px4OffboardNodeConfig& config);

  [[nodiscard]] std_msgs::msg::Header makeDebugHeader() const;

  void publishFinalTrajectoryDebug();

  void publishOffboardDebugMarkers();

  void publishRvizDroneFollowTransform();

  void clearFinalTrajectory();

  [[nodiscard]] bool trajectoryDiagnosticsMatchesCurrentPath(
      const TrajectoryPlannerDiagnosticsEnvelope& diagnostics) const;

  void mergePlannerDiagnosticsIntoCurrentTrajectoryStats(
      const TrajectoryPlannerDiagnosticsEnvelope& diagnostics);

  void updatePlannerStatsForReceivedTrajectory();

  void resetVelocitySmootherState(const std::string_view reason,
                                  const bool count_path_update_reset);

  [[nodiscard]] bool receivedFinalTrajectoryIsFreshEnough(
      const OffboardTrajectoryState& state, std::uint64_t candidate_update_id,
      std::uint64_t candidate_path_stamp_ns, std::size_t candidate_path_points) const;

  [[nodiscard]] TrajectoryContinuityResult
  evaluateReceivedTrajectoryContinuity(const OffboardTrajectoryState& state) const;

  void applyReceivedFinalTrajectoryPath(const char* source_label,
                                        const OffboardTrajectoryState& state,
                                        const TrajectoryContinuityResult& continuity);

  void onPath(const nav_msgs::msg::Path& path);

  void onPathId(const std_msgs::msg::UInt64& msg);

  void onTrajectoryDiagnostics(const std_msgs::msg::String& msg);

  void openFlightBlackbox();

  [[nodiscard]] std::filesystem::path
  diagnosticDumpDirectory(const std::string_view name) const;

  [[nodiscard]] std::filesystem::path finalTrajectorySamplesDirectory() const;

  bool writeFinalTrajectorySamplesCsvFile(const std::filesystem::path& path,
                                          const char* source_label) const;

  bool writeFinalTrajectorySummaryJsonFile(const std::filesystem::path& path) const;

  [[nodiscard]] std::string
  writeFinalTrajectorySamplesCsv(const char* source_label) const;

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg);

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg);

  void onVehicleStatus(const px4_msgs::msg::VehicleStatus& msg);

  void onProhibitedGrid(const nav_msgs::msg::OccupancyGrid& msg);

  void onTimer();

  void publishOffboardControlMode();

  void publishTrajectorySetpoint();

  [[nodiscard]] OffboardSetpointMode currentSetpointMode() const;

  [[nodiscard]] bool velocityCruiseReady() const;

  using TerminalCaptureState = TerminalStateMachineDecision;

  [[nodiscard]] TerminalCaptureState computeTerminalCaptureState() const;

  void updateTerminalCaptureState();

  [[nodiscard]] bool finalPathGoalReached() const;

  [[nodiscard]] bool finalPathGoalPassed() const;

  void updateFinalGoalHold();

  [[nodiscard]] double consumeVelocityPlanDtS();

  void clearTerminalPositionCaptureAltitude();

  void latchTerminalPositionCaptureAltitude(const char* reason);

  [[nodiscard]] double
  positionSetpointAltitudeM(bool terminal_position_capture_requested) const;

  [[nodiscard]] bool
  positionSetpointAltitudeValid(bool terminal_position_capture_requested) const;

  [[nodiscard]] VerticalSetpointPlan
  planVerticalSetpointForCurrentTrajectory(const VelocitySetpointPlan& velocity_plan,
                                           double dt_s) const;

  bool publishVelocityTrajectorySetpoint();

  void resetVelocityDiagnostics();

  void publishVehicleCommand(const std::uint32_t command, const float param1 = 0.0F,
                             const float param2 = 0.0F);

  void advanceWaypointIfNeeded();

  [[nodiscard]] Point2 currentTarget() const;

  [[nodiscard]] Point2 selectCommandTarget(const Point2 desired_target,
                                           const bool hold_position);

  [[nodiscard]] bool shouldHoldPosition() const;

  [[nodiscard]] bool finalTrajectoryReady() const;

  [[nodiscard]] bool trajectoryGoalReady() const;

  [[nodiscard]] bool pathFollowingReady() const;

  [[nodiscard]] bool missionStartReady() const;

  [[nodiscard]] UpcomingTurn upcomingTurnAtWaypoint(const std::size_t index) const;

  [[nodiscard]] const char* pathSegmentTypeName(const double turn_angle_rad) const;

  [[nodiscard]] const char* motionPhaseName(const bool hold_position) const noexcept;

  [[nodiscard]] bool prohibitedGridFresh() const;

  [[nodiscard]] bool localPositionFresh() const;

  [[nodiscard]] std::optional<OccupancyGrid2D> currentProhibitedGrid() const;

  [[nodiscard]] double localPositionAgeSeconds() const;

  [[nodiscard]] double attitudeAgeSeconds() const;

  [[nodiscard]] PathTrackingDiagnostics pathTrackingDiagnostics() const;

  [[nodiscard]] double estimateProhibitedGridClearanceM(const Point2 point) const;

  [[nodiscard]] NearestProhibitedCellDiagnostic
  nearestProhibitedCellDiagnostic(const Point2 point,
                                  const std::int8_t min_occupancy_value) const;

  [[nodiscard]] double
  estimateGridClearanceM(const Point2 point,
                         const std::int8_t min_occupancy_value) const;

  void updateCommandDiagnostics(const Point2 target, const Point2 previous_target,
                                const bool had_previous_target,
                                const double commanded_yaw_rad);

  [[nodiscard]] Point2 mapToPx4Local(const Point2 point) const noexcept;

  void updateNavigationStartState();

  [[nodiscard]] bool navigationAllowed() const;

  [[nodiscard]] bool isArmed() const;

  [[nodiscard]] bool isOffboard() const;

  [[nodiscard]] std::uint64_t nowMicros() const;

  void logControlSummary();

  void logTelemetry();

  void
  writeFlightBlackbox(const std::int64_t now_ns, const Point2 target,
                      const double target_distance_m, const double path_goal_distance_m,
                      const double mission_goal_distance_m,
                      const double prohibited_grid_clearance_m, const bool pose_fresh,
                      const double pose_age_s, const double attitude_age_s,
                      const UpcomingTurn& upcoming_turn, const bool hold_position,
                      const PathTrackingDiagnostics& path_tracking,
                      const NearestProhibitedCellDiagnostic& nearest_prohibited_cell);

  [[nodiscard]] Point2 loggedTarget() const;

  nav_msgs::msg::OccupancyGrid prohibited_grid_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  AttitudeEuler current_attitude_{};
  Point2 current_position_{};
  Point2 current_velocity_{};
  Point2 no_path_hold_target_{};
  Point2 final_goal_hold_target_{};
  Point2 takeoff_hold_target_{};
  Point2 commanded_target_{};
  Point2 last_published_target_{};
  Point2 last_velocity_setpoint_{};
  Point2 mission_goal_{85.0, 0.0};
  Point2 trajectory_goal_{};
  Point2 px4_local_origin_{};
  double current_heading_rad_{0.0};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double initial_altitude_m_{12.0};
  double min_navigation_altitude_m_{0.0};
  double takeoff_hover_s_{2.0};
  double acceptance_radius_m_{1.5};
  double diagnostic_turn_preview_distance_m_{32.0};
  double altitude_hold_kp_{0.5};
  double max_vertical_speed_mps_{4.0};
  double command_resend_period_s_{2.0};
  double hold_x_m_{0.0};
  double hold_y_m_{0.0};
  bool rviz_drone_follow_tf_enabled_{true};
  double current_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_target_delta_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_target_distance_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double last_velocity_setpoint_speed_mps_{0.0};
  double last_vertical_velocity_setpoint_mps_{0.0};
  double last_target_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double terminal_position_capture_altitude_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_altitude_error_m_{std::numeric_limits<double>::quiet_NaN()};
  double final_trajectory_debug_sample_step_m_{1.0};
  double trajectory_update_max_start_cross_track_m_{8.0};
  std::int64_t max_clearance_grid_staleness_ns_{1'500'000'000};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t telemetry_log_period_ns_{500'000'000};
  std::int64_t last_prohibited_grid_update_ns_{0};
  std::int64_t last_attitude_update_ns_{0};
  std::int64_t last_local_position_update_ns_{0};
  std::int64_t last_telemetry_log_ns_{0};
  std::uint64_t latest_planner_path_id_{0U};
  std::uint64_t accepted_planner_path_id_{0U};
  std::uint64_t received_path_update_id_{0U};
  std::uint64_t last_received_path_stamp_ns_{0U};
  std::uint64_t path_update_velocity_smoother_reset_count_{0U};
  std::size_t waypoint_index_{0U};
  int warmup_setpoints_{20};
  int setpoint_counter_{0};
  bool path_valid_{false};
  bool trajectory_goal_valid_{false};
  bool local_position_valid_{false};
  bool vehicle_status_valid_{false};
  bool attitude_valid_{false};
  bool altitude_valid_{false};
  bool local_position_seen_{false};
  bool auto_arm_{true};
  bool auto_offboard_{true};
  bool prohibited_grid_valid_{false};
  bool prohibited_grid_seen_logged_{false};
  bool current_velocity_valid_{false};
  bool no_path_hold_target_valid_{false};
  bool final_goal_hold_active_{false};
  bool takeoff_hold_target_valid_{false};
  bool terminal_position_capture_latched_{false};
  bool terminal_position_capture_altitude_valid_{false};
  bool commanded_target_valid_{false};
  bool last_published_target_valid_{false};
  bool last_trajectory_altitude_target_valid_{false};
  bool navigation_altitude_reached_{false};
  bool navigation_started_{false};
  bool last_terminal_position_capture_active_{false};
  bool latest_planner_path_id_seen_{false};
  bool accepted_planner_path_id_seen_{false};
  bool flight_blackbox_enabled_{true};
  std::uint8_t target_system_{1U};
  std::uint8_t target_component_{1U};
  std::uint8_t source_system_{1U};
  std::uint16_t source_component_{1U};
  int last_logged_arming_state_{-1};
  int last_logged_nav_state_{-1};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  VelocityFollowerConfig velocity_follower_config_{};
  VelocityFollowerState velocity_follower_state_{};
  VerticalFollowerConfig vertical_follower_config_{};
  VerticalFollowerState vertical_follower_state_{};
  VelocitySetpointPlan last_velocity_plan_{};
  VerticalSetpointPlan last_vertical_plan_{};
  TrajectoryPlannerStats last_trajectory_planner_stats_{};
  std::optional<TrajectoryPlannerDiagnosticsEnvelope> latest_trajectory_diagnostics_;
  TrajectoryMetrics last_trajectory_metrics_{};
  TrajectoryShapeDiagnostics last_trajectory_shape_diagnostics_{};
  TrajectorySpeedProfile trajectory_speed_profile_{};
  bool last_velocity_plan_valid_{false};
  bool last_vertical_plan_valid_{false};
  bool trajectory_valid_{false};
  OffboardSetpointMode last_offboard_setpoint_mode_{
      OffboardSetpointMode::kPositionHold};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time navigation_altitude_reached_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_plan_time_{0, 0, RCL_ROS_TIME};
  std::string flight_blackbox_path_{"log/offboard_blackbox.jsonl"};
  std::string final_trajectory_debug_topic_{"/drone_city_nav/final_trajectory_path"};
  std::string offboard_debug_marker_topic_{"/drone_city_nav/offboard_debug_markers"};
  std::string rviz_drone_follow_parent_frame_{"gazebo_map"};
  std::string rviz_drone_follow_frame_{"drone_follow"};
  std::string last_velocity_smoother_reset_reason_{"none"};
  std::string last_terminal_position_capture_reason_{"none"};
  std::ofstream flight_blackbox_stream_;
  std::vector<Point2> path_points_;
  std::vector<TrajectorySegment> trajectory_;
  std::vector<TrajectoryPointSample> final_trajectory_samples_;
  std::size_t last_final_trajectory_debug_samples_{0U};
  std::size_t last_trajectory_route_points_{0U};
  TerminalCaptureState terminal_capture_state_{};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr path_id_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_diagnostics_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prohibited_grid_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr final_trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      offboard_debug_marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> rviz_drone_follow_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double last_terminal_position_capture_goal_distance_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_terminal_position_capture_remaining_s_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_terminal_position_capture_speed_mps_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_terminal_position_capture_activation_radius_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_terminal_position_capture_max_entry_speed_mps_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_terminal_position_capture_stuck_speed_mps_{
      std::numeric_limits<double>::quiet_NaN()};
};

} // namespace drone_city_nav
