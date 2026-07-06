#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/offboard_velocity_follower.hpp"
#include "drone_city_nav/route_diagnostics.hpp"
#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <limits>
#include <string>
#include <string_view>

namespace drone_city_nav {

struct OffboardBlackboxPathId {
  std::uint64_t local_update{0U};
  std::uint64_t planner{0U};
  bool planner_seen{false};
  std::uint64_t stamp_ns{0U};
};

struct OffboardBlackboxPathTracking {
  bool valid{false};
  std::size_t segment_start_index{0U};
  double segment_t{0.0};
  double cross_track_error_m{0.0};
  double signed_cross_track_error_m{0.0};
  double path_heading_rad{0.0};
  double heading_error_rad{0.0};
  Point2 projection{};
  double projection_z_m{std::numeric_limits<double>::quiet_NaN()};
};

struct OffboardBlackboxNearestProhibitedCell {
  bool valid{false};
  double clearance_m{0.0};
  double bearing_map_rad{0.0};
  double bearing_body_rad{0.0};
  double bearing_body_deg{0.0};
  Point2 point{};
};

struct OffboardBlackboxRecord {
  std::int64_t time_ns{0};
  OffboardBlackboxPathId path_id{};
  bool pose_fresh{false};
  double pose_age_s{0.0};
  Point2 current_position{};
  double current_altitude_m{0.0};
  double current_heading_rad{0.0};
  bool attitude_valid{false};
  double attitude_age_s{0.0};
  AttitudeEuler current_attitude{};
  bool current_velocity_valid{false};
  Point2 current_velocity{};
  double current_speed_mps{0.0};
  Point2 target{};
  double target_distance_m{0.0};
  double last_commanded_target_delta_m{0.0};
  double last_commanded_yaw_rad{0.0};
  std::string control_mode;
  Point2 last_velocity_setpoint{};
  double last_vertical_velocity_setpoint_mps{0.0};
  double last_velocity_setpoint_speed_mps{0.0};
  VelocitySetpointPlan velocity_plan{};
  std::string velocity_smoother_reset_reason;
  std::uint64_t path_update_velocity_smoother_reset_count{0U};
  double target_altitude_m{std::numeric_limits<double>::quiet_NaN()};
  bool trajectory_altitude_target_valid{false};
  double last_altitude_error_m{0.0};
  bool trajectory_valid{false};
  TrajectoryMetrics trajectory_metrics{};
  std::size_t final_trajectory_samples{0U};
  TrajectoryPlannerStats trajectory_planner_stats{};
  TrajectoryShapeDiagnostics trajectory_shape_diagnostics{};
  bool path_valid{false};
  std::size_t waypoint_index{0U};
  std::size_t waypoint_count{0U};
  double path_goal_distance_m{0.0};
  double mission_goal_distance_m{0.0};
  UpcomingTurn upcoming_turn{};
  std::string rough_route_debug_segment_type;
  OffboardBlackboxPathTracking path_tracking{};
  std::string motion_phase;
  bool final_goal_hold_active{false};
  std::string terminal_state;
  bool terminal_position_capture_active{false};
  std::string terminal_position_capture_reason;
  double terminal_position_capture_goal_distance_m{
      std::numeric_limits<double>::quiet_NaN()};
  double terminal_position_capture_remaining_s_m{
      std::numeric_limits<double>::quiet_NaN()};
  double terminal_position_capture_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double terminal_position_capture_activation_radius_m{
      std::numeric_limits<double>::quiet_NaN()};
  double terminal_position_capture_max_entry_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double terminal_position_capture_stuck_speed_mps{
      std::numeric_limits<double>::quiet_NaN()};
  double prohibited_grid_clearance_m{0.0};
  OffboardBlackboxNearestProhibitedCell nearest_prohibited_cell{};
};

void writeBlackboxJsonBool(std::ostream& stream, bool value);

void writeBlackboxJsonNumberOrNull(std::ostream& stream, double value);

void writeBlackboxPathId(std::ostream& stream, const OffboardBlackboxPathId& path_id);

void writeBlackboxStringField(std::ostream& stream, std::string_view key,
                              std::string_view value);

void writeOffboardBlackboxRecord(std::ostream& stream,
                                 const OffboardBlackboxRecord& record);

} // namespace drone_city_nav
