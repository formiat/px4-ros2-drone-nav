#include "drone_city_nav/offboard_blackbox.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>
#include <string_view>

namespace drone_city_nav {
namespace {

void expectJsonField(const std::string& json, const std::string_view needle) {
  EXPECT_NE(json.find(needle), std::string::npos) << "missing field: " << needle;
}

} // namespace

TEST(OffboardBlackbox, WritesJsonPrimitives) {
  std::ostringstream stream;

  writeBlackboxJsonBool(stream, true);
  stream << ",";
  writeBlackboxJsonBool(stream, false);
  stream << ",";
  writeBlackboxJsonNumberOrNull(stream, 4.25);
  stream << ",";
  writeBlackboxJsonNumberOrNull(stream, std::numeric_limits<double>::quiet_NaN());
  stream << ",";
  writeBlackboxStringField(stream, "mode", "velocity_cruise");

  EXPECT_EQ(stream.str(), "true,false,4.25,null,\"mode\":\"velocity_cruise\"");
}

TEST(OffboardBlackbox, WritesPathIdContract) {
  std::ostringstream stream;

  writeBlackboxPathId(stream, OffboardBlackboxPathId{7U, 42U, true, 123456U});

  EXPECT_EQ(stream.str(), "\"path_id\":{\"local_update\":7,\"planner\":42,"
                          "\"planner_seen\":true,\"stamp_ns\":123456}");
}

TEST(OffboardBlackbox, WritesFullRecordJsonLine) {
  OffboardBlackboxRecord record;
  record.time_ns = 123456789;
  record.path_id = OffboardBlackboxPathId{7U, 42U, true, 987U};
  record.pose_fresh = true;
  record.pose_age_s = 0.25;
  record.current_position = Point2{1.0, 2.0};
  record.current_altitude_m = 12.0;
  record.current_heading_rad = 0.5;
  record.attitude_valid = true;
  record.attitude_age_s = 0.1;
  record.current_attitude = AttitudeEuler{0.1, 0.2, 0.3};
  record.current_velocity_valid = true;
  record.current_velocity = Point2{3.0, 4.0};
  record.current_speed_mps = 5.0;
  record.target = Point2{10.0, 11.0};
  record.target_distance_m = 6.0;
  record.last_commanded_target_delta_m = 0.4;
  record.last_commanded_yaw_rad = 0.9;
  record.control_mode = "velocity";
  record.last_velocity_setpoint = Point2{8.0, 9.0};
  record.last_vertical_velocity_setpoint_mps = -0.5;
  record.last_velocity_setpoint_speed_mps = 12.0;
  record.velocity_plan.valid = true;
  record.velocity_plan.reason = VelocitySetpointReason::kTrajectorySpeedProfile;
  record.velocity_plan.final_command_speed_mps = 11.0;
  record.velocity_plan.profile_speed_limit_mps = 13.0;
  record.velocity_plan.limiting_constraint_type = SpeedConstraintType::kArc;
  record.velocity_plan.trajectory_segment_kind = TrajectorySegmentKind::kArc;
  record.velocity_plan.trajectory_curvature_1pm = 0.1;
  record.velocity_plan.trajectory_arc_radius_m = 10.0;
  record.velocity_smoother_reset_reason = "path_update";
  record.path_update_velocity_smoother_reset_count = 3U;
  record.last_altitude_error_m = 0.2;
  record.trajectory_valid = true;
  record.trajectory_metrics.length_m = 100.0;
  record.trajectory_metrics.line_segments = 2U;
  record.trajectory_metrics.arc_segments = 1U;
  record.final_trajectory_samples = 25U;
  record.trajectory_planner_stats.samples = 25U;
  record.trajectory_planner_stats.status = TrajectoryPlannerStatus::kOk;
  record.trajectory_planner_stats.corridor.samples = 12U;
  record.trajectory_planner_stats.corridor.min_width_m = 5.0;
  record.trajectory_planner_stats.racing_line.iterations = 4U;
  record.trajectory_planner_stats.racing_line.optimizer_samples = 8U;
  record.trajectory_planner_stats.racing_line.final_cost = 1.5;
  record.trajectory_planner_stats.racing_line.candidate_point_build_duration_ms = 1.25;
  record.trajectory_planner_stats.racing_line.scratch_reused_candidates = 7U;
  record.trajectory_planner_stats.racing_line.parallel_candidate_evaluation_used = true;
  record.trajectory_shape_diagnostics.segment_count = 24U;
  record.trajectory_shape_diagnostics.max_heading_delta_rad = 0.3;
  record.path_valid = true;
  record.waypoint_index = 1U;
  record.waypoint_count = 25U;
  record.path_goal_distance_m = 7.0;
  record.mission_goal_distance_m = 8.0;
  record.upcoming_turn.valid = true;
  record.upcoming_turn.waypoint_index = 2U;
  record.upcoming_turn.distance_to_turn_m = 14.0;
  record.upcoming_turn.angle_rad = 1.0;
  record.upcoming_turn.turn_point = Point2{15.0, 16.0};
  record.final_trajectory_segment_type = "turn";
  record.path_tracking.valid = true;
  record.path_tracking.cross_track_error_m = 0.7;
  record.path_tracking.projection = Point2{3.0, 4.0};
  record.motion_phase = "cruise";
  record.final_goal_hold_active = false;
  record.prohibited_grid_clearance_m = 9.0;
  record.nearest_prohibited_cell.valid = true;
  record.nearest_prohibited_cell.clearance_m = 4.0;
  record.nearest_prohibited_cell.point = Point2{20.0, 21.0};

  std::ostringstream stream;
  writeOffboardBlackboxRecord(stream, record);
  const std::string json = stream.str();

  ASSERT_FALSE(json.empty());
  EXPECT_EQ(json.front(), '{');
  EXPECT_EQ(json.back(), '\n');
  expectJsonField(json, "\"time_ns\":123456789");
  expectJsonField(json, "\"path_id\":{\"local_update\":7,\"planner\":42");
  expectJsonField(json, "\"pose\":{\"fresh\":true");
  expectJsonField(json, "\"x\":1");
  expectJsonField(json, "\"attitude\":{\"valid\":true");
  expectJsonField(json, "\"velocity\":{\"valid\":true");
  expectJsonField(json, "\"target\":{\"x\":10");
  expectJsonField(json, "\"velocity_command\":{\"control_mode\":\"velocity\"");
  expectJsonField(json, "\"speed_limit_reason\":\"trajectory_profile\"");
  expectJsonField(json, "\"trajectory_segment_type\":\"arc\"");
  expectJsonField(json, "\"trajectory_planner_status\":\"none\"");
  expectJsonField(json, "\"corridor_samples\":12");
  expectJsonField(json, "\"racing_line_iterations\":4");
  expectJsonField(json, "\"racing_candidate_point_build_duration_ms\":1.25");
  expectJsonField(json, "\"racing_scratch_reused_candidates\":7");
  expectJsonField(json, "\"racing_parallel_candidate_evaluation_used\":true");
  expectJsonField(json, "\"trajectory_shape_segment_count\":24");
  expectJsonField(json, "\"path\":{\"valid\":true");
  expectJsonField(json, "\"final_trajectory_debug_segment_type\":\"turn\"");
  expectJsonField(json, "\"tracking\":{\"valid\":true");
  expectJsonField(json, "\"control\":{\"motion_phase\":\"cruise\"");
  expectJsonField(json, "\"obstacle\":{\"prohibited_grid_clearance_m\":9");
  expectJsonField(json, "\"nearest_prohibited_cell_valid\":true");
}

} // namespace drone_city_nav
