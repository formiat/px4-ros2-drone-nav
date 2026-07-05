#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_diagnostics_io.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"
#include "drone_city_nav/trajectory_update_continuity.hpp"
#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/path.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

struct OffboardTrajectoryState {
  std::vector<TrajectoryPointSample> samples;
  std::vector<TrajectorySegment> trajectory;
  TrajectorySpeedProfile speed_profile;
  TrajectoryMetrics metrics{};
  TrajectoryShapeDiagnostics shape{};
  TrajectoryPlannerStats stats{};
  bool valid{false};
};

[[nodiscard]] std::uint64_t
messageStampNanoseconds(const builtin_interfaces::msg::Time& stamp);

[[nodiscard]] std::vector<Point2>
pathPointsFromMessage(const nav_msgs::msg::Path& path);

[[nodiscard]] bool trajectoryDiagnosticsMatchesPath(
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics,
    std::uint64_t expected_path_stamp_ns, bool planner_path_id_seen,
    std::uint64_t expected_planner_path_id);

void mergePlannerDiagnosticsIntoTrajectoryStats(
    TrajectoryPlannerStats& output_stats,
    const TrajectoryPlannerDiagnosticsEnvelope& diagnostics);

[[nodiscard]] TrajectoryPlannerStats buildReceivedTrajectoryPlannerStats(
    std::span<const Point2> route_points,
    std::span<const TrajectoryPointSample> samples,
    std::span<const TrajectorySegment> trajectory, const TrajectoryMetrics& metrics,
    const TrajectorySpeedProfile& speed_profile,
    const VelocityFollowerConfig& velocity_config, bool trajectory_valid);

[[nodiscard]] OffboardTrajectoryState
buildOffboardTrajectoryState(std::span<const Point2> path_points,
                             const VelocityFollowerConfig& velocity_config);

[[nodiscard]] TrajectoryContinuityResult evaluateOffboardTrajectoryUpdateContinuity(
    std::span<const TrajectoryPointSample> current_samples,
    const TrajectorySpeedProfile& current_speed_profile,
    const OffboardTrajectoryState& candidate_state, Point2 current_position,
    Point2 previous_velocity_setpoint, bool previous_velocity_setpoint_valid,
    bool local_position_fresh);

} // namespace drone_city_nav
