#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>

namespace drone_city_nav {

struct OffboardPathFollowerConfig {
  double acceptance_radius_m{1.5};
  double lookahead_distance_m{6.0};
  double lookahead_time_s{1.2};
  double min_lookahead_distance_m{6.0};
  double max_lookahead_distance_m{6.0};
  double path_switch_hysteresis_m{3.0};
  double path_continuity_reuse_radius_m{6.0};
  double path_continuity_max_target_distance_m{20.0};
  double max_setpoint_distance_m{2.0};
  bool dynamic_lookahead_enabled{true};
};

struct OffboardPathProjection {
  std::size_t segment_start_index{0U};
  double segment_t{0.0};
  double distance_sq{std::numeric_limits<double>::infinity()};
  Point2 point{};
};

struct CommandTargetState {
  bool valid{false};
  Point2 target{};
};

[[nodiscard]] double
effectiveLookaheadDistanceM(const OffboardPathFollowerConfig& config,
                            double desired_speed_mps) noexcept;

[[nodiscard]] std::size_t closestWaypointIndex(std::span<const Point2> path,
                                               Point2 current_position);

[[nodiscard]] std::size_t
lookaheadWaypointIndex(std::span<const Point2> path, Point2 current_position,
                       Point2 mission_goal, const OffboardPathFollowerConfig& config,
                       double desired_speed_mps);

[[nodiscard]] std::optional<OffboardPathProjection>
closestOffboardPathProjection(std::span<const Point2> path, Point2 current_position);

[[nodiscard]] Point2 lookaheadTargetOnPath(std::span<const Point2> path,
                                           Point2 current_position,
                                           std::size_t waypoint_index,
                                           const OffboardPathFollowerConfig& config,
                                           double desired_speed_mps);

[[nodiscard]] Point2 targetOnPathAtDistance(std::span<const Point2> path,
                                            Point2 current_position,
                                            double path_distance_m);

[[nodiscard]] std::size_t
continuityWaypointIndex(std::span<const Point2> path, Point2 current_position,
                        Point2 previous_target, std::size_t candidate_index,
                        bool had_active_target,
                        const OffboardPathFollowerConfig& config);

[[nodiscard]] std::size_t
advanceWaypointIndex(std::span<const Point2> path, Point2 current_position,
                     std::size_t waypoint_index,
                     const OffboardPathFollowerConfig& config);

[[nodiscard]] Point2 limitedTarget(Point2 target, Point2 current_position,
                                   bool local_position_valid,
                                   double max_setpoint_distance_m);

[[nodiscard]] Point2 smoothedCommandTarget(Point2 desired_target, double target_step_m,
                                           bool snap_to_desired_target,
                                           Point2 current_position,
                                           bool local_position_valid,
                                           double max_setpoint_distance_m,
                                           CommandTargetState& state);
[[nodiscard]] Point2
enforceMinimumTargetLead(Point2 command_target, Point2 desired_target,
                         Point2 current_position, bool local_position_valid,
                         double minimum_target_lead_m, double max_setpoint_distance_m);

[[nodiscard]] double pathTurnAngleAtWaypoint(std::span<const Point2> path,
                                             std::size_t index, Point2 current_position,
                                             bool local_position_valid,
                                             const OffboardPathFollowerConfig& config,
                                             double desired_speed_mps);

} // namespace drone_city_nav
