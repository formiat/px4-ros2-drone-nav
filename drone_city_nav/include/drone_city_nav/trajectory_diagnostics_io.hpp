#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/trajectory_diagnostics.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/trajectory_speed_planner.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace drone_city_nav {

// Cross-node observability for one trajectory build. None of these values is
// consumed by planning, handover construction, or trajectory acceptance.
struct TrajectoryDeliveryDiagnostics {
  std::uint64_t generation{0U};
  std::uint64_t blocker_detected_stamp_ns{0U};
  std::uint64_t trajectory_build_started_stamp_ns{0U};
  std::uint64_t path_published_stamp_ns{0U};
  bool replan_triggered{false};
  Point2 blocker_position{};
  Point2 blocker_detection_position{};
  Point2 blocker_detection_velocity{};
  bool blocker_detection_velocity_valid{false};
  Point2 candidate_start_position{};
  Point2 planning_start_position{};
  Point2 planning_start_velocity{};
  bool planning_start_velocity_valid{false};
  Point2 predicted_publication_position{};
  bool predicted_publication_position_valid{false};
  Point2 actual_publication_position{};
  bool actual_publication_position_valid{false};
  double blocker_to_build_start_ms{std::numeric_limits<double>::quiet_NaN()};
  double build_start_to_publish_ms{std::numeric_limits<double>::quiet_NaN()};
  double blocker_to_publish_ms{std::numeric_limits<double>::quiet_NaN()};
  double publication_prediction_error_m{std::numeric_limits<double>::quiet_NaN()};
};

struct TrajectoryPlannerDiagnosticsEnvelope {
  std::uint64_t planner_path_id{0U};
  std::uint64_t path_stamp_ns{0U};
  TrajectoryPlannerStats stats{};
  TrajectoryDeliveryDiagnostics delivery{};
};

[[nodiscard]] std::string finalTrajectorySamplesCsvHeader();

[[nodiscard]] std::string
finalTrajectorySamplesCsvRow(std::size_t sample_index,
                             const TrajectoryPointSample& sample,
                             const TrajectorySpeedSample& speed_sample,
                             double time_from_start_s, double time_to_finish_s);

[[nodiscard]] std::string
trajectoryOptimizerDiagnosticsJsonFields(const TrajectoryPlannerStats& stats);

[[nodiscard]] std::string
turnSmoothingDiagnosticsJsonFields(const TrajectoryPlannerStats& stats);

[[nodiscard]] std::string
speedProfileConstraintDiagnosticsJsonFields(const TrajectoryPlannerStats& stats);

[[nodiscard]] std::string
trajectoryTimingDiagnosticsJsonFields(const TrajectoryPlannerStats& stats);

[[nodiscard]] std::string
finalTrajectoryDiagnosticsSummaryJson(const TrajectoryPlannerStats& stats,
                                      const TrajectoryShapeDiagnostics& shape);

[[nodiscard]] std::string
trajectoryPlannerDiagnosticsJson(std::uint64_t planner_path_id,
                                 std::uint64_t path_stamp_ns,
                                 const TrajectoryPlannerStats& stats,
                                 const TrajectoryDeliveryDiagnostics& delivery = {});

[[nodiscard]] std::optional<TrajectoryPlannerDiagnosticsEnvelope>
parseTrajectoryPlannerDiagnosticsJson(const std::string& json);

} // namespace drone_city_nav
