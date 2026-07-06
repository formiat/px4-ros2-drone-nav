#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class TrajectorySegmentKind {
  kLine,
  kArc,
};

struct TrajectorySegment {
  TrajectorySegmentKind kind{TrajectorySegmentKind::kLine};
  Point2 start{};
  Point2 end{};
  Point2 center{};
  double radius_m{std::numeric_limits<double>::quiet_NaN()};
  double start_angle_rad{std::numeric_limits<double>::quiet_NaN()};
  double sweep_rad{0.0};
  double s_start_m{0.0};
  double length_m{0.0};
};

struct TrajectoryProjection {
  bool valid{false};
  std::size_t segment_index{0U};
  double segment_t{0.0};
  double s_m{0.0};
  Point2 point{};
  Point2 tangent{};
  double curvature_1pm{0.0};
  double distance_sq{std::numeric_limits<double>::infinity()};
};

struct TrajectoryMetrics {
  std::size_t line_segments{0U};
  std::size_t arc_segments{0U};
  double length_m{0.0};
};

struct TrajectoryAltitudeStats {
  bool valid{false};
  double min_z_m{std::numeric_limits<double>::quiet_NaN()};
  double max_z_m{std::numeric_limits<double>::quiet_NaN()};
  double mean_z_m{std::numeric_limits<double>::quiet_NaN()};
};

struct TrajectoryPointSample {
  double s_m{0.0};
  Point2 point{};
  Point2 tangent{};
  double curvature_1pm{0.0};
  double z_m{0.0};
  double vertical_slope_dz_ds{0.0};
  double vertical_speed_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double vertical_accel_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  double vertical_jerk_limit_mps{std::numeric_limits<double>::quiet_NaN()};
  bool vertical_constraint_active{false};
  double left_bound_m{std::numeric_limits<double>::quiet_NaN()};
  double right_bound_m{std::numeric_limits<double>::quiet_NaN()};
  double lateral_offset_m{std::numeric_limits<double>::quiet_NaN()};
  std::string vertical_profile_passage_id;
};

[[nodiscard]] const char*
trajectorySegmentKindName(TrajectorySegmentKind kind) noexcept;

[[nodiscard]] TrajectorySegment makeLineSegment(Point2 start, Point2 end);

[[nodiscard]] TrajectorySegment makeArcSegment(Point2 start, Point2 end, Point2 center,
                                               double sweep_rad);

void assignTrajectoryStationing(std::vector<TrajectorySegment>& trajectory);

[[nodiscard]] std::vector<TrajectorySegment>
lineTrajectoryFromPoints(std::span<const Point2> points);

[[nodiscard]] std::vector<TrajectorySegment>
lineTrajectoryFromSamples(std::span<const TrajectoryPointSample> samples);

[[nodiscard]] bool trajectoryIsUsable(std::span<const TrajectorySegment> trajectory);

[[nodiscard]] bool
trajectorySamplesAreUsable(std::span<const TrajectoryPointSample> samples);

void assignTrajectorySampleAltitude(std::span<TrajectoryPointSample> samples,
                                    double altitude_m);

[[nodiscard]] double
trajectorySampleAltitudeAtS(std::span<const TrajectoryPointSample> samples, double s_m);

[[nodiscard]] TrajectoryAltitudeStats
trajectoryAltitudeStats(std::span<const TrajectoryPointSample> samples);

[[nodiscard]] TrajectoryMetrics
trajectoryMetrics(std::span<const TrajectorySegment> trajectory);

[[nodiscard]] double trajectoryLengthM(std::span<const TrajectorySegment> trajectory);

[[nodiscard]] Point2 trajectoryPointAtS(std::span<const TrajectorySegment> trajectory,
                                        double s_m);

[[nodiscard]] Point2 trajectoryTangentAtS(std::span<const TrajectorySegment> trajectory,
                                          double s_m);

[[nodiscard]] double
trajectoryCurvatureAtS(std::span<const TrajectorySegment> trajectory, double s_m);

[[nodiscard]] std::optional<TrajectoryProjection>
projectOnTrajectory(std::span<const TrajectorySegment> trajectory, Point2 point,
                    double minimum_s_m = 0.0);

[[nodiscard]] std::optional<TrajectoryProjection>
projectOnTrajectorySamples(std::span<const TrajectoryPointSample> samples, Point2 point,
                           double minimum_s_m = 0.0);

[[nodiscard]] std::vector<Point2>
sampleTrajectory(std::span<const TrajectorySegment> trajectory, double step_m);

[[nodiscard]] std::vector<TrajectoryPointSample>
sampleTrajectoryDetailed(std::span<const TrajectorySegment> trajectory, double step_m);

[[nodiscard]] std::vector<TrajectoryPointSample>
trajectoryPointSamplesFromPoints(std::span<const Point2> points);

} // namespace drone_city_nav
