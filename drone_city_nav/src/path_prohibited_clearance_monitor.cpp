#include "drone_city_nav/path_prohibited_clearance_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

struct SampleClearance {
  bool valid{false};
  double clearance_m{std::numeric_limits<double>::quiet_NaN()};
  GridIndex cell{};
};

[[nodiscard]] SampleClearance sampleClearance(const OccupancyGrid2D& grid,
                                              const ClearanceField2D& field,
                                              const Point2 point) {
  const std::optional<GridIndex> cell = grid.worldToCell(point);
  if (!cell.has_value() || !field.contains(*cell)) {
    return SampleClearance{};
  }
  return SampleClearance{true, field.distanceAt(*cell), *cell};
}

void populateNearestProhibitedCell(const OccupancyGrid2D& grid, const GridIndex query,
                                   const double search_radius_m,
                                   PathProhibitedClearanceViolation& violation) {
  const int radius_cells =
      static_cast<int>(std::ceil(std::max(0.0, search_radius_m) / grid.resolution()));
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const GridIndex candidate{query.x + dx, query.y + dy};
      if (!grid.contains(candidate) || !grid.isProhibited(candidate)) {
        continue;
      }
      const double distance_sq =
          squaredDistance(grid.cellCenter(query), grid.cellCenter(candidate));
      if (distance_sq >= nearest_distance_sq) {
        continue;
      }
      nearest_distance_sq = distance_sq;
      violation.nearest_prohibited_cell = candidate;
      violation.nearest_prohibited_cell_center = grid.cellCenter(candidate);
      violation.nearest_prohibited_cell_available = true;
    }
  }
}

} // namespace

PathProhibitedClearanceEvaluation evaluatePathProhibitedClearance(
    const OccupancyGrid2D& prohibited_grid,
    const ClearanceField2D& prohibited_clearance,
    const std::span<const Point2> remaining_path,
    const PathProhibitedClearanceMonitorConfig& input_config) {
  PathProhibitedClearanceEvaluation result{};
  if (remaining_path.size() < 2U || !(prohibited_grid.resolution() > 0.0) ||
      prohibited_clearance.source() != ClearanceSource::kProhibited ||
      prohibited_clearance.bounds().origin_x != prohibited_grid.originX() ||
      prohibited_clearance.bounds().origin_y != prohibited_grid.originY() ||
      prohibited_clearance.bounds().resolution_m != prohibited_grid.resolution() ||
      prohibited_clearance.bounds().width_cells != prohibited_grid.width() ||
      prohibited_clearance.bounds().height_cells != prohibited_grid.height()) {
    return result;
  }
  const PathProhibitedClearanceMonitorConfig config{
      .trigger_clearance_m = std::max(0.0, input_config.trigger_clearance_m),
      .arm_clearance_m =
          std::max(input_config.trigger_clearance_m, input_config.arm_clearance_m),
      .min_violation_length_m = std::max(0.0, input_config.min_violation_length_m),
      .sample_step_m = std::clamp(input_config.sample_step_m, 0.1, 5.0),
  };
  const SampleClearance current =
      sampleClearance(prohibited_grid, prohibited_clearance, remaining_path.front());
  if (!current.valid) {
    return result;
  }
  result.valid = true;
  result.current_clearance_m = current.clearance_m;
  result.current_position_arms =
      current.clearance_m + kTinyDistanceM >= config.arm_clearance_m;

  bool violation_active = false;
  double distance_m = 0.0;
  GridIndex entry_cell{};
  for (std::size_t segment_index = 1U; segment_index < remaining_path.size();
       ++segment_index) {
    const Point2 start = remaining_path[segment_index - 1U];
    const Point2 end = remaining_path[segment_index];
    const double segment_length_m = distance(start, end);
    if (!(segment_length_m > kTinyDistanceM)) {
      continue;
    }
    const std::size_t steps = std::max<std::size_t>(
        1U,
        static_cast<std::size_t>(std::ceil(segment_length_m / config.sample_step_m)));
    for (std::size_t step_index = 1U; step_index <= steps; ++step_index) {
      const double ratio = static_cast<double>(step_index) / static_cast<double>(steps);
      const Point2 point{start.x + (end.x - start.x) * ratio,
                         start.y + (end.y - start.y) * ratio};
      const double sample_distance_m = distance_m + segment_length_m * ratio;
      const SampleClearance sample =
          sampleClearance(prohibited_grid, prohibited_clearance, point);
      const bool below_trigger = sample.valid && sample.clearance_m + kTinyDistanceM <
                                                     config.trigger_clearance_m;
      if (!below_trigger) {
        violation_active = false;
        continue;
      }
      if (!violation_active) {
        violation_active = true;
        result.violation.entry_distance_m = sample_distance_m;
        result.violation.entry_point = point;
        result.violation.min_clearance_m = sample.clearance_m;
        entry_cell = sample.cell;
      }
      result.violation.length_m = sample_distance_m - result.violation.entry_distance_m;
      result.violation.min_clearance_m =
          std::min(result.violation.min_clearance_m, sample.clearance_m);
      if (result.violation.length_m + kTinyDistanceM >= config.min_violation_length_m) {
        result.violation.detected = true;
        populateNearestProhibitedCell(prohibited_grid, entry_cell,
                                      config.trigger_clearance_m, result.violation);
        return result;
      }
    }
    distance_m += segment_length_m;
  }
  return result;
}

} // namespace drone_city_nav
