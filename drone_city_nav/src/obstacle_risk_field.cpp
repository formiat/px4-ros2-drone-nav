#include "drone_city_nav/obstacle_risk_field.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kComparisonEpsilon = 1.0e-9;

[[nodiscard]] double finiteOrInfinity(const double value) noexcept {
  return std::isfinite(value) ? value : std::numeric_limits<double>::infinity();
}

[[nodiscard]] bool lessDouble(const double lhs, const double rhs) noexcept {
  return finiteOrInfinity(lhs) + kComparisonEpsilon < finiteOrInfinity(rhs);
}

[[nodiscard]] bool equalDouble(const double lhs, const double rhs) noexcept {
  if (std::isfinite(lhs) && std::isfinite(rhs)) {
    return std::abs(lhs - rhs) <= kComparisonEpsilon;
  }
  return std::isinf(lhs) && std::isinf(rhs);
}

[[nodiscard]] bool containsPoint(const GridBounds& bounds,
                                 const Point2 point) noexcept {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !(bounds.resolution_m > 0.0) || bounds.width_cells <= 0 ||
      bounds.height_cells <= 0) {
    return false;
  }
  const double max_x =
      bounds.origin_x + bounds.resolution_m * static_cast<double>(bounds.width_cells);
  const double max_y =
      bounds.origin_y + bounds.resolution_m * static_cast<double>(bounds.height_cells);
  return point.x >= bounds.origin_x && point.y >= bounds.origin_y && point.x < max_x &&
         point.y < max_y;
}

void mergeWorstTier(PathRiskScore& score, const ObstacleRiskTier tier) noexcept {
  if (static_cast<std::uint8_t>(tier) > static_cast<std::uint8_t>(score.worst_tier)) {
    score.worst_tier = tier;
  }
}

void classifyInterval(const ObstacleRiskField& field, const OccupancyGrid2D& raw_grid,
                      const Point2 point, const double length_m, PathRiskScore& score) {
  if (!containsPoint(field.evaluationBounds(), point)) {
    score.outside_bounds = true;
    return;
  }
  const std::optional<GridIndex> cell = raw_grid.worldToCell(point);
  if (!cell.has_value()) {
    score.outside_bounds = true;
    return;
  }
  if (raw_grid.isOccupied(*cell)) {
    score.intersects_raw_occupied = true;
    score.minimum_raw_clearance_m = 0.0;
    return;
  }
  const double clearance_m = field.occupiedDistance().distanceAt(*cell);
  score.minimum_raw_clearance_m = std::min(score.minimum_raw_clearance_m, clearance_m);
  const ObstacleRiskTier tier = field.tierAt(*cell);
  mergeWorstTier(score, tier);
  if (tier == ObstacleRiskTier::kCriticalBand) {
    score.critical_exposure_m += length_m;
  } else if (tier == ObstacleRiskTier::kPlanningBand) {
    score.planning_exposure_m += length_m;
  }
}

template<typename PointAccessor>
[[nodiscard]] PathRiskScore
evaluatePoints(const ObstacleRiskField& field, const OccupancyGrid2D& raw_grid,
               const std::size_t size, PointAccessor point_at) {
  PathRiskScore score{};
  if (size == 0U) {
    score.outside_bounds = true;
    return score;
  }

  const Point2 first = point_at(0U);
  classifyInterval(field, raw_grid, first, 0.0, score);
  for (std::size_t index = 0U; index + 1U < size; ++index) {
    const Point2 start = point_at(index);
    const Point2 end = point_at(index + 1U);
    const double segment_length_m = distance(start, end);
    if (!std::isfinite(segment_length_m)) {
      score.outside_bounds = true;
      continue;
    }
    const double sample_step_m = std::max(0.05, 0.5 * raw_grid.resolution());
    const std::size_t intervals = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(segment_length_m / sample_step_m)));
    const double interval_length_m =
        intervals > 0U ? segment_length_m / static_cast<double>(intervals) : 0.0;
    for (std::size_t interval = 0U; interval < intervals; ++interval) {
      const double ratio =
          (static_cast<double>(interval) + 0.5) / static_cast<double>(intervals);
      const Point2 midpoint{start.x + ((end.x - start.x) * ratio),
                            start.y + ((end.y - start.y) * ratio)};
      classifyInterval(field, raw_grid, midpoint, interval_length_m, score);
    }

    const std::optional<GridIndex> start_cell = raw_grid.worldToCell(start);
    const std::optional<GridIndex> end_cell = raw_grid.worldToCell(end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      score.outside_bounds = true;
      continue;
    }
    for (const GridIndex cell : raw_grid.cellsOnLine(*start_cell, *end_cell)) {
      if (!containsPoint(field.evaluationBounds(), raw_grid.cellCenter(cell))) {
        score.outside_bounds = true;
        continue;
      }
      if (raw_grid.isOccupied(cell)) {
        score.intersects_raw_occupied = true;
        score.minimum_raw_clearance_m = 0.0;
        break;
      }
    }
  }
  return score;
}

} // namespace

ObstacleRiskField ObstacleRiskField::build(const OccupancyGrid2D& raw_grid,
                                           ObstacleRiskPolicy policy) {
  return build(raw_grid, policy, raw_grid.bounds());
}

ObstacleRiskField ObstacleRiskField::build(const OccupancyGrid2D& raw_grid,
                                           ObstacleRiskPolicy policy,
                                           const GridBounds& evaluation_bounds) {
  if (!std::isfinite(policy.critical_distance_m) ||
      !std::isfinite(policy.preferred_distance_m)) {
    throw std::invalid_argument{"Obstacle risk distances must be finite"};
  }
  policy.critical_distance_m = std::max(0.0, policy.critical_distance_m);
  policy.preferred_distance_m =
      std::max(policy.critical_distance_m, policy.preferred_distance_m);

  ObstacleRiskField field;
  field.policy_ = policy;
  field.evaluation_bounds_ = evaluation_bounds;
  field.occupied_distance_ = DistanceField2D::build(
      raw_grid, policy.preferred_distance_m + raw_grid.resolution(),
      DistanceFieldSource::kOccupied);
  return field;
}

const ObstacleRiskPolicy& ObstacleRiskField::policy() const noexcept {
  return policy_;
}

const GridBounds& ObstacleRiskField::evaluationBounds() const noexcept {
  return evaluation_bounds_;
}

const DistanceField2D& ObstacleRiskField::occupiedDistance() const noexcept {
  return occupied_distance_;
}

ObstacleRiskTier ObstacleRiskField::tierAt(const GridIndex cell) const {
  const double clearance_m = occupied_distance_.distanceAt(cell);
  if (clearance_m < policy_.critical_distance_m) {
    return ObstacleRiskTier::kCriticalBand;
  }
  if (clearance_m < policy_.preferred_distance_m) {
    return ObstacleRiskTier::kPlanningBand;
  }
  return ObstacleRiskTier::kPreferred;
}

PathRiskScore ObstacleRiskField::evaluate(const OccupancyGrid2D& raw_grid,
                                          const std::span<const Point2> points) const {
  return evaluatePoints(*this, raw_grid, points.size(),
                        [&points](const std::size_t index) { return points[index]; });
}

PathRiskScore ObstacleRiskField::evaluate(
    const OccupancyGrid2D& raw_grid,
    const std::span<const TrajectoryPointSample> samples) const {
  return evaluatePoints(
      *this, raw_grid, samples.size(),
      [&samples](const std::size_t index) { return samples[index].point; });
}

bool pathRiskLess(const PathRiskScore& lhs, const PathRiskScore& rhs) noexcept {
  if (lhs.hardValid() != rhs.hardValid()) {
    return lhs.hardValid();
  }
  if (lhs.outside_bounds != rhs.outside_bounds) {
    return !lhs.outside_bounds;
  }
  if (lhs.intersects_raw_occupied != rhs.intersects_raw_occupied) {
    return !lhs.intersects_raw_occupied;
  }
  if (lhs.worst_tier != rhs.worst_tier) {
    return static_cast<std::uint8_t>(lhs.worst_tier) <
           static_cast<std::uint8_t>(rhs.worst_tier);
  }
  if (lessDouble(lhs.critical_exposure_m, rhs.critical_exposure_m)) {
    return true;
  }
  if (lessDouble(rhs.critical_exposure_m, lhs.critical_exposure_m)) {
    return false;
  }
  return lessDouble(lhs.planning_exposure_m, rhs.planning_exposure_m);
}

bool pathRiskEqual(const PathRiskScore& lhs, const PathRiskScore& rhs) noexcept {
  return lhs.outside_bounds == rhs.outside_bounds &&
         lhs.intersects_raw_occupied == rhs.intersects_raw_occupied &&
         lhs.worst_tier == rhs.worst_tier &&
         equalDouble(lhs.critical_exposure_m, rhs.critical_exposure_m) &&
         equalDouble(lhs.planning_exposure_m, rhs.planning_exposure_m);
}

bool rankedPathCostLess(const RankedPathCost& lhs, const RankedPathCost& rhs) noexcept {
  if (!pathRiskEqual(lhs.risk, rhs.risk)) {
    return pathRiskLess(lhs.risk, rhs.risk);
  }
  if (lessDouble(lhs.algorithm_cost, rhs.algorithm_cost)) {
    return true;
  }
  if (lessDouble(rhs.algorithm_cost, lhs.algorithm_cost)) {
    return false;
  }
  return lhs.deterministic_tiebreak < rhs.deterministic_tiebreak;
}

const char* obstacleRiskTierName(const ObstacleRiskTier tier) noexcept {
  switch (tier) {
    case ObstacleRiskTier::kPreferred:
      return "preferred";
    case ObstacleRiskTier::kPlanningBand:
      return "planning_band";
    case ObstacleRiskTier::kCriticalBand:
      return "critical_band";
  }
  return "unknown";
}

} // namespace drone_city_nav
