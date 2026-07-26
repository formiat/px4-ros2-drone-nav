#include "drone_city_nav/obstacle_risk_field.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kComparisonEpsilon = 1.0e-9;
constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

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

void hashUint64(std::uint64_t& hash, std::uint64_t value) noexcept {
  for (int byte = 0; byte < 8; ++byte) {
    hash ^= value & 0xffU;
    hash *= kFnvPrime;
    value >>= 8U;
  }
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

[[nodiscard]] bool nearInteger(const double value) noexcept {
  return std::abs(value - std::round(value)) <= 1.0e-10;
}

[[nodiscard]] std::vector<GridIndex> cellsTouchingPoint(const OccupancyGrid2D& grid,
                                                        const Point2 point) {
  std::vector<GridIndex> cells;
  const GridBounds& bounds = grid.bounds();
  const double grid_x = (point.x - bounds.origin_x) / bounds.resolution_m;
  const double grid_y = (point.y - bounds.origin_y) / bounds.resolution_m;
  const int base_x = static_cast<int>(std::floor(grid_x));
  const int base_y = static_cast<int>(std::floor(grid_y));
  const int min_x = nearInteger(grid_x) ? base_x - 1 : base_x;
  const int min_y = nearInteger(grid_y) ? base_y - 1 : base_y;
  cells.reserve(4U);
  for (int x = min_x; x <= base_x; ++x) {
    for (int y = min_y; y <= base_y; ++y) {
      const GridIndex cell{x, y};
      if (grid.contains(cell)) {
        cells.push_back(cell);
      }
    }
  }
  return cells;
}

void classifyInterval(const ObstacleRiskField& field, const OccupancyGrid2D& raw_grid,
                      const Point2 point, const double length_m, PathRiskScore& score) {
  if (!containsPoint(field.evaluationBounds(), point)) {
    score.outside_bounds = true;
    return;
  }
  const std::vector<GridIndex> cells = cellsTouchingPoint(raw_grid, point);
  if (cells.empty()) {
    score.outside_bounds = true;
    return;
  }

  ObstacleRiskTier interval_tier{ObstacleRiskTier::kPreferred};
  for (const GridIndex cell : cells) {
    if (!containsPoint(field.evaluationBounds(), raw_grid.cellCenter(cell))) {
      continue;
    }
    if (raw_grid.isOccupied(cell)) {
      score.intersects_raw_occupied = true;
      score.minimum_raw_clearance_m = 0.0;
      interval_tier = ObstacleRiskTier::kCriticalBand;
      continue;
    }
    const double clearance_m = field.occupiedClearance().distanceAt(cell);
    score.minimum_raw_clearance_m =
        std::min(score.minimum_raw_clearance_m, clearance_m);
    const ObstacleRiskTier tier = field.tierAt(cell);
    if (static_cast<std::uint8_t>(tier) > static_cast<std::uint8_t>(interval_tier)) {
      interval_tier = tier;
    }
  }
  mergeWorstTier(score, interval_tier);
  if (interval_tier == ObstacleRiskTier::kCriticalBand) {
    score.critical_exposure_m += length_m;
  } else if (interval_tier == ObstacleRiskTier::kPlanningBand) {
    score.planning_exposure_m += length_m;
  }
}

void appendGridBoundaryIntersections(const double start_coordinate,
                                     const double end_coordinate, const double origin,
                                     const double resolution,
                                     std::vector<double>& parameters) {
  const double delta = end_coordinate - start_coordinate;
  if (std::abs(delta) <= kComparisonEpsilon) {
    return;
  }
  const double start_grid = (start_coordinate - origin) / resolution;
  const double end_grid = (end_coordinate - origin) / resolution;
  const int first_boundary =
      static_cast<int>(std::floor(std::min(start_grid, end_grid))) + 1;
  const int last_boundary =
      static_cast<int>(std::floor(std::max(start_grid, end_grid)));
  for (int boundary = first_boundary; boundary <= last_boundary; ++boundary) {
    const double coordinate = origin + static_cast<double>(boundary) * resolution;
    const double parameter = (coordinate - start_coordinate) / delta;
    if (parameter > kComparisonEpsilon && parameter < 1.0 - kComparisonEpsilon) {
      parameters.push_back(parameter);
    }
  }
}

void classifySegment(const ObstacleRiskField& field, const OccupancyGrid2D& raw_grid,
                     const Point2 start, const Point2 end, PathRiskScore& score) {
  const double segment_length_m = distance(start, end);
  if (!std::isfinite(segment_length_m)) {
    score.outside_bounds = true;
    return;
  }
  if (!containsPoint(field.evaluationBounds(), start) ||
      !containsPoint(field.evaluationBounds(), end)) {
    score.outside_bounds = true;
    return;
  }
  if (segment_length_m <= kComparisonEpsilon) {
    classifyInterval(field, raw_grid, start, 0.0, score);
    return;
  }

  std::vector<double> parameters{0.0, 1.0};
  const GridBounds& bounds = raw_grid.bounds();
  appendGridBoundaryIntersections(start.x, end.x, bounds.origin_x, bounds.resolution_m,
                                  parameters);
  appendGridBoundaryIntersections(start.y, end.y, bounds.origin_y, bounds.resolution_m,
                                  parameters);
  std::sort(parameters.begin(), parameters.end());
  parameters.erase(std::unique(parameters.begin(), parameters.end(),
                               [](const double lhs, const double rhs) {
                                 return std::abs(lhs - rhs) <= kComparisonEpsilon;
                               }),
                   parameters.end());

  for (const double parameter : parameters) {
    const Point2 crossing{start.x + (end.x - start.x) * parameter,
                          start.y + (end.y - start.y) * parameter};
    classifyInterval(field, raw_grid, crossing, 0.0, score);
  }
  for (std::size_t index = 0U; index + 1U < parameters.size(); ++index) {
    const double begin = parameters[index];
    const double finish = parameters[index + 1U];
    const double midpoint_parameter = 0.5 * (begin + finish);
    const Point2 midpoint{start.x + (end.x - start.x) * midpoint_parameter,
                          start.y + (end.y - start.y) * midpoint_parameter};
    classifyInterval(field, raw_grid, midpoint, segment_length_m * (finish - begin),
                     score);
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
    classifySegment(field, raw_grid, start, end, score);
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
  return build(raw_grid, policy, evaluation_bounds,
               policy.preferred_distance_m + raw_grid.resolution());
}

ObstacleRiskField ObstacleRiskField::build(const OccupancyGrid2D& raw_grid,
                                           ObstacleRiskPolicy policy,
                                           const GridBounds& evaluation_bounds,
                                           const double maximum_distance_m) {
  if (!std::isfinite(policy.critical_distance_m) ||
      !std::isfinite(policy.preferred_distance_m) ||
      !std::isfinite(maximum_distance_m)) {
    throw std::invalid_argument{"Obstacle risk distances must be finite"};
  }
  policy.critical_distance_m = std::max(0.0, policy.critical_distance_m);
  policy.preferred_distance_m =
      std::max(policy.critical_distance_m, policy.preferred_distance_m);

  ObstacleRiskField field;
  field.policy_ = policy;
  field.evaluation_bounds_ = evaluation_bounds;
  field.occupied_clearance_ = ClearanceField2D::build(
      raw_grid,
      std::max(maximum_distance_m, policy.preferred_distance_m + raw_grid.resolution()),
      ClearanceSource::kOccupied);
  return field;
}

const ObstacleRiskPolicy& ObstacleRiskField::policy() const noexcept {
  return policy_;
}

const GridBounds& ObstacleRiskField::evaluationBounds() const noexcept {
  return evaluation_bounds_;
}

const ClearanceField2D& ObstacleRiskField::occupiedClearance() const noexcept {
  return occupied_clearance_;
}

bool ObstacleRiskField::containsEvaluationPoint(const Point2 point) const noexcept {
  return containsPoint(evaluation_bounds_, point);
}

ObstacleRiskTier ObstacleRiskField::tierAt(const GridIndex cell) const {
  const double clearance_m = occupied_clearance_.distanceAt(cell);
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

std::uint64_t
obstacleRiskContextFingerprint(const OccupancyGrid2D& raw_grid,
                               const ObstacleRiskField& risk_field) noexcept {
  const OccupancyGridFingerprint raw = raw_grid.rawFingerprint();
  const GridBounds& evaluation = risk_field.evaluationBounds();
  const ObstacleRiskPolicy& policy = risk_field.policy();
  std::uint64_t hash = kFnvOffsetBasis;
  hashUint64(hash, raw.cells_hash);
  hashUint64(hash, std::bit_cast<std::uint64_t>(evaluation.origin_x));
  hashUint64(hash, std::bit_cast<std::uint64_t>(evaluation.origin_y));
  hashUint64(hash, std::bit_cast<std::uint64_t>(evaluation.resolution_m));
  hashUint64(hash, static_cast<std::uint64_t>(evaluation.width_cells));
  hashUint64(hash, static_cast<std::uint64_t>(evaluation.height_cells));
  hashUint64(hash, std::bit_cast<std::uint64_t>(policy.critical_distance_m));
  hashUint64(hash, std::bit_cast<std::uint64_t>(policy.preferred_distance_m));
  return hash != 0U ? hash : 1U;
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
