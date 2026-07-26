#pragma once

#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

namespace drone_city_nav {

class ClearanceField2D;
class ObstacleRiskField;

enum class ObstacleRiskTier : std::uint8_t {
  kPreferred = 0,
  kPlanningBand = 1,
  kCriticalBand = 2,
};

struct ObstacleRiskPolicy {
  double critical_distance_m{1.0};
  double preferred_distance_m{4.0};
};

struct PathRiskScore {
  bool outside_bounds{false};
  bool intersects_raw_occupied{false};
  ObstacleRiskTier worst_tier{ObstacleRiskTier::kPreferred};
  double critical_exposure_m{0.0};
  double planning_exposure_m{0.0};
  double minimum_raw_clearance_m{std::numeric_limits<double>::infinity()};

  [[nodiscard]] bool hardValid() const noexcept {
    return !outside_bounds && !intersects_raw_occupied;
  }
};

struct RankedPathCost {
  PathRiskScore risk{};
  double algorithm_cost{0.0};
  std::uint64_t deterministic_tiebreak{0U};
};

struct TrajectoryRiskContext {
  std::string_view name{"raw_risk"};
  const OccupancyGrid2D* raw_occupancy{nullptr};
  const ObstacleRiskField* risk_field{nullptr};
  const ClearanceField2D* raw_clearance{nullptr};
  bool raw_clearance_cache_hit{false};

  [[nodiscard]] bool valid() const noexcept {
    return raw_occupancy != nullptr && risk_field != nullptr;
  }
};

class ObstacleRiskField {
public:
  [[nodiscard]] static ObstacleRiskField build(const OccupancyGrid2D& raw_grid,
                                               ObstacleRiskPolicy policy);
  [[nodiscard]] static ObstacleRiskField build(const OccupancyGrid2D& raw_grid,
                                               ObstacleRiskPolicy policy,
                                               const GridBounds& evaluation_bounds);

  [[nodiscard]] const ObstacleRiskPolicy& policy() const noexcept;
  [[nodiscard]] const GridBounds& evaluationBounds() const noexcept;
  [[nodiscard]] const DistanceField2D& occupiedDistance() const noexcept;
  [[nodiscard]] ObstacleRiskTier tierAt(GridIndex cell) const;
  [[nodiscard]] PathRiskScore evaluate(const OccupancyGrid2D& raw_grid,
                                       std::span<const Point2> points) const;
  [[nodiscard]] PathRiskScore
  evaluate(const OccupancyGrid2D& raw_grid,
           std::span<const TrajectoryPointSample> samples) const;

private:
  ObstacleRiskPolicy policy_{};
  GridBounds evaluation_bounds_{};
  DistanceField2D occupied_distance_{};
};

[[nodiscard]] bool pathRiskLess(const PathRiskScore& lhs,
                                const PathRiskScore& rhs) noexcept;
[[nodiscard]] bool pathRiskEqual(const PathRiskScore& lhs,
                                 const PathRiskScore& rhs) noexcept;
[[nodiscard]] bool rankedPathCostLess(const RankedPathCost& lhs,
                                      const RankedPathCost& rhs) noexcept;
[[nodiscard]] std::uint64_t
obstacleRiskContextFingerprint(const OccupancyGrid2D& raw_grid,
                               const ObstacleRiskField& risk_field) noexcept;
[[nodiscard]] const char* obstacleRiskTierName(ObstacleRiskTier tier) noexcept;

} // namespace drone_city_nav
