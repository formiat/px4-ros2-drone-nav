#pragma once

#include "drone_city_nav/obstacle_risk_field.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct RawObstacleVersion {
  std::uint64_t build_revision{0U};
  std::uint64_t memory_producer_instance_id{0U};
  std::uint64_t memory_sequence{0U};
  std::int64_t lidar_update_ns{0};
  std::uint64_t config_fingerprint{0U};
  OccupancyGridFingerprint raw_occupancy{};
  std::uint64_t risk_policy_fingerprint{0U};
};

struct ObstacleRiskPreparationInput {
  const ObstacleFieldBuildResult* build_result{nullptr};
  std::uint64_t config_fingerprint{0U};
};

struct PreparedObstacleRiskSnapshot {
  OccupancyGrid2D raw_occupancy;
  ObstacleRiskField risk_field;
  GridBounds evaluation_bounds{};
  RawObstacleVersion version{};

  [[nodiscard]] const ClearanceField2D& rawClearance() const noexcept {
    return risk_field.occupiedClearance();
  }
};

class ObstacleRiskSnapshotBuilder {
public:
  [[nodiscard]] std::optional<PreparedObstacleRiskSnapshot>
  prepare(const ObstacleRiskPreparationInput& input);

  [[nodiscard]] std::uint64_t nextRevision() const noexcept;

private:
  std::uint64_t next_revision_{1U};
};

[[nodiscard]] bool obstacleRiskVersionsEqual(const RawObstacleVersion& lhs,
                                             const RawObstacleVersion& rhs) noexcept;

} // namespace drone_city_nav
