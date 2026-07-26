#include "drone_city_nav/planning_grid_snapshot.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void hashUint64(std::uint64_t& hash, std::uint64_t value) noexcept {
  for (int byte = 0; byte < 8; ++byte) {
    hash ^= value & 0xffU;
    hash *= kFnvPrime;
    value >>= 8U;
  }
}

[[nodiscard]] std::uint64_t
riskPolicyFingerprint(const ObstacleRiskPolicy& policy) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  hashUint64(hash, std::bit_cast<std::uint64_t>(policy.critical_distance_m));
  hashUint64(hash, std::bit_cast<std::uint64_t>(policy.preferred_distance_m));
  return hash;
}

[[nodiscard]] bool sameBounds(const GridBounds& lhs, const GridBounds& rhs) noexcept {
  return lhs.origin_x == rhs.origin_x && lhs.origin_y == rhs.origin_y &&
         lhs.resolution_m == rhs.resolution_m && lhs.width_cells == rhs.width_cells &&
         lhs.height_cells == rhs.height_cells;
}

[[nodiscard]] bool sameFingerprint(const OccupancyGridFingerprint& lhs,
                                   const OccupancyGridFingerprint& rhs) noexcept {
  return sameBounds(lhs.bounds, rhs.bounds) && lhs.cells_hash == rhs.cells_hash;
}

} // namespace

std::optional<PreparedObstacleRiskSnapshot>
ObstacleRiskSnapshotBuilder::prepare(const ObstacleRiskPreparationInput& input) {
  if (input.build_result == nullptr ||
      input.build_result->status != PlanningGridStatus::kReady ||
      !input.build_result->raw_occupancy.has_value() ||
      !input.build_result->evaluation_bounds.has_value()) {
    return std::nullopt;
  }

  const ObstacleFieldBuildResult& build = *input.build_result;
  OccupancyGrid2D raw_occupancy = *build.raw_occupancy;
  ObstacleRiskField risk_field = ObstacleRiskField::build(
      raw_occupancy, build.risk_policy, *build.evaluation_bounds,
      input.clearance_diagnostic_radius_m);
  RawObstacleVersion version{
      .build_revision = next_revision_,
      .memory_producer_instance_id = build.applied_memory_producer_instance_id,
      .memory_sequence = build.applied_memory_sequence,
      .lidar_update_ns = build.applied_lidar_update_ns,
      .config_fingerprint = input.config_fingerprint,
      .raw_occupancy = raw_occupancy.rawFingerprint(),
      .risk_policy_fingerprint = riskPolicyFingerprint(build.risk_policy),
  };
  ++next_revision_;
  return PreparedObstacleRiskSnapshot{
      .raw_occupancy = std::move(raw_occupancy),
      .risk_field = std::move(risk_field),
      .evaluation_bounds = *build.evaluation_bounds,
      .version = version,
  };
}

std::uint64_t ObstacleRiskSnapshotBuilder::nextRevision() const noexcept {
  return next_revision_;
}

bool obstacleRiskVersionsEqual(const RawObstacleVersion& lhs,
                               const RawObstacleVersion& rhs) noexcept {
  return lhs.build_revision == rhs.build_revision &&
         lhs.memory_producer_instance_id == rhs.memory_producer_instance_id &&
         lhs.memory_sequence == rhs.memory_sequence &&
         lhs.lidar_update_ns == rhs.lidar_update_ns &&
         lhs.config_fingerprint == rhs.config_fingerprint &&
         lhs.risk_policy_fingerprint == rhs.risk_policy_fingerprint &&
         sameFingerprint(lhs.raw_occupancy, rhs.raw_occupancy);
}

} // namespace drone_city_nav
