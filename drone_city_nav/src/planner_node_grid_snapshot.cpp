#include <bit>
#include <cstdint>
#include <string_view>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  hash ^= value;
  hash *= kFnvPrime;
}

void hashValue(std::uint64_t& hash, const double value) noexcept {
  hashValue(hash, std::bit_cast<std::uint64_t>(value));
}

void hashValue(std::uint64_t& hash, const std::string_view value) noexcept {
  for (const char character : value) {
    hash ^= static_cast<std::uint8_t>(character);
    hash *= kFnvPrime;
  }
}

} // namespace

std::optional<PreparedPlanningGridSnapshot>
PlannerNode::preparePlanningGridSnapshot(const PlanningGridBuildResult& build_result,
                                         const Point2 relaxation_center) {
  std::uint64_t config_fingerprint = kFnvOffsetBasis;
  hashValue(config_fingerprint, static_cast<std::uint64_t>(use_static_map_ ? 1U : 0U));
  hashValue(config_fingerprint, inflation_radius_m_);
  hashValue(config_fingerprint, planning_clearance_m_);
  hashValue(config_fingerprint, local_inflation_relaxation_radius_m_);
  hashValue(config_fingerprint, fallback_grid_bounds_.origin_x);
  hashValue(config_fingerprint, fallback_grid_bounds_.origin_y);
  hashValue(config_fingerprint, fallback_grid_bounds_.resolution_m);
  hashValue(config_fingerprint,
            static_cast<std::uint64_t>(fallback_grid_bounds_.width_cells));
  hashValue(config_fingerprint,
            static_cast<std::uint64_t>(fallback_grid_bounds_.height_cells));
  hashValue(config_fingerprint, static_map_resolved_path_.string());

  return planning_grid_snapshot_builder_.prepare(PlanningGridPreparationInput{
      .build_result = &build_result,
      .relaxation_center = relaxation_center,
      .relaxation_radius_m = local_inflation_relaxation_radius_m_,
      .clearance_max_distance_m = planner_core_.config().clearance_diagnostic_radius_m,
      .sources =
          PlanningGridSourceIdentity{
              .memory_producer_instance_id =
                  last_memory_snapshot_applied_producer_instance_id_,
              .memory_sequence = last_memory_snapshot_applied_sequence_,
              .lidar_update_ns = last_scan_update_ns_,
              .config_fingerprint = config_fingerprint,
          },
  });
}

} // namespace drone_city_nav
