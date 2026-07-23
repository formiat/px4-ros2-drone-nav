#pragma once

#include "drone_city_nav/clearance_field.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct PlanningGridVersion {
  std::uint64_t build_revision{0U};
  std::uint64_t memory_producer_instance_id{0U};
  std::uint64_t memory_sequence{0U};
  std::int64_t lidar_update_ns{0};
  std::uint64_t config_fingerprint{0U};
  OccupancyGridFingerprint raw{};
  OccupancyGridFingerprint runtime_prohibited{};
  OccupancyGridFingerprint planning_clearance{};
};

struct PlanningGridPreparationInput {
  const PlanningGridBuildResult* build_result{nullptr};
  Point2 relaxation_center{};
  double relaxation_radius_m{0.0};
  double clearance_max_distance_m{0.0};
  std::uint64_t config_fingerprint{0U};
};

struct PreparedPlanningGridSnapshot {
  OccupancyGrid2D raw_grid;
  OccupancyGrid2D runtime_prohibited_grid;
  OccupancyGrid2D planning_clearance_grid;
  ClearanceField2D runtime_clearance;
  ClearanceField2D planning_clearance;
  PlanningGridVersion version{};
  LocalInflationRelaxationStats runtime_relaxation{};
  LocalInflationRelaxationStats planning_relaxation{};
};

class PlanningGridSnapshotBuilder {
public:
  [[nodiscard]] std::optional<PreparedPlanningGridSnapshot>
  prepare(const PlanningGridPreparationInput& input);

  [[nodiscard]] std::uint64_t nextRevision() const noexcept;

private:
  std::uint64_t next_revision_{1U};
};

[[nodiscard]] bool planningGridVersionsEqual(const PlanningGridVersion& lhs,
                                             const PlanningGridVersion& rhs) noexcept;

} // namespace drone_city_nav
