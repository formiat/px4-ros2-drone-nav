#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_passage_validation.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class PassageInsertionRejectReason {
  kNone,
  kDisabled,
  kNoMap,
  kInvalidInput,
  kNoRepairNeeded,
  kNoCandidate,
  kTooManyCandidates,
  kInvalidOpeningFrame,
  kExcessiveLateralShift,
  kInvalidGeometry,
  kNonTraversable,
  kEndpointMismatch,
  kJoinTangent,
  kJoinCurvature,
  kInsertedRadius,
  kValidationNotImproved,
};

struct PassageInsertionConfig {
  bool enabled{true};
  double sample_step_m{1.0};
  double min_anchor_margin_m{8.0};
  double max_anchor_margin_m{60.0};
  double opening_lateral_target_margin_m{0.0};
  double max_lateral_shift_m{80.0};
  double max_join_tangent_delta_rad{0.35};
  double max_join_curvature_jump_1pm{0.08};
  double min_inserted_radius_m{0.0};
  std::size_t max_candidates{8U};
  std::size_t max_diagnostics{8U};
};

struct PassageInsertionDiagnostic {
  std::string structure_id;
  std::string opening_id;
  double anchor_s_m{0.0};
  double entry_s_m{0.0};
  double exit_s_m{0.0};
  double reconnect_s_m{0.0};
  double lateral_miss_before_m{0.0};
  double lateral_miss_after_m{0.0};
  double join_tangent_delta_before_rad{0.0};
  double join_tangent_delta_after_rad{0.0};
  double join_curvature_jump_before_1pm{0.0};
  double join_curvature_jump_after_1pm{0.0};
  double min_inserted_radius_m{0.0};
  PassageInsertionRejectReason reason{PassageInsertionRejectReason::kNone};
  bool accepted{false};
};

struct PassageInsertionStats {
  bool enabled{false};
  bool applied{false};
  std::size_t candidates{0U};
  std::size_t inserted_count{0U};
  std::size_t rejected_join{0U};
  std::size_t rejected_traversability{0U};
  std::size_t rejected_validation{0U};
  std::size_t rejected_geometry{0U};
  std::size_t diagnostics_dropped{0U};
  PassageInsertionRejectReason final_reason{PassageInsertionRejectReason::kDisabled};
  std::vector<PassageInsertionDiagnostic> diagnostics;
};

struct PassageInsertionResult {
  std::vector<TrajectoryPointSample> samples;
  PassageInsertionStats stats{};
  bool valid{false};
  bool applied{false};
};

[[nodiscard]] const char*
passageInsertionRejectReasonName(PassageInsertionRejectReason reason) noexcept;

[[nodiscard]] PassageInsertionResult insertLocalPassageSegments(
    std::span<const TrajectoryPointSample> samples, const OccupancyGrid2D& grid,
    const KnownPassageMap* map, const KnownPassageValidationConfig& validation_config,
    const PassageInsertionConfig& config, double initial_altitude_m);

} // namespace drone_city_nav
