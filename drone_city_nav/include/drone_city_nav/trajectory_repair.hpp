#pragma once

#include "drone_city_nav/clearance_field.hpp"
#include "drone_city_nav/trajectory.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class BlockedSpanTrigger {
  kProhibited,
  kRawClearance,
};

struct BlockedSpan {
  BlockedSpanTrigger trigger{BlockedSpanTrigger::kProhibited};
  double first_blocked_s_m{std::numeric_limits<double>::quiet_NaN()};
  double last_blocked_s_m{std::numeric_limits<double>::quiet_NaN()};
  Point2 first_point{};
  Point2 last_point{};
  GridIndex first_cell{};
  GridIndex last_cell{};
  bool first_cell_available{false};
  bool last_cell_available{false};
  double min_raw_clearance_m{std::numeric_limits<double>::infinity()};
};

struct ExecutableTrajectoryArtifact {
  std::uint64_t path_id{0U};
  std::uint64_t geometry_fingerprint{0U};
  Point2 mission_goal{};
  std::vector<TrajectoryPointSample> samples;
  double current_s_m{0.0};
};

struct BlockedSpanScanConfig {
  double sample_step_m{0.5};
  double raw_clearance_trigger_m{5.0};
  double raw_min_violation_length_m{2.0};
};

struct ReconnectCandidate {
  double margin_m{0.0};
  double reconnect_s_m{0.0};
  TrajectoryPointSample reconnect_sample{};
};

struct TrajectoryRepairStitchResult {
  bool valid{false};
  const char* reason{"not_attempted"};
  std::vector<TrajectoryPointSample> samples;
};

[[nodiscard]] bool
updateExecutableTrajectoryProgress(ExecutableTrajectoryArtifact& artifact,
                                   Point2 current_position);

[[nodiscard]] std::optional<BlockedSpan> findFirstProhibitedBlockedSpan(
    const OccupancyGrid2D& grid, std::span<const TrajectoryPointSample> trajectory,
    double minimum_s_m, const BlockedSpanScanConfig& config = {});

[[nodiscard]] std::optional<BlockedSpan> findFirstRawClearanceBlockedSpan(
    const OccupancyGrid2D& raw_grid, std::span<const TrajectoryPointSample> trajectory,
    double minimum_s_m, const BlockedSpanScanConfig& config = {});

[[nodiscard]] std::vector<ReconnectCandidate>
makeReconnectCandidates(const ExecutableTrajectoryArtifact& artifact,
                        const BlockedSpan& blocked_span, double truncation_s_m,
                        std::span<const double> reconnect_margins_m,
                        std::span<const OccupancyGrid2D* const> candidate_grids,
                        double endpoint_tolerance_m = 1.0e-3);

[[nodiscard]] TrajectoryRepairStitchResult
stitchTrajectoryRepair(std::span<const TrajectoryPointSample> repaired_segment,
                       const ExecutableTrajectoryArtifact& artifact,
                       double reconnect_s_m, double endpoint_tolerance_m = 1.0);

[[nodiscard]] const char* blockedSpanTriggerName(BlockedSpanTrigger trigger) noexcept;

} // namespace drone_city_nav
