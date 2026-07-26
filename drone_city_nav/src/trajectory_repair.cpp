#include "drone_city_nav/trajectory_repair.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <type_traits>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] double segmentProjectionT(const Point2 start, const Point2 end,
                                        const Point2 point) noexcept {
  const Point2 direction{end.x - start.x, end.y - start.y};
  const double length_sq = squaredDistance(start, end);
  if (!(length_sq > kTinyDistanceM * kTinyDistanceM)) {
    return 0.0;
  }
  return std::clamp(
      ((point.x - start.x) * direction.x + (point.y - start.y) * direction.y) /
          length_sq,
      0.0, 1.0);
}

template<typename BlockedPredicate>
[[nodiscard]] std::optional<BlockedSpan>
findFirstBlockedCellSpan(const OccupancyGrid2D& grid,
                         const std::span<const TrajectoryPointSample> trajectory,
                         const double minimum_s_m, const BlockedSpanTrigger trigger,
                         BlockedPredicate cell_blocked) {
  static_assert(std::is_invocable_r_v<bool, BlockedPredicate, GridIndex>);
  if (!trajectorySamplesAreUsable(trajectory) || !(grid.resolution() > 0.0)) {
    return std::nullopt;
  }

  const double start_s_m = std::clamp(std::isfinite(minimum_s_m) ? minimum_s_m : 0.0,
                                      0.0, trajectory.back().s_m);
  bool run_active = false;
  BlockedSpan span{};
  span.trigger = trigger;

  auto observe = [&](const double station_m, const Point2 point,
                     const std::optional<GridIndex> cell, const bool blocked) -> bool {
    if (blocked && !run_active) {
      run_active = true;
      span.first_blocked_s_m = station_m;
      span.first_point = point;
      if (cell.has_value()) {
        span.first_cell = *cell;
        span.first_cell_available = true;
      }
    }
    if (blocked) {
      span.last_blocked_s_m = station_m;
      span.last_point = point;
      if (cell.has_value()) {
        span.last_cell = *cell;
        span.last_cell_available = true;
      }
      return false;
    }
    if (!run_active) {
      return false;
    }
    span.last_blocked_s_m = station_m;
    span.last_point = point;
    if (cell.has_value()) {
      span.last_cell = *cell;
      span.last_cell_available = true;
    }
    return true;
  };

  for (std::size_t index = 1U; index < trajectory.size(); ++index) {
    const TrajectoryPointSample& original_start = trajectory[index - 1U];
    const TrajectoryPointSample& original_end = trajectory[index];
    if (original_end.s_m + kTinyDistanceM < start_s_m) {
      continue;
    }
    const double segment_start_s_m = std::max(start_s_m, original_start.s_m);
    if (segment_start_s_m > original_end.s_m + kTinyDistanceM) {
      continue;
    }
    const TrajectoryPointSample segment_start =
        trajectorySampleAtS(trajectory, segment_start_s_m);
    const Point2 segment_end = original_end.point;
    const std::optional<GridIndex> start_cell = grid.worldToCell(segment_start.point);
    const std::optional<GridIndex> end_cell = grid.worldToCell(segment_end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      if (observe(segment_start_s_m, segment_start.point, std::nullopt, true) ||
          observe(original_end.s_m, segment_end, std::nullopt, true)) {
        return span;
      }
      continue;
    }

    const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
    if (cells.empty()) {
      if (observe(segment_start_s_m, segment_start.point, std::nullopt, true)) {
        return span;
      }
      continue;
    }
    double previous_station_m = segment_start_s_m;
    for (const GridIndex cell : cells) {
      const double segment_t =
          segmentProjectionT(segment_start.point, segment_end, grid.cellCenter(cell));
      const double station_m = std::max(
          previous_station_m,
          segment_start_s_m + segment_t * (original_end.s_m - segment_start_s_m));
      previous_station_m = station_m;
      const Point2 point = trajectorySampleAtS(trajectory, station_m).point;
      if (observe(station_m, point, cell, cell_blocked(cell))) {
        return span;
      }
    }
  }

  if (run_active) {
    span.last_blocked_s_m = trajectory.back().s_m;
    span.last_point = trajectory.back().point;
    return span;
  }
  return std::nullopt;
}

[[nodiscard]] bool
endpointAllowed(const TrajectoryPointSample& sample,
                const std::span<const OccupancyGrid2D* const> grids) {
  return std::ranges::any_of(grids, [&sample](const OccupancyGrid2D* grid) {
    if (grid == nullptr) {
      return false;
    }
    const std::optional<GridIndex> cell = grid->worldToCell(sample.point);
    return cell.has_value() && !grid->isOccupied(*cell);
  });
}

void appendDistinct(std::vector<TrajectoryPointSample>& samples,
                    const TrajectoryPointSample& sample) {
  if (!samples.empty() &&
      distance(samples.back().point, sample.point) <= kTinyDistanceM) {
    samples.back() = sample;
    return;
  }
  samples.push_back(sample);
}

} // namespace

ExecutableTrajectoryProgress
updateExecutableTrajectoryProgress(ExecutableTrajectoryArtifact& artifact,
                                   const Point2 current_position,
                                   const double max_cross_track_m) {
  ExecutableTrajectoryProgress result{};
  result.previous_s_m = artifact.current_s_m;
  if (!trajectorySamplesAreUsable(artifact.samples)) {
    return result;
  }
  result.terminal_distance_m =
      distance(current_position, artifact.samples.back().point);
  const std::optional<TrajectoryProjection> projection = projectOnTrajectorySamples(
      artifact.samples, current_position, artifact.current_s_m);
  if (!projection.has_value()) {
    return result;
  }
  result.projected_s_m = projection->s_m;
  result.projected_point = projection->point;
  result.cross_track_m = std::sqrt(std::max(0.0, projection->distance_sq));
  result.remaining_m =
      std::max(0.0, artifact.samples.back().s_m - result.projected_s_m);
  result.diverged =
      std::isfinite(max_cross_track_m) && result.cross_track_m > max_cross_track_m;
  if (result.diverged) {
    return result;
  }
  artifact.current_s_m = projection->s_m;
  result.valid = true;
  return result;
}

ExecutableSuffixDecision evaluateExecutableSuffix(
    const OccupancyGrid2D& grid, const ObstacleRiskField& risk_field,
    const ExecutableTrajectoryArtifact& artifact,
    const ExecutableTrajectoryProgress& progress, const double exhaustion_epsilon_m,
    const double minimum_tracking_clearance_m) {
  ExecutableSuffixDecision decision{};
  decision.progress = progress;
  if (!progress.valid || !trajectorySamplesAreUsable(artifact.samples)) {
    return decision;
  }
  decision.exhausted = progress.remaining_m <= std::max(0.0, exhaustion_epsilon_m);
  const std::optional<BlockedSpan> raw_blocked =
      findFirstRawOccupiedBlockedSpan(grid, artifact.samples, progress.projected_s_m);
  const std::optional<BlockedSpan> tracking_blocked =
      findFirstTrackingEnvelopeBlockedSpan(grid, risk_field.occupiedClearance(),
                                           artifact.samples, progress.projected_s_m,
                                           minimum_tracking_clearance_m);
  if (raw_blocked.has_value() &&
      (!tracking_blocked.has_value() ||
       raw_blocked->first_blocked_s_m <=
           tracking_blocked->first_blocked_s_m + kTinyDistanceM)) {
    decision.blocked_span = raw_blocked;
  } else {
    decision.blocked_span = tracking_blocked;
  }
  decision.blocked = decision.blocked_span.has_value();
  return decision;
}

std::optional<BlockedSpan>
findFirstRawOccupiedBlockedSpan(const OccupancyGrid2D& grid,
                                const std::span<const TrajectoryPointSample> trajectory,
                                const double minimum_s_m) {
  return findFirstBlockedCellSpan(
      grid, trajectory, minimum_s_m, BlockedSpanTrigger::kRawOccupied,
      [&grid](const GridIndex cell) { return grid.isOccupied(cell); });
}

std::optional<BlockedSpan> findFirstTrackingEnvelopeBlockedSpan(
    const OccupancyGrid2D& grid, const ClearanceField2D& raw_clearance,
    const std::span<const TrajectoryPointSample> trajectory, const double minimum_s_m,
    const double minimum_clearance_m) {
  if (!std::isfinite(minimum_clearance_m) || minimum_clearance_m <= 0.0) {
    return std::nullopt;
  }
  return findFirstBlockedCellSpan(
      grid, trajectory, minimum_s_m, BlockedSpanTrigger::kTrackingEnvelope,
      [&raw_clearance, minimum_clearance_m](const GridIndex cell) {
        return raw_clearance.distanceAt(cell) + kTinyDistanceM < minimum_clearance_m;
      });
}

std::vector<ReconnectCandidate>
makeReconnectCandidates(const ExecutableTrajectoryArtifact& artifact,
                        const BlockedSpan& blocked_span, const double truncation_s_m,
                        const std::span<const double> reconnect_margins_m,
                        const std::span<const OccupancyGrid2D* const> candidate_grids,
                        const double endpoint_tolerance_m) {
  std::vector<ReconnectCandidate> candidates;
  if (!trajectorySamplesAreUsable(artifact.samples) ||
      !std::isfinite(blocked_span.last_blocked_s_m) || !std::isfinite(truncation_s_m)) {
    return candidates;
  }
  candidates.reserve(reconnect_margins_m.size());
  const double lower_bound_m = std::max(artifact.current_s_m, truncation_s_m) +
                               std::max(0.0, endpoint_tolerance_m);
  for (const double margin_m : reconnect_margins_m) {
    if (!std::isfinite(margin_m) || margin_m <= 0.0) {
      continue;
    }
    const double reconnect_s_m = blocked_span.last_blocked_s_m + margin_m;
    if (reconnect_s_m <= lower_bound_m ||
        reconnect_s_m <= blocked_span.last_blocked_s_m ||
        reconnect_s_m >= artifact.samples.back().s_m - kTinyDistanceM) {
      continue;
    }
    const TrajectoryPointSample reconnect_sample =
        trajectorySampleAtS(artifact.samples, reconnect_s_m);
    if (!endpointAllowed(reconnect_sample, candidate_grids)) {
      continue;
    }
    candidates.push_back(ReconnectCandidate{
        .margin_m = margin_m,
        .reconnect_s_m = reconnect_s_m,
        .reconnect_sample = reconnect_sample,
    });
  }
  return candidates;
}

TrajectoryRepairStitchResult
stitchTrajectoryRepair(const std::span<const TrajectoryPointSample> repaired_segment,
                       const ExecutableTrajectoryArtifact& artifact,
                       const double reconnect_s_m, const double endpoint_tolerance_m) {
  TrajectoryRepairStitchResult result{};
  if (!trajectorySamplesAreUsable(repaired_segment) ||
      !trajectorySamplesAreUsable(artifact.samples) || !std::isfinite(reconnect_s_m)) {
    result.reason = "invalid_input";
    return result;
  }
  const TrajectoryPointSample reconnect =
      trajectorySampleAtS(artifact.samples, reconnect_s_m);
  if (distance(repaired_segment.back().point, reconnect.point) >
      std::max(0.0, endpoint_tolerance_m)) {
    result.reason = "endpoint_mismatch";
    return result;
  }

  result.samples.reserve(repaired_segment.size() + artifact.samples.size());
  for (const TrajectoryPointSample& sample : repaired_segment) {
    TrajectoryPointSample geometry_sample{};
    geometry_sample.point = sample.point;
    appendDistinct(result.samples, geometry_sample);
  }
  TrajectoryPointSample reconnect_geometry{};
  reconnect_geometry.point = reconnect.point;
  appendDistinct(result.samples, reconnect_geometry);
  for (const TrajectoryPointSample& sample : artifact.samples) {
    if (sample.s_m <= reconnect_s_m + kTinyDistanceM) {
      continue;
    }
    TrajectoryPointSample geometry_sample{};
    geometry_sample.point = sample.point;
    appendDistinct(result.samples, geometry_sample);
  }
  populateTrajectorySampleGeometry(result.samples);
  if (!trajectorySamplesAreUsable(result.samples)) {
    result.samples.clear();
    result.reason = "stitched_trajectory_invalid";
    return result;
  }
  result.valid = true;
  result.reason = "ok";
  return result;
}

const char* blockedSpanTriggerName(const BlockedSpanTrigger trigger) noexcept {
  switch (trigger) {
    case BlockedSpanTrigger::kRawOccupied:
      return "raw_occupied";
    case BlockedSpanTrigger::kTrackingEnvelope:
      return "tracking_envelope";
  }
  return "unknown";
}

} // namespace drone_city_nav
