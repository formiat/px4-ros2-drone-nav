#include "drone_city_nav/trajectory_repair.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

template<typename Predicate>
[[nodiscard]] std::optional<BlockedSpan>
findFirstSampledBlockedSpan(const OccupancyGrid2D& grid,
                            const std::span<const TrajectoryPointSample> trajectory,
                            const double minimum_s_m, const double requested_step_m,
                            const double minimum_run_length_m,
                            const BlockedSpanTrigger trigger, Predicate predicate) {
  if (!trajectorySamplesAreUsable(trajectory) || !(grid.resolution() > 0.0)) {
    return std::nullopt;
  }
  const double start_s_m = std::clamp(std::isfinite(minimum_s_m) ? minimum_s_m : 0.0,
                                      0.0, trajectory.back().s_m);
  const double step_m =
      std::clamp(std::min(requested_step_m, 0.5 * grid.resolution()), 0.05, 5.0);
  bool run_active = false;
  BlockedSpan span{};
  span.trigger = trigger;

  const double remaining_length_m = trajectory.back().s_m - start_s_m;
  const std::size_t probe_count =
      static_cast<std::size_t>(std::ceil(remaining_length_m / step_m)) + 1U;
  for (std::size_t probe_index = 0U; probe_index < probe_count; ++probe_index) {
    const double sample_s_m = std::min(
        start_s_m + static_cast<double>(probe_index) * step_m, trajectory.back().s_m);
    const TrajectoryPointSample sample = trajectorySampleAtS(trajectory, sample_s_m);
    const std::optional<GridIndex> cell = grid.worldToCell(sample.point);
    const auto state = predicate(sample, cell);
    const bool blocked = state.first;
    const double clearance_m = state.second;
    if (blocked && !run_active) {
      run_active = true;
      span.first_blocked_s_m = sample_s_m;
      span.first_point = sample.point;
      if (cell.has_value()) {
        span.first_cell = *cell;
        span.first_cell_available = true;
      }
    }
    if (blocked) {
      span.last_blocked_s_m = sample_s_m;
      span.last_point = sample.point;
      span.min_raw_clearance_m = std::min(span.min_raw_clearance_m, clearance_m);
      if (cell.has_value()) {
        span.last_cell = *cell;
        span.last_cell_available = true;
      }
    } else if (run_active) {
      const double run_length_m = sample_s_m - span.first_blocked_s_m;
      if (run_length_m + kTinyDistanceM >= minimum_run_length_m) {
        span.last_blocked_s_m = sample_s_m;
        span.last_point = sample.point;
        if (cell.has_value()) {
          span.last_cell = *cell;
          span.last_cell_available = true;
        }
        return span;
      }
      run_active = false;
      span = BlockedSpan{};
      span.trigger = trigger;
    }
    if (sample_s_m >= trajectory.back().s_m) {
      break;
    }
  }

  if (run_active && trajectory.back().s_m - span.first_blocked_s_m + kTinyDistanceM >=
                        minimum_run_length_m) {
    span.last_blocked_s_m = trajectory.back().s_m;
    span.last_point = trajectory.back().point;
    return span;
  }
  return std::nullopt;
}

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

[[nodiscard]] std::optional<BlockedSpan>
findFirstProhibitedCellSpan(const OccupancyGrid2D& grid,
                            const std::span<const TrajectoryPointSample> trajectory,
                            const double minimum_s_m) {
  if (!trajectorySamplesAreUsable(trajectory) || !(grid.resolution() > 0.0)) {
    return std::nullopt;
  }

  const double start_s_m = std::clamp(std::isfinite(minimum_s_m) ? minimum_s_m : 0.0,
                                      0.0, trajectory.back().s_m);
  bool run_active = false;
  BlockedSpan span{};
  span.trigger = BlockedSpanTrigger::kProhibited;

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
      if (observe(station_m, point, cell, grid.isProhibited(cell))) {
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
    return cell.has_value() && !grid->isProhibited(*cell);
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

bool updateExecutableTrajectoryProgress(ExecutableTrajectoryArtifact& artifact,
                                        const Point2 current_position) {
  const std::optional<TrajectoryProjection> projection = projectOnTrajectorySamples(
      artifact.samples, current_position, artifact.current_s_m);
  if (!projection.has_value()) {
    return false;
  }
  artifact.current_s_m = projection->s_m;
  return true;
}

std::optional<BlockedSpan>
findFirstProhibitedBlockedSpan(const OccupancyGrid2D& grid,
                               const std::span<const TrajectoryPointSample> trajectory,
                               const double minimum_s_m,
                               [[maybe_unused]] const BlockedSpanScanConfig& config) {
  return findFirstProhibitedCellSpan(grid, trajectory, minimum_s_m);
}

std::optional<BlockedSpan> findFirstRawClearanceBlockedSpan(
    const OccupancyGrid2D& raw_grid,
    const std::span<const TrajectoryPointSample> trajectory, const double minimum_s_m,
    const BlockedSpanScanConfig& config) {
  const double trigger_m = std::max(0.0, config.raw_clearance_trigger_m);
  const ClearanceField2D field =
      ClearanceField2D::build(raw_grid, trigger_m, ClearanceSource::kOccupied);
  return findFirstSampledBlockedSpan(
      raw_grid, trajectory, minimum_s_m, config.sample_step_m,
      std::max(0.0, config.raw_min_violation_length_m),
      BlockedSpanTrigger::kRawClearance,
      [&field, trigger_m](const TrajectoryPointSample&,
                          const std::optional<GridIndex> cell) {
        const double clearance_m = cell.has_value() && field.contains(*cell)
                                       ? field.distanceAt(*cell)
                                       : std::numeric_limits<double>::quiet_NaN();
        return std::pair{std::isnan(clearance_m) ||
                             clearance_m + kTinyDistanceM < trigger_m,
                         clearance_m};
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
    case BlockedSpanTrigger::kProhibited:
      return "prohibited";
    case BlockedSpanTrigger::kRawClearance:
      return "raw_clearance";
  }
  return "unknown";
}

} // namespace drone_city_nav
