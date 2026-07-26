#include "drone_city_nav/receding_horizon_trajectory_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] double clampAngle(double angle) noexcept {
  while (angle > std::numbers::pi) {
    angle -= 2.0 * std::numbers::pi;
  }
  while (angle < -std::numbers::pi) {
    angle += 2.0 * std::numbers::pi;
  }
  return angle;
}

[[nodiscard]] bool finitePoint(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

void recordHardRejection(const OccupancyGrid2D& grid,
                         std::span<const TrajectoryPointSample> samples,
                         const std::size_t deterministic_index,
                         RolloutDiagnostics& diagnostics) {
  for (std::size_t index = 0U; index + 1U < samples.size(); ++index) {
    const std::optional<GridIndex> start = grid.worldToCell(samples[index].point);
    const std::optional<GridIndex> end = grid.worldToCell(samples[index + 1U].point);
    if (!start.has_value() || !end.has_value()) {
      ++diagnostics.outside_grid_rejections;
      if (!diagnostics.first_grid_rejection.has_value()) {
        diagnostics.first_grid_rejection = RolloutGridRejectionDiagnostic{
            .reason = RolloutGridRejectReason::kOutsideGrid,
            .deterministic_index = deterministic_index,
            .segment_index = index,
            .position =
                !start.has_value() ? samples[index].point : samples[index + 1U].point,
            .cell = std::nullopt,
        };
      }
      return;
    }
    for (const GridIndex cell : grid.cellsOnLine(*start, *end)) {
      if (grid.isOccupied(cell)) {
        if (!diagnostics.first_grid_rejection.has_value()) {
          diagnostics.first_grid_rejection = RolloutGridRejectionDiagnostic{
              .reason = RolloutGridRejectReason::kRawOccupied,
              .deterministic_index = deterministic_index,
              .segment_index = index,
              .position = grid.cellCenter(cell),
              .cell = cell,
          };
        }
        return;
      }
    }
  }
}

void recordClearanceRejection(const OccupancyGrid2D& grid,
                              const ClearanceField2D& clearance,
                              std::span<const TrajectoryPointSample> samples,
                              const std::size_t deterministic_index,
                              const double minimum_clearance_m,
                              RolloutDiagnostics& diagnostics) {
  if (diagnostics.first_grid_rejection.has_value()) {
    return;
  }
  for (std::size_t index = 0U; index + 1U < samples.size(); ++index) {
    const std::optional<GridIndex> start = grid.worldToCell(samples[index].point);
    const std::optional<GridIndex> end = grid.worldToCell(samples[index + 1U].point);
    if (!start.has_value() || !end.has_value()) {
      continue;
    }
    for (const GridIndex cell : grid.cellsOnLine(*start, *end)) {
      if (clearance.distanceAt(cell) + 1.0e-9 < minimum_clearance_m) {
        diagnostics.first_grid_rejection = RolloutGridRejectionDiagnostic{
            .reason = RolloutGridRejectReason::kVehicleClearanceEnvelope,
            .deterministic_index = deterministic_index,
            .segment_index = index,
            .position = grid.cellCenter(cell),
            .cell = cell,
        };
        return;
      }
    }
  }
}

void recordTerminalStoppingRejection(
    const OccupancyGrid2D& grid, const ClearanceField2D& clearance,
    std::span<const TrajectoryPointSample> stopping_samples,
    const std::size_t deterministic_index, const std::size_t terminal_segment_index,
    const double minimum_clearance_m, RolloutDiagnostics& diagnostics) {
  if (diagnostics.first_grid_rejection.has_value()) {
    return;
  }
  for (std::size_t index = 0U; index + 1U < stopping_samples.size(); ++index) {
    const std::optional<GridIndex> start =
        grid.worldToCell(stopping_samples[index].point);
    const std::optional<GridIndex> end =
        grid.worldToCell(stopping_samples[index + 1U].point);
    if (!start.has_value() || !end.has_value()) {
      ++diagnostics.outside_grid_rejections;
      diagnostics.first_grid_rejection = RolloutGridRejectionDiagnostic{
          .reason = RolloutGridRejectReason::kTerminalStoppingEnvelope,
          .deterministic_index = deterministic_index,
          .segment_index = terminal_segment_index,
          .position = !start.has_value() ? stopping_samples[index].point
                                         : stopping_samples[index + 1U].point,
          .cell = std::nullopt,
      };
      return;
    }
    for (const GridIndex cell : grid.cellsOnLine(*start, *end)) {
      if (grid.isOccupied(cell) ||
          clearance.distanceAt(cell) + 1.0e-9 < minimum_clearance_m) {
        diagnostics.first_grid_rejection = RolloutGridRejectionDiagnostic{
            .reason = RolloutGridRejectReason::kTerminalStoppingEnvelope,
            .deterministic_index = deterministic_index,
            .segment_index = terminal_segment_index,
            .position = grid.cellCenter(cell),
            .cell = cell,
        };
        return;
      }
    }
  }
}

} // namespace

std::span<const RolloutCandidate>
RolloutResult::rankedShortlist(const std::size_t maximum) const noexcept {
  return std::span<const RolloutCandidate>{ranked_candidates}.first(
      std::min(maximum, ranked_candidates.size()));
}

RecedingHorizonTrajectoryPlanner::RecedingHorizonTrajectoryPlanner(
    const RolloutPlannerConfig& config)
    : config_{config} {
  config_.horizon_m = std::max(1.0, config_.horizon_m);
  config_.sample_step_m = std::clamp(config_.sample_step_m, 0.1, config_.horizon_m);
  config_.heading_samples = std::clamp<std::size_t>(config_.heading_samples, 1U, 63U);
  if ((config_.heading_samples % 2U) == 0U) {
    ++config_.heading_samples;
  }
  config_.speed_samples = std::clamp<std::size_t>(config_.speed_samples, 1U, 9U);
  config_.max_heading_offset_rad =
      std::clamp(config_.max_heading_offset_rad, 0.0, std::numbers::pi);
  config_.horizon_time_s = std::clamp(config_.horizon_time_s, 0.5, 10.0);
  config_.minimum_speed_mps = std::max(0.1, config_.minimum_speed_mps);
  config_.maximum_speed_mps =
      std::max(config_.minimum_speed_mps, config_.maximum_speed_mps);
  config_.maximum_acceleration_mps2 = std::max(0.1, config_.maximum_acceleration_mps2);
  config_.maximum_curvature_1pm = std::max(0.0, config_.maximum_curvature_1pm);
  config_.maximum_lateral_acceleration_mps2 =
      std::max(0.1, config_.maximum_lateral_acceleration_mps2);
  config_.max_finalists = std::clamp<std::size_t>(config_.max_finalists, 1U, 16U);
}

RolloutResult RecedingHorizonTrajectoryPlanner::plan(const RolloutInput& input) const {
  RolloutResult result;
  result.generation = input.generation;
  result.grid_revision = input.grid_revision;
  if (!finitePoint(input.position) || !finitePoint(input.velocity) ||
      !finitePoint(input.preferred_target) || input.grid == nullptr) {
    result.reject_reason = RolloutRejectReason::kInvalidInput;
    return result;
  }
  std::optional<ObstacleRiskField> owned_risk;
  if (input.risk_field == nullptr) {
    owned_risk = ObstacleRiskField::build(*input.grid, ObstacleRiskPolicy{});
  }
  const ObstacleRiskField& risk_field =
      input.risk_field != nullptr ? *input.risk_field : owned_risk.value();
  const std::optional<GridIndex> start_cell = input.grid->worldToCell(input.position);
  const double requested_path_clearance_m =
      std::max(0.0, input.minimum_path_clearance_m);
  const double start_clearance_m =
      start_cell.has_value() ? risk_field.occupiedClearance().distanceAt(*start_cell)
                             : 0.0;
  const double effective_path_clearance_m =
      std::min(requested_path_clearance_m, start_clearance_m);

  const Point2 goal_delta{input.preferred_target.x - input.position.x,
                          input.preferred_target.y - input.position.y};
  const double goal_distance = std::hypot(goal_delta.x, goal_delta.y);
  if (!(goal_distance > 1.0e-6)) {
    result.reject_reason = RolloutRejectReason::kInvalidInput;
    return result;
  }
  const double target_heading = std::atan2(goal_delta.y, goal_delta.x);
  const double speed =
      input.stationary_restart ? 0.0 : std::hypot(input.velocity.x, input.velocity.y);
  const double initial_heading = !input.stationary_restart && speed > 0.25
                                     ? std::atan2(input.velocity.y, input.velocity.x)
                                     : target_heading;
  const double base_heading_error = clampAngle(target_heading - initial_heading);
  const double heading_search_limit =
      input.stationary_restart ? std::numbers::pi : config_.max_heading_offset_rad;
  for (std::size_t candidate_index = 0U; candidate_index < config_.heading_samples;
       ++candidate_index) {
    const double normalized =
        config_.heading_samples == 1U
            ? 0.0
            : (2.0 * static_cast<double>(candidate_index) /
                   static_cast<double>(config_.heading_samples - 1U) -
               1.0);
    const double offset = normalized * heading_search_limit;
    const double terminal_heading = target_heading + offset;
    const double candidate_initial_heading =
        input.stationary_restart ? terminal_heading : initial_heading;
    for (std::size_t speed_index = 0U; speed_index < config_.speed_samples;
         ++speed_index) {
      const double speed_ratio =
          config_.speed_samples == 1U
              ? 1.0
              : static_cast<double>(speed_index) /
                    static_cast<double>(config_.speed_samples - 1U);
      const double requested_speed =
          config_.minimum_speed_mps +
          speed_ratio * (config_.maximum_speed_mps - config_.minimum_speed_mps);
      const double acceleration = (requested_speed - speed) / config_.horizon_time_s;
      if (std::abs(acceleration) > config_.maximum_acceleration_mps2) {
        ++result.diagnostics.dynamic_limit_rejections;
        ++result.diagnostics.acceleration_rejections;
        continue;
      }
      const double average_speed = 0.5 * (speed + requested_speed);
      const double rollout_length = std::min(
          {config_.horizon_m, goal_distance,
           std::max({config_.sample_step_m, average_speed * config_.horizon_time_s,
                     std::max(0.0, input.minimum_length_m)})});
      const double heading_delta =
          clampAngle(terminal_heading - candidate_initial_heading);
      const double curvature = heading_delta / rollout_length;
      if (std::abs(curvature) > config_.maximum_curvature_1pm) {
        ++result.diagnostics.dynamic_limit_rejections;
        ++result.diagnostics.curvature_rejections;
        continue;
      }
      if (requested_speed * requested_speed * std::abs(curvature) >
          config_.maximum_lateral_acceleration_mps2) {
        ++result.diagnostics.dynamic_limit_rejections;
        ++result.diagnostics.lateral_acceleration_rejections;
        continue;
      }
      const std::size_t sample_count = std::max<std::size_t>(
          2U,
          static_cast<std::size_t>(std::ceil(rollout_length / config_.sample_step_m)) +
              1U);
      RolloutCandidate candidate;
      candidate.heading_offset_rad = offset;
      candidate.target_speed_mps = requested_speed;
      candidate.curvature_1pm = curvature;
      candidate.deterministic_index =
          candidate_index * config_.speed_samples + speed_index;
      candidate.samples.reserve(sample_count);
      for (std::size_t index = 0U; index < sample_count; ++index) {
        const double s = rollout_length * static_cast<double>(index) /
                         static_cast<double>(sample_count - 1U);
        const double heading = candidate_initial_heading + curvature * s;
        Point2 point{};
        if (std::abs(curvature) <= 1.0e-9) {
          point = Point2{input.position.x + s * std::cos(candidate_initial_heading),
                         input.position.y + s * std::sin(candidate_initial_heading)};
        } else {
          point = Point2{input.position.x +
                             (std::sin(heading) - std::sin(candidate_initial_heading)) /
                                 curvature,
                         input.position.y -
                             (std::cos(heading) - std::cos(candidate_initial_heading)) /
                                 curvature};
        }
        TrajectoryPointSample sample;
        sample.s_m = s;
        sample.point = point;
        sample.tangent = Point2{std::cos(heading), std::sin(heading)};
        sample.curvature_1pm = curvature;
        candidate.samples.push_back(std::move(sample));
      }
      populateTrajectorySampleGeometry(candidate.samples);
      ++result.diagnostics.generated;
      candidate.risk = risk_field.evaluate(*input.grid, candidate.samples);
      if (!candidate.risk.hardValid()) {
        recordHardRejection(*input.grid, candidate.samples,
                            candidate.deterministic_index, result.diagnostics);
        ++result.diagnostics.grid_rejections;
        continue;
      }
      if (candidate.risk.minimum_raw_clearance_m + 1.0e-9 <
          effective_path_clearance_m) {
        recordClearanceRejection(*input.grid, risk_field.occupiedClearance(),
                                 candidate.samples, candidate.deterministic_index,
                                 effective_path_clearance_m, result.diagnostics);
        ++result.diagnostics.grid_rejections;
        continue;
      }
      const std::optional<GridIndex> terminal_cell =
          input.grid->worldToCell(candidate.samples.back().point);
      const double required_terminal_clearance_m =
          std::max(0.0, input.minimum_terminal_clearance_m);
      if (!terminal_cell.has_value() ||
          risk_field.occupiedClearance().distanceAt(*terminal_cell) + 1.0e-9 <
              required_terminal_clearance_m) {
        if (!result.diagnostics.first_grid_rejection.has_value()) {
          result.diagnostics.first_grid_rejection = RolloutGridRejectionDiagnostic{
              .reason = terminal_cell.has_value()
                            ? RolloutGridRejectReason::kTerminalResponseEnvelope
                            : RolloutGridRejectReason::kOutsideGrid,
              .deterministic_index = candidate.deterministic_index,
              .segment_index = candidate.samples.size() - 1U,
              .position = candidate.samples.back().point,
              .cell = terminal_cell,
          };
        }
        if (!terminal_cell.has_value()) {
          ++result.diagnostics.outside_grid_rejections;
        }
        ++result.diagnostics.grid_rejections;
        continue;
      }
      const double terminal_stopping_distance_m =
          std::max(0.0, input.minimum_terminal_stopping_distance_m);
      if (terminal_stopping_distance_m > 0.0) {
        const TrajectoryPointSample& terminal = candidate.samples.back();
        std::array<TrajectoryPointSample, 2U> stopping_samples{terminal, terminal};
        stopping_samples.back().s_m = terminal.s_m + terminal_stopping_distance_m;
        stopping_samples.back().point = Point2{
            terminal.point.x + terminal.tangent.x * terminal_stopping_distance_m,
            terminal.point.y + terminal.tangent.y * terminal_stopping_distance_m,
        };
        stopping_samples.back().curvature_1pm = 0.0;
        const PathRiskScore stopping_risk =
            risk_field.evaluate(*input.grid, stopping_samples);
        if (!stopping_risk.hardValid() ||
            stopping_risk.minimum_raw_clearance_m + 1.0e-9 <
                requested_path_clearance_m) {
          recordTerminalStoppingRejection(
              *input.grid, risk_field.occupiedClearance(), stopping_samples,
              candidate.deterministic_index, candidate.samples.size() - 1U,
              requested_path_clearance_m, result.diagnostics);
          ++result.diagnostics.grid_rejections;
          continue;
        }
      }
      const double after =
          distance(candidate.samples.back().point, input.preferred_target);
      candidate.progress_m = goal_distance - after;
      candidate.progress_cost = -config_.progress_weight * candidate.progress_m;
      candidate.lateral_deviation_cost =
          config_.lateral_deviation_weight * std::abs(offset) * rollout_length;
      candidate.heading_change_cost =
          config_.heading_change_weight * std::abs(base_heading_error + offset);
      candidate.curvature_cost =
          config_.curvature_weight * std::abs(curvature) * rollout_length;
      candidate.score = candidate.progress_cost + candidate.lateral_deviation_cost +
                        candidate.heading_change_cost + candidate.curvature_cost;
      result.ranked_candidates.push_back(std::move(candidate));
    }
  }

  std::ranges::sort(result.ranked_candidates, [](const RolloutCandidate& lhs,
                                                 const RolloutCandidate& rhs) {
    RankedPathCost lhs_cost{
        .risk = lhs.risk,
        .algorithm_cost = lhs.score,
        .deterministic_tiebreak = static_cast<std::uint64_t>(lhs.deterministic_index),
    };
    RankedPathCost rhs_cost{
        .risk = rhs.risk,
        .algorithm_cost = rhs.score,
        .deterministic_tiebreak = static_cast<std::uint64_t>(rhs.deterministic_index),
    };
    return rankedPathCostLess(lhs_cost, rhs_cost);
  });
  if (result.ranked_candidates.empty()) {
    result.reject_reason = RolloutRejectReason::kNoCandidate;
  }
  return result;
}

const RolloutPlannerConfig& RecedingHorizonTrajectoryPlanner::config() const noexcept {
  return config_;
}

const char* rolloutRejectReasonName(const RolloutRejectReason reason) noexcept {
  switch (reason) {
    case RolloutRejectReason::kNone:
      return "none";
    case RolloutRejectReason::kInvalidInput:
      return "invalid_input";
    case RolloutRejectReason::kOutsideGrid:
      return "outside_grid";
    case RolloutRejectReason::kNoCandidate:
      return "no_candidate";
  }
  return "unknown";
}

const char* rolloutGridRejectReasonName(const RolloutGridRejectReason reason) noexcept {
  switch (reason) {
    case RolloutGridRejectReason::kOutsideGrid:
      return "outside_grid";
    case RolloutGridRejectReason::kRawOccupied:
      return "raw_occupied";
    case RolloutGridRejectReason::kVehicleClearanceEnvelope:
      return "vehicle_clearance_envelope";
    case RolloutGridRejectReason::kTerminalResponseEnvelope:
      return "terminal_response_envelope";
    case RolloutGridRejectReason::kTerminalStoppingEnvelope:
      return "terminal_stopping_envelope";
  }
  return "unknown";
}

} // namespace drone_city_nav
