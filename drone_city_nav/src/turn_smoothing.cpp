#include "turn_smoothing_internal.hpp"

namespace drone_city_nav {

using namespace turn_smoothing_detail;

TurnSmoothingResult
smoothTrajectoryTurns(const std::span<const TrajectoryPointSample> samples,
                      const std::span<const CorridorSample> corridor_samples,
                      const OccupancyGrid2D& prohibited_grid,
                      const TurnSmoothingConfig& config,
                      const VelocityFollowerConfig& speed_config) {
  TurnSmoothingResult result{};
  result.stats.input_samples = samples.size();
  result.samples.assign(samples.begin(), samples.end());
  if (samples.size() < 3U || corridor_samples.empty()) {
    result.stats.output_samples = result.samples.size();
    result.valid = trajectorySamplesAreUsable(result.samples);
    return result;
  }
  populateSampleGeometry(result.samples);

  const TrajectoryShapeDiagnostics initial_shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.max_heading_delta_before_rad = initial_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_before_1pm = initial_shape.max_curvature_jump_1pm;

  const std::size_t max_passes =
      std::clamp<std::size_t>(config.max_passes, 0U, static_cast<std::size_t>(100U));
  for (std::size_t pass = 0U; pass < max_passes; ++pass) {
    TurnSmoothingStats detection_stats{};
    const std::optional<CornerCandidate> corner =
        worstCorner(result.samples, config, speed_config, detection_stats);
    result.stats.detected_corners += detection_stats.detected_corners;
    if (std::isfinite(detection_stats.min_inner_margin_m)) {
      if (!std::isfinite(result.stats.min_inner_margin_m)) {
        result.stats.min_inner_margin_m = detection_stats.min_inner_margin_m;
      } else {
        result.stats.min_inner_margin_m = std::min(result.stats.min_inner_margin_m,
                                                   detection_stats.min_inner_margin_m);
      }
    }
    if (!corner.has_value()) {
      break;
    }

    TurnSmoothingWorkBuffer work_buffer{};
    work_buffer.before_local.reserve(result.samples.size());
    work_buffer.replacement.reserve(result.samples.size());
    work_buffer.candidate.reserve(result.samples.size() * 2U);
    work_buffer.after_local.reserve(result.samples.size());
    ++result.stats.attempted_corners;
    const double entry_distance =
        sanitizedPositive(config.entry_distance_m, 45.0, 0.1, 5000.0);
    const double exit_distance =
        sanitizedPositive(config.exit_distance_m, 45.0, 0.1, 5000.0);
    const std::vector<double> entry_candidates =
        distanceFallbackCandidates(entry_distance);
    constexpr std::array<double, 4U> kShiftScales = {1.0, 0.5, 0.25, 0.0};
    SmoothingRejectReason dominant_reject_reason = SmoothingRejectReason::kNone;
    std::optional<SmoothingAttempt> accepted_attempt;
    std::optional<SmoothingAttempt> rejected_attempt;
    std::optional<std::size_t> selected_candidate_diagnostic_index;
    const double corner_s_m = corner->index < result.samples.size()
                                  ? result.samples[corner->index].s_m
                                  : std::numeric_limits<double>::quiet_NaN();
    const auto record_attempt = [&](const SmoothingAttempt& attempt,
                                    const std::size_t attempt_index) {
      result.stats.candidate_diagnostics.push_back(candidateDiagnosticFromAttempt(
          attempt, *corner, pass, attempt_index, corner_s_m));
      return result.stats.candidate_diagnostics.size() - 1U;
    };
    const auto consider_attempt = [&](SmoothingAttempt&& attempt,
                                      const std::size_t diagnostic_index) {
      if (attempt.accepted) {
        if (!accepted_attempt.has_value() || attempt.score < accepted_attempt->score) {
          accepted_attempt = std::move(attempt);
          selected_candidate_diagnostic_index = diagnostic_index;
        }
        return;
      }
      if (rejectReasonDominates(attempt.reject_reason, dominant_reject_reason)) {
        dominant_reject_reason = attempt.reject_reason;
      }
      if (!rejected_attempt.has_value() ||
          (std::isfinite(attempt.score) && attempt.score < rejected_attempt->score)) {
        rejected_attempt = std::move(attempt);
      }
    };
    for (const double entry_candidate : entry_candidates) {
      const double distance_scale =
          entry_distance > kTinyDistanceM ? entry_candidate / entry_distance : 1.0;
      for (const double shift_scale : kShiftScales) {
        const std::size_t attempt_index = result.stats.candidate_attempts++;
        SmoothingAttempt attempt = trySmoothCorner(
            result.samples, corridor_samples, prohibited_grid, *corner, config,
            entry_distance * distance_scale, exit_distance * distance_scale,
            shift_scale, 0.0, speed_config, work_buffer, result.stats);
        const std::size_t diagnostic_index = record_attempt(attempt, attempt_index);
        consider_attempt(std::move(attempt), diagnostic_index);
      }
    }
    for (const double entry_candidate : entry_candidates) {
      const double distance_scale =
          entry_distance > kTinyDistanceM ? entry_candidate / entry_distance : 1.0;
      for (const double relaxed_angle : relaxedTangentAngleCandidatesRad()) {
        for (const double shift_scale : kShiftScales) {
          const std::size_t attempt_index = result.stats.candidate_attempts++;
          ++result.stats.relaxed_candidate_attempts;
          SmoothingAttempt attempt = trySmoothCorner(
              result.samples, corridor_samples, prohibited_grid, *corner, config,
              entry_distance * distance_scale, exit_distance * distance_scale,
              shift_scale, relaxed_angle, speed_config, work_buffer, result.stats);
          const std::size_t diagnostic_index = record_attempt(attempt, attempt_index);
          consider_attempt(std::move(attempt), diagnostic_index);
        }
      }
    }
    if (!accepted_attempt.has_value()) {
      incrementRejectStat(result.stats,
                          dominant_reject_reason == SmoothingRejectReason::kNone
                              ? SmoothingRejectReason::kNotImproved
                              : dominant_reject_reason);
      if (rejected_attempt.has_value()) {
        result.stats.corner_diagnostics.push_back(
            cornerDiagnosticFromAttempt(*rejected_attempt, corner_s_m));
      }
      break;
    }

    populateAttemptSpeedDiagnostics(result.samples, *accepted_attempt, speed_config,
                                    work_buffer, result.stats);
    if (selected_candidate_diagnostic_index.has_value() &&
        *selected_candidate_diagnostic_index <
            result.stats.candidate_diagnostics.size()) {
      result.stats.candidate_diagnostics[*selected_candidate_diagnostic_index]
          .decision = "selected";
      updateCandidateSpeedDiagnostics(
          result.stats.candidate_diagnostics[*selected_candidate_diagnostic_index],
          *accepted_attempt);
    }
    result.stats.corner_diagnostics.push_back(
        cornerDiagnosticFromAttempt(*accepted_attempt, corner_s_m));
    result.samples = std::move(accepted_attempt->samples);
    result.changed = true;
    ++result.stats.smoothed_corners;
    result.stats.max_applied_outer_shift_m = std::max(
        result.stats.max_applied_outer_shift_m, accepted_attempt->applied_shift_m);
    result.stats.accepted_entry_distance_m = accepted_attempt->entry_distance_m;
    result.stats.accepted_exit_distance_m = accepted_attempt->exit_distance_m;
    result.stats.accepted_shift_scale = accepted_attempt->shift_scale;
    result.stats.accepted_relaxed_angle_deg =
        radiansToDegrees(accepted_attempt->relaxed_angle_rad);
    result.stats.accepted_score = accepted_attempt->score;
    result.stats.accepted_min_radius_before_m =
        accepted_attempt->before_metrics.min_radius_m;
    result.stats.accepted_min_radius_after_m =
        accepted_attempt->after_metrics.min_radius_m;
    result.stats.accepted_min_speed_before_mps =
        accepted_attempt->before_metrics.min_speed_limit_mps;
    result.stats.accepted_min_speed_after_mps =
        accepted_attempt->after_metrics.min_speed_limit_mps;
    result.stats.accepted_local_time_before_s =
        accepted_attempt->before_metrics.estimated_time_s;
    result.stats.accepted_local_time_after_s =
        accepted_attempt->after_metrics.estimated_time_s;
  }

  populateSampleGeometry(result.samples);
  const TrajectoryShapeDiagnostics final_shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.max_heading_delta_after_rad = final_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_after_1pm = final_shape.max_curvature_jump_1pm;
  result.valid = trajectorySamplesAreUsable(result.samples) &&
                 pathIsTraversable(prohibited_grid, result.samples);
  return result;
}

} // namespace drone_city_nav
