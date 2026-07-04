#include "turn_smoothing_internal.hpp"

namespace drone_city_nav::turn_smoothing_detail {

void populateAttemptSpeedDiagnostics(
    const std::span<const TrajectoryPointSample> before_samples,
    SmoothingAttempt& attempt, const VelocityFollowerConfig& speed_config,
    TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats) {
  if (attempt.samples.empty() || before_samples.empty() ||
      attempt.entry_index >= before_samples.size() ||
      attempt.exit_index >= before_samples.size() ||
      attempt.entry_index >= attempt.exit_index ||
      attempt.replacement_sample_count < 2U) {
    return;
  }
  sampleRangeInto(before_samples, attempt.entry_index, attempt.exit_index,
                  buffer.before_local);
  attempt.before_metrics =
      localTrajectoryMetrics(buffer.before_local, speed_config, &stats, true);

  const std::size_t after_end_index =
      attempt.entry_index + attempt.replacement_sample_count - 1U;
  if (after_end_index >= attempt.samples.size()) {
    return;
  }
  sampleRangeInto(attempt.samples, attempt.entry_index, after_end_index,
                  buffer.after_local);
  attempt.after_metrics =
      localTrajectoryMetrics(buffer.after_local, speed_config, &stats, true);
  attempt.score = smoothingAttemptScore(attempt.after_metrics);
}

void updateCandidateSpeedDiagnostics(TurnSmoothingCandidateDiagnostic& diagnostic,
                                     const SmoothingAttempt& attempt) noexcept {
  diagnostic.min_speed_before_mps = attempt.before_metrics.min_speed_limit_mps;
  diagnostic.min_speed_after_mps = attempt.after_metrics.min_speed_limit_mps;
  diagnostic.local_time_before_s = attempt.before_metrics.estimated_time_s;
  diagnostic.local_time_after_s = attempt.after_metrics.estimated_time_s;
}

[[nodiscard]] bool rejectReasonDominates(const SmoothingRejectReason candidate,
                                         const SmoothingRejectReason current) noexcept {
  const auto priority = [](const SmoothingRejectReason reason) {
    switch (reason) {
      case SmoothingRejectReason::kProhibited:
        return 7;
      case SmoothingRejectReason::kCurvatureRegression:
        return 5;
      case SmoothingRejectReason::kRadiusRegression:
        return 4;
      case SmoothingRejectReason::kNotImproved:
        return 1;
      case SmoothingRejectReason::kNone:
        return 0;
    }
    return 0;
  };
  return priority(candidate) > priority(current);
}

[[nodiscard]] const char*
rejectReasonName(const SmoothingRejectReason reason) noexcept {
  switch (reason) {
    case SmoothingRejectReason::kNone:
      return "none";
    case SmoothingRejectReason::kProhibited:
      return "prohibited";
    case SmoothingRejectReason::kNotImproved:
      return "not_improved";
    case SmoothingRejectReason::kCurvatureRegression:
      return "curvature_regression";
    case SmoothingRejectReason::kRadiusRegression:
      return "radius_regression";
  }
  return "unknown";
}

[[nodiscard]] TurnSmoothingCornerDiagnostic
cornerDiagnosticFromAttempt(const SmoothingAttempt& attempt,
                            const double corner_s_m) noexcept {
  TurnSmoothingCornerDiagnostic diagnostic{};
  diagnostic.accepted = attempt.accepted;
  diagnostic.reject_reason = rejectReasonName(attempt.reject_reason);
  diagnostic.reject_detail = attempt.reject_detail;
  diagnostic.corner_s_m = corner_s_m;
  diagnostic.entry_distance_m = attempt.entry_distance_m;
  diagnostic.exit_distance_m = attempt.exit_distance_m;
  diagnostic.shift_scale = attempt.shift_scale;
  diagnostic.relaxed_angle_deg = radiansToDegrees(attempt.relaxed_angle_rad);
  diagnostic.score = attempt.score;
  diagnostic.min_radius_before_m = attempt.before_metrics.min_radius_m;
  diagnostic.min_radius_after_m = attempt.after_metrics.min_radius_m;
  diagnostic.min_speed_before_mps = attempt.before_metrics.min_speed_limit_mps;
  diagnostic.min_speed_after_mps = attempt.after_metrics.min_speed_limit_mps;
  diagnostic.local_time_before_s = attempt.before_metrics.estimated_time_s;
  diagnostic.local_time_after_s = attempt.after_metrics.estimated_time_s;
  diagnostic.curvature_jump_before_1pm =
      attempt.before_metrics.shape.max_curvature_jump_1pm;
  diagnostic.curvature_jump_after_1pm =
      attempt.after_metrics.shape.max_curvature_jump_1pm;
  diagnostic.heading_delta_before_rad =
      attempt.before_metrics.shape.max_heading_delta_rad;
  diagnostic.heading_delta_after_rad =
      attempt.after_metrics.shape.max_heading_delta_rad;
  return diagnostic;
}

[[nodiscard]] TurnSmoothingCandidateDiagnostic candidateDiagnosticFromAttempt(
    const SmoothingAttempt& attempt, const CornerCandidate& corner,
    const std::size_t pass, const std::size_t attempt_index, const double corner_s_m) {
  TurnSmoothingCandidateDiagnostic diagnostic{};
  diagnostic.decision = attempt.accepted ? "valid_not_best" : "rejected";
  diagnostic.reject_reason = rejectReasonName(attempt.reject_reason);
  diagnostic.reject_detail = attempt.reject_detail;
  diagnostic.pass = pass;
  diagnostic.attempt_index = attempt_index;
  diagnostic.corner_index = corner.index;
  diagnostic.corner_s_m = corner_s_m;
  diagnostic.entry_distance_m = attempt.entry_distance_m;
  diagnostic.exit_distance_m = attempt.exit_distance_m;
  diagnostic.shift_scale = attempt.shift_scale;
  diagnostic.applied_shift_m = attempt.applied_shift_m;
  diagnostic.relaxed_angle_deg = radiansToDegrees(attempt.relaxed_angle_rad);
  diagnostic.score = attempt.score;
  diagnostic.min_radius_before_m = attempt.before_metrics.min_radius_m;
  diagnostic.min_radius_after_m = attempt.after_metrics.min_radius_m;
  diagnostic.min_speed_before_mps = attempt.before_metrics.min_speed_limit_mps;
  diagnostic.min_speed_after_mps = attempt.after_metrics.min_speed_limit_mps;
  diagnostic.local_time_before_s = attempt.before_metrics.estimated_time_s;
  diagnostic.local_time_after_s = attempt.after_metrics.estimated_time_s;
  diagnostic.curvature_jump_before_1pm =
      attempt.before_metrics.shape.max_curvature_jump_1pm;
  diagnostic.curvature_jump_after_1pm =
      attempt.after_metrics.shape.max_curvature_jump_1pm;
  diagnostic.heading_delta_before_rad =
      attempt.before_metrics.shape.max_heading_delta_rad;
  diagnostic.heading_delta_after_rad =
      attempt.after_metrics.shape.max_heading_delta_rad;
  return diagnostic;
}

void incrementRejectStat(TurnSmoothingStats& stats,
                         const SmoothingRejectReason reason) noexcept {
  switch (reason) {
    case SmoothingRejectReason::kProhibited:
      ++stats.rejected_prohibited;
      break;
    case SmoothingRejectReason::kNotImproved:
      ++stats.rejected_not_improved;
      break;
    case SmoothingRejectReason::kCurvatureRegression:
      ++stats.rejected_curvature_regression;
      break;
    case SmoothingRejectReason::kRadiusRegression:
      ++stats.rejected_radius_regression;
      break;
    case SmoothingRejectReason::kNone:
      break;
  }
}

} // namespace drone_city_nav::turn_smoothing_detail
