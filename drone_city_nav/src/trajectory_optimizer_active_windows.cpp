#include "trajectory_optimizer_internal.hpp"

namespace drone_city_nav::trajectory_optimizer_detail {

void addActiveWindow(std::vector<ActiveWindow>& windows,
                     const std::span<const CorridorSample> samples,
                     const std::size_t center_index, const double pre_margin_m,
                     const double post_margin_m) {
  if (samples.empty()) {
    return;
  }
  const double begin_s = samples[center_index].s_m - pre_margin_m;
  const double end_s = samples[center_index].s_m + post_margin_m;
  std::size_t begin = center_index;
  while (begin > 0U && samples[begin].s_m > begin_s) {
    --begin;
  }
  std::size_t end = center_index;
  while (end + 1U < samples.size() && samples[end].s_m < end_s) {
    ++end;
  }
  if (begin >= end) {
    return;
  }
  if (!windows.empty() && begin >= windows.back().begin_index &&
      begin <= windows.back().end_index + 1U) {
    windows.back().end_index = std::max(windows.back().end_index, end);
    return;
  }
  windows.push_back(ActiveWindow{begin, end});
}

[[nodiscard]] bool addActiveWindowRange(std::vector<ActiveWindow>& windows,
                                        const std::span<const CorridorSample> samples,
                                        const double begin_s_m, const double end_s_m) {
  if (samples.empty() || !std::isfinite(begin_s_m) || !std::isfinite(end_s_m) ||
      begin_s_m >= end_s_m) {
    return false;
  }

  std::size_t begin = 0U;
  while (begin + 1U < samples.size() && samples[begin + 1U].s_m <= begin_s_m) {
    ++begin;
  }
  std::size_t end = begin;
  while (end + 1U < samples.size() && samples[end].s_m < end_s_m) {
    ++end;
  }
  if (begin >= end) {
    return false;
  }
  if (!windows.empty() && begin >= windows.back().begin_index &&
      begin <= windows.back().end_index + 1U) {
    windows.back().end_index = std::max(windows.back().end_index, end);
    return true;
  }
  windows.push_back(ActiveWindow{begin, end});
  return true;
}

void mergeActiveWindows(std::vector<ActiveWindow>& windows) {
  if (windows.size() < 2U) {
    return;
  }
  std::sort(windows.begin(), windows.end(),
            [](const ActiveWindow& lhs, const ActiveWindow& rhs) {
              if (lhs.begin_index == rhs.begin_index) {
                return lhs.end_index < rhs.end_index;
              }
              return lhs.begin_index < rhs.begin_index;
            });
  std::vector<ActiveWindow> merged;
  merged.reserve(windows.size());
  for (const ActiveWindow& window : windows) {
    if (window.begin_index >= window.end_index) {
      continue;
    }
    if (!merged.empty() && window.begin_index <= merged.back().end_index + 1U) {
      merged.back().end_index = std::max(merged.back().end_index, window.end_index);
      continue;
    }
    merged.push_back(window);
  }
  windows = std::move(merged);
}

[[nodiscard]] std::size_t
activeWindowControlSampleCount(const std::span<const ActiveWindow> windows) {
  std::size_t count = 0U;
  for (const ActiveWindow& window : windows) {
    if (window.end_index > window.begin_index + 1U) {
      count += window.end_index - window.begin_index - 1U;
    }
  }
  return count;
}

[[nodiscard]] std::size_t fullPathActiveSampleCount(const std::size_t sample_count) {
  return sample_count > 2U ? sample_count - 2U : 0U;
}

[[nodiscard]] double headingSpanAround(const std::span<const CorridorSample> samples,
                                       const std::size_t center_index,
                                       const double pre_margin_m,
                                       const double post_margin_m) {
  if (samples.empty() || center_index >= samples.size()) {
    return 0.0;
  }
  const double begin_s = samples[center_index].s_m - pre_margin_m;
  const double end_s = samples[center_index].s_m + post_margin_m;
  std::size_t begin = center_index;
  while (begin > 0U && samples[begin].s_m > begin_s) {
    --begin;
  }
  std::size_t end = center_index;
  while (end + 1U < samples.size() && samples[end].s_m < end_s) {
    ++end;
  }
  if (begin >= end) {
    return 0.0;
  }

  const double reference =
      std::atan2(samples[begin].tangent.y, samples[begin].tangent.x);
  double min_heading = 0.0;
  double max_heading = 0.0;
  for (std::size_t i = begin; i <= end; ++i) {
    double heading = std::atan2(samples[i].tangent.y, samples[i].tangent.x);
    while (heading - reference > std::numbers::pi) {
      heading -= 2.0 * std::numbers::pi;
    }
    while (heading - reference < -std::numbers::pi) {
      heading += 2.0 * std::numbers::pi;
    }
    const double relative_heading = heading - reference;
    min_heading = std::min(min_heading, relative_heading);
    max_heading = std::max(max_heading, relative_heading);
  }
  return max_heading - min_heading;
}

[[nodiscard]] std::vector<ActiveWindow> detectActiveWindows(
    const std::span<const CorridorSample> samples,
    const std::span<const Point2> centerline, const OccupancyGrid2D& prohibited_grid,
    const TrajectoryOptimizerConfig& config, TrajectoryOptimizerStats& stats) {
  const auto started_at = std::chrono::steady_clock::now();
  std::vector<ActiveWindow> windows;
  std::vector<ActiveWindow> shadow_no_width_asymmetry_windows;
  std::vector<ActiveWindow> shadow_no_width_triggers_windows;
  std::vector<ActiveWindow> shadow_no_heading_span_windows;
  if (samples.size() < 3U) {
    stats.window_detection_duration_ms = elapsedMilliseconds(started_at);
    return windows;
  }

  const PathEvaluation centerline_evaluation =
      evaluatePath(prohibited_grid, centerline);
  const double pre_margin_m =
      sanitizedPositive(config.window_pre_margin_m, 25.0, 0.0, 5000.0);
  const double post_margin_m =
      sanitizedPositive(config.window_post_margin_m, 25.0, 0.0, 5000.0);
  if (!centerline_evaluation.traversable()) {
    stats.active_window_centerline_blocked = 1U;
    stats.centerline_blocked_prohibited_cells = centerline_evaluation.prohibited_cells;
    stats.centerline_blocked_outside_grid_segments =
        centerline_evaluation.outside_grid_segments;
    stats.centerline_blocked_segment_count =
        centerline_evaluation.blocked_segment_count;
    stats.centerline_blocked_span_count = centerline_evaluation.blocked_span_count;
    stats.centerline_blocked_span_diagnostic_count =
        centerline_evaluation.blocked_span_diagnostic_count;
    stats.centerline_blocked_span_diagnostics =
        centerline_evaluation.blocked_span_diagnostics;
    if (centerline_evaluation.has_first_blocked_point) {
      stats.centerline_blocked_first_segment_index =
          centerline_evaluation.first_blocked_segment_index;
      stats.centerline_blocked_first_s_m = centerline_evaluation.first_blocked_s_m;
      stats.centerline_blocked_first_x_m = centerline_evaluation.first_blocked_point.x;
      stats.centerline_blocked_first_y_m = centerline_evaluation.first_blocked_point.y;
      stats.centerline_blocked_first_outside_grid =
          centerline_evaluation.first_blocked_outside_grid;
    }
    if (centerline_evaluation.has_last_blocked_point) {
      stats.centerline_blocked_last_segment_index =
          centerline_evaluation.last_blocked_segment_index;
      stats.centerline_blocked_last_s_m = centerline_evaluation.last_blocked_s_m;
      stats.centerline_blocked_last_x_m = centerline_evaluation.last_blocked_point.x;
      stats.centerline_blocked_last_y_m = centerline_evaluation.last_blocked_point.y;
      stats.centerline_blocked_last_outside_grid =
          centerline_evaluation.last_blocked_outside_grid;
    }
    if (std::isfinite(stats.centerline_blocked_first_s_m) &&
        std::isfinite(stats.centerline_blocked_last_s_m)) {
      stats.centerline_blocked_span_length_m =
          stats.centerline_blocked_last_s_m - stats.centerline_blocked_first_s_m;
    }

    bool use_full_path_fallback = centerline_evaluation.outside_grid_segments > 0U ||
                                  centerline_evaluation.blocked_span_count == 0U ||
                                  centerline_evaluation.blocked_span_count >
                                      kMaxCenterlineBlockedSpanDiagnostics ||
                                  centerline_evaluation.blocked_span_diagnostic_count !=
                                      centerline_evaluation.blocked_span_count;
    if (!use_full_path_fallback) {
      for (std::size_t i = 0U; i < centerline_evaluation.blocked_span_diagnostic_count;
           ++i) {
        const TrajectoryOptimizerBlockedSpanDiagnostic& span =
            centerline_evaluation.blocked_span_diagnostics.at(i);
        if (span.outside_grid_segments > 0U || !std::isfinite(span.begin_s_m) ||
            !std::isfinite(span.end_s_m) || span.begin_s_m >= span.end_s_m) {
          use_full_path_fallback = true;
          break;
        }
        if (!addActiveWindowRange(windows, samples, span.begin_s_m - pre_margin_m,
                                  span.end_s_m + post_margin_m)) {
          use_full_path_fallback = true;
          break;
        }
        ++stats.centerline_blocked_windows;
      }
    }

    if (!use_full_path_fallback) {
      mergeActiveWindows(windows);
      stats.centerline_blocked_window_merged_count = windows.size();
      stats.centerline_blocked_window_samples = activeWindowControlSampleCount(windows);
      const std::size_t full_path_samples = fullPathActiveSampleCount(samples.size());
      if (windows.empty() || full_path_samples == 0U ||
          stats.centerline_blocked_window_samples * 10U >= full_path_samples * 9U) {
        use_full_path_fallback = true;
      }
    }

    if (use_full_path_fallback) {
      windows.clear();
      windows.push_back(ActiveWindow{0U, samples.size() - 1U});
      stats.window_count = 1U;
      stats.active_window_count = 1U;
      stats.active_window_samples = fullPathActiveSampleCount(samples.size());
      stats.shadow_active_window_no_width_asymmetry_count = 1U;
      stats.shadow_active_window_no_width_asymmetry_samples =
          stats.active_window_samples;
      stats.shadow_active_window_no_width_triggers_count = 1U;
      stats.shadow_active_window_no_width_triggers_samples =
          stats.active_window_samples;
      stats.shadow_active_window_no_heading_span_count = 1U;
      stats.shadow_active_window_no_heading_span_samples = stats.active_window_samples;
      stats.window_detection_duration_ms = elapsedMilliseconds(started_at);
      return windows;
    }
  }
  shadow_no_width_asymmetry_windows = windows;
  shadow_no_width_triggers_windows = windows;
  shadow_no_heading_span_windows = windows;

  const double heading_threshold_rad =
      sanitizedPositive(config.window_heading_threshold_rad,
                        10.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  const double heading_span_threshold_rad =
      sanitizedPositive(config.window_min_heading_span_rad,
                        10.0 * std::numbers::pi / 180.0, 0.0, std::numbers::pi);
  const double curvature_threshold =
      sanitizedPositive(config.window_min_curvature_1pm, 0.01, 0.0, 1000.0);
  const double width_threshold_m =
      sanitizedPositive(config.window_width_change_threshold_m, 2.0, 0.0, 5000.0);
  const double width_asymmetry_threshold_m =
      sanitizedPositive(config.window_min_width_asymmetry_m, 1.0, 0.0, 5000.0);
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const double heading_change =
        headingDeltaRad(samples[i - 1U].tangent, samples[i + 1U].tangent);
    const double heading_span =
        headingSpanAround(samples, i, pre_margin_m, post_margin_m);
    const double curvature = std::abs(
        discreteCurvature(centerline[i - 1U], centerline[i], centerline[i + 1U]));
    const double previous_width =
        samples[i - 1U].left_bound_m + samples[i - 1U].right_bound_m;
    const double next_width =
        samples[i + 1U].left_bound_m + samples[i + 1U].right_bound_m;
    const double width_asymmetry =
        std::abs(samples[i].left_bound_m - samples[i].right_bound_m);
    const bool heading_change_trigger = heading_change >= heading_threshold_rad;
    const bool heading_span_trigger = heading_span >= heading_span_threshold_rad;
    const bool curvature_trigger = curvature >= curvature_threshold;
    const bool width_change_trigger =
        std::abs(next_width - previous_width) >= width_threshold_m;
    const bool width_asymmetry_trigger = width_asymmetry >= width_asymmetry_threshold_m;
    if (heading_change_trigger) {
      ++stats.active_window_heading_change_samples;
    }
    if (heading_span_trigger) {
      ++stats.active_window_heading_span_samples;
    }
    if (curvature_trigger) {
      ++stats.active_window_curvature_samples;
    }
    if (width_change_trigger) {
      ++stats.active_window_width_change_samples;
    }
    if (width_asymmetry_trigger) {
      ++stats.active_window_width_asymmetry_samples;
    }
    const bool turn_zone =
        heading_change_trigger || heading_span_trigger || curvature_trigger;
    const bool width_zone = width_change_trigger || width_asymmetry_trigger;
    if (turn_zone || width_zone) {
      addActiveWindow(windows, samples, i, pre_margin_m, post_margin_m);
    }
    if (turn_zone || width_change_trigger) {
      addActiveWindow(shadow_no_width_asymmetry_windows, samples, i, pre_margin_m,
                      post_margin_m);
    }
    if (turn_zone) {
      addActiveWindow(shadow_no_width_triggers_windows, samples, i, pre_margin_m,
                      post_margin_m);
    }
    if (heading_change_trigger || curvature_trigger || width_zone) {
      addActiveWindow(shadow_no_heading_span_windows, samples, i, pre_margin_m,
                      post_margin_m);
    }
  }

  mergeActiveWindows(windows);
  mergeActiveWindows(shadow_no_width_asymmetry_windows);
  mergeActiveWindows(shadow_no_width_triggers_windows);
  mergeActiveWindows(shadow_no_heading_span_windows);
  stats.window_count = windows.size();
  stats.active_window_count = windows.size();
  stats.active_window_samples = activeWindowControlSampleCount(windows);
  stats.shadow_active_window_no_width_asymmetry_count =
      shadow_no_width_asymmetry_windows.size();
  stats.shadow_active_window_no_width_asymmetry_samples =
      activeWindowControlSampleCount(shadow_no_width_asymmetry_windows);
  stats.shadow_active_window_no_width_triggers_count =
      shadow_no_width_triggers_windows.size();
  stats.shadow_active_window_no_width_triggers_samples =
      activeWindowControlSampleCount(shadow_no_width_triggers_windows);
  stats.shadow_active_window_no_heading_span_count =
      shadow_no_heading_span_windows.size();
  stats.shadow_active_window_no_heading_span_samples =
      activeWindowControlSampleCount(shadow_no_heading_span_windows);
  stats.window_detection_duration_ms = elapsedMilliseconds(started_at);
  return windows;
}

[[nodiscard]] std::vector<std::size_t>
activeControlIndices(const std::span<const ActiveWindow> windows,
                     const std::size_t sample_count,
                     std::vector<std::uint8_t>& mutable_indices) {
  mutable_indices.assign(sample_count, 0U);
  std::vector<std::size_t> indices;
  for (const ActiveWindow& window : windows) {
    const std::size_t begin = std::min(window.begin_index + 1U, sample_count);
    const std::size_t end = std::min(window.end_index, sample_count);
    for (std::size_t i = begin; i < end; ++i) {
      if (i == 0U || i + 1U >= sample_count || mutable_indices[i] != 0U) {
        continue;
      }
      mutable_indices[i] = 1U;
      indices.push_back(i);
    }
  }
  return indices;
}

[[nodiscard]] std::vector<TrajectoryOptimizerWindowMetadata>
windowMetadata(const std::span<const ActiveWindow> windows,
               const std::span<const CorridorSample> samples) {
  std::vector<TrajectoryOptimizerWindowMetadata> metadata;
  metadata.reserve(windows.size());
  for (std::size_t i = 0U; i < windows.size(); ++i) {
    const ActiveWindow& window = windows[i];
    if (window.begin_index >= samples.size() || window.end_index >= samples.size()) {
      continue;
    }
    metadata.push_back(TrajectoryOptimizerWindowMetadata{
        .id = i + 1U,
        .begin_s_m = samples[window.begin_index].s_m,
        .end_s_m = samples[window.end_index].s_m,
    });
  }
  return metadata;
}

} // namespace drone_city_nav::trajectory_optimizer_detail
