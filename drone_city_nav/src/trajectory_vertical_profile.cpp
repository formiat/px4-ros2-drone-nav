#include "drone_city_nav/trajectory_vertical_profile.hpp"

#include "drone_city_nav/known_passage_matching.hpp"
#include "drone_city_nav/known_passage_solid_volumes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kSmootherstepMaxDerivative = 1.875;

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double smootherstep(const double t) noexcept {
  const double x = std::clamp(t, 0.0, 1.0);
  return x * x * x * (x * (x * 6.0 - 15.0) + 10.0);
}

struct ProfileWindow {
  KnownPassageTraversalMatch match{};
  double start_z_m{std::numeric_limits<double>::quiet_NaN()};
  double gate_z_m{std::numeric_limits<double>::quiet_NaN()};
  double approach_start_s_m{0.0};
  double gate_hold_start_s_m{0.0};
  double exit_end_s_m{0.0};
  double transition_required_m{std::numeric_limits<double>::quiet_NaN()};
  double transition_available_m{std::numeric_limits<double>::quiet_NaN()};
  double desired_gate_hold_m{std::numeric_limits<double>::quiet_NaN()};
  double actual_gate_hold_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] double interpolateProfile(const double start_s, const double end_s,
                                        const double start_z, const double end_z,
                                        const double s_m) noexcept {
  const double length = end_s - start_s;
  if (!(length > kTinyDistanceM)) {
    return end_z;
  }
  return start_z + (end_z - start_z) * smootherstep((s_m - start_s) / length);
}

[[nodiscard]] double targetGateAltitude(const PassageOpening& opening,
                                        const VerticalProfileConfig& config,
                                        const double reference_altitude_m) noexcept {
  const double hard_margin_m = std::max(0.0, config.gate_clearance_margin_m);
  const double preferred_margin_m =
      std::max(hard_margin_m, config.preferred_gate_clearance_margin_m);
  const double hard_min_z = opening.min_z_m + hard_margin_m;
  const double hard_max_z = opening.max_z_m - hard_margin_m;
  const double preferred_min_z = opening.min_z_m + preferred_margin_m;
  const double preferred_max_z = opening.max_z_m - preferred_margin_m;
  if (preferred_max_z >= preferred_min_z) {
    return std::clamp(reference_altitude_m, preferred_min_z, preferred_max_z);
  }
  if (hard_max_z >= hard_min_z) {
    if (reference_altitude_m >= hard_min_z && reference_altitude_m <= hard_max_z) {
      return reference_altitude_m;
    }
    return 0.5 * (hard_min_z + hard_max_z);
  }
  return 0.5 * (opening.min_z_m + opening.max_z_m);
}

[[nodiscard]] double safeOpeningMinZ(const PassageOpening& opening,
                                     const VerticalProfileConfig& config) noexcept {
  return opening.min_z_m + std::max(0.0, config.gate_clearance_margin_m);
}

[[nodiscard]] double safeOpeningMaxZ(const PassageOpening& opening,
                                     const VerticalProfileConfig& config) noexcept {
  return opening.max_z_m - std::max(0.0, config.gate_clearance_margin_m);
}

[[nodiscard]] double
transitionDistanceForAltitudeDelta(const double dz_m, const double requested_distance_m,
                                   const VerticalProfileConfig& config,
                                   const double min_transition_m,
                                   const double max_transition_m) noexcept {
  if (dz_m <= kTinyDistanceM) {
    return 0.0;
  }
  const double max_slope = std::tan(std::max(1.0e-6, config.max_climb_angle_rad));
  const double slope_distance = max_slope > kTinyDistanceM
                                    ? kSmootherstepMaxDerivative * dz_m / max_slope
                                    : max_transition_m;
  return std::max({min_transition_m, slope_distance, requested_distance_m});
}

[[nodiscard]] double preGateHoldDistanceM(const VerticalProfileConfig& config) {
  const double nominal_speed =
      sanitizedPositive(config.nominal_horizontal_speed_mps, 12.0, 0.0, 100.0);
  const double hold_time =
      sanitizedPositive(config.pre_gate_hold_time_s, 1.0, 0.0, 60.0);
  const double min_distance =
      sanitizedPositive(config.pre_gate_hold_min_distance_m, 15.0, 0.0, 1000.0);
  const double max_distance =
      std::max(min_distance, sanitizedPositive(config.pre_gate_hold_max_distance_m,
                                               80.0, 0.0, 5000.0));
  return std::clamp(std::max(min_distance, nominal_speed * hold_time), min_distance,
                    max_distance);
}

[[nodiscard]] bool verticalSegmentIntersectsKnownSolid(const Point2 point,
                                                       const double start_z_m,
                                                       const double end_z_m,
                                                       const KnownPassageMap& map) {
  const double segment_min_z_m = std::min(start_z_m, end_z_m);
  const double segment_max_z_m = std::max(start_z_m, end_z_m);
  const std::vector<KnownPassageSolidVolume> volumes = knownPassageSolidVolumes(map);
  return std::ranges::any_of(volumes, [&](const KnownPassageSolidVolume& volume) {
    const Point2 offset{point.x - volume.center.x, point.y - volume.center.y};
    const double depth_coordinate =
        offset.x * volume.normal_xy.x + offset.y * volume.normal_xy.y;
    const double lateral_coordinate =
        offset.x * volume.lateral_xy.x + offset.y * volume.lateral_xy.y;
    const bool inside_xy =
        std::abs(depth_coordinate) <= 0.5 * volume.depth_m + kTinyDistanceM &&
        std::abs(lateral_coordinate) <= 0.5 * volume.width_m + kTinyDistanceM;
    const bool overlaps_z = segment_max_z_m >= volume.min_z_m - kTinyDistanceM &&
                            segment_min_z_m <= volume.max_z_m + kTinyDistanceM;
    return inside_xy && overlaps_z;
  });
}

void resetVerticalMetadata(std::span<TrajectoryPointSample> samples,
                           const double initial_altitude_m) {
  for (TrajectoryPointSample& sample : samples) {
    sample.z_m = initial_altitude_m;
    sample.vertical_slope_dz_ds = 0.0;
    sample.vertical_speed_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_accel_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_jerk_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_constraint_active = false;
    sample.vertical_hard_window_active = false;
    sample.vertical_safe_min_z_m = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_safe_max_z_m = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_gate_z_m = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_profile_passage_id.clear();
  }
}

void appendDiagnostic(VerticalProfileStats& stats, const VerticalProfileConfig& config,
                      const VerticalProfilePassageDiagnostic& diagnostic) {
  if (stats.diagnostics.size() < config.max_diagnostics) {
    stats.diagnostics.push_back(diagnostic);
  }
}

[[nodiscard]] bool hasFiniteSamples(std::span<const TrajectoryPointSample> samples) {
  return std::ranges::all_of(samples, [](const TrajectoryPointSample& sample) {
    return std::isfinite(sample.s_m) && std::isfinite(sample.point.x) &&
           std::isfinite(sample.point.y) && std::isfinite(sample.z_m);
  });
}

void updateAltitudeStats(VerticalProfileStats& stats,
                         std::span<const TrajectoryPointSample> samples) {
  if (samples.empty()) {
    return;
  }
  stats.min_z_m = samples.front().z_m;
  stats.max_z_m = samples.front().z_m;
  for (const TrajectoryPointSample& sample : samples) {
    stats.min_z_m = std::min(stats.min_z_m, sample.z_m);
    stats.max_z_m = std::max(stats.max_z_m, sample.z_m);
  }
}

void computeVerticalDerivatives(std::span<TrajectoryPointSample> samples,
                                const VerticalProfileConfig& config,
                                VerticalProfileStats& stats) {
  const double max_climb_vz =
      sanitizedPositive(config.max_climb_speed_mps, 3.2, 1.0e-6, 100.0);
  const double max_descent_vz =
      sanitizedPositive(config.max_descent_speed_mps, 3.2, 1.0e-6, 100.0);
  const double max_accel =
      sanitizedPositive(config.max_vertical_accel_mps2, 3.0, 1.0e-6, 100.0);
  const double max_jerk =
      sanitizedPositive(config.max_vertical_jerk_mps3, 9.0, 1.0e-6, 1000.0);
  std::vector<double> slopes(samples.size(), 0.0);
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    std::size_t prev = i == 0U ? i : i - 1U;
    std::size_t next = i + 1U < samples.size() ? i + 1U : i;
    if (prev == next && samples.size() > 1U) {
      next = 1U;
    }
    const double ds = samples[next].s_m - samples[prev].s_m;
    const double slope =
        ds > kTinyDistanceM ? (samples[next].z_m - samples[prev].z_m) / ds : 0.0;
    slopes[i] = slope;
    samples[i].vertical_slope_dz_ds = slope;
    stats.max_abs_dz_ds = std::max(stats.max_abs_dz_ds, std::abs(slope));
    if (std::abs(slope) > kTinyDistanceM) {
      const double directional_limit_mps = slope > 0.0 ? max_climb_vz : max_descent_vz;
      samples[i].vertical_speed_limit_mps = directional_limit_mps / std::abs(slope);
      stats.min_vertical_speed_cap_mps =
          std::isfinite(stats.min_vertical_speed_cap_mps)
              ? std::min(stats.min_vertical_speed_cap_mps,
                         samples[i].vertical_speed_limit_mps)
              : samples[i].vertical_speed_limit_mps;
    }
  }

  std::vector<double> curvature(samples.size(), 0.0);
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const double ds = samples[i + 1U].s_m - samples[i - 1U].s_m;
    if (!(ds > kTinyDistanceM)) {
      continue;
    }
    curvature[i] = (slopes[i + 1U] - slopes[i - 1U]) / ds;
    stats.max_abs_d2z_ds2 = std::max(stats.max_abs_d2z_ds2, std::abs(curvature[i]));
    if (std::abs(curvature[i]) > kTinyDistanceM) {
      samples[i].vertical_accel_limit_mps =
          std::sqrt(max_accel / std::abs(curvature[i]));
      stats.min_vertical_speed_cap_mps =
          std::isfinite(stats.min_vertical_speed_cap_mps)
              ? std::min(stats.min_vertical_speed_cap_mps,
                         samples[i].vertical_accel_limit_mps)
              : samples[i].vertical_accel_limit_mps;
    }
  }

  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const double ds = samples[i + 1U].s_m - samples[i - 1U].s_m;
    if (!(ds > kTinyDistanceM)) {
      continue;
    }
    const double jerk = (curvature[i + 1U] - curvature[i - 1U]) / ds;
    stats.max_abs_d3z_ds3 = std::max(stats.max_abs_d3z_ds3, std::abs(jerk));
    if (std::abs(jerk) > kTinyDistanceM) {
      samples[i].vertical_jerk_limit_mps = std::cbrt(max_jerk / std::abs(jerk));
      stats.min_vertical_speed_cap_mps =
          std::isfinite(stats.min_vertical_speed_cap_mps)
              ? std::min(stats.min_vertical_speed_cap_mps,
                         samples[i].vertical_jerk_limit_mps)
              : samples[i].vertical_jerk_limit_mps;
    }
  }

  for (TrajectoryPointSample& sample : samples) {
    sample.vertical_constraint_active =
        sample.vertical_hard_window_active ||
        std::isfinite(sample.vertical_speed_limit_mps) ||
        std::isfinite(sample.vertical_accel_limit_mps) ||
        std::isfinite(sample.vertical_jerk_limit_mps);
  }
}

[[nodiscard]] bool
profileWithinClimbAngle(std::span<const TrajectoryPointSample> samples,
                        const VerticalProfileConfig& config) {
  const double max_slope = std::tan(std::max(0.0, config.max_climb_angle_rad));
  return std::ranges::all_of(samples, [max_slope](const TrajectoryPointSample& sample) {
    return std::abs(sample.vertical_slope_dz_ds) <= max_slope + 1.0e-9;
  });
}

[[nodiscard]] VerticalProfilePassageDiagnostic
diagnosticFromWindow(const ProfileWindow& window, const char* const reason,
                     const VerticalProfileConfig& config, const bool valid) {
  return VerticalProfilePassageDiagnostic{
      .structure_id = window.match.structure_id,
      .opening_id = window.match.opening_id,
      .entry_s_m = window.match.entry_s_m,
      .exit_s_m = window.match.exit_s_m,
      .approach_start_s_m = window.approach_start_s_m,
      .gate_hold_start_s_m = window.gate_hold_start_s_m,
      .exit_end_s_m = window.exit_end_s_m,
      .gate_z_m = window.gate_z_m,
      .min_z_m = window.match.opening.min_z_m,
      .max_z_m = window.match.opening.max_z_m,
      .safe_min_z_m = safeOpeningMinZ(window.match.opening, config),
      .safe_max_z_m = safeOpeningMaxZ(window.match.opening, config),
      .transition_required_m = window.transition_required_m,
      .transition_available_m = window.transition_available_m,
      .desired_gate_hold_m = window.desired_gate_hold_m,
      .actual_gate_hold_m = window.actual_gate_hold_m,
      .reason = reason,
      .valid = valid,
  };
}

[[nodiscard]] bool profileWindowsOverlap(const ProfileWindow& lhs,
                                         const ProfileWindow& rhs) noexcept {
  return rhs.approach_start_s_m < lhs.exit_end_s_m - kTinyDistanceM;
}

[[nodiscard]] bool
rejectOverlappingInfeasibleWindows(std::span<const ProfileWindow> windows,
                                   VerticalProfileStats& stats,
                                   const VerticalProfileConfig& config) {
  bool conflict_found = false;
  for (std::size_t i = 0U; i < windows.size(); ++i) {
    for (std::size_t j = i + 1U; j < windows.size(); ++j) {
      if (windows[j].approach_start_s_m >= windows[i].exit_end_s_m - kTinyDistanceM) {
        break;
      }
      if (!profileWindowsOverlap(windows[i], windows[j])) {
        continue;
      }
      if (std::abs(windows[i].gate_z_m - windows[j].gate_z_m) <= kTinyDistanceM) {
        continue;
      }
      conflict_found = true;
      stats.infeasible_count += 2U;
      appendDiagnostic(stats, config,
                       diagnosticFromWindow(windows[i],
                                            "overlapping_infeasible_windows", config,
                                            false));
      appendDiagnostic(stats, config,
                       diagnosticFromWindow(windows[j],
                                            "overlapping_infeasible_windows", config,
                                            false));
    }
  }
  return conflict_found;
}

} // namespace

const char* verticalProfileStatusName(const bool valid) noexcept {
  return valid ? "valid" : "invalid";
}

VerticalProfileResult applyVerticalProfile(
    const std::span<TrajectoryPointSample> samples, const KnownPassageMap* const map,
    const KnownPassageValidationConfig& validation_config,
    const VerticalProfileConfig& config, const double initial_altitude_m,
    const VerticalProfileStartMode start_mode) {
  VerticalProfileResult result{};
  result.stats.enabled = config.enabled;
  if (samples.empty() || !std::isfinite(initial_altitude_m) || !config.enabled ||
      map == nullptr) {
    resetVerticalMetadata(samples, initial_altitude_m);
    result.stats.applied = true;
    updateAltitudeStats(result.stats, samples);
    return result;
  }

  resetVerticalMetadata(samples, initial_altitude_m);
  result.stats.applied = true;
  if (!hasFiniteSamples(samples)) {
    result.valid = false;
    result.stats.valid = false;
    return result;
  }

  const std::vector<KnownPassageTraversalMatch> matches =
      findKnownPassageTraversalMatches(samples, *map, validation_config, true);
  const double first_s = samples.front().s_m;
  const double min_transition =
      sanitizedPositive(config.min_transition_distance_m, 6.0, 0.0, 1000.0);
  const double max_transition =
      std::max(min_transition,
               sanitizedPositive(config.max_transition_distance_m, 80.0, 0.0, 10000.0));
  const double pre_gate_hold_distance = preGateHoldDistanceM(config);

  std::vector<KnownPassageTraversalMatch> valid_matches;
  valid_matches.reserve(matches.size());
  for (const KnownPassageTraversalMatch& match : matches) {
    if (!match.valid) {
      // A footprint/opening miss is a diagnostics signal, not a vertical-profile
      // infeasibility. Ordinary building collision volumes remain the runtime
      // source of truth for solid-vs-free space.
      appendDiagnostic(result.stats, config,
                       VerticalProfilePassageDiagnostic{
                           .structure_id = match.structure_id,
                           .opening_id = match.opening_id,
                           .entry_s_m = match.entry_s_m,
                           .exit_s_m = match.exit_s_m,
                           .reason = knownPassageValidationReasonName(match.reason),
                           .valid = false,
                       });
      continue;
    }

    ++result.stats.passages_matched;
    valid_matches.push_back(match);
  }
  std::ranges::sort(valid_matches, [](const KnownPassageTraversalMatch& lhs,
                                      const KnownPassageTraversalMatch& rhs) {
    if (std::abs(lhs.entry_s_m - rhs.entry_s_m) > kTinyDistanceM) {
      return lhs.entry_s_m < rhs.entry_s_m;
    }
    return lhs.exit_s_m < rhs.exit_s_m;
  });

  double effective_initial_altitude_m = initial_altitude_m;
  if (start_mode == VerticalProfileStartMode::kStationaryHoldRestart &&
      !valid_matches.empty() && !valid_matches.front().starts_inside_opening) {
    const KnownPassageTraversalMatch& first_match = valid_matches.front();
    const double gate_z_m =
        targetGateAltitude(first_match.opening, config, initial_altitude_m);
    const double required_m =
        transitionDistanceForAltitudeDelta(std::abs(gate_z_m - initial_altitude_m),
                                           first_match.opening.approach_distance_m,
                                           config, min_transition, max_transition);
    const double available_m = std::max(0.0, first_match.entry_s_m - first_s);
    if (required_m > available_m + kTinyDistanceM) {
      result.stats.pre_alignment_start_z_m = initial_altitude_m;
      result.stats.pre_alignment_target_z_m = gate_z_m;
      if (verticalSegmentIntersectsKnownSolid(samples.front().point, initial_altitude_m,
                                              gate_z_m, *map)) {
        ++result.stats.infeasible_count;
        appendDiagnostic(
            result.stats, config,
            VerticalProfilePassageDiagnostic{
                .structure_id = first_match.structure_id,
                .opening_id = first_match.opening_id,
                .entry_s_m = first_match.entry_s_m,
                .exit_s_m = first_match.exit_s_m,
                .gate_z_m = gate_z_m,
                .min_z_m = first_match.opening.min_z_m,
                .max_z_m = first_match.opening.max_z_m,
                .safe_min_z_m = safeOpeningMinZ(first_match.opening, config),
                .safe_max_z_m = safeOpeningMaxZ(first_match.opening, config),
                .transition_required_m = required_m,
                .transition_available_m = available_m,
                .reason = "vertical_pre_alignment_blocked_by_known_solid",
                .valid = false,
            });
      } else {
        result.stats.pre_alignment_required = true;
        effective_initial_altitude_m = gate_z_m;
        resetVerticalMetadata(samples, effective_initial_altitude_m);
      }
    }
  }

  std::vector<ProfileWindow> windows;
  windows.reserve(valid_matches.size());
  double carried_altitude_m = effective_initial_altitude_m;
  double transition_available_after_s_m = first_s;
  for (const KnownPassageTraversalMatch& match : valid_matches) {
    const double start_z = carried_altitude_m;
    const bool starts_inside = match.starts_inside_opening;
    const double gate_z = starts_inside
                              ? effective_initial_altitude_m
                              : targetGateAltitude(match.opening, config, start_z);
    const double dz = std::abs(gate_z - start_z);
    const double transition_distance_required = transitionDistanceForAltitudeDelta(
        dz, match.opening.approach_distance_m, config, min_transition, max_transition);
    const bool transition_required = transition_distance_required > kTinyDistanceM;
    const double transition_window_start_s =
        std::max(first_s, transition_available_after_s_m);
    const double available_before_entry_s =
        std::max(0.0, match.entry_s_m - transition_window_start_s);
    const double actual_gate_hold_distance = std::min(
        pre_gate_hold_distance,
        std::max(0.0, available_before_entry_s - transition_distance_required));
    const double gate_hold_start_s =
        starts_inside ? first_s : match.entry_s_m - actual_gate_hold_distance;
    const double approach_start_s =
        starts_inside ? first_s
                      : gate_hold_start_s -
                            std::min(transition_distance_required, max_transition);
    if (transition_distance_required > max_transition + kTinyDistanceM ||
        !(match.exit_s_m >= match.entry_s_m) ||
        (transition_required &&
         (available_before_entry_s + kTinyDistanceM < transition_distance_required ||
          approach_start_s + kTinyDistanceM < transition_window_start_s ||
          !(match.entry_s_m > approach_start_s + kTinyDistanceM)))) {
      ++result.stats.infeasible_count;
      appendDiagnostic(result.stats, config,
                       VerticalProfilePassageDiagnostic{
                           .structure_id = match.structure_id,
                           .opening_id = match.opening_id,
                           .entry_s_m = match.entry_s_m,
                           .exit_s_m = match.exit_s_m,
                           .approach_start_s_m = approach_start_s,
                           .gate_hold_start_s_m = gate_hold_start_s,
                           .exit_end_s_m = match.exit_s_m,
                           .gate_z_m = gate_z,
                           .min_z_m = match.opening.min_z_m,
                           .max_z_m = match.opening.max_z_m,
                           .safe_min_z_m = safeOpeningMinZ(match.opening, config),
                           .safe_max_z_m = safeOpeningMaxZ(match.opening, config),
                           .transition_required_m = transition_distance_required,
                           .transition_available_m = available_before_entry_s,
                           .desired_gate_hold_m = pre_gate_hold_distance,
                           .actual_gate_hold_m = actual_gate_hold_distance,
                           .reason = "insufficient_transition_distance",
                           .valid = false,
                       });
      continue;
    }

    windows.push_back(ProfileWindow{
        .match = match,
        .start_z_m = start_z,
        .gate_z_m = gate_z,
        .approach_start_s_m = approach_start_s,
        .gate_hold_start_s_m = gate_hold_start_s,
        .exit_end_s_m = match.exit_s_m,
        .transition_required_m = transition_distance_required,
        .transition_available_m = available_before_entry_s,
        .desired_gate_hold_m = pre_gate_hold_distance,
        .actual_gate_hold_m = actual_gate_hold_distance,
    });
    carried_altitude_m = gate_z;
    transition_available_after_s_m =
        std::max(transition_available_after_s_m, match.exit_s_m);
  }

  std::ranges::sort(windows, [](const ProfileWindow& lhs, const ProfileWindow& rhs) {
    if (std::abs(lhs.approach_start_s_m - rhs.approach_start_s_m) > kTinyDistanceM) {
      return lhs.approach_start_s_m < rhs.approach_start_s_m;
    }
    return lhs.match.entry_s_m < rhs.match.entry_s_m;
  });
  if (rejectOverlappingInfeasibleWindows(windows, result.stats, config)) {
    computeVerticalDerivatives(samples, config, result.stats);
    updateAltitudeStats(result.stats, samples);
    result.valid = false;
    result.stats.valid = false;
    return result;
  }
  if (result.stats.infeasible_count > 0U) {
    computeVerticalDerivatives(samples, config, result.stats);
    updateAltitudeStats(result.stats, samples);
    result.valid = false;
    result.stats.valid = false;
    return result;
  }

  std::size_t window_index = 0U;
  double sample_altitude_m = effective_initial_altitude_m;
  for (TrajectoryPointSample& sample : samples) {
    while (window_index < windows.size() &&
           sample.s_m > windows[window_index].exit_end_s_m + kTinyDistanceM) {
      sample_altitude_m = windows[window_index].gate_z_m;
      ++window_index;
    }
    if (window_index >= windows.size()) {
      sample.z_m = sample_altitude_m;
      continue;
    }

    const ProfileWindow& window = windows[window_index];
    const KnownPassageTraversalMatch& match = window.match;
    const double safe_min_z_m = safeOpeningMinZ(match.opening, config);
    const double safe_max_z_m = safeOpeningMaxZ(match.opening, config);
    if (sample.s_m < window.approach_start_s_m) {
      sample.z_m = sample_altitude_m;
      continue;
    }
    if (!match.starts_inside_opening && sample.s_m <= window.gate_hold_start_s_m) {
      sample.z_m =
          interpolateProfile(window.approach_start_s_m, window.gate_hold_start_s_m,
                             window.start_z_m, window.gate_z_m, sample.s_m);
      sample.vertical_profile_passage_id = match.opening_id;
    } else if (sample.s_m <= match.exit_s_m) {
      sample.z_m = window.gate_z_m;
      sample.vertical_profile_passage_id = match.opening_id;
      sample.vertical_hard_window_active = true;
      sample.vertical_safe_min_z_m = safe_min_z_m;
      sample.vertical_safe_max_z_m = safe_max_z_m;
      sample.vertical_gate_z_m = window.gate_z_m;
    } else {
      sample_altitude_m = window.gate_z_m;
      sample.z_m = sample_altitude_m;
    }
  }

  for (const ProfileWindow& window : windows) {
    ++result.stats.passages_profiled;
    result.stats.active = true;
    appendDiagnostic(result.stats, config,
                     diagnosticFromWindow(window, "profiled", config, true));
  }

  computeVerticalDerivatives(samples, config, result.stats);
  updateAltitudeStats(result.stats, samples);
  result.valid =
      result.stats.infeasible_count == 0U && profileWithinClimbAngle(samples, config);
  result.stats.valid = result.valid;
  return result;
}

} // namespace drone_city_nav
