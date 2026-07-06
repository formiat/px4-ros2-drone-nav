#include "drone_city_nav/trajectory_vertical_profile.hpp"

#include "drone_city_nav/known_passage_matching.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

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

[[nodiscard]] double interpolateProfile(const double start_s, const double end_s,
                                        const double start_z, const double end_z,
                                        const double s_m) noexcept {
  const double length = end_s - start_s;
  if (!(length > kTinyDistanceM)) {
    return end_z;
  }
  return start_z + (end_z - start_z) * smootherstep((s_m - start_s) / length);
}

[[nodiscard]] double nearestValidGateAltitude(const PassageOpening& opening,
                                              const VerticalProfileConfig& config,
                                              const double cruise_altitude_m) noexcept {
  const double min_z = opening.min_z_m + std::max(0.0, config.gate_clearance_margin_m);
  const double max_z = opening.max_z_m - std::max(0.0, config.gate_clearance_margin_m);
  if (max_z >= min_z) {
    return std::clamp(cruise_altitude_m, min_z, max_z);
  }
  return 0.5 * (opening.min_z_m + opening.max_z_m);
}

void resetVerticalMetadata(std::span<TrajectoryPointSample> samples,
                           const double cruise_altitude_m) {
  for (TrajectoryPointSample& sample : samples) {
    sample.z_m = cruise_altitude_m;
    sample.vertical_slope_dz_ds = 0.0;
    sample.vertical_speed_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_accel_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_jerk_limit_mps = std::numeric_limits<double>::quiet_NaN();
    sample.vertical_constraint_active = false;
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
  const double max_vz =
      sanitizedPositive(config.max_vertical_speed_mps, 2.5, 1.0e-6, 100.0);
  const double max_accel =
      sanitizedPositive(config.max_vertical_accel_mps2, 2.0, 1.0e-6, 100.0);
  const double max_jerk =
      sanitizedPositive(config.max_vertical_jerk_mps3, 6.0, 1.0e-6, 1000.0);
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
      samples[i].vertical_speed_limit_mps = max_vz / std::abs(slope);
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

} // namespace

const char* verticalProfileStatusName(const bool valid) noexcept {
  return valid ? "valid" : "invalid";
}

VerticalProfileResult applyVerticalProfile(
    const std::span<TrajectoryPointSample> samples, const KnownPassageMap* const map,
    const KnownPassageValidationConfig& validation_config,
    const VerticalProfileConfig& config, const double cruise_altitude_m) {
  VerticalProfileResult result{};
  result.stats.enabled = config.enabled;
  if (samples.empty() || !std::isfinite(cruise_altitude_m) || !config.enabled ||
      map == nullptr) {
    resetVerticalMetadata(samples, cruise_altitude_m);
    result.stats.applied = true;
    updateAltitudeStats(result.stats, samples);
    return result;
  }

  resetVerticalMetadata(samples, cruise_altitude_m);
  result.stats.applied = true;
  if (!hasFiniteSamples(samples)) {
    result.valid = false;
    result.stats.valid = false;
    return result;
  }

  const std::vector<KnownPassageTraversalMatch> matches =
      findKnownPassageTraversalMatches(samples, *map, validation_config, true);
  const double total_s = samples.back().s_m;
  const double min_transition =
      sanitizedPositive(config.min_transition_distance_m, 6.0, 0.0, 1000.0);
  const double max_transition =
      std::max(min_transition,
               sanitizedPositive(config.max_transition_distance_m, 80.0, 0.0, 10000.0));

  for (const KnownPassageTraversalMatch& match : matches) {
    if (!match.valid) {
      ++result.stats.infeasible_count;
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
    const double gate_z =
        nearestValidGateAltitude(match.opening, config, cruise_altitude_m);
    const double dz = std::abs(gate_z - cruise_altitude_m);
    const double slope_distance =
        std::tan(std::max(1.0e-6, config.max_climb_angle_rad)) > kTinyDistanceM
            ? dz / std::tan(config.max_climb_angle_rad)
            : max_transition;
    const double transition_distance = std::clamp(
        std::max({min_transition, slope_distance, match.opening.approach_distance_m}),
        min_transition, max_transition);
    const double exit_transition_distance = std::clamp(
        std::max({min_transition, slope_distance, match.opening.exit_distance_m}),
        min_transition, max_transition);
    const double approach_start_s =
        std::clamp(match.entry_s_m - transition_distance, 0.0, total_s);
    const double exit_end_s =
        std::clamp(match.exit_s_m + exit_transition_distance, 0.0, total_s);
    if (!(match.exit_s_m >= match.entry_s_m) ||
        !(match.entry_s_m > approach_start_s + kTinyDistanceM) ||
        !(exit_end_s > match.exit_s_m + kTinyDistanceM)) {
      ++result.stats.infeasible_count;
      appendDiagnostic(result.stats, config,
                       VerticalProfilePassageDiagnostic{
                           .structure_id = match.structure_id,
                           .opening_id = match.opening_id,
                           .entry_s_m = match.entry_s_m,
                           .exit_s_m = match.exit_s_m,
                           .approach_start_s_m = approach_start_s,
                           .exit_end_s_m = exit_end_s,
                           .gate_z_m = gate_z,
                           .reason = "insufficient_transition_distance",
                           .valid = false,
                       });
      continue;
    }

    for (TrajectoryPointSample& sample : samples) {
      if (sample.s_m < approach_start_s || sample.s_m > exit_end_s) {
        continue;
      }
      if (sample.s_m <= match.entry_s_m) {
        sample.z_m = interpolateProfile(approach_start_s, match.entry_s_m,
                                        cruise_altitude_m, gate_z, sample.s_m);
      } else if (sample.s_m <= match.exit_s_m) {
        sample.z_m = gate_z;
      } else {
        sample.z_m = interpolateProfile(match.exit_s_m, exit_end_s, gate_z,
                                        cruise_altitude_m, sample.s_m);
      }
      sample.vertical_profile_passage_id = match.opening_id;
    }
    ++result.stats.passages_profiled;
    result.stats.active = true;
    appendDiagnostic(result.stats, config,
                     VerticalProfilePassageDiagnostic{
                         .structure_id = match.structure_id,
                         .opening_id = match.opening_id,
                         .entry_s_m = match.entry_s_m,
                         .exit_s_m = match.exit_s_m,
                         .approach_start_s_m = approach_start_s,
                         .exit_end_s_m = exit_end_s,
                         .gate_z_m = gate_z,
                         .min_z_m = match.opening.min_z_m,
                         .max_z_m = match.opening.max_z_m,
                         .reason = "profiled",
                         .valid = true,
                     });
  }

  computeVerticalDerivatives(samples, config, result.stats);
  updateAltitudeStats(result.stats, samples);
  result.valid =
      result.stats.infeasible_count == 0U && profileWithinClimbAngle(samples, config);
  result.stats.valid = result.valid;
  return result;
}

} // namespace drone_city_nav
