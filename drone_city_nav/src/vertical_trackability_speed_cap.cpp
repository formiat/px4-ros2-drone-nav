#include "drone_city_nav/vertical_trackability_speed_cap.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

struct VerticalHardWindow {
  bool valid{false};
  double start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double safe_min_z_m{std::numeric_limits<double>::quiet_NaN()};
  double safe_max_z_m{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] double boundedFiniteDouble(const double value, const double fallback,
                                         const double min_value,
                                         const double max_value) noexcept {
  return std::isfinite(value) ? std::clamp(value, min_value, max_value) : fallback;
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  return boundedFiniteDouble(value, fallback, min_value, max_value);
}

[[nodiscard]] std::size_t
firstSampleIndexAtOrAfterS(const std::span<const TrajectoryPointSample> samples,
                           const double s_m) noexcept {
  if (samples.empty()) {
    return 0U;
  }
  const double clamped_s =
      std::clamp(std::isfinite(s_m) ? s_m : 0.0, 0.0, samples.back().s_m);
  const auto it =
      std::lower_bound(samples.begin(), samples.end(), clamped_s,
                       [](const TrajectoryPointSample& sample, const double station_m) {
                         return sample.s_m < station_m;
                       });
  if (it == samples.end()) {
    return samples.size() - 1U;
  }
  return static_cast<std::size_t>(std::distance(samples.begin(), it));
}

[[nodiscard]] double
verticalProfileWindowEndS(const std::span<const TrajectoryPointSample> samples,
                          const std::size_t start_index,
                          const std::string& passage_id) noexcept {
  if (samples.empty() || passage_id.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  for (std::size_t i = start_index; i < samples.size(); ++i) {
    if (!samples[i].vertical_profile_passage_id.empty() &&
        samples[i].vertical_profile_passage_id != passage_id) {
      return samples[i].s_m;
    }
    if (samples[i].vertical_profile_passage_id.empty() && i > start_index) {
      return samples[i].s_m;
    }
  }
  return samples.back().s_m;
}

[[nodiscard]] bool
hardWindowSampleUsable(const TrajectoryPointSample& sample) noexcept {
  return sample.vertical_hard_window_active &&
         std::isfinite(sample.vertical_safe_min_z_m) &&
         std::isfinite(sample.vertical_safe_max_z_m) &&
         sample.vertical_safe_max_z_m >= sample.vertical_safe_min_z_m;
}

[[nodiscard]] bool sameHardWindow(const TrajectoryPointSample& lhs,
                                  const TrajectoryPointSample& rhs) noexcept {
  if (!hardWindowSampleUsable(lhs) || !hardWindowSampleUsable(rhs)) {
    return false;
  }
  if (!lhs.vertical_profile_passage_id.empty() ||
      !rhs.vertical_profile_passage_id.empty()) {
    return lhs.vertical_profile_passage_id == rhs.vertical_profile_passage_id;
  }
  return std::abs(lhs.vertical_safe_min_z_m - rhs.vertical_safe_min_z_m) <= 1.0e-6 &&
         std::abs(lhs.vertical_safe_max_z_m - rhs.vertical_safe_max_z_m) <= 1.0e-6;
}

[[nodiscard]] VerticalHardWindow
upcomingVerticalHardWindow(const std::span<const TrajectoryPointSample> samples,
                           const double trajectory_s_m) noexcept {
  VerticalHardWindow window{};
  if (samples.empty()) {
    return window;
  }

  std::size_t search_index = firstSampleIndexAtOrAfterS(samples, trajectory_s_m);
  if (search_index > 0U && hardWindowSampleUsable(samples[search_index - 1U]) &&
      samples[search_index - 1U].s_m <= trajectory_s_m + kTinyDistanceM) {
    --search_index;
  }
  while (search_index < samples.size() &&
         !hardWindowSampleUsable(samples[search_index])) {
    ++search_index;
  }
  if (search_index >= samples.size()) {
    return window;
  }

  std::size_t begin_index = search_index;
  while (begin_index > 0U &&
         sameHardWindow(samples[begin_index - 1U], samples[search_index])) {
    --begin_index;
  }
  std::size_t end_index = search_index;
  while (end_index + 1U < samples.size() &&
         sameHardWindow(samples[end_index + 1U], samples[search_index])) {
    ++end_index;
  }

  double safe_min_z_m = samples[begin_index].vertical_safe_min_z_m;
  double safe_max_z_m = samples[begin_index].vertical_safe_max_z_m;
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    safe_min_z_m = std::max(safe_min_z_m, samples[i].vertical_safe_min_z_m);
    safe_max_z_m = std::min(safe_max_z_m, samples[i].vertical_safe_max_z_m);
  }
  if (!(safe_max_z_m >= safe_min_z_m)) {
    return window;
  }

  window.valid = true;
  window.start_s_m = samples[begin_index].s_m;
  window.safe_min_z_m = safe_min_z_m;
  window.safe_max_z_m = safe_max_z_m;
  return window;
}

[[nodiscard]] double altitudeErrorToSafeWindow(const double altitude_m,
                                               const VerticalHardWindow& window,
                                               const double safe_margin_m) noexcept {
  if (!window.valid || !std::isfinite(altitude_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (altitude_m < window.safe_min_z_m - safe_margin_m) {
    return window.safe_min_z_m - altitude_m;
  }
  if (altitude_m > window.safe_max_z_m + safe_margin_m) {
    return window.safe_max_z_m - altitude_m;
  }
  return 0.0;
}

[[nodiscard]] double minimumVerticalTravelTimeS(const double distance_m,
                                                const double velocity_toward_target_mps,
                                                const double max_speed_mps,
                                                const double max_accel_mps2) noexcept {
  if (!(distance_m > kTinyDistanceM) || !(max_speed_mps > kTinyDistanceM)) {
    return 0.0;
  }
  if (!(max_accel_mps2 > kTinyDistanceM)) {
    return distance_m / max_speed_mps;
  }

  double remaining_distance_m = distance_m;
  double initial_speed_mps =
      std::clamp(velocity_toward_target_mps, -max_speed_mps, max_speed_mps);
  double time_s = 0.0;
  if (initial_speed_mps < 0.0) {
    const double stop_time_s = -initial_speed_mps / max_accel_mps2;
    remaining_distance_m += 0.5 * (-initial_speed_mps) * stop_time_s;
    time_s += stop_time_s;
    initial_speed_mps = 0.0;
  }

  const double accel_distance_m = std::max(
      0.0, (max_speed_mps * max_speed_mps - initial_speed_mps * initial_speed_mps) /
               (2.0 * max_accel_mps2));
  if (remaining_distance_m <= accel_distance_m) {
    const double discriminant = initial_speed_mps * initial_speed_mps +
                                2.0 * max_accel_mps2 * remaining_distance_m;
    return time_s + (std::sqrt(std::max(0.0, discriminant)) - initial_speed_mps) /
                        max_accel_mps2;
  }

  time_s += (max_speed_mps - initial_speed_mps) / max_accel_mps2;
  return time_s + (remaining_distance_m - accel_distance_m) / max_speed_mps;
}

} // namespace

VerticalTrackabilitySpeedCap computeVerticalTrackabilitySpeedCap(
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const double trajectory_s_m, const double current_altitude_m,
    const bool altitude_valid, const double current_vertical_velocity_mps,
    const bool vertical_velocity_valid, const VelocityFollowerConfig& config) {
  VerticalTrackabilitySpeedCap cap{};
  if (!altitude_valid || !std::isfinite(current_altitude_m) ||
      !trajectorySamplesAreUsable(trajectory_samples)) {
    return cap;
  }

  const TrajectoryVerticalTarget target =
      trajectoryVerticalTargetAtS(trajectory_samples, trajectory_s_m);
  if (!target.valid || !std::isfinite(target.z_m)) {
    return cap;
  }

  const double tolerance = sanitizedPositive(
      config.vertical_trackability_altitude_tolerance_m, 0.4, 0.0, 100.0);
  const VerticalHardWindow hard_window =
      upcomingVerticalHardWindow(trajectory_samples, target.s_m);
  const bool hard_window_started =
      hard_window.valid && target.s_m + kTinyDistanceM >= hard_window.start_s_m;
  const double safe_margin_m = hard_window_started ? 0.0 : tolerance;
  const double altitude_error =
      hard_window.valid
          ? altitudeErrorToSafeWindow(current_altitude_m, hard_window, safe_margin_m)
          : target.z_m - current_altitude_m;
  if (!std::isfinite(altitude_error)) {
    return cap;
  }
  const double required_error =
      std::abs(altitude_error) - (hard_window_started ? 0.0 : tolerance);
  if (!(required_error > 0.0)) {
    return cap;
  }

  double constraint_distance_m = std::numeric_limits<double>::quiet_NaN();
  if (hard_window.valid) {
    constraint_distance_m = target.s_m < hard_window.start_s_m
                                ? std::max(0.0, hard_window.start_s_m - target.s_m)
                                : 0.0;
  } else {
    if (target.vertical_profile_passage_id.empty()) {
      return cap;
    }
    const std::size_t start_index =
        firstSampleIndexAtOrAfterS(trajectory_samples, target.s_m);
    const double window_end_s = verticalProfileWindowEndS(
        trajectory_samples, start_index, target.vertical_profile_passage_id);
    if (!std::isfinite(window_end_s)) {
      return cap;
    }
    constraint_distance_m = std::max(0.0, window_end_s - target.s_m);
  }

  const bool climbing = altitude_error > 0.0;
  const double vertical_speed_mps = sanitizedPositive(
      climbing ? config.vertical_trackability_max_climb_speed_mps
               : config.vertical_trackability_max_descent_speed_mps,
      std::max(1.0, climbing ? config.vertical_profile_max_climb_speed_mps
                             : config.vertical_profile_max_descent_speed_mps),
      1.0e-6, 100.0);
  const double vertical_accel_mps2 = sanitizedPositive(
      config.vertical_trackability_max_vertical_accel_mps2, 3.5, 1.0e-6, 100.0);
  const double response_time_s =
      sanitizedPositive(config.vertical_trackability_response_time_s, 0.4, 0.0, 10.0);
  const double direction = climbing ? 1.0 : -1.0;
  const double velocity_toward_target_mps =
      vertical_velocity_valid && std::isfinite(current_vertical_velocity_mps)
          ? direction * current_vertical_velocity_mps
          : 0.0;
  const double time_needed_s =
      minimumVerticalTravelTimeS(required_error, velocity_toward_target_mps,
                                 vertical_speed_mps, vertical_accel_mps2) +
      response_time_s;
  if (!(time_needed_s > kTinyDistanceM)) {
    return cap;
  }

  const double cruise_speed_mps =
      sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
  const double min_speed_mps = std::min(
      cruise_speed_mps,
      sanitizedPositive(config.vertical_trackability_min_speed_mps, 1.0, 0.0, 100.0));
  const double speed_limit_mps = std::clamp(constraint_distance_m / time_needed_s,
                                            min_speed_mps, cruise_speed_mps);
  if (speed_limit_mps + 1.0e-9 >= cruise_speed_mps) {
    return cap;
  }

  cap.active = true;
  cap.speed_limit_mps = speed_limit_mps;
  cap.constraint_distance_m = constraint_distance_m;
  cap.altitude_error_m = altitude_error;
  return cap;
}

} // namespace drone_city_nav
