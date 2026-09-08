#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

// The two passes every published route goes through before it becomes a
// candidate: shortcut simplification and clearance centering.

namespace drone_city_nav::detail {
namespace {

constexpr double kCostTolerance{1.0e-12};

} // namespace

std::vector<Point3>
PathPostprocessor3D::shortcut(const std::vector<Point3>& path,
                              const PathPostprocessorContext3D& context,
                              std::size_t& checks, std::size_t& applied) const {
  if (path.size() < 3U || context.maximum_shortcut_checks == 0U ||
      !context.segment_valid || !context.time_profile || !context.segment_factor) {
    return path;
  }
  const auto past_deadline = [&context]() {
    return context.deadline.has_value() &&
           std::chrono::steady_clock::now() >= *context.deadline;
  };
  // Ranked execution time of a path from its time profile and the factor of
  // each of its segments; stationary turn time is not scaled.
  const auto ranked_time = [](const FlightPathTimeProfile3D& profile,
                              const std::vector<double>& factors) {
    double ranked_s = 0.0;
    for (std::size_t index = 0U; index < factors.size(); ++index) {
      const double segment_s = std::max(0.0, profile.arrival_times_s[index + 1U] -
                                                 profile.departure_times_s[index]);
      const double turn_s = std::max(0.0, profile.departure_times_s[index] -
                                              profile.arrival_times_s[index]);
      ranked_s += turn_s + segment_s * factors[index];
    }
    return ranked_s;
  };
  std::vector<Point3> result = path;
  FlightPathTimeProfile3D current_profile = context.time_profile(result);
  if (!current_profile.valid ||
      current_profile.arrival_times_s.size() != result.size() ||
      current_profile.departure_times_s.size() != result.size()) {
    return path;
  }
  std::vector<double> factors;
  factors.reserve(result.size() - 1U);
  for (std::size_t index = 0U; index + 1U < result.size(); ++index) {
    if (past_deadline()) {
      return path;
    }
    factors.push_back(context.segment_factor(result[index], result[index + 1U]));
  }
  double current_ranked_time_s = ranked_time(current_profile, factors);
  std::size_t anchor = 0U;
  while (anchor + 2U < result.size()) {
    bool shortcut_applied{false};
    for (std::size_t candidate = result.size() - 1U; candidate > anchor + 1U;
         --candidate) {
      if (checks >= context.maximum_shortcut_checks || past_deadline()) {
        return result;
      }
      ++checks;
      const bool shortcut_valid =
          context.segment_valid(result[anchor], result[candidate], anchor == 0U);
      if (!shortcut_valid) {
        continue;
      }
      std::vector<Point3> trial = result;
      trial.erase(std::next(trial.begin(), static_cast<std::ptrdiff_t>(anchor + 1U)),
                  std::next(trial.begin(), static_cast<std::ptrdiff_t>(candidate)));
      FlightPathTimeProfile3D trial_profile = context.time_profile(trial);
      if (!trial_profile.valid ||
          trial_profile.arrival_times_s.size() != trial.size() ||
          trial_profile.departure_times_s.size() != trial.size()) {
        continue;
      }
      // The shortcut replaces the segments from the anchor to the candidate
      // with one; every other segment keeps the factor it already has.
      std::vector<double> trial_factors;
      trial_factors.reserve(trial.size() - 1U);
      trial_factors.insert(
          trial_factors.end(), factors.begin(),
          std::next(factors.begin(), static_cast<std::ptrdiff_t>(anchor)));
      trial_factors.push_back(
          context.segment_factor(result[anchor], result[candidate]));
      trial_factors.insert(
          trial_factors.end(),
          std::next(factors.begin(), static_cast<std::ptrdiff_t>(candidate)),
          factors.end());
      const double trial_ranked_time_s = ranked_time(trial_profile, trial_factors);
      if (trial_ranked_time_s > current_ranked_time_s + kCostTolerance) {
        continue;
      }
      applied += candidate - anchor - 1U;
      result = std::move(trial);
      factors = std::move(trial_factors);
      current_profile = std::move(trial_profile);
      current_ranked_time_s = trial_ranked_time_s;
      shortcut_applied = true;
      break;
    }
    ++anchor;
    if (checks >= context.maximum_shortcut_checks && !shortcut_applied) {
      break;
    }
  }
  return result;
}

std::vector<Point3>
PathPostprocessor3D::centerOnClearance(const std::vector<Point3>& path,
                                       const PathClearanceCenteringContext3D& context,
                                       std::size_t& queries, std::size_t& moved) const {
  if (path.size() < 3U || !context.segment_valid || !context.clearance ||
      !(context.target_clearance_m > 0.0) || !(context.probe_step_m > 0.0) ||
      context.maximum_passes == 0U || context.maximum_clearance_queries == 0U) {
    return path;
  }
  std::vector<Point3> result = path;
  const auto measure = [&](const Point3& point) {
    ++queries;
    return context.clearance(point);
  };
  const auto observed = [&](const Point3& point) {
    return !context.observed || context.observed(point);
  };
  // The vertex slides across the passage, not along the route: a move along
  // the local tangent only re-parameterises the path and can lengthen it.
  const auto acrossPath = [](const Vec3& gradient, const Point3& previous,
                             const Point3& next) {
    const Vec3 tangent{next.x - previous.x, next.y - previous.y, next.z - previous.z};
    const double tangent_length = std::sqrt(
        tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z);
    if (!(tangent_length > 1.0e-6)) {
      return gradient;
    }
    const Vec3 unit{tangent.x / tangent_length, tangent.y / tangent_length,
                    tangent.z / tangent_length};
    const double along =
        gradient.x * unit.x + gradient.y * unit.y + gradient.z * unit.z;
    return Vec3{gradient.x - along * unit.x, gradient.y - along * unit.y,
                gradient.z - along * unit.z};
  };
  for (std::size_t pass = 0U; pass < context.maximum_passes; ++pass) {
    bool moved_in_pass{false};
    for (std::size_t index = 1U; index + 1U < result.size(); ++index) {
      if (queries >= context.maximum_clearance_queries ||
          (context.deadline.has_value() &&
           std::chrono::steady_clock::now() >= *context.deadline)) {
        return result;
      }
      const Point3 vertex = result[index];
      const double clearance_m = measure(vertex);
      if (clearance_m >= context.target_clearance_m) {
        continue;
      }
      const double step_m = context.probe_step_m;
      // A probe in unobserved space says nothing about the clearance there:
      // the measured distance runs to observed evidence only, so it would
      // point the vertex into whatever the next scan reveals.
      const auto axisGradient = [&](const Vec3& axis) {
        const Point3 forward{vertex.x + step_m * axis.x, vertex.y + step_m * axis.y,
                             vertex.z + step_m * axis.z};
        const Point3 backward{vertex.x - step_m * axis.x, vertex.y - step_m * axis.y,
                              vertex.z - step_m * axis.z};
        if (!observed(forward) || !observed(backward)) {
          return 0.0;
        }
        return measure(forward) - measure(backward);
      };
      Vec3 gradient{axisGradient(Vec3{1.0, 0.0, 0.0}),
                    axisGradient(Vec3{0.0, 1.0, 0.0}),
                    axisGradient(Vec3{0.0, 0.0, 1.0})};
      gradient = acrossPath(gradient, result[index - 1U], result[index + 1U]);
      const double gradient_length = std::sqrt(
          gradient.x * gradient.x + gradient.y * gradient.y + gradient.z * gradient.z);
      if (!(gradient_length > 1.0e-9)) {
        continue;
      }
      const double scale = step_m / gradient_length;
      const Point3 candidate{vertex.x + scale * gradient.x,
                             vertex.y + scale * gradient.y,
                             vertex.z + scale * gradient.z};
      if (!observed(candidate) || measure(candidate) <= clearance_m) {
        continue;
      }
      // A departure segment leaves the vehicle's own position, where contact
      // evidence is exempt; the ordinary raw rule applies everywhere else.
      if (!context.segment_valid(result[index - 1U], candidate, index == 1U) ||
          !context.segment_valid(candidate, result[index + 1U], false)) {
        continue;
      }
      result[index] = candidate;
      ++moved;
      moved_in_pass = true;
    }
    if (!moved_in_pass) {
      break;
    }
  }
  return result;
}

} // namespace drone_city_nav::detail
