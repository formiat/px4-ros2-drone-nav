#include "drone_city_nav/mppi/mppi_noncooperative_acquisition.hpp"

#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
#include <utility>

namespace drone_city_nav::mppi {
namespace {

#include "mppi_engine_host_validation.hpp"

struct CandidateEvaluation {
  std::size_t index{0U};
  float minimum_separation_m{-std::numeric_limits<float>::infinity()};
  float head_progress_m{0.0F};
  float terminal_progress_m{0.0F};
  bool raw_safe{false};
  bool preserves_required_separation{false};
  bool positive_progress{false};
  bool backward_candidate{false};
};

[[nodiscard]] float
requiredSeparation(const NonCooperativeAcquisitionEvaluationInput& input) noexcept {
  float required_m = input.cost_policy.strong_separation_m;
  for (const DynamicAircraftTrajectory& aircraft : input.aircraft) {
    required_m = std::max(required_m, input.config.footprint.radius_m +
                                          aircraft.footprint_radius_m);
  }
  return required_m;
}

[[nodiscard]] std::pair<float, float>
routeDirection(const NonCooperativeAcquisitionEvaluationInput& input) noexcept {
  if (input.route.size() >= 2U) {
    const auto sample =
        std::ranges::find_if(input.route, [&](const RouteSample3D& point) {
          return point.station_m + 1.0e-5F >= input.initial_route_station_m;
        });
    const RouteSample3D& point =
        sample != input.route.end() ? *sample : input.route.back();
    const float tangent_norm = std::hypot(point.tangent_x, point.tangent_y);
    if (tangent_norm > 1.0e-5F) {
      return {point.tangent_x / tangent_norm, point.tangent_y / tangent_norm};
    }
  }
  const float dx = input.target.x - input.initial_state.x;
  const float dy = input.target.y - input.initial_state.y;
  const float distance_m = std::hypot(dx, dy);
  return distance_m > 1.0e-5F ? std::pair{dx / distance_m, dy / distance_m}
                              : std::pair{1.0F, 0.0F};
}

[[nodiscard]] CandidateEvaluation
evaluateCandidate(const NonCooperativeAcquisitionEvaluationInput& input,
                  const std::span<const Control> controls, const std::size_t index,
                  const std::pair<float, float> direction,
                  const std::span<const Control> zero_noise,
                  const float required_separation_m) {
  ReferenceSimulationTrace trace;
  const RolloutMetrics metrics = simulateReference(
      input.initial_state, controls, zero_noise, input.config.dynamics,
      input.config.risk, input.config.costs, input.grid, input.esdf, input.target.x,
      input.target.y, input.config.early_exit_on_collision,
      input.previous_applied_control, input.reference_speed_mps, input.config.footprint,
      std::nullopt, &trace, input.aircraft, std::nullopt, input.config.cooperative,
      input.cost_policy, input.config.altitude_envelope);
  const bool solid_collision = hostSweptSolidCollision(
      trace.horizon, controls, input.config.footprint, input.known_solids);
  const std::size_t head_step = std::clamp<std::size_t>(
      static_cast<std::size_t>(std::ceil(input.config.costs.head_progress_horizon_s /
                                         input.config.dynamics.dt_s)),
      1U, controls.size());
  const auto progress = [&](const State& state) {
    return (state.x - input.initial_state.x) * direction.first +
           (state.y - input.initial_state.y) * direction.second;
  };
  const float head_progress_m = progress(trace.horizon[head_step]);
  const float terminal_progress_m = progress(trace.horizon.back());
  const bool backward_candidate =
      index == static_cast<std::size_t>(NonCooperativeManeuver::kBackward);
  return CandidateEvaluation{
      .index = index,
      .minimum_separation_m = metrics.minimum_peer_separation_m,
      .head_progress_m = head_progress_m,
      .terminal_progress_m = terminal_progress_m,
      .raw_safe = !metrics.altitude_envelope_violation && !metrics.collision &&
                  !solid_collision,
      .preserves_required_separation =
          metrics.minimum_peer_separation_m >= required_separation_m,
      .positive_progress =
          !backward_candidate && head_progress_m >= 0.0F && terminal_progress_m >= 0.0F,
      .backward_candidate = backward_candidate,
  };
}

[[nodiscard]] bool
betterPositiveProgress(const CandidateEvaluation& candidate,
                       const CandidateEvaluation& selected) noexcept {
  return std::tie(candidate.head_progress_m, candidate.terminal_progress_m,
                  candidate.minimum_separation_m) >
         std::tie(selected.head_progress_m, selected.terminal_progress_m,
                  selected.minimum_separation_m);
}

[[nodiscard]] bool
betterProgressFallback(const CandidateEvaluation& candidate,
                       const CandidateEvaluation& selected) noexcept {
  return std::tie(candidate.terminal_progress_m, candidate.head_progress_m,
                  candidate.minimum_separation_m) >
         std::tie(selected.terminal_progress_m, selected.head_progress_m,
                  selected.minimum_separation_m);
}

[[nodiscard]] bool
betterSurvivalFallback(const CandidateEvaluation& candidate,
                       const CandidateEvaluation& selected) noexcept {
  return std::tie(candidate.minimum_separation_m, candidate.head_progress_m,
                  candidate.terminal_progress_m) >
         std::tie(selected.minimum_separation_m, selected.head_progress_m,
                  selected.terminal_progress_m);
}

} // namespace

NonCooperativeAcquisitionResult evaluateNonCooperativeAcquisition(
    const NonCooperativeAcquisitionEvaluationInput& input) {
  NonCooperativeAcquisitionResult result;
  if (input.aircraft.empty() || input.config.steps == 0U || input.esdf.empty()) {
    return result;
  }
  const std::vector<Control> candidates =
      buildNonCooperativeSeparationAcquisitionCandidates(
          input.initial_state, input.target, input.route, input.initial_route_station_m,
          input.reference_speed_mps, input.acquisition, input.config.dynamics,
          input.config.steps, input.previous_applied_control,
          input.first_control_interval_s);
  const std::vector<Control> zero_noise(input.config.steps);
  const std::pair<float, float> direction = routeDirection(input);
  const float required_separation_m = requiredSeparation(input);
  std::optional<CandidateEvaluation> baseline;
  std::optional<CandidateEvaluation> positive_progress;
  std::optional<CandidateEvaluation> non_backward_fallback;
  std::optional<CandidateEvaluation> preserving_fallback;
  std::optional<CandidateEvaluation> survival_fallback;
  for (std::size_t index = 0U; index < kNonCooperativeAcquisitionCandidateCount;
       ++index) {
    const std::span<const Control> controls =
        std::span<const Control>{candidates}.subspan(index * input.config.steps,
                                                     input.config.steps);
    const CandidateEvaluation evaluation = evaluateCandidate(
        input, controls, index, direction, zero_noise, required_separation_m);
    if (index == static_cast<std::size_t>(NonCooperativeManeuver::kRouteCruise)) {
      baseline = evaluation;
    }
    if (!evaluation.raw_safe) {
      continue;
    }
    if (!survival_fallback || betterSurvivalFallback(evaluation, *survival_fallback)) {
      survival_fallback = evaluation;
    }
    if (!evaluation.preserves_required_separation) {
      continue;
    }
    if (!preserving_fallback ||
        betterProgressFallback(evaluation, *preserving_fallback)) {
      preserving_fallback = evaluation;
    }
    if (!evaluation.backward_candidate &&
        (!non_backward_fallback ||
         betterProgressFallback(evaluation, *non_backward_fallback))) {
      non_backward_fallback = evaluation;
    }
    if (evaluation.positive_progress &&
        (!positive_progress ||
         betterPositiveProgress(evaluation, *positive_progress))) {
      positive_progress = evaluation;
    }
  }
  std::optional<CandidateEvaluation> selected = positive_progress;
  if (!selected) {
    selected = non_backward_fallback;
  }
  if (!selected) {
    selected = preserving_fallback;
  }
  if (!selected) {
    selected = survival_fallback;
  }
  if (!selected) {
    return result;
  }
  const std::size_t offset = selected->index * input.config.steps;
  result.controls.assign(candidates.begin() + static_cast<std::ptrdiff_t>(offset),
                         candidates.begin() +
                             static_cast<std::ptrdiff_t>(offset + input.config.steps));
  result.candidate_index = selected->index;
  result.maneuver = static_cast<NonCooperativeManeuver>(selected->index);
  result.minimum_separation_m = selected->minimum_separation_m;
  result.separation_gain_m =
      baseline && std::isfinite(baseline->minimum_separation_m)
          ? selected->minimum_separation_m - baseline->minimum_separation_m
          : 0.0F;
  result.head_progress_m = selected->head_progress_m;
  result.terminal_progress_m = selected->terminal_progress_m;
  result.available = true;
  return result;
}

NonCooperativeAcquisitionLifecycleResult NonCooperativeAcquisitionLifecycle::update(
    const NonCooperativeAcquisitionLifecycleInput& input) {
  NonCooperativeAcquisitionLifecycleResult result;
  const bool entered = input.avoidance_active && !avoidance_active_;
  const bool released = !input.avoidance_active && avoidance_active_;
  avoidance_active_ = input.avoidance_active;
  acquisition_pending_ = entered || (acquisition_pending_ && input.avoidance_active);

  if (acquisition_pending_ && input.acquisition && !input.evaluation.aircraft.empty()) {
    NonCooperativeAcquisitionEvaluationInput evaluation = input.evaluation;
    evaluation.acquisition = *input.acquisition;
    result.acquisition = evaluateNonCooperativeAcquisition(evaluation);
    result.nominal_reseed =
        result.acquisition.available
            ? std::move(result.acquisition.controls)
            : buildGuideDirectedNominalSeed(
                  evaluation.initial_state, evaluation.target, evaluation.route,
                  evaluation.initial_route_station_m, evaluation.reference_speed_mps,
                  evaluation.config.dynamics, evaluation.config.steps,
                  evaluation.previous_applied_control);
    acquisition_pending_ = false;
    acquisition_applied_ = true;
    result.acquisition_reseeded = true;
    return result;
  }
  if (!released) {
    return result;
  }

  acquisition_pending_ = false;
  if (acquisition_applied_) {
    const NonCooperativeAcquisitionEvaluationInput& evaluation = input.evaluation;
    result.nominal_reseed = buildGuideDirectedNominalSeed(
        evaluation.initial_state, evaluation.target, evaluation.route,
        evaluation.initial_route_station_m, evaluation.reference_speed_mps,
        evaluation.config.dynamics, evaluation.config.steps,
        evaluation.previous_applied_control);
    result.release_reseeded = true;
  }
  acquisition_applied_ = false;
  return result;
}

} // namespace drone_city_nav::mppi
