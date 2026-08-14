#include "drone_city_nav/mppi/mppi_separation_acquisition.hpp"

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
  float head_progress_m{0.0F};
  float terminal_progress_m{0.0F};
  float separation_gain_m{-std::numeric_limits<float>::infinity()};
  bool raw_safe{false};
  bool separating{false};
  bool positive_progress{false};
  bool backward_candidate{false};
};

[[nodiscard]] std::pair<float, float>
routeDirection(const CooperativeSeparationAcquisitionEvaluationInput& input) noexcept {
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

[[nodiscard]] float minimumAircraftSeparationGain(
    const State& initial_state, const std::span<const State> horizon,
    const std::span<const DynamicAircraftTrajectory> aircraft) noexcept {
  float separation_gain_m = std::numeric_limits<float>::infinity();
  for (const DynamicAircraftTrajectory& tracked_aircraft : aircraft) {
    if (!tracked_aircraft.samples || tracked_aircraft.active_steps == 0U) {
      continue;
    }
    const DynamicAircraftSample& first = tracked_aircraft.samples->front();
    const std::size_t step = tracked_aircraft.active_steps - 1U;
    const DynamicAircraftSample& last = (*tracked_aircraft.samples)[step];
    const State& terminal = horizon[std::min(step + 1U, horizon.size() - 1U)];
    const float initial_separation_m =
        std::hypot(std::hypot(first.x - initial_state.x, first.y - initial_state.y),
                   first.z - initial_state.z);
    const float terminal_separation_m = std::hypot(
        std::hypot(last.x - terminal.x, last.y - terminal.y), last.z - terminal.z);
    separation_gain_m =
        std::min(separation_gain_m, terminal_separation_m - initial_separation_m);
  }
  return separation_gain_m;
}

[[nodiscard]] CandidateEvaluation
evaluateCandidate(const CooperativeSeparationAcquisitionEvaluationInput& input,
                  const std::span<const Control> controls, const std::size_t index,
                  const std::pair<float, float> direction,
                  const std::span<const Control> zero_noise) {
  ReferenceSimulationTrace trace;
  const RolloutMetrics metrics = simulateReference(
      input.initial_state, controls, zero_noise, input.config.dynamics,
      input.config.risk, input.config.costs, input.grid, input.esdf, input.target.x,
      input.target.y, input.config.early_exit_on_collision,
      input.previous_applied_control, input.reference_speed_mps, input.config.footprint,
      std::nullopt, &trace, input.aircraft, input.acquisition.preference,
      input.config.cooperative);
  const bool solid_collision = hostSweptSolidCollision(
      trace.horizon, controls, input.config.footprint, input.known_solids);
  const std::size_t head_step = std::clamp<std::size_t>(
      static_cast<std::size_t>(std::ceil(input.config.costs.head_progress_horizon_s /
                                         input.config.dynamics.dt_s)),
      1U, controls.size());
  const State& head = trace.horizon[head_step];
  const State& terminal = trace.horizon.back();
  const auto progress = [&](const State& state) {
    return (state.x - input.initial_state.x) * direction.first +
           (state.y - input.initial_state.y) * direction.second;
  };
  const float separation_gain_m =
      minimumAircraftSeparationGain(input.initial_state, trace.horizon, input.aircraft);
  const float head_progress_m = progress(head);
  const bool backward_candidate = index + 1U == kCooperativeAcquisitionCandidateCount;
  return CandidateEvaluation{
      .index = index,
      .head_progress_m = head_progress_m,
      .terminal_progress_m = progress(terminal),
      .separation_gain_m = separation_gain_m,
      .raw_safe = !metrics.collision && !solid_collision,
      .separating = std::isfinite(separation_gain_m) &&
                    separation_gain_m >= input.acquisition.minimum_separation_gain_m,
      .positive_progress =
          !backward_candidate &&
          head_progress_m >= input.acquisition.minimum_positive_progress_m &&
          progress(terminal) >= input.acquisition.minimum_positive_progress_m,
      .backward_candidate = backward_candidate,
  };
}

[[nodiscard]] bool betterPositive(const CandidateEvaluation& candidate,
                                  const CandidateEvaluation& selected) noexcept {
  return std::tie(candidate.head_progress_m, candidate.terminal_progress_m,
                  candidate.separation_gain_m) > std::tie(selected.head_progress_m,
                                                          selected.terminal_progress_m,
                                                          selected.separation_gain_m);
}

[[nodiscard]] bool
betterNonBackwardFallback(const CandidateEvaluation& candidate,
                          const CandidateEvaluation& selected) noexcept {
  return std::tie(candidate.terminal_progress_m, candidate.head_progress_m,
                  candidate.separation_gain_m) > std::tie(selected.terminal_progress_m,
                                                          selected.head_progress_m,
                                                          selected.separation_gain_m);
}

} // namespace

CooperativeSeparationAcquisitionResult evaluateCooperativeSeparationAcquisition(
    const CooperativeSeparationAcquisitionEvaluationInput& input) {
  CooperativeSeparationAcquisitionResult result;
  if (input.aircraft.empty() || input.config.steps == 0U || input.esdf.empty()) {
    return result;
  }
  const std::vector<Control> candidates =
      buildCooperativeSeparationAcquisitionCandidates(
          input.initial_state, input.target, input.route, input.initial_route_station_m,
          input.reference_speed_mps, input.acquisition, input.config.dynamics,
          input.config.cooperative, input.config.steps, input.previous_applied_control,
          input.first_control_interval_s);
  const std::vector<Control> zero_noise(input.config.steps);
  const std::pair<float, float> direction = routeDirection(input);
  std::optional<CandidateEvaluation> positive;
  std::optional<CandidateEvaluation> non_backward_fallback;
  std::optional<CandidateEvaluation> backward_fallback;
  for (std::size_t index = 0U; index < kCooperativeAcquisitionCandidateCount; ++index) {
    const std::span<const Control> controls =
        std::span<const Control>{candidates}.subspan(index * input.config.steps,
                                                     input.config.steps);
    const CandidateEvaluation evaluation =
        evaluateCandidate(input, controls, index, direction, zero_noise);
    if (!evaluation.raw_safe || !evaluation.separating) {
      continue;
    }
    if (evaluation.positive_progress &&
        (!positive || betterPositive(evaluation, *positive))) {
      positive = evaluation;
    }
    if (evaluation.backward_candidate) {
      backward_fallback = evaluation;
    } else if (!non_backward_fallback ||
               betterNonBackwardFallback(evaluation, *non_backward_fallback)) {
      non_backward_fallback = evaluation;
    }
  }
  std::optional<CandidateEvaluation> selected = positive;
  if (!selected) {
    selected = non_backward_fallback;
  }
  if (!selected) {
    selected = backward_fallback;
  }
  if (!selected) {
    return result;
  }
  const std::size_t offset = selected->index * input.config.steps;
  result.controls.assign(candidates.begin() + static_cast<std::ptrdiff_t>(offset),
                         candidates.begin() +
                             static_cast<std::ptrdiff_t>(offset + input.config.steps));
  result.candidate_index = selected->index;
  result.head_progress_m = selected->head_progress_m;
  result.terminal_progress_m = selected->terminal_progress_m;
  result.separation_gain_m = selected->separation_gain_m;
  result.available = true;
  result.positive_progress = selected->positive_progress;
  result.backward_fallback = selected->backward_candidate;
  return result;
}

CooperativeSeparationAcquisitionLifecycleResult
CooperativeSeparationAcquisitionLifecycle::update(
    const CooperativeSeparationAcquisitionLifecycleInput& input) {
  CooperativeSeparationAcquisitionLifecycleResult result;
  const bool entered = input.avoidance_active && !avoidance_active_;
  const bool released = !input.avoidance_active && avoidance_active_;
  avoidance_active_ = input.avoidance_active;
  acquisition_pending_ = entered || (acquisition_pending_ && input.avoidance_active);

  if (acquisition_pending_ && input.acquisition && !input.evaluation.aircraft.empty()) {
    CooperativeSeparationAcquisitionEvaluationInput evaluation = input.evaluation;
    evaluation.acquisition = *input.acquisition;
    result.acquisition = evaluateCooperativeSeparationAcquisition(evaluation);
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
    const auto& evaluation = input.evaluation;
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
