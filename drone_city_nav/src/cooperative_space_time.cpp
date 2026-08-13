#include "drone_city_nav/cooperative_space_time.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};

struct Candidate {
  CooperativeManeuver maneuver{CooperativeManeuver::kKeep};
  Vec3 direction{};
  double spatial_offset_m{0.0};
  double time_shift_s{0.0};
  double pair_preference_alignment{0.0};
};

struct CandidateEvaluation {
  Candidate candidate{};
  bool valid{false};
  bool separation_satisfied{false};
  double minimum_separation_m{std::numeric_limits<double>::infinity()};
  double time_to_minimum_s{0.0};
  double integrated_shortfall_m2_s{0.0};
  double forward_progress_delta_m{0.0};
};

[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] double norm(const Vec3& value) noexcept {
  return std::hypot(std::hypot(value.x, value.y), value.z);
}

[[nodiscard]] Vec3 normalized(const Vec3& value) noexcept {
  const double length = norm(value);
  return length > 1.0e-9 ? Vec3{value.x / length, value.y / length, value.z / length}
                         : Vec3{};
}

[[nodiscard]] Vec3 scaled(const Vec3& value, const double scale) noexcept {
  return Vec3{scale * value.x, scale * value.y, scale * value.z};
}

[[nodiscard]] Point3 translated(const Point3& value, const Vec3& offset) noexcept {
  return Point3{value.x + offset.x, value.y + offset.y, value.z + offset.z};
}

[[nodiscard]] double smoothStep(const double value) noexcept {
  const double bounded = std::clamp(value, 0.0, 1.0);
  return bounded * bounded * (3.0 - 2.0 * bounded);
}

[[nodiscard]] bool validConfig(const CooperativeSpaceTimeConfig& config) noexcept {
  return std::isfinite(config.prediction_horizon_s) &&
         config.prediction_horizon_s > 0.0 &&
         std::isfinite(config.desired_minimum_separation_m) &&
         config.desired_minimum_separation_m > 0.0 &&
         std::isfinite(config.spatial_transition_s) &&
         config.spatial_transition_s > 0.0 &&
         std::isfinite(config.minimum_spatial_offset_m) &&
         config.minimum_spatial_offset_m > 0.0 &&
         std::isfinite(config.maximum_spatial_offset_m) &&
         config.maximum_spatial_offset_m >= config.minimum_spatial_offset_m &&
         std::isfinite(config.minimum_time_shift_s) &&
         config.minimum_time_shift_s > 0.0 &&
         std::isfinite(config.maximum_time_shift_s) &&
         config.maximum_time_shift_s >= config.minimum_time_shift_s &&
         std::isfinite(config.sample_period_s) && config.sample_period_s > 0.0 &&
         std::isfinite(config.spatial_margin_m) && config.spatial_margin_m >= 0.0 &&
         std::isfinite(config.incumbent_hysteresis_m) &&
         config.incumbent_hysteresis_m >= 0.0;
}

[[nodiscard]] Vec3 routeForward(const CooperativeFlightIntentData& ownship,
                                const std::int64_t now_ns) noexcept {
  const std::int64_t future_ns =
      std::min(ownship.valid_until_ns,
               now_ns + static_cast<std::int64_t>(std::llround(kNanosecondsPerSecond)));
  const auto current =
      sampleCooperativeTrajectory(ownship, std::max(now_ns, ownship.valid_from_ns));
  const auto future = sampleCooperativeTrajectory(ownship, future_ns);
  if (current.has_value() && future.has_value()) {
    const Vec3 displacement{future->position.x - current->position.x,
                            future->position.y - current->position.y,
                            future->position.z - current->position.z};
    if (norm(displacement) > 1.0e-6) {
      return normalized(displacement);
    }
  }
  if (norm(ownship.current_velocity) > 1.0e-6) {
    return normalized(ownship.current_velocity);
  }
  return Vec3{1.0, 0.0, 0.0};
}

[[nodiscard]] Vec3 horizontalLeft(const Vec3& forward) noexcept {
  const double horizontal = std::hypot(forward.x, forward.y);
  return horizontal > 1.0e-6
             ? Vec3{-forward.y / horizontal, forward.x / horizontal, 0.0}
             : Vec3{0.0, 1.0, 0.0};
}

[[nodiscard]] std::int64_t secondsToNanoseconds(const double seconds) noexcept {
  return static_cast<std::int64_t>(std::llround(seconds * kNanosecondsPerSecond));
}

[[nodiscard]] Point3 candidatePosition(const Candidate& candidate,
                                       const CooperativeFlightIntentData& ownship,
                                       const std::int64_t time_ns,
                                       const std::int64_t now_ns,
                                       const double transition_s) noexcept {
  const double elapsed_s =
      std::max(0.0, static_cast<double>(time_ns - now_ns) / kNanosecondsPerSecond);
  if (candidate.time_shift_s > 0.0) {
    const double transition_ratio = smoothStep(elapsed_s / transition_s);
    const std::int64_t delayed_time_ns = std::max(
        ownship.valid_from_ns,
        time_ns - secondsToNanoseconds(transition_ratio * candidate.time_shift_s));
    if (const auto delayed = sampleCooperativeTrajectory(ownship, delayed_time_ns)) {
      return delayed->position;
    }
  }
  const auto base = sampleCooperativeTrajectory(ownship, time_ns);
  if (!base.has_value()) {
    return ownship.current_position;
  }
  const double offset_ratio = smoothStep(elapsed_s / transition_s);
  return translated(base->position, scaled(candidate.direction,
                                           offset_ratio * candidate.spatial_offset_m));
}

[[nodiscard]] bool sameCandidate(const Candidate& first,
                                 const Candidate& second) noexcept {
  return first.maneuver == second.maneuver &&
         std::abs(first.direction.x - second.direction.x) <= 1.0e-6 &&
         std::abs(first.direction.y - second.direction.y) <= 1.0e-6 &&
         std::abs(first.direction.z - second.direction.z) <= 1.0e-6 &&
         std::abs(first.spatial_offset_m - second.spatial_offset_m) <= 1.0e-6 &&
         std::abs(first.time_shift_s - second.time_shift_s) <= 1.0e-6;
}

[[nodiscard]] bool candidateOrder(const Candidate& first,
                                  const Candidate& second) noexcept {
  const auto first_maneuver = static_cast<std::uint8_t>(first.maneuver);
  const auto second_maneuver = static_cast<std::uint8_t>(second.maneuver);
  if (first_maneuver != second_maneuver) {
    return first_maneuver < second_maneuver;
  }
  return std::tie(first.direction.x, first.direction.y, first.direction.z,
                  first.spatial_offset_m, first.time_shift_s) <
         std::tie(second.direction.x, second.direction.y, second.direction.z,
                  second.spatial_offset_m, second.time_shift_s);
}

void appendCandidate(std::vector<Candidate>& candidates, Candidate candidate) {
  candidate.direction = normalized(candidate.direction);
  if (std::ranges::none_of(candidates, [&](const Candidate& existing) {
        return sameCandidate(existing, candidate);
      })) {
    candidates.push_back(candidate);
  }
}

[[nodiscard]] double candidateEffort(const CandidateEvaluation& evaluation,
                                     const double nominal_speed_mps) noexcept {
  return evaluation.candidate.spatial_offset_m +
         evaluation.candidate.time_shift_s * std::max(1.0, nominal_speed_mps);
}

[[nodiscard]] bool betterCandidate(const CandidateEvaluation& candidate,
                                   const CandidateEvaluation& incumbent,
                                   const double nominal_speed_mps) noexcept {
  if (candidate.separation_satisfied != incumbent.separation_satisfied) {
    return candidate.separation_satisfied;
  }
  if (!candidate.separation_satisfied) {
    if (std::abs(candidate.integrated_shortfall_m2_s -
                 incumbent.integrated_shortfall_m2_s) > 1.0e-6) {
      return candidate.integrated_shortfall_m2_s < incumbent.integrated_shortfall_m2_s;
    }
    if (std::abs(candidate.minimum_separation_m - incumbent.minimum_separation_m) >
        1.0e-6) {
      return candidate.minimum_separation_m > incumbent.minimum_separation_m;
    }
  } else {
    const double candidate_effort = candidateEffort(candidate, nominal_speed_mps);
    const double incumbent_effort = candidateEffort(incumbent, nominal_speed_mps);
    if (std::abs(candidate_effort - incumbent_effort) > 1.0e-6) {
      return candidate_effort < incumbent_effort;
    }
  }
  if (std::abs(candidate.forward_progress_delta_m -
               incumbent.forward_progress_delta_m) > 1.0e-6) {
    return candidate.forward_progress_delta_m > incumbent.forward_progress_delta_m;
  }
  if (std::abs(candidate.candidate.pair_preference_alignment -
               incumbent.candidate.pair_preference_alignment) > 1.0e-6) {
    return candidate.candidate.pair_preference_alignment >
           incumbent.candidate.pair_preference_alignment;
  }
  return static_cast<std::uint8_t>(candidate.candidate.maneuver) <
         static_cast<std::uint8_t>(incumbent.candidate.maneuver);
}

[[nodiscard]] CandidateEvaluation
evaluateCandidate(const Candidate& candidate,
                  const CooperativeFlightIntentData& ownship,
                  const std::span<const CooperativeConflictPeer* const> peers,
                  const std::int64_t now_ns, const CooperativeSpaceTimeConfig& config,
                  const Vec3& forward) noexcept {
  CandidateEvaluation result{.candidate = candidate};
  const std::int64_t horizon_end_ns =
      now_ns + secondsToNanoseconds(config.prediction_horizon_s);
  const std::int64_t step_ns = secondsToNanoseconds(config.sample_period_s);
  if (step_ns <= 0) {
    return result;
  }
  double maximum_shortfall_m = 0.0;
  for (const CooperativeConflictPeer* conflict : peers) {
    if (conflict == nullptr || conflict->intent.trajectory.empty()) {
      return result;
    }
    const CooperativeFlightIntentData& peer = conflict->intent;
    const std::int64_t begin_ns =
        std::max({now_ns, ownship.valid_from_ns, peer.valid_from_ns,
                  ownship.trajectory.front().time_ns, peer.trajectory.front().time_ns});
    const std::int64_t end_ns =
        std::min({horizon_end_ns, ownship.valid_until_ns, peer.valid_until_ns,
                  ownship.trajectory.back().time_ns, peer.trajectory.back().time_ns});
    if (end_ns < begin_ns) {
      continue;
    }
    const double desired_separation_m =
        std::max(config.desired_minimum_separation_m,
                 ownship.footprint_radius_m + peer.footprint_radius_m);
    std::int64_t sample_ns = begin_ns;
    while (true) {
      const std::optional<CooperativeTrajectorySample> peer_sample =
          sampleCooperativeTrajectory(peer, sample_ns);
      if (!peer_sample.has_value()) {
        return result;
      }
      const Point3 own_position = candidatePosition(
          candidate, ownship, sample_ns, now_ns, config.spatial_transition_s);
      const double separation_m = distance3D(own_position, peer_sample->position);
      if (separation_m < result.minimum_separation_m) {
        result.minimum_separation_m = separation_m;
        result.time_to_minimum_s = std::max(
            0.0, static_cast<double>(sample_ns - now_ns) / kNanosecondsPerSecond);
      }
      const double shortfall_m = std::max(0.0, desired_separation_m - separation_m);
      maximum_shortfall_m = std::max(maximum_shortfall_m, shortfall_m);
      result.integrated_shortfall_m2_s +=
          shortfall_m * shortfall_m * config.sample_period_s;
      result.valid = true;
      if (sample_ns == end_ns) {
        break;
      }
      sample_ns = std::min(end_ns, sample_ns + step_ns);
    }
  }
  if (!result.valid) {
    return result;
  }
  const std::int64_t terminal_ns =
      std::min(horizon_end_ns,
               std::min(ownship.valid_until_ns, ownship.trajectory.back().time_ns));
  if (const auto base = sampleCooperativeTrajectory(ownship, terminal_ns)) {
    const Point3 candidate_terminal = candidatePosition(
        candidate, ownship, terminal_ns, now_ns, config.spatial_transition_s);
    result.forward_progress_delta_m = dot(Vec3{candidate_terminal.x - base->position.x,
                                               candidate_terminal.y - base->position.y,
                                               candidate_terminal.z - base->position.z},
                                          forward);
  }
  result.separation_satisfied = maximum_shortfall_m <= 1.0e-6;
  return result;
}

} // namespace

bool cooperativeSpaceTimeConfigIsValid(
    const CooperativeSpaceTimeConfig& config) noexcept {
  return validConfig(config);
}

CooperativeSpaceTimeDecision optimizeCooperativeSpaceTime(
    const CooperativeFlightIntentData& ownship,
    const std::span<const CooperativeConflictPeer> conflicting_peers,
    const std::int64_t now_ns, const CooperativeSpaceTimeConfig& config,
    const std::optional<CooperativeManeuver> incumbent) {
  CooperativeSpaceTimeDecision result;
  if (now_ns <= 0 || ownship.vehicle_id.empty() || ownship.trajectory.empty() ||
      conflicting_peers.empty() || !validConfig(config)) {
    return result;
  }

  const Vec3 forward = routeForward(ownship, now_ns);
  const Vec3 left = horizontalLeft(forward);
  std::vector<const CooperativeConflictPeer*> ordered_peers;
  ordered_peers.reserve(conflicting_peers.size());
  for (const CooperativeConflictPeer& peer : conflicting_peers) {
    ordered_peers.push_back(&peer);
  }
  std::ranges::sort(ordered_peers, {}, [](const CooperativeConflictPeer* peer) {
    return peer->intent.vehicle_id;
  });
  double baseline_minimum_m = std::numeric_limits<double>::infinity();
  double maximum_closing_speed_mps = 0.0;
  std::vector<CooperativePairManeuverPreference> pair_preferences;
  pair_preferences.reserve(conflicting_peers.size());
  for (const CooperativeConflictPeer* peer : ordered_peers) {
    if (peer == nullptr || peer->intent.trajectory.empty() || !peer->prediction.valid) {
      continue;
    }
    baseline_minimum_m =
        std::min(baseline_minimum_m, peer->prediction.minimum_separation_m);
    maximum_closing_speed_mps =
        std::max(maximum_closing_speed_mps, peer->prediction.current_closing_speed_mps);
    pair_preferences.push_back(preferredCooperativePairManeuver(ownship, peer->intent));
  }
  if (pair_preferences.empty() || !std::isfinite(baseline_minimum_m)) {
    return result;
  }

  const double spatial_offset_m =
      std::clamp(config.desired_minimum_separation_m - baseline_minimum_m +
                     config.spatial_margin_m,
                 config.minimum_spatial_offset_m, config.maximum_spatial_offset_m);
  const double time_shift_s =
      std::clamp((config.desired_minimum_separation_m - baseline_minimum_m +
                  config.spatial_margin_m) /
                     std::max(1.0, maximum_closing_speed_mps),
                 config.minimum_time_shift_s, config.maximum_time_shift_s);

  std::vector<Candidate> candidates;
  candidates.reserve(8U + pair_preferences.size());
  appendCandidate(candidates, Candidate{});
  for (const CooperativePairManeuverPreference& preference : pair_preferences) {
    appendCandidate(candidates,
                    Candidate{.maneuver = preference.maneuver,
                              .direction = preference.acceleration_direction,
                              .spatial_offset_m = spatial_offset_m});
  }
  appendCandidate(candidates, Candidate{.maneuver = CooperativeManeuver::kClimb,
                                        .direction = Vec3{0.0, 0.0, 1.0},
                                        .spatial_offset_m = spatial_offset_m});
  appendCandidate(candidates, Candidate{.maneuver = CooperativeManeuver::kDescend,
                                        .direction = Vec3{0.0, 0.0, -1.0},
                                        .spatial_offset_m = spatial_offset_m});
  appendCandidate(candidates, Candidate{.maneuver = CooperativeManeuver::kLeft,
                                        .direction = left,
                                        .spatial_offset_m = spatial_offset_m});
  appendCandidate(candidates, Candidate{.maneuver = CooperativeManeuver::kRight,
                                        .direction = scaled(left, -1.0),
                                        .spatial_offset_m = spatial_offset_m});
  if (!ordered_peers.empty() &&
      ownship.vehicle_id > ordered_peers.front()->intent.vehicle_id) {
    appendCandidate(candidates, Candidate{.maneuver = CooperativeManeuver::kSlow,
                                          .direction = scaled(forward, -1.0),
                                          .time_shift_s = time_shift_s});
  }

  for (Candidate& candidate : candidates) {
    if (candidate.maneuver == CooperativeManeuver::kKeep ||
        candidate.maneuver == CooperativeManeuver::kSlow) {
      continue;
    }
    for (const CooperativePairManeuverPreference& preference : pair_preferences) {
      candidate.pair_preference_alignment +=
          dot(candidate.direction, normalized(preference.acceleration_direction));
    }
    candidate.pair_preference_alignment /= static_cast<double>(pair_preferences.size());
  }
  std::ranges::sort(candidates, candidateOrder);

  std::vector<CandidateEvaluation> evaluations;
  evaluations.reserve(candidates.size());
  for (const Candidate& candidate : candidates) {
    CandidateEvaluation evaluation =
        evaluateCandidate(candidate, ownship, ordered_peers, now_ns, config, forward);
    if (evaluation.valid) {
      evaluations.push_back(evaluation);
    }
  }
  if (evaluations.empty()) {
    return result;
  }
  const double nominal_speed_mps = norm(ownship.current_velocity);
  auto best = evaluations.begin();
  for (auto candidate = std::next(evaluations.begin()); candidate != evaluations.end();
       ++candidate) {
    if (betterCandidate(*candidate, *best, nominal_speed_mps)) {
      best = candidate;
    }
  }
  if (incumbent.has_value() && best->candidate.maneuver != *incumbent) {
    auto incumbent_evaluation = evaluations.end();
    for (auto candidate = evaluations.begin(); candidate != evaluations.end();
         ++candidate) {
      if (candidate->candidate.maneuver == *incumbent &&
          (incumbent_evaluation == evaluations.end() ||
           betterCandidate(*candidate, *incumbent_evaluation, nominal_speed_mps))) {
        incumbent_evaluation = candidate;
      }
    }
    if (incumbent_evaluation != evaluations.end() &&
        incumbent_evaluation->separation_satisfied == best->separation_satisfied) {
      const bool retain_safe = incumbent_evaluation->separation_satisfied;
      const bool retain_degraded =
          incumbent_evaluation->minimum_separation_m + config.incumbent_hysteresis_m >=
              best->minimum_separation_m &&
          incumbent_evaluation->integrated_shortfall_m2_s <=
              1.1 * best->integrated_shortfall_m2_s + 1.0e-6;
      if (retain_safe || retain_degraded) {
        best = incumbent_evaluation;
      }
    }
  }

  result.valid = true;
  result.active = best->candidate.maneuver != CooperativeManeuver::kKeep;
  result.changed = !incumbent.has_value() || best->candidate.maneuver != *incumbent;
  result.maneuver = best->candidate.maneuver;
  result.preferred_acceleration_direction = best->candidate.direction;
  result.lateral_offset_m =
      dot(best->candidate.direction, left) * best->candidate.spatial_offset_m;
  result.vertical_offset_m =
      best->candidate.direction.z * best->candidate.spatial_offset_m;
  result.time_shift_s = best->candidate.time_shift_s;
  result.predicted_minimum_separation_m = best->minimum_separation_m;
  result.time_to_minimum_s = best->time_to_minimum_s;
  result.integrated_separation_shortfall_m2_s = best->integrated_shortfall_m2_s;
  result.evaluated_candidate_count = evaluations.size();
  return result;
}

} // namespace drone_city_nav
