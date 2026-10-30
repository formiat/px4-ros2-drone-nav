#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_post_update_classification.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <vector>

// Host-side evaluation of one control sequence and construction of the
// post-update repair candidates. The CUDA engine ranks the population on the
// device; what it selected is re-simulated here by the reference
// implementation, classified, and repaired from these candidates when it is
// not executable.

namespace drone_city_nav::mppi::detail {

struct EvaluatedControlSequence {
  ReferenceSimulationTrace trace;
  RolloutMetrics metrics{};
  MppiPostUpdateClassificationResult classification{};
  bool route_terminal_cross_track_violation{false};
  float terminal_route_cross_track_m{-1.0F};
  std::size_t route_terminal_arrival_shaping_attempts{0U};
  std::size_t route_terminal_nominal_prefix_control_count{0U};
};

// Everything one tick's evaluation reads besides the sequence itself.
struct ControlSequenceEvaluationContext {
  const MppiTickInput& input;
  const BenchmarkConfig& config;
  const EsdfGrid& grid;
  std::span<const float> esdf;
  std::span<const Control> zero_noise;
  // The route the tick follows, empty when none is active.
  std::span<const RouteSample3D> active_route;
  Control previous_applied_control{};
  DynamicAircraftCostPolicy dynamic_aircraft_cost_policy{};
  const MppiFeasibilityContract& feasibility_contract;
};

[[nodiscard]] EvaluatedControlSequence
evaluateControlSequence(const ControlSequenceEvaluationContext& context,
                        std::span<const Control> controls);

// Blends of the limited nominal sequence toward the weighted update, then the
// best feasible rollout, then the deterministic candidate when one was
// injected: the order the repair tries them in.
inline constexpr std::array<float, 5U> kRepairBacktrackRatios{0.5F, 0.25F, 0.125F,
                                                              0.0625F, 0.0F};
inline constexpr std::size_t kMaximumRepairCandidateCount{
    kRepairBacktrackRatios.size() + 2U};

struct RepairCandidateInput {
  std::span<const Control> nominal;
  std::span<const Control> updated;
  std::span<const Control> best_feasible;
  // Absent when no deterministic candidate was injected this tick.
  std::span<const Control> deterministic_candidate;
  const DynamicsConfig& dynamics;
  Control previous_applied_control{};
  float first_control_interval_s{0.0F};
};

// Fills `candidates`, laid out as consecutive sequences of `steps` controls,
// and returns how many it wrote.
[[nodiscard]] std::size_t
buildRepairCandidateSequences(const RepairCandidateInput& input, std::size_t steps,
                              std::span<Control> candidates);

// The gaze looks where the horizon is 1.5 s on, about the time a quarter turn
// of the heading took under the original yaw limits (1.5 rad/s, 2 rad/s^2;
// the configured ones are in urban_mvp.yaml), and holds the heading where the
// horizon moves less than the body's radius in that time.
inline constexpr float kGazeLookaheadS{1.5F};
inline constexpr float kGazeMinimumDisplacementM{0.55F};

struct ResultReconstructionInput {
  const MppiTickInput& input;
  const BenchmarkConfig& config;
  // The selected sequence; the gaze writes its yaw controls in place.
  std::span<Control> controls;
  Control previous_applied_control{};
  float first_control_interval_s{0.0F};
};

// The part of a tick every backend shares once its sequence is selected and
// evaluated: the horizon, the gaze, the exposures, the first control's delta,
// the progress along the route or toward the target, the reconstruction time.
void fillResultFromEvaluation(MppiTickResult& result,
                              EvaluatedControlSequence&& selected_evaluation,
                              const ResultReconstructionInput& in);

} // namespace drone_city_nav::mppi::detail
