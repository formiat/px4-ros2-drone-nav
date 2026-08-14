#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

struct NonCooperativeAcquisitionEvaluationInput {
  State initial_state{};
  State target{};
  std::span<const RouteSample3D> route;
  float initial_route_station_m{0.0F};
  float reference_speed_mps{0.0F};
  Control previous_applied_control{};
  float first_control_interval_s{0.0F};
  EsdfGrid grid{};
  std::span<const float> esdf;
  std::span<const KnownSolid> known_solids;
  std::span<const DynamicAircraftTrajectory> aircraft;
  NonCooperativeSeparationAcquisition acquisition{};
  DynamicAircraftCostPolicy cost_policy{};
  BenchmarkConfig config{};
};

struct NonCooperativeAcquisitionResult {
  std::vector<Control> controls;
  std::size_t candidate_index{0U};
  NonCooperativeManeuver maneuver{NonCooperativeManeuver::kRouteCruise};
  float minimum_separation_m{0.0F};
  float separation_gain_m{0.0F};
  float head_progress_m{0.0F};
  float terminal_progress_m{0.0F};
  bool available{false};
};

[[nodiscard]] NonCooperativeAcquisitionResult evaluateNonCooperativeAcquisition(
    const NonCooperativeAcquisitionEvaluationInput& input);

struct NonCooperativeAcquisitionLifecycleInput {
  bool avoidance_active{false};
  std::optional<NonCooperativeSeparationAcquisition> acquisition;
  NonCooperativeAcquisitionEvaluationInput evaluation{};
};

struct NonCooperativeAcquisitionLifecycleResult {
  std::optional<std::vector<Control>> nominal_reseed;
  NonCooperativeAcquisitionResult acquisition{};
  bool acquisition_reseeded{false};
  bool release_reseeded{false};
};

class NonCooperativeAcquisitionLifecycle {
public:
  [[nodiscard]] NonCooperativeAcquisitionLifecycleResult
  update(const NonCooperativeAcquisitionLifecycleInput& input);

private:
  bool avoidance_active_{false};
  bool acquisition_pending_{false};
  bool acquisition_applied_{false};
};

} // namespace drone_city_nav::mppi
