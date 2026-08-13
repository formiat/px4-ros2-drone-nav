#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

struct CooperativeSeparationAcquisitionEvaluationInput {
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
  std::span<const CooperativePeerTrajectory> peers;
  CooperativeSeparationAcquisition acquisition{};
  BenchmarkConfig config{};
};

struct CooperativeSeparationAcquisitionResult {
  std::vector<Control> controls;
  std::size_t candidate_index{0U};
  float head_progress_m{0.0F};
  float terminal_progress_m{0.0F};
  float separation_gain_m{0.0F};
  bool available{false};
  bool positive_progress{false};
  bool backward_fallback{false};
};

[[nodiscard]] CooperativeSeparationAcquisitionResult
evaluateCooperativeSeparationAcquisition(
    const CooperativeSeparationAcquisitionEvaluationInput& input);

struct CooperativeSeparationAcquisitionLifecycleInput {
  bool avoidance_active{false};
  std::optional<CooperativeSeparationAcquisition> acquisition;
  CooperativeSeparationAcquisitionEvaluationInput evaluation{};
};

struct CooperativeSeparationAcquisitionLifecycleResult {
  std::optional<std::vector<Control>> nominal_reseed;
  CooperativeSeparationAcquisitionResult acquisition{};
  bool acquisition_reseeded{false};
  bool release_reseeded{false};
};

class CooperativeSeparationAcquisitionLifecycle {
public:
  [[nodiscard]] CooperativeSeparationAcquisitionLifecycleResult
  update(const CooperativeSeparationAcquisitionLifecycleInput& input);

private:
  bool avoidance_active_{false};
  bool acquisition_pending_{false};
  bool acquisition_applied_{false};
};

} // namespace drone_city_nav::mppi
