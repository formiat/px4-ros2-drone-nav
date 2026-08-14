#pragma once

#include "drone_city_nav/mppi/mppi_noncooperative_acquisition.hpp"
#include "drone_city_nav/mppi/mppi_separation_acquisition.hpp"

#include <optional>
#include <span>

namespace drone_city_nav::mppi {

struct SeparationAcquisitionCoordinatorInput {
  bool cooperative_avoidance_active{false};
  bool noncooperative_avoidance_active{false};
  std::optional<CooperativeSeparationAcquisition> cooperative_acquisition;
  std::optional<NonCooperativeSeparationAcquisition> noncooperative_acquisition;
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
  DynamicAircraftCostPolicy dynamic_aircraft_cost_policy{};
  BenchmarkConfig config{};
};

struct SeparationAcquisitionCoordinatorResult {
  std::optional<std::vector<Control>> nominal_reseed;
  CooperativeSeparationAcquisitionResult cooperative{};
  NonCooperativeAcquisitionResult noncooperative{};
  bool cooperative_acquisition_reseeded{false};
  bool cooperative_release_reseeded{false};
  bool noncooperative_acquisition_reseeded{false};
  bool noncooperative_release_reseeded{false};
};

class SeparationAcquisitionCoordinator {
public:
  [[nodiscard]] SeparationAcquisitionCoordinatorResult
  update(const SeparationAcquisitionCoordinatorInput& input);

private:
  CooperativeSeparationAcquisitionLifecycle cooperative_{};
  NonCooperativeAcquisitionLifecycle noncooperative_{};
};

} // namespace drone_city_nav::mppi
