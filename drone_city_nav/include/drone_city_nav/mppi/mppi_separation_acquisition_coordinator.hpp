#pragma once

#include "drone_city_nav/mppi/mppi_separation_acquisition.hpp"

#include <optional>
#include <span>

namespace drone_city_nav::mppi {

struct SeparationAcquisitionCoordinatorInput {
  bool cooperative_avoidance_active{false};
  std::optional<CooperativeSeparationAcquisition> cooperative_acquisition;
  State initial_state{};
  State target{};
  std::span<const RouteSample3D> route;
  float initial_route_station_m{0.0F};
  float reference_speed_mps{0.0F};
  Control previous_applied_control{};
  float first_control_interval_s{0.0F};
  EsdfGrid grid{};
  std::span<const float> esdf;
  std::span<const DynamicAircraftTrajectory> aircraft;
  BenchmarkConfig config{};
};

struct SeparationAcquisitionCoordinatorResult {
  std::optional<std::vector<Control>> nominal_reseed;
  CooperativeSeparationAcquisitionResult cooperative{};
  bool cooperative_acquisition_reseeded{false};
  bool cooperative_release_reseeded{false};
};

class SeparationAcquisitionCoordinator {
public:
  [[nodiscard]] SeparationAcquisitionCoordinatorResult
  update(const SeparationAcquisitionCoordinatorInput& input);

private:
  CooperativeSeparationAcquisitionLifecycle cooperative_{};
};

} // namespace drone_city_nav::mppi
