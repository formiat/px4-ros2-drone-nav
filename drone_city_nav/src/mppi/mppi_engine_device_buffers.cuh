// Textual include for mppi_engine.cu only: the persistent device buffer set and
// the evaluated reference-sequence record live in the engine's anonymous
// namespace together with the kernels.
#pragma once

struct DeviceBuffers {
  DeviceBuffer<float> noise_ax;
  DeviceBuffer<float> noise_ay;
  DeviceBuffer<float> noise_az;
  DeviceBuffer<float> noise_yaw;
  DeviceBuffer<float> soft_cost;
  DeviceBuffer<float> critical_exposure;
  DeviceBuffer<float> planning_exposure;
  DeviceBuffer<float> minimum_clearance;
  DeviceBuffer<std::uint8_t> altitude_envelope_violation;
  DeviceBuffer<std::uint8_t> collision_violation;
  DeviceBuffer<std::uint8_t> worst_tier;
  DeviceBuffer<float> weights;
  DeviceBuffer<Control> nominal;
  DeviceBuffer<Control> updated;
  DeviceBuffer<Control> control_update_partials;
  DeviceBuffer<Control> best_feasible;
  DeviceBuffer<Control> repair_candidates;
  DeviceBuffer<int> best_rollout;
  DeviceBuffer<float> minimum_soft;
  DeviceBuffer<float> weight_sum;
  DeviceBuffer<float> feasible_cost_sum;
  DeviceBuffer<unsigned int> feasible_count;
  DeviceBuffer<float> effective_temperature;
  DeviceBuffer<RouteSample3D> route_points{kMaximumDeviceRoutePoints};
  DeviceBuffer<DynamicAircraftSample> dynamic_aircraft_samples;
  DeviceBuffer<float> dynamic_aircraft_radii{kMaximumDynamicAircraft};
  DeviceBuffer<std::uint32_t> dynamic_aircraft_active_steps{kMaximumDynamicAircraft};

  DeviceBuffers(const std::size_t rollouts, const std::size_t steps)
      : noise_ax{rollouts * steps},
        noise_ay{rollouts * steps},
        noise_az{rollouts * steps},
        noise_yaw{rollouts * steps},
        soft_cost{rollouts},
        critical_exposure{rollouts},
        planning_exposure{rollouts},
        minimum_clearance{rollouts},
        altitude_envelope_violation{rollouts},
        collision_violation{rollouts},
        worst_tier{rollouts},
        weights{rollouts},
        nominal{steps},
        updated{steps},
        control_update_partials{kControlUpdatePartitions * steps},
        best_feasible{steps},
        repair_candidates{kMaximumRepairCandidateCount * steps},
        best_rollout{1U},
        minimum_soft{1U},
        weight_sum{1U},
        feasible_cost_sum{1U},
        feasible_count{1U},
        effective_temperature{1U},
        dynamic_aircraft_samples{kMaximumDynamicAircraft * steps} {
  }

  [[nodiscard]] std::size_t bytes() const noexcept {
    return noise_ax.bytes() + noise_ay.bytes() + noise_az.bytes() + noise_yaw.bytes() +
           soft_cost.bytes() + critical_exposure.bytes() + planning_exposure.bytes() +
           minimum_clearance.bytes() + altitude_envelope_violation.bytes() +
           collision_violation.bytes() + worst_tier.bytes() + weights.bytes() +
           nominal.bytes() + updated.bytes() + control_update_partials.bytes() +
           best_feasible.bytes() + repair_candidates.bytes() + best_rollout.bytes() +
           minimum_soft.bytes() + weight_sum.bytes() + feasible_cost_sum.bytes() +
           feasible_count.bytes() + effective_temperature.bytes() +
           route_points.bytes() + dynamic_aircraft_samples.bytes() +
           dynamic_aircraft_radii.bytes() + dynamic_aircraft_active_steps.bytes();
  }
};

struct EvaluatedControlSequence {
  ReferenceSimulationTrace trace;
  RolloutMetrics metrics{};
  MppiPostUpdateClassificationResult classification{};
  bool route_terminal_cross_track_violation{false};
  float terminal_route_cross_track_m{-1.0F};
  std::size_t route_terminal_arrival_shaping_attempts{0U};
  std::size_t route_terminal_nominal_prefix_control_count{0U};
};
