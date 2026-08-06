#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

struct MppiHorizonSafetyConfig {
  double reaction_latency_s{0.10};
  double maximum_braking_acceleration_mps2{8.0};
  double minimum_time_to_collision_s{0.50};
  double fallback_duration_s{2.0};
  double dt_s{0.05};
  double swept_validation_step_m{0.25};
  double physical_footprint_radius_m{0.82};
  double physical_footprint_lower_extent_m{0.23};
  double physical_footprint_upper_extent_m{0.35};
  std::size_t physical_footprint_samples{12U};
  std::size_t physical_footprint_radial_rings{2U};
  std::size_t physical_footprint_axial_samples{3U};
  double position_hold_capture_speed_mps{0.20};
  FlightEnvelopeConfig flight_envelope{.minimum_target_z_m = -1.0e9,
                                       .maximum_target_z_m = 1.0e9};
};

enum class MppiHorizonSafetyDecision {
  kExecute,
  kExecuteUntilDeadline,
  kBrake,
  kHold,
};

struct MppiHorizonSafetyResult {
  MppiHorizonSafetyDecision decision{MppiHorizonSafetyDecision::kHold};
  double time_to_collision_s{0.0};
  double stopping_time_s{0.0};
  double stopping_distance_m{0.0};
  double latest_safe_intervention_time_s{0.0};
  bool flight_envelope_violation{false};
  bool global_raw_collision{false};
  std::size_t global_raw_fallback_samples{0U};
  std::vector<mppi::State> fallback_horizon;
  std::vector<mppi::Control> fallback_controls;
};

struct MppiSafetyInterventionUpdate {
  MppiHorizonSafetyDecision decision{MppiHorizonSafetyDecision::kHold};
  std::optional<std::int64_t> deadline_ns;
};

class MppiSafetyInterventionTracker {
public:
  [[nodiscard]] MppiSafetyInterventionUpdate
  update(std::int64_t now_ns, const MppiHorizonSafetyResult& result) noexcept;
  void reset() noexcept;

private:
  std::optional<std::int64_t> deadline_ns_;
};

struct MppiBrakeHoldUpdate {
  bool position_hold{false};
  mppi::State hold_state{};
};

class MppiBrakeHoldLifecycle {
public:
  [[nodiscard]] MppiBrakeHoldUpdate
  update(bool braking_required, const mppi::State& current_state,
         double capture_speed_mps,
         const FlightEnvelopeConfig& flight_envelope) noexcept;
  void reset() noexcept;

private:
  std::optional<mppi::State> hold_state_;
};

[[nodiscard]] MppiHorizonSafetyResult
buildMppiBrakingFallback(const mppi::State& current_state,
                         const MppiHorizonSafetyConfig& config);

[[nodiscard]] MppiHorizonSafetyResult evaluateMppiHorizonSafety(
    const mppi::State& current_state, std::span<const mppi::State> horizon,
    std::span<const float> esdf_m, const mppi::EsdfGrid& grid,
    const MppiHorizonSafetyConfig& config, bool engine_collision = false,
    std::span<const mppi::KnownSolid> known_solids = {},
    const OccupancyGrid3D* global_raw_occupancy = nullptr);

} // namespace drone_city_nav
