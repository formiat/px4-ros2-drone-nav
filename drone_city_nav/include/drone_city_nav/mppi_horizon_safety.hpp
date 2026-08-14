#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

struct MppiHorizonSafetyConfig {
  double reaction_latency_s{0.10};
  double maximum_braking_acceleration_mps2{8.0};
  double guaranteed_braking_deceleration_mps2{3.0};
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
  std::size_t braking_release_safe_observations{5U};
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
  bool latest_lidar_collision{false};
  bool raw_stopping_path_collision{false};
  bool latest_lidar_stopping_path_collision{false};
  std::size_t global_raw_validation_samples{0U};
  std::size_t global_raw_fallback_samples{0U};
  std::size_t latest_lidar_validation_samples{0U};
  std::size_t latest_lidar_point_checks{0U};
  std::size_t raw_stopping_validation_samples{0U};
  std::size_t raw_stopping_footprint_checks{0U};
  std::size_t latest_lidar_stopping_validation_samples{0U};
  std::size_t latest_lidar_stopping_point_checks{0U};
  double raw_stopping_time_to_collision_s{std::numeric_limits<double>::infinity()};
  double latest_lidar_stopping_time_to_collision_s{
      std::numeric_limits<double>::infinity()};
  std::vector<mppi::State> fallback_horizon;
  std::vector<mppi::Control> fallback_controls;
};

struct MppiSafetyInterventionUpdate {
  MppiHorizonSafetyDecision decision{MppiHorizonSafetyDecision::kHold};
  std::optional<std::int64_t> deadline_ns;
  bool braking_latched{false};
  bool latch_entered{false};
  bool latch_released{false};
  std::size_t safe_release_observations{0U};
};

struct MppiSafetyInterventionObservation {
  std::int64_t now_ns{0};
  MppiHorizonSafetyDecision decision{MppiHorizonSafetyDecision::kHold};
  double latest_safe_intervention_time_s{0.0};
  double current_speed_mps{0.0};
  double release_speed_mps{0.20};
  std::size_t required_safe_release_observations{5U};
  bool persistent_braking_required{false};
};

class MppiSafetyInterventionTracker {
public:
  [[nodiscard]] MppiSafetyInterventionUpdate
  update(const MppiSafetyInterventionObservation& observation) noexcept;
  void reset() noexcept;

private:
  [[nodiscard]] MppiSafetyInterventionUpdate latchBraking(std::int64_t now_ns,
                                                          bool entered) noexcept;

  std::optional<std::int64_t> deadline_ns_;
  bool braking_latched_{false};
  std::size_t safe_release_observations_{0U};
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
    const OccupancyGrid3D* global_raw_occupancy = nullptr,
    const OccupancyGrid2D* latest_raw_occupancy = nullptr,
    std::span<const Point3> latest_lidar_hit_points_map_m = {});

} // namespace drone_city_nav
