#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_city_nav {

enum class NonCooperativeThreatReason : std::uint8_t {
  kNone,
  kNearby,
  kConverging,
  kStrongCurrentSeparation,
  kStrongPredictedClosestApproach,
};

enum class NonCooperativeAvoidanceLifecycleState : std::uint8_t {
  kInactive,
  kEntered,
  kHolding,
  kReleased,
};

[[nodiscard]] const char*
nonCooperativeThreatReasonName(NonCooperativeThreatReason reason) noexcept;

[[nodiscard]] const char* nonCooperativeAvoidanceLifecycleStateName(
    NonCooperativeAvoidanceLifecycleState state) noexcept;

struct NonCooperativeAircraftTrack {
  std::uint64_t local_track_id{0U};
  Point3 position{};
  Vec3 velocity{};
  std::int64_t measurement_stamp_ns{0};
  bool position_valid{false};
  bool velocity_valid{false};
};

struct NonCooperativeAvoidanceConfig {
  double prediction_horizon_s{4.0};
  double strong_separation_m{10.0};
  double anticipation_separation_m{20.0};
  double release_separation_m{15.0};
  double release_hysteresis_s{1.0};
  double maximum_track_age_s{0.75};
  double tracked_aircraft_radius_m{0.82};
  double minimum_relative_speed_mps{0.05};
  double candidate_acceleration_fraction{0.95};
  double candidate_duration_s{1.5};
  double strong_cost_weight{4000.0};
  double anticipation_cost_weight{40.0};
  double time_to_collision_gain_s{1.0};
  double maximum_time_to_collision_multiplier{4.0};
};

struct NonCooperativeClosestApproach {
  std::uint64_t local_track_id{0U};
  double track_age_s{0.0};
  double current_range_m{0.0};
  double closing_speed_mps{0.0};
  double time_to_closest_approach_s{0.0};
  double closest_approach_distance_m{0.0};
  Point3 current_position{};
  Vec3 estimated_velocity{};
  bool velocity_valid{false};
  NonCooperativeThreatReason reason{NonCooperativeThreatReason::kNone};
};

struct NonCooperativeAvoidanceInput {
  mppi::State ownship{};
  std::vector<NonCooperativeAircraftTrack> tracks;
  std::int64_t now_ns{0};
  std::size_t horizon_steps{0U};
  double step_s{0.0};
};

struct NonCooperativeAvoidanceUpdate {
  std::vector<mppi::DynamicAircraftTrajectory> trajectories;
  std::optional<NonCooperativeClosestApproach> primary_threat;
  std::optional<mppi::NonCooperativeSeparationAcquisition> acquisition;
  mppi::DynamicAircraftCostPolicy cost_policy{};
  NonCooperativeAvoidanceLifecycleState lifecycle_state{
      NonCooperativeAvoidanceLifecycleState::kInactive};
  std::size_t received_track_count{0U};
  std::size_t fresh_track_count{0U};
  double maximum_radar_age_s{-1.0};
  std::uint64_t lifecycle_generation{0U};
  bool active{false};
};

class NonCooperativeCollisionAvoidance {
public:
  explicit NonCooperativeCollisionAvoidance(
      const NonCooperativeAvoidanceConfig& config);

  [[nodiscard]] NonCooperativeAvoidanceUpdate
  update(const NonCooperativeAvoidanceInput& input);

private:
  NonCooperativeAvoidanceConfig config_{};
  std::int64_t clear_since_ns_{0};
  std::uint64_t lifecycle_generation_{0U};
  bool active_{false};
};

} // namespace drone_city_nav
