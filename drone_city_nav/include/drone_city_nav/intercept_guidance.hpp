#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/intercept_mission.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

enum class InterceptGuidanceMode : std::uint8_t {
  kDirect,
  kAnalyticIntercept,
  kAheadIntercept,
};

struct InterceptGuidanceConfig {
  double interceptor_speed_mps{20.0};
  double minimum_prediction_horizon_s{0.0};
  double maximum_prediction_horizon_s{15.0};
  double ahead_maximum_prediction_horizon_s{1.0};
  double fallback_prediction_horizon_s{1.0};
  double minimum_target_speed_mps{0.5};
  double ahead_enter_m{5.0};
  double ahead_exit_m{0.0};
  double ahead_corridor_enter_m{15.0};
  double ahead_corridor_exit_m{20.0};
  double horizon_smoothing_time_constant_s{0.5};
  double prediction_heading_offset_rad{0.0};
  double hypothesis_zero_distance_m{30.0};
  double hypothesis_full_distance_m{120.0};
  double maximum_hypothesis_lateral_offset_m{70.0};
  double target_vertical_deceleration_mps2{4.0};
  FlightEnvelopeConfig target_flight_envelope{};
};

struct TargetVerticalPrediction {
  double z_m{0.0};
  double velocity_mps{0.0};
  bool envelope_limited{false};
  bool valid{false};
};

struct InterceptGuidanceResult {
  Point3 observed_position{};
  Point3 predicted_position{};
  Vec3 observed_velocity{};
  InterceptGuidanceMode mode{InterceptGuidanceMode::kDirect};
  std::int64_t observation_stamp_ns{0};
  double prediction_age_s{0.0};
  double prediction_horizon_s{0.0};
  double analytic_intercept_time_s{0.0};
  double target_speed_mps{0.0};
  double ahead_m{0.0};
  double cross_track_m{0.0};
  double configured_heading_offset_rad{0.0};
  double effective_heading_offset_rad{0.0};
  double hypothesis_lateral_offset_m{0.0};
  bool valid{false};
  bool vertical_prediction_limited{false};
};

[[nodiscard]] TargetVerticalPrediction
predictTargetVerticalMotion(double initial_z_m, double initial_velocity_mps,
                            double elapsed_s, double deceleration_mps2,
                            const FlightEnvelopeConfig& flight_envelope) noexcept;

class InterceptGuidance final {
public:
  explicit InterceptGuidance(const InterceptGuidanceConfig& config = {});

  [[nodiscard]] InterceptGuidanceResult update(const TimedVehicleState& interceptor,
                                               const TimedVehicleState& target,
                                               std::int64_t now_ns);

private:
  void resetPredictionState() noexcept;

  InterceptGuidanceConfig config_{};
  std::optional<double> smoothed_prediction_horizon_s_;
  std::optional<std::int64_t> previous_update_stamp_ns_;
  bool ahead_mode_{false};
};

[[nodiscard]] const char*
interceptGuidanceModeName(InterceptGuidanceMode mode) noexcept;

} // namespace drone_city_nav
