#pragma once

#include "drone_city_nav/intercept_mission.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

enum class InterceptGuidanceMode : std::uint8_t {
  kDirect,
  kFarLead,
  kAheadLead,
};

struct InterceptGuidanceConfig {
  double far_prediction_horizon_s{3.0};
  double ahead_prediction_horizon_s{1.0};
  double minimum_target_speed_mps{0.5};
  double ahead_enter_m{5.0};
  double ahead_exit_m{0.0};
  double ahead_corridor_enter_m{15.0};
  double ahead_corridor_exit_m{20.0};
  double horizon_smoothing_time_constant_s{0.5};
};

struct InterceptGuidanceResult {
  Point3 observed_position{};
  Point3 predicted_position{};
  Vec3 observed_velocity{};
  InterceptGuidanceMode mode{InterceptGuidanceMode::kDirect};
  std::int64_t observation_stamp_ns{0};
  double prediction_age_s{0.0};
  double prediction_horizon_s{0.0};
  double target_speed_mps{0.0};
  double ahead_m{0.0};
  double cross_track_m{0.0};
  bool valid{false};
};

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
