#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct InterceptMissionConfig {
  double capture_radius_m{5.0};
  double evader_goal_radius_m{2.0};
};

struct TimedVehicleState {
  Point3 position{};
  Vec3 velocity{};
  std::int64_t stamp_ns{0};
  double heading_rad{0.0};
  bool position_valid{false};
  bool velocity_valid{false};
  bool heading_valid{false};
  bool armed{false};
  bool airborne{false};
  bool navigation_ready{false};
};

enum class InterceptMissionOutcome : std::uint8_t {
  kRunning,
  kIntercepted,
  kEvaderReachedGoal,
};

struct InterceptMissionUpdate {
  InterceptMissionOutcome outcome{InterceptMissionOutcome::kRunning};
  bool newly_terminal{false};
  bool capture_detected{false};
  bool newly_captured{false};
  double separation_m{0.0};
};

struct InterceptMissionReadiness {
  bool interceptor_navigation_ready{false};
  bool evader_navigation_ready{false};
  bool interceptor_world_ready{false};
  bool evader_world_ready{false};
  bool target_track_ready{false};
};

[[nodiscard]] bool
interceptMissionReady(const InterceptMissionReadiness& readiness) noexcept;

struct InterceptStateAdjudicationConfig {
  double maximum_state_age_s{1.0};
  double maximum_degraded_duration_s{5.0};
};

enum class InterceptStateAdjudicationStatus : std::uint8_t {
  kHealthy,
  kDegraded,
  kProlongedFailure,
};

struct InterceptStateAdjudicationUpdate {
  InterceptStateAdjudicationStatus status{InterceptStateAdjudicationStatus::kHealthy};
  bool interceptor_fresh{false};
  bool evader_fresh{false};
  bool newly_degraded{false};
  bool newly_recovered{false};
  bool newly_prolonged_failure{false};
  double interceptor_age_s{0.0};
  double evader_age_s{0.0};
  double degraded_duration_s{0.0};
};

class InterceptStateAdjudicationLifecycle final {
public:
  explicit InterceptStateAdjudicationLifecycle(
      const InterceptStateAdjudicationConfig& config = {});

  [[nodiscard]] InterceptStateAdjudicationUpdate
  update(std::int64_t now_ns, const TimedVehicleState& interceptor,
         const TimedVehicleState& evader) noexcept;

private:
  [[nodiscard]] bool stateFresh(std::int64_t now_ns,
                                const TimedVehicleState& state) const noexcept;
  [[nodiscard]] static double stateAgeSeconds(std::int64_t now_ns,
                                              const TimedVehicleState& state) noexcept;

  std::int64_t maximum_state_age_ns_{0};
  std::int64_t maximum_degraded_duration_ns_{0};
  std::optional<std::int64_t> degraded_since_ns_;
  bool prolonged_failure_reported_{false};
};

struct InterceptorHoldConfig {
  double position_tolerance_m{2.0};
  double maximum_speed_mps{0.8};
  double confirmation_duration_s{1.0};
};

struct InterceptorHoldUpdate {
  bool confirmed{false};
  bool newly_confirmed{false};
  double position_error_m{0.0};
  double speed_mps{0.0};
};

class InterceptorHoldConfirmation final {
public:
  InterceptorHoldConfirmation(const Point3& hold_position,
                              const InterceptorHoldConfig& config = {});

  [[nodiscard]] InterceptorHoldUpdate update(const TimedVehicleState& interceptor);

private:
  Point3 hold_position_{};
  InterceptorHoldConfig config_{};
  std::optional<std::int64_t> stable_since_ns_;
  bool confirmed_{false};
};

class InterceptMissionEvaluator final {
public:
  InterceptMissionEvaluator(const Point3& evader_goal,
                            const InterceptMissionConfig& config = {});

  [[nodiscard]] InterceptMissionUpdate update(const TimedVehicleState& interceptor,
                                              const TimedVehicleState& evader);
  void resetTemporalContinuity() noexcept;

  [[nodiscard]] InterceptMissionOutcome outcome() const noexcept {
    return outcome_;
  }

private:
  [[nodiscard]] double sweptSeparation(const TimedVehicleState& interceptor,
                                       const TimedVehicleState& evader) const noexcept;

  Point3 evader_goal_{};
  InterceptMissionConfig config_{};
  std::optional<TimedVehicleState> previous_interceptor_;
  std::optional<TimedVehicleState> previous_evader_;
  InterceptMissionOutcome outcome_{InterceptMissionOutcome::kRunning};
  bool capture_detected_{false};
};

[[nodiscard]] const char*
interceptMissionOutcomeName(InterceptMissionOutcome outcome) noexcept;

} // namespace drone_city_nav
