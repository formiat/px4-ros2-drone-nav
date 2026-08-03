#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct InterceptMissionConfig {
  double capture_radius_m{5.0};
  double evader_goal_radius_m{2.0};
  double evader_goal_stop_speed_mps{0.8};
  double evader_goal_hold_s{2.0};
};

struct TimedVehicleState {
  Point3 position{};
  Vec3 velocity{};
  std::int64_t stamp_ns{0};
  bool position_valid{false};
  bool velocity_valid{false};
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
  double separation_m{0.0};
};

class InterceptMissionEvaluator final {
public:
  InterceptMissionEvaluator(const Point3& evader_goal,
                            const InterceptMissionConfig& config = {});

  [[nodiscard]] InterceptMissionUpdate update(const TimedVehicleState& interceptor,
                                              const TimedVehicleState& evader);

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
  std::optional<std::int64_t> evader_goal_hold_started_ns_;
  InterceptMissionOutcome outcome_{InterceptMissionOutcome::kRunning};
};

[[nodiscard]] const char*
interceptMissionOutcomeName(InterceptMissionOutcome outcome) noexcept;

} // namespace drone_city_nav
