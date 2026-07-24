#pragma once

#include <cstdint>

namespace drone_city_nav {

enum class TrajectoryEndpointSemantics : std::uint8_t {
  kMissionGoal = 0U,
  kLocalHorizon = 1U,
  kTemporaryReplanHold = 2U,
};

struct LocalHorizonExecutionConfig {
  double minimum_buffer_m{3.0};
  double successor_timeout_s{0.5};
};

struct LocalHorizonExecutionInput {
  TrajectoryEndpointSemantics semantics{TrajectoryEndpointSemantics::kMissionGoal};
  double remaining_s_m{0.0};
  double low_buffer_duration_s{0.0};
  bool endpoint_captured{false};
  bool successor_received{false};
};

struct LocalHorizonExecutionDecision {
  bool terminal_capture_enabled{false};
  bool latch_temporary_hold{false};
  bool clear_temporary_state{false};
  bool mission_goal_eligible{false};
};

[[nodiscard]] LocalHorizonExecutionDecision
evaluateLocalHorizonExecution(const LocalHorizonExecutionInput& input,
                              const LocalHorizonExecutionConfig& config = {});

[[nodiscard]] const char*
trajectoryEndpointSemanticsName(TrajectoryEndpointSemantics semantics) noexcept;

} // namespace drone_city_nav
