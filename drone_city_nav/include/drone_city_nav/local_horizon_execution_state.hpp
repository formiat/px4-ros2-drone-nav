#pragma once

#include <cstdint>
#include <optional>

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

struct MissionGoalSettlementInput {
  bool active_artifact_available{false};
  TrajectoryEndpointSemantics endpoint_semantics{
      TrajectoryEndpointSemantics::kLocalHorizon};
  double current_goal_distance_m{0.0};
  double endpoint_goal_distance_m{0.0};
  double tolerance_m{0.0};
};

[[nodiscard]] LocalHorizonExecutionDecision
evaluateLocalHorizonExecution(const LocalHorizonExecutionInput& input,
                              const LocalHorizonExecutionConfig& config = {});

[[nodiscard]] bool
missionGoalSettlementOwned(const MissionGoalSettlementInput& input) noexcept;

[[nodiscard]] const char*
trajectoryEndpointSemanticsName(TrajectoryEndpointSemantics semantics) noexcept;

[[nodiscard]] std::uint8_t
trajectoryEndpointSemanticsToWire(TrajectoryEndpointSemantics semantics) noexcept;

[[nodiscard]] std::optional<TrajectoryEndpointSemantics>
trajectoryEndpointSemanticsFromWire(std::uint8_t value) noexcept;

} // namespace drone_city_nav
