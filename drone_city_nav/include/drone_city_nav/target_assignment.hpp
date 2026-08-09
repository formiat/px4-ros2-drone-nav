#pragma once

#include "drone_city_nav/intercept_mission.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace drone_city_nav {

struct TargetAssignmentTrack {
  TimedVehicleState state{};
  std::uint64_t detection_id{0U};
  std::uint64_t track_id{0U};
};

struct TargetAssignmentAgent {
  std::string interceptor_id;
  TimedVehicleState ownship{};
  std::vector<TargetAssignmentTrack> tracks;
};

struct TargetAssignmentDecision {
  std::string interceptor_id;
  std::uint64_t detection_id{0U};
  std::uint64_t track_id{0U};
  double estimated_intercept_time_s{0.0};
};

enum class TargetAssignmentReason : std::uint8_t {
  kInitial,
  kTargetSetChanged,
  kCostImprovement,
  kRefresh,
};

struct TargetAssignmentConfig {
  double interceptor_speed_mps{20.0};
  double maximum_track_age_s{3.5};
  double switch_penalty_s{2.0};
  double minimum_switch_improvement_s{1.0};
  double minimum_switch_improvement_ratio{0.10};
  double minimum_assignment_hold_s{1.0};
  double switch_confirmation_s{0.5};
  double no_intercept_solution_penalty_s{30.0};
};

struct TargetAssignmentUpdate {
  std::vector<TargetAssignmentDecision> decisions;
  TargetAssignmentReason reason{TargetAssignmentReason::kRefresh};
  std::uint64_t generation{0U};
  bool changed{false};
};

class AdaptiveTargetAssignment final {
public:
  explicit AdaptiveTargetAssignment(const TargetAssignmentConfig& config = {});

  [[nodiscard]] TargetAssignmentUpdate
  update(std::int64_t now_ns, const std::vector<TargetAssignmentAgent>& agents);

  void reset() noexcept;

private:
  struct AssignmentState {
    TargetAssignmentDecision decision;
    std::int64_t assigned_since_ns{0};
  };

  TargetAssignmentConfig config_{};
  std::vector<AssignmentState> assignments_;
  std::vector<TargetAssignmentDecision> pending_decisions_;
  std::int64_t pending_since_ns_{0};
  std::uint64_t generation_{0U};
};

[[nodiscard]] const char*
targetAssignmentReasonName(TargetAssignmentReason reason) noexcept;

} // namespace drone_city_nav
