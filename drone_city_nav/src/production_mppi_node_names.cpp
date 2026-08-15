#include "production_mppi_node.hpp"

namespace drone_city_nav {

const char*
productionMppiPlanningStateName(const ProductionMppiPlanningState state) noexcept {
  switch (state) {
    case ProductionMppiPlanningState::kPlanned:
      return "planned";
    case ProductionMppiPlanningState::kMissionCommandPositionHold:
      return "mission_command_position_hold";
    case ProductionMppiPlanningState::kCooperativePassageYieldHold:
      return "cooperative_passage_yield_hold";
    case ProductionMppiPlanningState::kMissionGoalPositionHold:
      return "mission_goal_position_hold";
    case ProductionMppiPlanningState::kNoExecutableRouteHold:
      return "no_executable_route_hold";
  }
  return "unknown";
}

const char* productionMppiPreviousControlSourceName(
    const ProductionMppiPreviousControlSource source) noexcept {
  switch (source) {
    case ProductionMppiPreviousControlSource::kEngineFallback:
      return "engine_fallback";
    case ProductionMppiPreviousControlSource::kMeasuredAcceleration:
      return "measured_acceleration";
    case ProductionMppiPreviousControlSource::kOffboardFeedback:
      return "offboard_feedback";
  }
  return "unknown";
}

const char*
productionMppiExecutionModeName(const ProductionMppiExecutionMode mode) noexcept {
  switch (mode) {
    case ProductionMppiExecutionMode::kPlanned:
      return "planned";
    case ProductionMppiExecutionMode::kPositionHold:
      return "position_hold";
  }
  return "unknown";
}

const char*
productionMppiExecutionReasonName(const ProductionMppiExecutionReason reason) noexcept {
  switch (reason) {
    case ProductionMppiExecutionReason::kNone:
      return "none";
    case ProductionMppiExecutionReason::kNoExecutableHorizon:
      return "no_executable_horizon";
    case ProductionMppiExecutionReason::kCooperativePassageYield:
      return "cooperative_passage_yield";
    case ProductionMppiExecutionReason::kGoalCapture:
      return "goal_capture";
    case ProductionMppiExecutionReason::kNoExecutableRoute:
      return "no_executable_route";
  }
  return "unknown";
}

const char*
productionPlanningSearchKindName(const ProductionPlanningSearchKind kind) noexcept {
  switch (kind) {
    case ProductionPlanningSearchKind::kNone:
      return "none";
    case ProductionPlanningSearchKind::kLattice2D:
      return "lattice_2d";
    case ProductionPlanningSearchKind::kLattice3D:
      return "lattice_3d";
  }
  return "unknown";
}

const char* productionGuideCandidateValidationStatusName(
    const ProductionGuideCandidateValidationStatus status) noexcept {
  switch (status) {
    case ProductionGuideCandidateValidationStatus::kNotAttempted:
      return "not_attempted";
    case ProductionGuideCandidateValidationStatus::kAccepted:
      return "accepted";
    case ProductionGuideCandidateValidationStatus::kUnavailableLatestWorld:
      return "unavailable_latest_world";
    case ProductionGuideCandidateValidationStatus::kInvalidProjection:
      return "invalid_projection";
    case ProductionGuideCandidateValidationStatus::kExcessiveCrossTrack:
      return "excessive_cross_track";
    case ProductionGuideCandidateValidationStatus::kRawValidationRejected:
      return "raw_validation_rejected";
    case ProductionGuideCandidateValidationStatus::kLifecycleRejected:
      return "lifecycle_rejected";
  }
  return "unknown";
}

} // namespace drone_city_nav
