#pragma once

#include <chrono>
#include <cstdint>

#include "mppi_controller_3d.hpp"
#include "production_mppi_node.hpp"

namespace drone_city_nav {

[[nodiscard]] inline ProductionMppiExecutionReason
terminalExecutionReason(const NavigationTerminalFailure failure) noexcept {
  switch (failure) {
    case NavigationTerminalFailure::kUnavailableWorld:
      return ProductionMppiExecutionReason::kUnavailableWorld;
    case NavigationTerminalFailure::kNoAcknowledgedHorizon:
      return ProductionMppiExecutionReason::kNoExecutableHorizon;
    case NavigationTerminalFailure::kNone:
    case NavigationTerminalFailure::kNoExecutableRoute:
    case NavigationTerminalFailure::kRecoveryBudgetExhausted:
      return ProductionMppiExecutionReason::kNoExecutableRoute;
  }
  return ProductionMppiExecutionReason::kNoExecutableRoute;
}

[[nodiscard]] inline std::optional<DirectTrackingOwnerIdentity3D>
makeDirectTrackingOwnerIdentity(const ProductionNavigationObjective* const objective,
                                const bool direct_tracking_interception,
                                const std::uint64_t line_of_sight_generation) {
  if (!direct_tracking_interception || objective == nullptr) {
    return std::nullopt;
  }
  return DirectTrackingOwnerIdentity3D{
      .mission_epoch = objective->mission_epoch,
      .assignment_generation = objective->assignment_generation,
      .target_detection_id = objective->target_detection_id,
      .target_track_id = objective->target_track_id,
      .objective_sample_sequence = objective->sample_sequence,
      .line_of_sight_generation = line_of_sight_generation,
  };
}

[[nodiscard]] inline std::uint64_t
directTrackingRouteGeneration(const bool direct_tracking_interception,
                              const std::uint64_t line_of_sight_generation) noexcept {
  return direct_tracking_interception
             ? (std::uint64_t{1} << 63U) | line_of_sight_generation
             : 0U;
}

struct PreviousControlEvidence3D {
  mppi::Control control{};
  ExecutionPreviousControlEvidenceSource3D source{
      ExecutionPreviousControlEvidenceSource3D::kUnknown};
  std::uint64_t source_producer_instance_id{0U};
  std::uint64_t source_sequence{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
};

struct ProductionMppiControllerTick {
  const WorldSnapshot3D& world;
  MppiControllerRequest3D request{};
  std::uint64_t route_generation{0U};
  std::int64_t now_ns{0};
  double route_cross_track_m{0.0};
  bool direct_tracking_interception{false};
};

} // namespace drone_city_nav
