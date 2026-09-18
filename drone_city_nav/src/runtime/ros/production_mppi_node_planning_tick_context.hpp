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
};

} // namespace drone_city_nav
