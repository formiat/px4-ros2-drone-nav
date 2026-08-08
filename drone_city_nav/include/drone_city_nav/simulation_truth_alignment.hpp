#pragma once

#include "drone_city_nav/intercept_mission.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace drone_city_nav {

struct SimulationTruthAlignmentConfig {
  double maximum_position_error_m{2.0};
  double maximum_state_age_s{0.5};
  double maximum_time_alignment_s{0.15};
  double failure_confirmation_s{1.0};
  std::size_t readiness_confirmation_samples{5U};
};

struct SimulationTruthAlignmentSample {
  std::optional<TimedVehicleState> navigation;
  std::optional<TimedVehicleState> physical_truth;
};

enum class SimulationTruthAlignmentReason : std::uint8_t {
  kAligned,
  kConfirming,
  kMissingNavigation,
  kMissingPhysicalTruth,
  kStaleNavigation,
  kStalePhysicalTruth,
  kTimeAlignmentUnavailable,
  kPositionMismatch,
};

struct SimulationTruthAlignmentUpdate {
  SimulationTruthAlignmentReason reason{
      SimulationTruthAlignmentReason::kMissingNavigation};
  std::size_t aligned_vehicle_count{0U};
  std::size_t offending_vehicle_index{0U};
  double maximum_position_error_m{0.0};
  bool ready{false};
  bool newly_ready{false};
  bool failure_confirmed{false};
  bool newly_failed{false};
};

struct SimulationTruthAlignmentObservation {
  bool ready{false};
  bool sample_aligned{false};
  bool failure_confirmed{false};
};

struct SimulationTruthAlignmentMissionUpdate {
  bool startup_ready{false};
  bool startup_failure_confirmed{false};
  bool runtime_residual{false};
  bool newly_runtime_degraded{false};
  bool newly_runtime_recovered{false};
};

class SimulationTruthAlignmentMissionLifecycle final {
public:
  [[nodiscard]] SimulationTruthAlignmentMissionUpdate
  update(const SimulationTruthAlignmentObservation& observation) noexcept;

  [[nodiscard]] bool latchStartupContract() noexcept;

private:
  SimulationTruthAlignmentObservation observation_{};
  bool startup_contract_latched_{false};
  bool runtime_residual_{false};
};

class SimulationTruthAlignmentMonitor final {
public:
  explicit SimulationTruthAlignmentMonitor(
      const SimulationTruthAlignmentConfig& config = {});

  [[nodiscard]] SimulationTruthAlignmentUpdate
  update(std::int64_t now_ns,
         std::span<const SimulationTruthAlignmentSample> samples) noexcept;

private:
  SimulationTruthAlignmentConfig config_{};
  std::int64_t maximum_state_age_ns_{0};
  std::int64_t maximum_time_alignment_ns_{0};
  std::int64_t failure_confirmation_ns_{0};
  std::optional<std::int64_t> failure_started_ns_;
  std::optional<SimulationTruthAlignmentReason> failure_reason_;
  std::size_t consecutive_aligned_samples_{0U};
  bool ready_{false};
  bool failure_confirmed_{false};
};

[[nodiscard]] const char*
simulationTruthAlignmentReasonName(SimulationTruthAlignmentReason reason) noexcept;

} // namespace drone_city_nav
