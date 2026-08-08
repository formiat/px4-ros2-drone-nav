#include "drone_city_nav/simulation_truth_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] double distance(const Point3& first, const Point3& second) noexcept {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

[[nodiscard]] bool fresh(const TimedVehicleState& state, const std::int64_t now_ns,
                         const std::int64_t maximum_age_ns) noexcept {
  return state.position_valid && finite(state.position) && state.stamp_ns > 0 &&
         now_ns >= state.stamp_ns && now_ns - state.stamp_ns <= maximum_age_ns;
}

[[nodiscard]] std::optional<Point3>
positionAt(const TimedVehicleState& state, const std::int64_t stamp_ns,
           const std::int64_t maximum_alignment_ns) noexcept {
  if (!state.position_valid || !finite(state.position) || state.stamp_ns <= 0 ||
      stamp_ns < state.stamp_ns || stamp_ns - state.stamp_ns > maximum_alignment_ns) {
    return std::nullopt;
  }
  if (stamp_ns == state.stamp_ns) {
    return state.position;
  }
  if (!state.velocity_valid || !finite(state.velocity)) {
    return std::nullopt;
  }
  const double delta_s = static_cast<double>(stamp_ns - state.stamp_ns) * 1.0e-9;
  return Point3{state.position.x + state.velocity.x * delta_s,
                state.position.y + state.velocity.y * delta_s,
                state.position.z + state.velocity.z * delta_s};
}

} // namespace

SimulationTruthAlignmentMissionUpdate SimulationTruthAlignmentMissionLifecycle::update(
    const SimulationTruthAlignmentObservation& observation) noexcept {
  observation_ = observation;
  const bool previous_runtime_residual = runtime_residual_;
  runtime_residual_ = startup_contract_latched_ &&
                      (!observation_.ready || !observation_.sample_aligned ||
                       observation_.failure_confirmed);
  return SimulationTruthAlignmentMissionUpdate{
      .startup_ready = startup_contract_latched_ ||
                       (observation_.ready && observation_.sample_aligned &&
                        !observation_.failure_confirmed),
      .startup_failure_confirmed =
          !startup_contract_latched_ && observation_.failure_confirmed,
      .runtime_residual = runtime_residual_,
      .newly_runtime_degraded = runtime_residual_ && !previous_runtime_residual,
      .newly_runtime_recovered = !runtime_residual_ && previous_runtime_residual,
  };
}

bool SimulationTruthAlignmentMissionLifecycle::latchStartupContract() noexcept {
  if (startup_contract_latched_) {
    return true;
  }
  if (!observation_.ready || !observation_.sample_aligned ||
      observation_.failure_confirmed) {
    return false;
  }
  startup_contract_latched_ = true;
  runtime_residual_ = false;
  return true;
}

SimulationTruthAlignmentMonitor::SimulationTruthAlignmentMonitor(
    const SimulationTruthAlignmentConfig& config)
    : config_{config} {
  if (!(config_.maximum_position_error_m > 0.0) ||
      !(config_.maximum_state_age_s > 0.0) ||
      !(config_.maximum_time_alignment_s >= 0.0) ||
      !(config_.failure_confirmation_s >= 0.0) ||
      config_.readiness_confirmation_samples == 0U ||
      !std::isfinite(config_.maximum_position_error_m) ||
      !std::isfinite(config_.maximum_state_age_s) ||
      !std::isfinite(config_.maximum_time_alignment_s) ||
      !std::isfinite(config_.failure_confirmation_s)) {
    throw std::invalid_argument{"invalid simulation truth alignment configuration"};
  }
  maximum_state_age_ns_ =
      static_cast<std::int64_t>(config_.maximum_state_age_s * 1.0e9);
  maximum_time_alignment_ns_ =
      static_cast<std::int64_t>(config_.maximum_time_alignment_s * 1.0e9);
  failure_confirmation_ns_ =
      static_cast<std::int64_t>(config_.failure_confirmation_s * 1.0e9);
}

SimulationTruthAlignmentUpdate SimulationTruthAlignmentMonitor::update(
    const std::int64_t now_ns,
    const std::span<const SimulationTruthAlignmentSample> samples) noexcept {
  SimulationTruthAlignmentUpdate result;
  bool aligned = !samples.empty() && now_ns > 0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const SimulationTruthAlignmentSample& sample = samples[index];
    auto reject = [&](const SimulationTruthAlignmentReason reason) {
      if (aligned) {
        result.reason = reason;
        result.offending_vehicle_index = index;
      }
      aligned = false;
    };
    if (!sample.navigation.has_value()) {
      reject(SimulationTruthAlignmentReason::kMissingNavigation);
      continue;
    }
    if (!sample.physical_truth.has_value()) {
      reject(SimulationTruthAlignmentReason::kMissingPhysicalTruth);
      continue;
    }
    if (!fresh(*sample.navigation, now_ns, maximum_state_age_ns_)) {
      reject(SimulationTruthAlignmentReason::kStaleNavigation);
      continue;
    }
    if (!fresh(*sample.physical_truth, now_ns, maximum_state_age_ns_)) {
      reject(SimulationTruthAlignmentReason::kStalePhysicalTruth);
      continue;
    }
    const std::int64_t comparison_stamp =
        std::max(sample.navigation->stamp_ns, sample.physical_truth->stamp_ns);
    const std::optional<Point3> navigation_position =
        positionAt(*sample.navigation, comparison_stamp, maximum_time_alignment_ns_);
    const std::optional<Point3> truth_position = positionAt(
        *sample.physical_truth, comparison_stamp, maximum_time_alignment_ns_);
    if (!navigation_position || !truth_position) {
      reject(SimulationTruthAlignmentReason::kTimeAlignmentUnavailable);
      continue;
    }
    const double position_error = distance(*navigation_position, *truth_position);
    result.maximum_position_error_m =
        std::max(result.maximum_position_error_m, position_error);
    if (position_error > config_.maximum_position_error_m) {
      reject(SimulationTruthAlignmentReason::kPositionMismatch);
      continue;
    }
    ++result.aligned_vehicle_count;
  }

  result.newly_ready = false;
  result.newly_failed = false;
  if (aligned) {
    failure_started_ns_.reset();
    failure_reason_.reset();
    failure_confirmed_ = false;
    ++consecutive_aligned_samples_;
    if (!ready_ &&
        consecutive_aligned_samples_ >= config_.readiness_confirmation_samples) {
      ready_ = true;
      result.newly_ready = true;
    }
    result.reason = ready_ ? SimulationTruthAlignmentReason::kAligned
                           : SimulationTruthAlignmentReason::kConfirming;
  } else {
    consecutive_aligned_samples_ = 0U;
    const bool confirmable =
        ready_ || result.reason == SimulationTruthAlignmentReason::kPositionMismatch;
    if (!confirmable) {
      failure_started_ns_.reset();
      failure_reason_.reset();
    } else if (!failure_started_ns_.has_value() || now_ns < *failure_started_ns_ ||
               failure_reason_ != result.reason) {
      failure_started_ns_ = now_ns;
      failure_reason_ = result.reason;
    }
    if (confirmable && failure_started_ns_.has_value() &&
        now_ns - *failure_started_ns_ >= failure_confirmation_ns_ &&
        !failure_confirmed_) {
      ready_ = false;
      failure_confirmed_ = true;
      result.newly_failed = true;
    }
  }
  result.ready = ready_;
  result.failure_confirmed = failure_confirmed_;
  return result;
}

const char* simulationTruthAlignmentReasonName(
    const SimulationTruthAlignmentReason reason) noexcept {
  switch (reason) {
    case SimulationTruthAlignmentReason::kAligned:
      return "aligned";
    case SimulationTruthAlignmentReason::kConfirming:
      return "confirming";
    case SimulationTruthAlignmentReason::kMissingNavigation:
      return "missing_navigation";
    case SimulationTruthAlignmentReason::kMissingPhysicalTruth:
      return "missing_physical_truth";
    case SimulationTruthAlignmentReason::kStaleNavigation:
      return "stale_navigation";
    case SimulationTruthAlignmentReason::kStalePhysicalTruth:
      return "stale_physical_truth";
    case SimulationTruthAlignmentReason::kTimeAlignmentUnavailable:
      return "time_alignment_unavailable";
    case SimulationTruthAlignmentReason::kPositionMismatch:
      return "position_mismatch";
  }
  return "unknown";
}

} // namespace drone_city_nav
