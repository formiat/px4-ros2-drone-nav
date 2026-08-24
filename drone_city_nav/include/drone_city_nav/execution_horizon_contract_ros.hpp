#pragma once

#include "drone_city_nav/execution_horizon_witness.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"

#include <cstdint>
#include <string_view>

namespace drone_city_nav {

enum class ExecutionHorizonPayloadStatus : std::uint8_t {
  kValid,
  kInvalidIdentity,
  kInvalidFrame,
  kInvalidTiming,
  kNonFiniteMetadata,
  kNonFinitePoint,
  kOutsideFlightEnvelope,
  kInvalidEnum,
  kInconsistentExecutionMode,
  kInconsistentRevocation,
  kMissingTerminalRestState,
  kInconsistentStationaryHold,
};

struct ExecutionHorizonPayloadValidationConfig {
  std::string_view expected_frame_id{"map"};
  const FlightEnvelopeConfig* flight_envelope{nullptr};
};

enum class ExecutionControlFeedbackStatus : std::uint8_t {
  kValid,
  kInvalidFrame,
  kInvalidTiming,
  kInvalidIdentity,
  kInvalidEnum,
  kInconsistentExecutionMode,
  kNonFiniteControl,
};

struct ExecutionControlFeedbackAssessment {
  ExecutionControlFeedbackStatus status{
      ExecutionControlFeedbackStatus::kInvalidIdentity};
  ExecutionHorizonFeedbackCandidate candidate{};

  [[nodiscard]] bool valid() const noexcept {
    return status == ExecutionControlFeedbackStatus::kValid;
  }

  // This reports structural candidate validity only. Consumers must still
  // route every non-kValid assessment through malformed admission so frame and
  // canonical-time failures cannot refresh session liveness.
  [[nodiscard]] bool admissionCandidateValid() const noexcept {
    return candidate.valid();
  }
};

[[nodiscard]] std::int64_t
executionHorizonTimeNanoseconds(const builtin_interfaces::msg::Time& time) noexcept;

[[nodiscard]] ExecutionHorizonPayloadStatus assessExecutionHorizonPayload(
    const msg::MppiTrajectoryHorizon& horizon,
    const ExecutionHorizonPayloadValidationConfig& config) noexcept;

// Canonical authority content bound to (producer_instance_id, sequence). The
// fingerprint includes timestamps and every executable/revocation field so a
// same-identity wire mutation cannot be treated as a replay.
[[nodiscard]] std::uint64_t
executionHorizonContentFingerprint(const msg::MppiTrajectoryHorizon& horizon) noexcept;

[[nodiscard]] std::string_view
executionHorizonPayloadStatusName(ExecutionHorizonPayloadStatus status) noexcept;

// Validates the complete ROS feedback payload. The raw source time and an exact
// wire fingerprint are captured before frame, canonical-time, enum, or mode
// gates so consumers can claim and tombstone malformed source identities.
[[nodiscard]] ExecutionControlFeedbackAssessment
assessExecutionControlFeedback(const msg::MppiControlFeedback& feedback,
                               std::string_view expected_frame_id,
                               std::int64_t receive_stamp_ns) noexcept;

[[nodiscard]] std::string_view
executionControlFeedbackStatusName(ExecutionControlFeedbackStatus status) noexcept;

} // namespace drone_city_nav
