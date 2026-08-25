#include "drone_city_nav/navigation_angular_derivative.hpp"

#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {
namespace {

constexpr double kSecondsPerMicrosecond{1.0e-6};
constexpr double kSecondsPerNanosecond{1.0e-9};
constexpr double kPhysicalLimitTolerance{1.0e-5};

[[nodiscard]] bool withinPhysicalLimit(const double value,
                                       const double limit) noexcept {
  return std::isfinite(value) && std::isfinite(limit) && limit > 0.0 &&
         std::abs(value) <= limit + kPhysicalLimitTolerance;
}

[[nodiscard]] bool midpointTwice(const std::uint64_t first_timestamp_us,
                                 const std::uint64_t second_timestamp_us,
                                 std::uint64_t& result) noexcept {
  if (first_timestamp_us >
      std::numeric_limits<std::uint64_t>::max() - second_timestamp_us) {
    return false;
  }
  result = first_timestamp_us + second_timestamp_us;
  return true;
}

void clearPendingTimestampAdmission(Px4TimestampEpochAdmissionState& state) noexcept {
  state.pending_primary_timestamp_us = 0U;
  state.pending_corroborating_timestamp_us = 0U;
  state.pending_receive_timestamp_ns = 0;
  state.pending_confirmation_count = 0U;
  state.pending_kind = Px4TimestampEpochAdmissionPendingKind::kNone;
}

void beginPendingTimestampAdmission(
    Px4TimestampEpochAdmissionState& state,
    const Px4TimestampEpochObservation& observation,
    const Px4TimestampEpochAdmissionPendingKind pending_kind) noexcept {
  state.pending_primary_timestamp_us = observation.primary_timestamp_us;
  state.pending_corroborating_timestamp_us = observation.corroborating_timestamp_us;
  state.pending_receive_timestamp_ns = observation.receive_timestamp_ns;
  state.pending_confirmation_count = 1U;
  state.pending_kind = pending_kind;
}

[[nodiscard]] bool
timestampBackwardByAtLeast(const std::uint64_t high_water_us,
                           const std::uint64_t candidate_us,
                           const double minimum_backward_jump_s) noexcept {
  return high_water_us != 0U && candidate_us != 0U && candidate_us < high_water_us &&
         static_cast<double>(high_water_us - candidate_us) * kSecondsPerMicrosecond >=
             minimum_backward_jump_s;
}

[[nodiscard]] bool timestampProgressPlausible(
    const std::uint64_t high_water_us, const std::int64_t high_water_receive_ns,
    const std::uint64_t candidate_us, const std::int64_t candidate_receive_ns,
    const double maximum_source_lead_s) noexcept {
  if (candidate_us == 0U) {
    return true;
  }
  if (high_water_us == 0U) {
    return high_water_receive_ns == 0;
  }
  if (candidate_us <= high_water_us || high_water_receive_ns <= 0 ||
      candidate_receive_ns <= high_water_receive_ns) {
    return false;
  }
  const double source_progress_s =
      static_cast<double>(candidate_us - high_water_us) * kSecondsPerMicrosecond;
  const double receive_progress_s =
      static_cast<double>(candidate_receive_ns - high_water_receive_ns) *
      kSecondsPerNanosecond;
  return std::isfinite(source_progress_s) && std::isfinite(receive_progress_s) &&
         source_progress_s <= receive_progress_s + maximum_source_lead_s;
}

[[nodiscard]] bool timestampEpochAdmissionStateIsValid(
    const Px4TimestampEpochAdmissionConfig& config,
    const Px4TimestampEpochAdmissionState& state) noexcept {
  const bool primary_consistent = (state.primary_timestamp_high_water_us == 0U) ==
                                  (state.primary_timestamp_receive_ns == 0);
  const bool corroborating_consistent =
      (state.corroborating_timestamp_high_water_us == 0U) ==
      (state.corroborating_timestamp_receive_ns == 0);
  if (!primary_consistent || !corroborating_consistent ||
      state.primary_timestamp_receive_ns < 0 ||
      state.corroborating_timestamp_receive_ns < 0 ||
      state.last_observation_receive_ns < 0 ||
      state.last_observation_receive_ns < state.primary_timestamp_receive_ns ||
      state.last_observation_receive_ns < state.corroborating_timestamp_receive_ns) {
    return false;
  }
  if (state.pending_confirmation_count == 0U) {
    return state.pending_primary_timestamp_us == 0U &&
           state.pending_corroborating_timestamp_us == 0U &&
           state.pending_receive_timestamp_ns == 0 &&
           state.pending_kind == Px4TimestampEpochAdmissionPendingKind::kNone;
  }
  return state.pending_confirmation_count < config.epoch_reset_confirmation_samples &&
         state.pending_receive_timestamp_ns > 0 &&
         state.pending_receive_timestamp_ns == state.last_observation_receive_ns &&
         (state.pending_primary_timestamp_us != 0U ||
          state.pending_corroborating_timestamp_us != 0U) &&
         (state.pending_kind == Px4TimestampEpochAdmissionPendingKind::kEpochReset ||
          state.pending_kind ==
              Px4TimestampEpochAdmissionPendingKind::kForwardReacquisition) &&
         (!config.require_corroborating_timestamp_for_reset ||
          (state.pending_primary_timestamp_us != 0U &&
           state.pending_corroborating_timestamp_us != 0U));
}

[[nodiscard]] bool pendingTimestampObservationConsecutive(
    const Px4TimestampEpochAdmissionConfig& config,
    const Px4TimestampEpochAdmissionState& state,
    const Px4TimestampEpochObservation& observation) noexcept {
  if (observation.receive_timestamp_ns <= state.pending_receive_timestamp_ns ||
      (observation.primary_timestamp_us == 0U) !=
          (state.pending_primary_timestamp_us == 0U) ||
      (observation.corroborating_timestamp_us == 0U) !=
          (state.pending_corroborating_timestamp_us == 0U)) {
    return false;
  }
  const double receive_interval_s =
      static_cast<double>(observation.receive_timestamp_ns -
                          state.pending_receive_timestamp_ns) *
      kSecondsPerNanosecond;
  if (!std::isfinite(receive_interval_s) || receive_interval_s <= 0.0 ||
      receive_interval_s > config.maximum_epoch_confirmation_interval_s) {
    return false;
  }
  const auto timestamp_consecutive = [&](const std::uint64_t previous,
                                         const std::uint64_t current) noexcept {
    if (current == 0U) {
      return true;
    }
    if (current <= previous) {
      return false;
    }
    const double source_interval_s =
        static_cast<double>(current - previous) * kSecondsPerMicrosecond;
    return std::isfinite(source_interval_s) && source_interval_s > 0.0 &&
           source_interval_s <= config.maximum_epoch_confirmation_interval_s &&
           source_interval_s <=
               receive_interval_s + config.maximum_source_timestamp_lead_s;
  };
  return timestamp_consecutive(state.pending_primary_timestamp_us,
                               observation.primary_timestamp_us) &&
         timestamp_consecutive(state.pending_corroborating_timestamp_us,
                               observation.corroborating_timestamp_us);
}

[[nodiscard]] bool postResetReceiveGapRequiresReacquisition(
    const Px4TimestampEpochAdmissionConfig& config,
    const Px4TimestampEpochAdmissionState& state,
    const Px4TimestampEpochObservation& observation) noexcept {
  if (!state.post_reset_replay_guard_active) {
    return false;
  }
  const auto stream_requires_reacquisition =
      [&](const bool present, const std::uint64_t high_water_us,
          const std::int64_t accepted_receive_ns) noexcept {
        if (!present) {
          return false;
        }
        if (high_water_us == 0U || accepted_receive_ns <= 0 ||
            observation.receive_timestamp_ns <= accepted_receive_ns) {
          return true;
        }
        const double receive_gap_s =
            static_cast<double>(observation.receive_timestamp_ns -
                                accepted_receive_ns) *
            kSecondsPerNanosecond;
        return !std::isfinite(receive_gap_s) ||
               receive_gap_s > config.maximum_post_reset_unprobated_receive_gap_s;
      };
  return stream_requires_reacquisition(observation.primary_timestamp_us != 0U,
                                       state.primary_timestamp_high_water_us,
                                       state.primary_timestamp_receive_ns) ||
         stream_requires_reacquisition(observation.corroborating_timestamp_us != 0U,
                                       state.corroborating_timestamp_high_water_us,
                                       state.corroborating_timestamp_receive_ns);
}

} // namespace

bool px4TimestampEpochAdmissionConfigIsValid(
    const Px4TimestampEpochAdmissionConfig& config) noexcept {
  return std::isfinite(config.minimum_epoch_reset_backward_jump_s) &&
         config.minimum_epoch_reset_backward_jump_s > 0.0 &&
         std::isfinite(config.maximum_epoch_confirmation_interval_s) &&
         config.maximum_epoch_confirmation_interval_s > 0.0 &&
         std::isfinite(config.maximum_source_timestamp_lead_s) &&
         config.maximum_source_timestamp_lead_s >= 0.0 &&
         std::isfinite(config.maximum_post_reset_unprobated_receive_gap_s) &&
         config.maximum_post_reset_unprobated_receive_gap_s > 0.0 &&
         config.epoch_reset_confirmation_samples >= 2U;
}

const char* px4TimestampEpochAdmissionStatusName(
    const Px4TimestampEpochAdmissionStatus status) noexcept {
  switch (status) {
    case Px4TimestampEpochAdmissionStatus::kAcceptedInitial:
      return "accepted_initial";
    case Px4TimestampEpochAdmissionStatus::kAcceptedNewer:
      return "accepted_newer";
    case Px4TimestampEpochAdmissionStatus::kAcceptedEpochReset:
      return "accepted_epoch_reset";
    case Px4TimestampEpochAdmissionStatus::kAcceptedForwardReacquisition:
      return "accepted_forward_reacquisition";
    case Px4TimestampEpochAdmissionStatus::kPendingEpochReset:
      return "pending_epoch_reset";
    case Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition:
      return "pending_forward_reacquisition";
    case Px4TimestampEpochAdmissionStatus::kRejectedInvalidConfiguration:
      return "invalid_configuration";
    case Px4TimestampEpochAdmissionStatus::kRejectedInvalidState:
      return "invalid_state";
    case Px4TimestampEpochAdmissionStatus::kRejectedMissingTimestamp:
      return "missing_timestamp";
    case Px4TimestampEpochAdmissionStatus::kRejectedMissingReceiveTimestamp:
      return "missing_receive_timestamp";
    case Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicReceiveTimestamp:
      return "nonmonotonic_receive_timestamp";
    case Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicPrimaryTimestamp:
      return "nonmonotonic_primary_timestamp";
    case Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicCorroboratingTimestamp:
      return "nonmonotonic_corroborating_timestamp";
    case Px4TimestampEpochAdmissionStatus::kRejectedImplausibleTimestampProgress:
      return "implausible_timestamp_progress";
  }
  return "unknown";
}

bool px4TimestampEpochAdmissionAccepted(
    const Px4TimestampEpochAdmissionStatus status) noexcept {
  return status == Px4TimestampEpochAdmissionStatus::kAcceptedInitial ||
         status == Px4TimestampEpochAdmissionStatus::kAcceptedNewer ||
         status == Px4TimestampEpochAdmissionStatus::kAcceptedEpochReset ||
         status == Px4TimestampEpochAdmissionStatus::kAcceptedForwardReacquisition;
}

Px4TimestampEpochAdmissionResult
admitPx4TimestampEpoch(const Px4TimestampEpochAdmissionConfig& config,
                       const Px4TimestampEpochAdmissionState& state,
                       const Px4TimestampEpochObservation& observation) noexcept {
  Px4TimestampEpochAdmissionResult result{.next_state = state};
  if (!px4TimestampEpochAdmissionConfigIsValid(config)) {
    result.status = Px4TimestampEpochAdmissionStatus::kRejectedInvalidConfiguration;
    return result;
  }
  if (!timestampEpochAdmissionStateIsValid(config, state)) {
    result.status = Px4TimestampEpochAdmissionStatus::kRejectedInvalidState;
    return result;
  }
  const bool has_primary = observation.primary_timestamp_us != 0U;
  const bool has_corroborating = observation.corroborating_timestamp_us != 0U;
  if (!has_primary && !has_corroborating) {
    result.status = Px4TimestampEpochAdmissionStatus::kRejectedMissingTimestamp;
    return result;
  }
  if (observation.receive_timestamp_ns <= 0) {
    result.status = Px4TimestampEpochAdmissionStatus::kRejectedMissingReceiveTimestamp;
    return result;
  }
  if (state.last_observation_receive_ns != 0 &&
      observation.receive_timestamp_ns <= state.last_observation_receive_ns) {
    clearPendingTimestampAdmission(result.next_state);
    result.status =
        Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicReceiveTimestamp;
    return result;
  }

  result.next_state.last_observation_receive_ns = observation.receive_timestamp_ns;
  const bool primary_forward =
      !has_primary || state.primary_timestamp_high_water_us == 0U ||
      observation.primary_timestamp_us > state.primary_timestamp_high_water_us;
  const bool corroborating_forward =
      !has_corroborating || state.corroborating_timestamp_high_water_us == 0U ||
      observation.corroborating_timestamp_us >
          state.corroborating_timestamp_high_water_us;
  if (primary_forward && corroborating_forward) {
    const bool initial = state.primary_timestamp_high_water_us == 0U &&
                         state.corroborating_timestamp_high_water_us == 0U;
    const bool progress_plausible =
        timestampProgressPlausible(
            state.primary_timestamp_high_water_us, state.primary_timestamp_receive_ns,
            observation.primary_timestamp_us, observation.receive_timestamp_ns,
            config.maximum_source_timestamp_lead_s) &&
        timestampProgressPlausible(state.corroborating_timestamp_high_water_us,
                                   state.corroborating_timestamp_receive_ns,
                                   observation.corroborating_timestamp_us,
                                   observation.receive_timestamp_ns,
                                   config.maximum_source_timestamp_lead_s);
    // After a confirmed backward epoch reset, an implausibly far-ahead sample
    // is indistinguishable from delayed replay of the retired epoch. Keep the
    // replay guard strict; ordinary time-sync corrections are handled below.
    if (!progress_plausible && state.post_reset_replay_guard_active) {
      clearPendingTimestampAdmission(result.next_state);
      result.status =
          Px4TimestampEpochAdmissionStatus::kRejectedImplausibleTimestampProgress;
      return result;
    }
    // A coordinated PX4 time-sync correction can advance source time farther
    // than wall receive time. Treat it as a bounded new-epoch probation rather
    // than permanently pinning the high-water mark behind the correction.
    const bool forward_reacquisition_required =
        !initial && (!progress_plausible || postResetReceiveGapRequiresReacquisition(
                                                config, state, observation));
    if (forward_reacquisition_required) {
      if (config.require_corroborating_timestamp_for_reset &&
          (!has_primary || !has_corroborating)) {
        clearPendingTimestampAdmission(result.next_state);
        result.status = Px4TimestampEpochAdmissionStatus::kRejectedMissingTimestamp;
        return result;
      }
      constexpr auto kPendingKind =
          Px4TimestampEpochAdmissionPendingKind::kForwardReacquisition;
      if (state.pending_kind != kPendingKind ||
          !pendingTimestampObservationConsecutive(config, state, observation)) {
        beginPendingTimestampAdmission(result.next_state, observation, kPendingKind);
        result.status = Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition;
        return result;
      }
      result.next_state.pending_primary_timestamp_us = observation.primary_timestamp_us;
      result.next_state.pending_corroborating_timestamp_us =
          observation.corroborating_timestamp_us;
      result.next_state.pending_receive_timestamp_ns = observation.receive_timestamp_ns;
      ++result.next_state.pending_confirmation_count;
      if (result.next_state.pending_confirmation_count <
          config.epoch_reset_confirmation_samples) {
        result.status = Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition;
        return result;
      }
    }
    if (has_primary) {
      result.next_state.primary_timestamp_high_water_us =
          observation.primary_timestamp_us;
      result.next_state.primary_timestamp_receive_ns = observation.receive_timestamp_ns;
    }
    if (has_corroborating) {
      result.next_state.corroborating_timestamp_high_water_us =
          observation.corroborating_timestamp_us;
      result.next_state.corroborating_timestamp_receive_ns =
          observation.receive_timestamp_ns;
    }
    clearPendingTimestampAdmission(result.next_state);
    if (forward_reacquisition_required) {
      result.status = Px4TimestampEpochAdmissionStatus::kAcceptedForwardReacquisition;
      result.forward_reacquisition = true;
    } else {
      result.status = initial ? Px4TimestampEpochAdmissionStatus::kAcceptedInitial
                              : Px4TimestampEpochAdmissionStatus::kAcceptedNewer;
    }
    return result;
  }

  const bool primary_deep_reset =
      !has_primary || state.primary_timestamp_high_water_us == 0U ||
      timestampBackwardByAtLeast(state.primary_timestamp_high_water_us,
                                 observation.primary_timestamp_us,
                                 config.minimum_epoch_reset_backward_jump_s);
  const bool corroborating_deep_reset =
      !has_corroborating || state.corroborating_timestamp_high_water_us == 0U ||
      timestampBackwardByAtLeast(state.corroborating_timestamp_high_water_us,
                                 observation.corroborating_timestamp_us,
                                 config.minimum_epoch_reset_backward_jump_s);
  const bool required_timestamps_present =
      !config.require_corroborating_timestamp_for_reset ||
      (has_primary && has_corroborating);
  const bool reset_candidate = required_timestamps_present && primary_deep_reset &&
                               corroborating_deep_reset &&
                               ((has_primary && !primary_forward) ||
                                (has_corroborating && !corroborating_forward));
  if (!reset_candidate) {
    clearPendingTimestampAdmission(result.next_state);
    result.status =
        !primary_forward
            ? Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicPrimaryTimestamp
            : Px4TimestampEpochAdmissionStatus::
                  kRejectedNonmonotonicCorroboratingTimestamp;
    return result;
  }

  constexpr auto kPendingKind = Px4TimestampEpochAdmissionPendingKind::kEpochReset;
  if (state.pending_kind != kPendingKind) {
    beginPendingTimestampAdmission(result.next_state, observation, kPendingKind);
    result.status = Px4TimestampEpochAdmissionStatus::kPendingEpochReset;
    return result;
  }

  if (!pendingTimestampObservationConsecutive(config, state, observation)) {
    beginPendingTimestampAdmission(result.next_state, observation, kPendingKind);
    result.status = Px4TimestampEpochAdmissionStatus::kPendingEpochReset;
    return result;
  }
  result.next_state.pending_primary_timestamp_us = observation.primary_timestamp_us;
  result.next_state.pending_corroborating_timestamp_us =
      observation.corroborating_timestamp_us;
  result.next_state.pending_receive_timestamp_ns = observation.receive_timestamp_ns;
  ++result.next_state.pending_confirmation_count;
  if (result.next_state.pending_confirmation_count <
      config.epoch_reset_confirmation_samples) {
    result.status = Px4TimestampEpochAdmissionStatus::kPendingEpochReset;
    return result;
  }

  if (has_primary) {
    result.next_state.primary_timestamp_high_water_us =
        observation.primary_timestamp_us;
    result.next_state.primary_timestamp_receive_ns = observation.receive_timestamp_ns;
  }
  if (has_corroborating) {
    result.next_state.corroborating_timestamp_high_water_us =
        observation.corroborating_timestamp_us;
    result.next_state.corroborating_timestamp_receive_ns =
        observation.receive_timestamp_ns;
  }
  result.next_state.post_reset_replay_guard_active = true;
  clearPendingTimestampAdmission(result.next_state);
  result.status = Px4TimestampEpochAdmissionStatus::kAcceptedEpochReset;
  result.epoch_reset = true;
  return result;
}

NavigationLocalStateResetAssessment
assessNavigationLocalStateReset(const NavigationLocalStateResetCounters& previous,
                                const NavigationLocalStateResetCounters& candidate,
                                const bool has_baseline,
                                const bool timestamp_epoch_reset) noexcept {
  const bool xy_reset = has_baseline && candidate.xy != previous.xy;
  const bool z_reset = has_baseline && candidate.z != previous.z;
  const bool velocity_or_heading_reset =
      has_baseline && (candidate.vxy != previous.vxy || candidate.vz != previous.vz ||
                       candidate.heading != previous.heading);
  return NavigationLocalStateResetAssessment{
      .state_lineage_reset = xy_reset || z_reset || velocity_or_heading_reset,
      .frame_compensation_required = timestamp_epoch_reset || xy_reset || z_reset,
  };
}

bool navigationAngularDerivativeConfigIsValid(
    const NavigationAngularDerivativeConfig& config) noexcept {
  return std::isfinite(config.minimum_interval_s) && config.minimum_interval_s > 0.0 &&
         std::isfinite(config.maximum_interval_s) &&
         config.maximum_interval_s >= config.minimum_interval_s &&
         std::isfinite(config.maximum_yaw_rate_radps) &&
         config.maximum_yaw_rate_radps > 0.0 &&
         std::isfinite(config.maximum_yaw_acceleration_radps2) &&
         config.maximum_yaw_acceleration_radps2 > 0.0 &&
         config.timestamp_epoch_admission.require_corroborating_timestamp_for_reset &&
         px4TimestampEpochAdmissionConfigIsValid(config.timestamp_epoch_admission);
}

const char*
navigationAngularUpdateStatusName(const NavigationAngularUpdateStatus status) noexcept {
  switch (status) {
    case NavigationAngularUpdateStatus::kAcceptedSample:
      return "accepted_sample";
    case NavigationAngularUpdateStatus::kAcceptedPublicationFallback:
      return "accepted_publication_fallback";
    case NavigationAngularUpdateStatus::kAcceptedTimestampEpochReset:
      return "accepted_timestamp_epoch_reset";
    case NavigationAngularUpdateStatus::kIdempotentSourceIdentityDuplicate:
      return "idempotent_source_identity_duplicate";
    case NavigationAngularUpdateStatus::kRejectedInvalidConfiguration:
      return "invalid_configuration";
    case NavigationAngularUpdateStatus::kRejectedMissingTimestamp:
      return "missing_timestamp";
    case NavigationAngularUpdateStatus::kRejectedMissingReceiveTimestamp:
      return "missing_receive_timestamp";
    case NavigationAngularUpdateStatus::kRejectedSourceIdentityConflict:
      return "source_identity_conflict";
    case NavigationAngularUpdateStatus::kRejectedSourceIdentityQuarantined:
      return "source_identity_quarantined";
    case NavigationAngularUpdateStatus::kRejectedNonmonotonicReceiveTimestamp:
      return "nonmonotonic_receive_timestamp";
    case NavigationAngularUpdateStatus::kRejectedNonmonotonicSampleTimestamp:
      return "nonmonotonic_sample_timestamp";
    case NavigationAngularUpdateStatus::kRejectedNonmonotonicPublicationTimestamp:
      return "nonmonotonic_publication_timestamp";
    case NavigationAngularUpdateStatus::kRejectedTimestampEpochResetPending:
      return "timestamp_epoch_reset_pending";
    case NavigationAngularUpdateStatus::kRejectedTimestampReacquisitionPending:
      return "timestamp_reacquisition_pending";
    case NavigationAngularUpdateStatus::kRejectedImplausibleTimestampProgress:
      return "implausible_timestamp_progress";
  }
  return "unknown";
}

bool navigationAngularUpdateAccepted(
    const NavigationAngularUpdateStatus status) noexcept {
  return status == NavigationAngularUpdateStatus::kAcceptedSample ||
         status == NavigationAngularUpdateStatus::kAcceptedPublicationFallback ||
         status == NavigationAngularUpdateStatus::kAcceptedTimestampEpochReset;
}

NavigationAngularDerivativeEstimator::NavigationAngularDerivativeEstimator(
    const NavigationAngularDerivativeConfig& config) noexcept
    : config_{config} {
}

NavigationAngularDerivativeEstimate NavigationAngularDerivativeEstimator::observe(
    const NavigationAngularObservation& observation) noexcept {
  NavigationAngularDerivativeEstimate result;
  if (!navigationAngularDerivativeConfigIsValid(config_)) {
    resetDerivativeChain(NavigationTimestampProvenance::kNone);
    result.status = NavigationAngularUpdateStatus::kRejectedInvalidConfiguration;
    return result;
  }

  const bool has_sample_timestamp = observation.sample_timestamp_us != 0U;
  const bool has_publication_timestamp = observation.publication_timestamp_us != 0U;
  if (!has_sample_timestamp && !has_publication_timestamp) {
    result.status = NavigationAngularUpdateStatus::kRejectedMissingTimestamp;
    return result;
  }
  if (observation.receive_timestamp_ns <= 0) {
    result.status = NavigationAngularUpdateStatus::kRejectedMissingReceiveTimestamp;
    return result;
  }

  const bool same_source_identity =
      accepted_source_identity_valid_ &&
      observation.sample_timestamp_us == accepted_sample_timestamp_us_ &&
      observation.publication_timestamp_us == accepted_publication_timestamp_us_;
  if (same_source_identity) {
    if (accepted_source_identity_conflicted_) {
      result.status = NavigationAngularUpdateStatus::kRejectedSourceIdentityQuarantined;
      result.source_identity_conflicted = true;
      return result;
    }
    if (observation.source_payload_fingerprint ==
        accepted_source_payload_fingerprint_) {
      result.status = NavigationAngularUpdateStatus::kIdempotentSourceIdentityDuplicate;
      return result;
    }
    accepted_source_identity_conflicted_ = true;
    resetDerivativeChain(NavigationTimestampProvenance::kNone);
    result.status = NavigationAngularUpdateStatus::kRejectedSourceIdentityConflict;
    result.source_identity_conflict = true;
    result.source_identity_conflicted = true;
    return result;
  }

  const Px4TimestampEpochAdmissionResult timestamp_admission = admitPx4TimestampEpoch(
      config_.timestamp_epoch_admission, timestamp_epoch_admission_state_,
      Px4TimestampEpochObservation{
          .primary_timestamp_us = observation.publication_timestamp_us,
          .corroborating_timestamp_us = observation.sample_timestamp_us,
          .receive_timestamp_ns = observation.receive_timestamp_ns,
      });
  timestamp_epoch_admission_state_ = timestamp_admission.next_state;
  if (!px4TimestampEpochAdmissionAccepted(timestamp_admission.status)) {
    if (timestamp_admission.status ==
            Px4TimestampEpochAdmissionStatus::kRejectedInvalidConfiguration ||
        timestamp_admission.status ==
            Px4TimestampEpochAdmissionStatus::kRejectedInvalidState) {
      resetDerivativeChain(NavigationTimestampProvenance::kNone);
    }
    switch (timestamp_admission.status) {
      case Px4TimestampEpochAdmissionStatus::kPendingEpochReset:
        result.status =
            NavigationAngularUpdateStatus::kRejectedTimestampEpochResetPending;
        break;
      case Px4TimestampEpochAdmissionStatus::kPendingForwardReacquisition:
        result.status =
            NavigationAngularUpdateStatus::kRejectedTimestampReacquisitionPending;
        break;
      case Px4TimestampEpochAdmissionStatus::kRejectedMissingTimestamp:
        result.status = NavigationAngularUpdateStatus::kRejectedMissingTimestamp;
        break;
      case Px4TimestampEpochAdmissionStatus::kRejectedMissingReceiveTimestamp:
        result.status = NavigationAngularUpdateStatus::kRejectedMissingReceiveTimestamp;
        break;
      case Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicReceiveTimestamp:
        result.status =
            NavigationAngularUpdateStatus::kRejectedNonmonotonicReceiveTimestamp;
        break;
      case Px4TimestampEpochAdmissionStatus::kRejectedNonmonotonicPrimaryTimestamp:
        result.status =
            NavigationAngularUpdateStatus::kRejectedNonmonotonicPublicationTimestamp;
        break;
      case Px4TimestampEpochAdmissionStatus::
          kRejectedNonmonotonicCorroboratingTimestamp:
        result.status =
            NavigationAngularUpdateStatus::kRejectedNonmonotonicSampleTimestamp;
        break;
      case Px4TimestampEpochAdmissionStatus::kRejectedImplausibleTimestampProgress:
        result.status =
            NavigationAngularUpdateStatus::kRejectedImplausibleTimestampProgress;
        break;
      case Px4TimestampEpochAdmissionStatus::kRejectedInvalidConfiguration:
      case Px4TimestampEpochAdmissionStatus::kRejectedInvalidState:
      case Px4TimestampEpochAdmissionStatus::kAcceptedInitial:
      case Px4TimestampEpochAdmissionStatus::kAcceptedNewer:
      case Px4TimestampEpochAdmissionStatus::kAcceptedEpochReset:
      case Px4TimestampEpochAdmissionStatus::kAcceptedForwardReacquisition:
        result.status = NavigationAngularUpdateStatus::kRejectedInvalidConfiguration;
        break;
    }
    return result;
  }

  accepted_sample_timestamp_us_ = observation.sample_timestamp_us;
  accepted_publication_timestamp_us_ = observation.publication_timestamp_us;
  accepted_source_payload_fingerprint_ = observation.source_payload_fingerprint;
  accepted_source_identity_valid_ = true;
  accepted_source_identity_conflicted_ = false;

  if (timestamp_admission.epoch_reset) {
    result.status = NavigationAngularUpdateStatus::kAcceptedTimestampEpochReset;
    result.provenance = NavigationTimestampProvenance::kSample;
    result.source_timestamp_us = observation.sample_timestamp_us;
    result.timestamp_epoch_reset = true;
    resetDerivativeChain(NavigationTimestampProvenance::kSample);
    if (observation.angular_state_authoritative && std::isfinite(observation.yaw_rad)) {
      seedSample(observation);
    }
    return result;
  }

  if (!has_sample_timestamp) {
    resetDerivativeChain(NavigationTimestampProvenance::kPublication);
    result.status = NavigationAngularUpdateStatus::kAcceptedPublicationFallback;
    result.provenance = NavigationTimestampProvenance::kPublication;
    result.source_timestamp_us = observation.publication_timestamp_us;
    return result;
  }

  result.status = NavigationAngularUpdateStatus::kAcceptedSample;
  result.provenance = NavigationTimestampProvenance::kSample;
  result.source_timestamp_us = observation.sample_timestamp_us;
  if (active_provenance_ != NavigationTimestampProvenance::kSample) {
    resetDerivativeChain(NavigationTimestampProvenance::kSample);
  }
  if (!observation.angular_state_authoritative || !std::isfinite(observation.yaw_rad)) {
    resetDerivativeChain(NavigationTimestampProvenance::kSample);
    return result;
  }
  if (!previous_sample_valid_) {
    seedSample(observation);
    return result;
  }

  const std::uint64_t interval_us =
      observation.sample_timestamp_us - previous_sample_timestamp_us_;
  const double interval_s = static_cast<double>(interval_us) * kSecondsPerMicrosecond;
  const bool state_lineage_reset =
      observation.xy_reset_counter != previous_xy_reset_counter_ ||
      observation.z_reset_counter != previous_z_reset_counter_ ||
      observation.vxy_reset_counter != previous_vxy_reset_counter_ ||
      observation.vz_reset_counter != previous_vz_reset_counter_ ||
      observation.heading_reset_counter != previous_heading_reset_counter_;
  if (state_lineage_reset || !std::isfinite(interval_s) ||
      interval_s < config_.minimum_interval_s ||
      interval_s > config_.maximum_interval_s) {
    seedSample(observation);
    return result;
  }

  const double yaw_delta_rad =
      std::remainder(observation.yaw_rad - previous_yaw_rad_, 2.0 * std::numbers::pi);
  const double yaw_rate_radps = yaw_delta_rad / interval_s;
  if (!withinPhysicalLimit(yaw_rate_radps, config_.maximum_yaw_rate_radps)) {
    seedSample(observation);
    return result;
  }

  result.yaw_rate_radps = static_cast<float>(yaw_rate_radps);
  result.yaw_rate_authoritative = std::isfinite(result.yaw_rate_radps);
  if (!result.yaw_rate_authoritative) {
    seedSample(observation);
    return result;
  }

  std::uint64_t current_rate_midpoint_twice_us{0U};
  const bool current_midpoint_valid =
      midpointTwice(previous_sample_timestamp_us_, observation.sample_timestamp_us,
                    current_rate_midpoint_twice_us);
  if (previous_rate_valid_ && current_midpoint_valid &&
      current_rate_midpoint_twice_us > previous_rate_midpoint_twice_us_) {
    const double acceleration_interval_s =
        static_cast<double>(current_rate_midpoint_twice_us -
                            previous_rate_midpoint_twice_us_) *
        0.5 * kSecondsPerMicrosecond;
    const double yaw_acceleration_radps2 =
        (yaw_rate_radps - previous_yaw_rate_radps_) / acceleration_interval_s;
    if (withinPhysicalLimit(yaw_acceleration_radps2,
                            config_.maximum_yaw_acceleration_radps2)) {
      result.yaw_acceleration_radps2 = static_cast<float>(yaw_acceleration_radps2);
      result.yaw_acceleration_authoritative =
          std::isfinite(result.yaw_acceleration_radps2);
    }
  }

  previous_rate_valid_ = current_midpoint_valid;
  if (current_midpoint_valid) {
    previous_rate_midpoint_twice_us_ = current_rate_midpoint_twice_us;
    previous_yaw_rate_radps_ = yaw_rate_radps;
  }
  previous_sample_timestamp_us_ = observation.sample_timestamp_us;
  previous_yaw_rad_ = observation.yaw_rad;
  previous_xy_reset_counter_ = observation.xy_reset_counter;
  previous_z_reset_counter_ = observation.z_reset_counter;
  previous_vxy_reset_counter_ = observation.vxy_reset_counter;
  previous_vz_reset_counter_ = observation.vz_reset_counter;
  previous_heading_reset_counter_ = observation.heading_reset_counter;
  previous_sample_valid_ = true;
  return result;
}

void NavigationAngularDerivativeEstimator::resetDerivativeChain(
    const NavigationTimestampProvenance provenance) noexcept {
  active_provenance_ = provenance;
  previous_sample_timestamp_us_ = 0U;
  previous_yaw_rad_ = 0.0;
  previous_xy_reset_counter_ = 0U;
  previous_z_reset_counter_ = 0U;
  previous_vxy_reset_counter_ = 0U;
  previous_vz_reset_counter_ = 0U;
  previous_heading_reset_counter_ = 0U;
  previous_sample_valid_ = false;
  previous_rate_midpoint_twice_us_ = 0U;
  previous_yaw_rate_radps_ = 0.0;
  previous_rate_valid_ = false;
}

void NavigationAngularDerivativeEstimator::seedSample(
    const NavigationAngularObservation& observation) noexcept {
  previous_sample_timestamp_us_ = observation.sample_timestamp_us;
  previous_yaw_rad_ = observation.yaw_rad;
  previous_xy_reset_counter_ = observation.xy_reset_counter;
  previous_z_reset_counter_ = observation.z_reset_counter;
  previous_vxy_reset_counter_ = observation.vxy_reset_counter;
  previous_vz_reset_counter_ = observation.vz_reset_counter;
  previous_heading_reset_counter_ = observation.heading_reset_counter;
  previous_sample_valid_ = true;
  previous_rate_midpoint_twice_us_ = 0U;
  previous_yaw_rate_radps_ = 0.0;
  previous_rate_valid_ = false;
}

} // namespace drone_city_nav
