#pragma once

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

struct Px4TimestampEpochAdmissionConfig {
  double minimum_epoch_reset_backward_jump_s{1.0};
  double maximum_epoch_confirmation_interval_s{0.25};
  double maximum_source_timestamp_lead_s{0.25};
  double maximum_post_reset_unprobated_receive_gap_s{0.5};
  std::size_t epoch_reset_confirmation_samples{3U};
  bool require_corroborating_timestamp_for_reset{false};
};

[[nodiscard]] bool px4TimestampEpochAdmissionConfigIsValid(
    const Px4TimestampEpochAdmissionConfig& config) noexcept;

enum class Px4TimestampEpochAdmissionStatus : std::uint8_t {
  kAcceptedInitial,
  kAcceptedNewer,
  kAcceptedEpochReset,
  kAcceptedForwardReacquisition,
  kPendingEpochReset,
  kPendingForwardReacquisition,
  kRejectedInvalidConfiguration,
  kRejectedInvalidState,
  kRejectedMissingTimestamp,
  kRejectedMissingReceiveTimestamp,
  kRejectedNonmonotonicReceiveTimestamp,
  kRejectedNonmonotonicPrimaryTimestamp,
  kRejectedNonmonotonicCorroboratingTimestamp,
  kRejectedImplausibleTimestampProgress,
};

[[nodiscard]] const char*
px4TimestampEpochAdmissionStatusName(Px4TimestampEpochAdmissionStatus status) noexcept;

[[nodiscard]] bool
px4TimestampEpochAdmissionAccepted(Px4TimestampEpochAdmissionStatus status) noexcept;

struct Px4TimestampEpochObservation {
  std::uint64_t primary_timestamp_us{0U};
  std::uint64_t corroborating_timestamp_us{0U};
  std::int64_t receive_timestamp_ns{0};
};

enum class Px4TimestampEpochAdmissionPendingKind : std::uint8_t {
  kNone,
  kEpochReset,
  kForwardReacquisition,
};

struct Px4TimestampEpochAdmissionState {
  std::uint64_t primary_timestamp_high_water_us{0U};
  std::uint64_t corroborating_timestamp_high_water_us{0U};
  std::int64_t primary_timestamp_receive_ns{0};
  std::int64_t corroborating_timestamp_receive_ns{0};
  std::int64_t last_observation_receive_ns{0};
  bool post_reset_replay_guard_active{false};
  std::uint64_t pending_primary_timestamp_us{0U};
  std::uint64_t pending_corroborating_timestamp_us{0U};
  std::int64_t pending_receive_timestamp_ns{0};
  std::size_t pending_confirmation_count{0U};
  Px4TimestampEpochAdmissionPendingKind pending_kind{
      Px4TimestampEpochAdmissionPendingKind::kNone};
};

struct Px4TimestampEpochAdmissionResult {
  Px4TimestampEpochAdmissionState next_state{};
  Px4TimestampEpochAdmissionStatus status{
      Px4TimestampEpochAdmissionStatus::kRejectedMissingTimestamp};
  bool epoch_reset{false};
  bool forward_reacquisition{false};
};

// Admits one or two PX4 boot-relative timestamp streams as one epoch. A large
// backward jump enters multi-sample probation; ordinary reorder remains
// rejected. After a confirmed reset, any forward jump following a long receive
// gap also needs bounded multi-sample reacquisition, so a lone delayed sample
// from the retired epoch cannot regain authority.
[[nodiscard]] Px4TimestampEpochAdmissionResult
admitPx4TimestampEpoch(const Px4TimestampEpochAdmissionConfig& config,
                       const Px4TimestampEpochAdmissionState& state,
                       const Px4TimestampEpochObservation& observation) noexcept;

struct NavigationAngularDerivativeConfig {
  double minimum_interval_s{0.001};
  double maximum_interval_s{0.1};
  double maximum_yaw_rate_radps{1.5};
  double maximum_yaw_acceleration_radps2{2.0};
  Px4TimestampEpochAdmissionConfig timestamp_epoch_admission{
      .require_corroborating_timestamp_for_reset = true,
  };
};

struct NavigationLocalStateResetCounters {
  std::uint8_t xy{0U};
  std::uint8_t z{0U};
  std::uint8_t vxy{0U};
  std::uint8_t vz{0U};
  std::uint8_t heading{0U};
};

struct NavigationLocalStateResetAssessment {
  bool state_lineage_reset{false};
  bool frame_compensation_required{false};
};

// Initial counters establish a baseline. Later XY/Z resets and a confirmed PX4
// timestamp epoch reset require coordinated planner/offboard/world frame
// compensation; velocity and heading resets only break the dynamic lineage.
[[nodiscard]] NavigationLocalStateResetAssessment
assessNavigationLocalStateReset(const NavigationLocalStateResetCounters& previous,
                                const NavigationLocalStateResetCounters& candidate,
                                bool has_baseline, bool timestamp_epoch_reset) noexcept;

[[nodiscard]] bool navigationAngularDerivativeConfigIsValid(
    const NavigationAngularDerivativeConfig& config) noexcept;

enum class NavigationTimestampProvenance : std::uint8_t {
  kNone,
  kSample,
  kPublication,
};

enum class NavigationAngularUpdateStatus : std::uint8_t {
  kAcceptedSample,
  kAcceptedPublicationFallback,
  kAcceptedTimestampEpochReset,
  kIdempotentSourceIdentityDuplicate,
  kRejectedInvalidConfiguration,
  kRejectedMissingTimestamp,
  kRejectedMissingReceiveTimestamp,
  kRejectedSourceIdentityConflict,
  kRejectedSourceIdentityQuarantined,
  kRejectedNonmonotonicReceiveTimestamp,
  kRejectedNonmonotonicSampleTimestamp,
  kRejectedNonmonotonicPublicationTimestamp,
  kRejectedTimestampEpochResetPending,
  kRejectedTimestampReacquisitionPending,
  kRejectedImplausibleTimestampProgress,
};

[[nodiscard]] const char*
navigationAngularUpdateStatusName(NavigationAngularUpdateStatus status) noexcept;

[[nodiscard]] bool
navigationAngularUpdateAccepted(NavigationAngularUpdateStatus status) noexcept;

struct NavigationAngularObservation {
  std::uint64_t sample_timestamp_us{0U};
  std::uint64_t publication_timestamp_us{0U};
  std::int64_t receive_timestamp_ns{0};
  double yaw_rad{0.0};
  std::uint64_t source_payload_fingerprint{0U};
  std::uint8_t xy_reset_counter{0U};
  std::uint8_t z_reset_counter{0U};
  std::uint8_t vxy_reset_counter{0U};
  std::uint8_t vz_reset_counter{0U};
  std::uint8_t heading_reset_counter{0U};
  bool angular_state_authoritative{false};
};

struct NavigationAngularDerivativeEstimate {
  NavigationAngularUpdateStatus status{
      NavigationAngularUpdateStatus::kRejectedMissingTimestamp};
  NavigationTimestampProvenance provenance{NavigationTimestampProvenance::kNone};
  std::uint64_t source_timestamp_us{0U};
  float yaw_rate_radps{0.0F};
  float yaw_acceleration_radps2{0.0F};
  bool yaw_rate_authoritative{false};
  bool yaw_acceleration_authoritative{false};
  bool timestamp_epoch_reset{false};
  bool source_identity_conflict{false};
  bool source_identity_conflicted{false};
};

// Owns timestamp/identity admission and angular finite-difference state for PX4
// local-position samples. A same-timestamp payload conflict quarantines that
// identity until timestamp admission accepts a different observation.
// Publication timestamps can carry state through the node, but only consecutive
// source-sample timestamps can authorize derivatives.
class NavigationAngularDerivativeEstimator {
public:
  NavigationAngularDerivativeEstimator() = default;
  explicit NavigationAngularDerivativeEstimator(
      const NavigationAngularDerivativeConfig& config) noexcept;

  [[nodiscard]] NavigationAngularDerivativeEstimate
  observe(const NavigationAngularObservation& observation) noexcept;

private:
  void resetDerivativeChain(NavigationTimestampProvenance provenance) noexcept;
  void seedSample(const NavigationAngularObservation& observation) noexcept;

  NavigationAngularDerivativeConfig config_{};
  Px4TimestampEpochAdmissionState timestamp_epoch_admission_state_{};
  NavigationTimestampProvenance active_provenance_{
      NavigationTimestampProvenance::kNone};
  std::uint64_t previous_sample_timestamp_us_{0U};
  double previous_yaw_rad_{0.0};
  std::uint8_t previous_xy_reset_counter_{0U};
  std::uint8_t previous_z_reset_counter_{0U};
  std::uint8_t previous_vxy_reset_counter_{0U};
  std::uint8_t previous_vz_reset_counter_{0U};
  std::uint8_t previous_heading_reset_counter_{0U};
  bool previous_sample_valid_{false};
  std::uint64_t previous_rate_midpoint_twice_us_{0U};
  double previous_yaw_rate_radps_{0.0};
  bool previous_rate_valid_{false};
  std::uint64_t accepted_sample_timestamp_us_{0U};
  std::uint64_t accepted_publication_timestamp_us_{0U};
  std::uint64_t accepted_source_payload_fingerprint_{0U};
  bool accepted_source_identity_valid_{false};
  bool accepted_source_identity_conflicted_{false};
};

} // namespace drone_city_nav
