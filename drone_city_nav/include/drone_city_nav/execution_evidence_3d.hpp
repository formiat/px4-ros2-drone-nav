#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

namespace drone_city_nav {

struct ExecutionValidationPolicyId3D {
  std::uint64_t value{0U};

  [[nodiscard]] bool valid() const noexcept {
    return value != 0U;
  }

  friend bool operator==(const ExecutionValidationPolicyId3D&,
                         const ExecutionValidationPolicyId3D&) = default;
};

class VersionedExecutionValidationPolicy3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const VersionedExecutionValidationPolicy3D>
  capture(FlightEnvelopeConfig flight_envelope, mppi::DynamicsConfig dynamics,
          mppi::AltitudeEnvelopeConfig altitude_envelope,
          SweptFootprintConfig swept_footprint,
          double latest_lidar_maximum_age_ms = 1000.0,
          double execution_input_maximum_pose_age_ms = 1000.0,
          double execution_input_maximum_control_age_ms = 1000.0,
          bool route_cross_track_constraints_enabled = false,
          bool latest_lidar_freshness_required = true,
          bool route_tracking_tube_constraints_enabled = true);

  [[nodiscard]] const FlightEnvelopeConfig& flightEnvelope() const noexcept;
  [[nodiscard]] const mppi::DynamicsConfig& dynamics() const noexcept;
  [[nodiscard]] const mppi::AltitudeEnvelopeConfig& altitudeEnvelope() const noexcept;
  [[nodiscard]] const SweptFootprintConfig& sweptFootprint() const noexcept;
  [[nodiscard]] double latestLidarMaximumAgeMs() const noexcept;
  [[nodiscard]] double executionInputMaximumPoseAgeMs() const noexcept;
  [[nodiscard]] double executionInputMaximumControlAgeMs() const noexcept;
  [[nodiscard]] bool routeCrossTrackConstraintsEnabled() const noexcept;
  [[nodiscard]] bool latestLidarFreshnessRequired() const noexcept;
  [[nodiscard]] bool routeTrackingTubeConstraintsEnabled() const noexcept;
  [[nodiscard]] ExecutionValidationPolicyId3D policyId() const noexcept;
  [[nodiscard]] std::uint64_t contentFingerprint() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  VersionedExecutionValidationPolicy3D(
      CaptureToken, FlightEnvelopeConfig flight_envelope, mppi::DynamicsConfig dynamics,
      mppi::AltitudeEnvelopeConfig altitude_envelope,
      SweptFootprintConfig swept_footprint, double latest_lidar_maximum_age_ms,
      double execution_input_maximum_pose_age_ms,
      double execution_input_maximum_control_age_ms,
      bool route_cross_track_constraints_enabled, bool latest_lidar_freshness_required,
      bool route_tracking_tube_constraints_enabled);

private:
  FlightEnvelopeConfig flight_envelope_{};
  mppi::DynamicsConfig dynamics_{};
  mppi::AltitudeEnvelopeConfig altitude_envelope_{};
  SweptFootprintConfig swept_footprint_{};
  double latest_lidar_maximum_age_ms_{0.0};
  double execution_input_maximum_pose_age_ms_{0.0};
  double execution_input_maximum_control_age_ms_{0.0};
  bool route_cross_track_constraints_enabled_{false};
  bool latest_lidar_freshness_required_{true};
  bool route_tracking_tube_constraints_enabled_{true};
  std::uint64_t content_fingerprint_{0U};
};

enum class ExecutionStateFieldProvenance3D : std::uint8_t {
  kUnknown,
  kSourceSample,
  kEffectiveTimePrediction,
  kAssumedZero,
};

struct ExecutionStateProvenance3D {
  ExecutionStateFieldProvenance3D x{ExecutionStateFieldProvenance3D::kUnknown};
  ExecutionStateFieldProvenance3D y{ExecutionStateFieldProvenance3D::kUnknown};
  ExecutionStateFieldProvenance3D z{ExecutionStateFieldProvenance3D::kUnknown};
  ExecutionStateFieldProvenance3D vx{ExecutionStateFieldProvenance3D::kUnknown};
  ExecutionStateFieldProvenance3D vy{ExecutionStateFieldProvenance3D::kUnknown};
  ExecutionStateFieldProvenance3D vz{ExecutionStateFieldProvenance3D::kUnknown};
  ExecutionStateFieldProvenance3D yaw{ExecutionStateFieldProvenance3D::kUnknown};
  ExecutionStateFieldProvenance3D yaw_rate{ExecutionStateFieldProvenance3D::kUnknown};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] bool fullyAuthoritative() const noexcept;
  [[nodiscard]] bool yawAuthoritative() const noexcept;
};

enum class ExecutionPreviousControlEvidenceSource3D : std::uint8_t {
  kUnknown,
  kOffboardFeedback,
  kMeasuredAcceleration,
  kAssumedZero,
  kEngineFallback,
};

enum class ExecutionInputPurpose3D : std::uint8_t {
  kGeneralExecution,
  kStationaryCaptureRearm,
};

struct ExecutionInputCapture3D {
  std::uint64_t capture_sequence{0U};
  std::uint64_t pose_revision{0U};
  std::uint64_t pose_source_timestamp_us{0U};
  std::int64_t pose_receive_stamp_ns{0};
  std::int64_t effective_stamp_ns{0};
  mppi::State state{};
  bool full_state_authoritative{false};
  ExecutionStateProvenance3D state_provenance{};
  ExecutionInputPurpose3D purpose{ExecutionInputPurpose3D::kGeneralExecution};
  mppi::Control previous_control{};
  ExecutionPreviousControlEvidenceSource3D previous_control_source{
      ExecutionPreviousControlEvidenceSource3D::kUnknown};
  std::uint64_t previous_control_source_producer_instance_id{0U};
  std::uint64_t previous_control_source_sequence{0U};
  std::int64_t previous_control_source_stamp_ns{0};
  std::int64_t previous_control_receive_stamp_ns{0};
};

class VersionedExecutionInput3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const VersionedExecutionInput3D>
  capture(const ExecutionInputCapture3D& capture);

  [[nodiscard]] std::uint64_t captureSequence() const noexcept;
  [[nodiscard]] std::uint64_t poseRevision() const noexcept;
  [[nodiscard]] std::uint64_t poseSourceTimestampUs() const noexcept;
  [[nodiscard]] std::int64_t poseReceiveStampNs() const noexcept;
  [[nodiscard]] std::int64_t effectiveStampNs() const noexcept;
  [[nodiscard]] const mppi::State& state() const noexcept;
  [[nodiscard]] bool fullStateAuthoritative() const noexcept;
  [[nodiscard]] const ExecutionStateProvenance3D& stateProvenance() const noexcept;
  [[nodiscard]] bool nominalStateAuthoritative() const noexcept;
  [[nodiscard]] bool stationaryCaptureStateAuthoritative() const noexcept;
  [[nodiscard]] ExecutionInputPurpose3D purpose() const noexcept;
  [[nodiscard]] const mppi::Control& previousControl() const noexcept;
  [[nodiscard]] ExecutionPreviousControlEvidenceSource3D
  previousControlSource() const noexcept;
  [[nodiscard]] std::uint64_t previousControlSourceProducerInstanceId() const noexcept;
  [[nodiscard]] std::uint64_t previousControlSourceSequence() const noexcept;
  [[nodiscard]] std::int64_t previousControlSourceStampNs() const noexcept;
  [[nodiscard]] std::int64_t previousControlReceiveStampNs() const noexcept;
  [[nodiscard]] std::uint64_t contentFingerprint() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  VersionedExecutionInput3D(CaptureToken, ExecutionInputCapture3D capture);

private:
  ExecutionInputCapture3D capture_{};
  std::uint64_t content_fingerprint_{0U};
};

// Publication must recheck both captured pose and previous-control receipt
// against its actual commit time. Certification at the planning tick's
// effective time does not make a delayed publication current.
[[nodiscard]] bool
executionInputFreshAt(const VersionedExecutionInput3D& input,
                      const VersionedExecutionValidationPolicy3D& policy,
                      std::int64_t validation_stamp_ns) noexcept;

struct LatestLidarEvidenceCapture3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::uint64_t pose_generation{0U};
  // Acquisition time is producer lineage. It may be ahead of the consumer's
  // local clock; receive time is the authoritative local freshness fallback.
  std::int64_t acquisition_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::size_t source_beam_count{0U};
  std::size_t invalid_beam_count{0U};
  std::vector<Point3> hit_points_map_m;
};

enum class LatestLidarEvidenceUpdateStatus3D : std::uint8_t {
  kAcceptedInitial,
  kAcceptedNewer,
  kAcceptedAcquisitionEpochReset,
  kAcceptedProducerHandoff,
  kIdempotentDuplicate,
  kPendingProducerHandoff,
  kRejectedInvalid,
  kRejectedStaleCandidate,
  kRejectedRegression,
  kRejectedIdentityConflict,
  kRejectedUnauthenticatedProducer,
  kRejectedRetiredProducer,
  kRejectedHandoffCapacity,
  kRejectedClaimReplay,
  kRejectedClaimCapacity,
};

struct LatestLidarEvidenceFreshness3D {
  double age_ms{-1.0};
  bool fresh{false};
  bool receive_time_fallback{false};
};

struct LatestLidarEvidenceId3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::uint64_t pose_generation{0U};
  std::int64_t acquisition_stamp_ns{0};

  [[nodiscard]] bool valid() const noexcept {
    return producer_instance_id != 0U && sequence != 0U && pose_generation != 0U &&
           acquisition_stamp_ns > 0;
  }

  friend bool operator==(const LatestLidarEvidenceId3D&,
                         const LatestLidarEvidenceId3D&) = default;
};

class VersionedLatestLidarEvidence3D final {
private:
  struct CaptureToken final {};

public:
  [[nodiscard]] static std::shared_ptr<const VersionedLatestLidarEvidence3D>
  capture(LatestLidarEvidenceCapture3D capture);

  [[nodiscard]] std::uint64_t producerInstanceId() const noexcept;
  [[nodiscard]] std::uint64_t sequence() const noexcept;
  [[nodiscard]] std::uint64_t poseGeneration() const noexcept;
  [[nodiscard]] std::int64_t acquisitionStampNs() const noexcept;
  [[nodiscard]] std::int64_t receiveStampNs() const noexcept;
  [[nodiscard]] std::size_t sourceBeamCount() const noexcept;
  [[nodiscard]] std::size_t invalidBeamCount() const noexcept;
  [[nodiscard]] const std::vector<Point3>& hitPointsMapM() const noexcept;
  [[nodiscard]] LatestLidarEvidenceId3D evidenceId() const noexcept;
  // Fingerprint of producer-owned content. Local receipt time is deliberately
  // excluded so retransmission cannot rejuvenate an observation.
  [[nodiscard]] std::uint64_t sourceContentFingerprint() const noexcept;
  [[nodiscard]] std::uint64_t contentFingerprint() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

  VersionedLatestLidarEvidence3D(CaptureToken, LatestLidarEvidenceCapture3D capture);

private:
  LatestLidarEvidenceCapture3D capture_{};
  std::uint64_t source_content_fingerprint_{0U};
  std::uint64_t content_fingerprint_{0U};
  bool valid_{false};
};

inline constexpr std::size_t kLatestLidarRetiredProducerCapacity3D{16U};
inline constexpr std::size_t kLatestLidarProspectiveClaimCapacity3D{16U};

// The transport computes the canonical fingerprint over the unvalidated wire
// fields. Claiming this identity before interpreting the payload prevents a
// malformed or stale replay from acquiring a newer local receipt time.
struct LatestLidarEvidenceIdentityClaim3D {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t sequence{0U};
  std::uint64_t raw_wire_fingerprint{0U};
  std::int64_t first_receive_stamp_ns{0};
};

struct LatestLidarEvidenceProspectiveClaim3D {
  LatestLidarEvidenceIdentityClaim3D identity{};
  bool conflicted{false};
};

enum class LatestLidarEvidenceClaimStatus3D : std::uint8_t {
  kClaimed,
  kInstalledReplay,
  kPendingReplay,
  kRejectedReplay,
  kRejectedRegression,
  kRejectedIdentityConflict,
  kRejectedCapacity,
  kRejectedInvalid,
};

// Consumer-owned authority state. Retired producer identifiers are never
// evicted: exhausting the bounded tombstone set fails closed instead of making
// an old producer replay eligible again.
struct LatestLidarEvidenceAdmissionState3D {
  std::uint64_t current_producer_instance_id{0U};
  std::uint64_t current_sequence{0U};
  std::uint64_t current_raw_wire_fingerprint{0U};
  std::int64_t current_first_receive_stamp_ns{0};
  // Same producer/sequence with a different raw-wire fingerprint quarantines the
  // resident identity until strictly newer evidence resolves the ambiguity.
  bool current_identity_conflicted{false};
  std::array<std::uint64_t, kLatestLidarRetiredProducerCapacity3D>
      retired_producer_instance_ids{};
  std::size_t retired_producer_count{0U};
  std::array<LatestLidarEvidenceProspectiveClaim3D,
             kLatestLidarProspectiveClaimCapacity3D>
      prospective_claims{};
  std::size_t prospective_claim_count{0U};
  // This latch is deliberately irreversible: after bounded identity memory is
  // exhausted, accepting any later tuple could reopen a forgotten replay.
  bool prospective_claim_capacity_exhausted{false};
  std::uint64_t pending_producer_instance_id{0U};
  std::uint64_t pending_sequence{0U};
  std::uint64_t pending_pose_generation{0U};
  std::uint64_t pending_source_content_fingerprint{0U};
  std::int64_t pending_receive_stamp_ns{0};
  std::size_t pending_confirmation_count{0U};
  bool pending_identity_conflicted{false};
};

struct LatestLidarEvidenceClaimResult3D {
  LatestLidarEvidenceAdmissionState3D next_state{};
  LatestLidarEvidenceIdentityClaim3D claim{};
  LatestLidarEvidenceClaimStatus3D status{
      LatestLidarEvidenceClaimStatus3D::kRejectedInvalid};
  bool assess_candidate{false};
  bool authority_quarantine_opened{false};
};

struct LatestLidarEvidenceAdmissionResult3D {
  LatestLidarEvidenceAdmissionState3D next_state{};
  LatestLidarEvidenceUpdateStatus3D status{
      LatestLidarEvidenceUpdateStatus3D::kRejectedInvalid};
  bool install_candidate{false};
  bool acquisition_epoch_reset{false};
  bool producer_handoff{false};
  bool current_identity_conflict{false};
};

[[nodiscard]] LatestLidarEvidenceUpdateStatus3D assessLatestLidarEvidenceUpdate3D(
    const VersionedLatestLidarEvidence3D* current,
    const VersionedLatestLidarEvidence3D& candidate) noexcept;

// This is phase one of admission and must run before frame, count, point,
// freshness, acquisition-epoch, probation, or owner checks.
[[nodiscard]] LatestLidarEvidenceClaimResult3D
claimLatestLidarEvidenceIdentity3D(const LatestLidarEvidenceAdmissionState3D& state,
                                   const VersionedLatestLidarEvidence3D* current,
                                   LatestLidarEvidenceIdentityClaim3D claim) noexcept;

// Only the caller that received assess_candidate=true may finalize its claim.
[[nodiscard]] LatestLidarEvidenceAdmissionResult3D
admitClaimedLatestLidarEvidence3D(const LatestLidarEvidenceAdmissionState3D& state,
                                  const VersionedLatestLidarEvidence3D* current,
                                  const VersionedLatestLidarEvidence3D& candidate,
                                  const LatestLidarEvidenceIdentityClaim3D& claim,
                                  std::int64_t now_ns, double maximum_age_ms) noexcept;

// A new producer may take authority only after the resident evidence is stale
// and two fresh, strictly ordered observations within one freshness window
// establish the new producer epoch. The old producer is tombstoned on handoff.
[[nodiscard]] LatestLidarEvidenceAdmissionResult3D
admitLatestLidarEvidence3D(const LatestLidarEvidenceAdmissionState3D& state,
                           const VersionedLatestLidarEvidence3D* current,
                           const VersionedLatestLidarEvidence3D& candidate,
                           std::int64_t now_ns, double maximum_age_ms) noexcept;

[[nodiscard]] std::string_view latestLidarEvidenceUpdateStatus3DName(
    LatestLidarEvidenceUpdateStatus3D status) noexcept;

[[nodiscard]] std::string_view
latestLidarEvidenceClaimStatus3DName(LatestLidarEvidenceClaimStatus3D status) noexcept;

[[nodiscard]] bool latestLidarEvidenceAuthorityQuarantined3D(
    const LatestLidarEvidenceAdmissionState3D& state) noexcept;

[[nodiscard]] LatestLidarEvidenceFreshness3D
assessLatestLidarEvidenceFreshness3D(const VersionedLatestLidarEvidence3D& evidence,
                                     std::int64_t now_ns,
                                     double maximum_age_ms) noexcept;

} // namespace drone_city_nav
