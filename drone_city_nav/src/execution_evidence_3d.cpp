#include "drone_city_nav/execution_evidence_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "execution_evidence_3d_admission_internal.hpp"
#include "execution_evidence_3d_hash_internal.hpp"

namespace drone_city_nav {
namespace {

using execution_evidence_hash::canonicalDoubleBits;
using execution_evidence_hash::canonicalFloatBits;
using execution_evidence_hash::hashValue;
using execution_evidence_hash::kFnvOffset;

constexpr std::uint64_t kValidationPolicyDomain{0x56504f4c49435933ULL};
constexpr std::uint64_t kExecutionInputDomain{0x45584543494e5033ULL};

[[nodiscard]] bool finiteState(const MotionState3D& state) noexcept {
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.z) &&
         std::isfinite(state.vx) && std::isfinite(state.vy) &&
         std::isfinite(state.vz) && std::isfinite(state.yaw) &&
         std::isfinite(state.yaw_rate);
}

[[nodiscard]] bool finiteControl(const MotionControl3D& control) noexcept {
  return std::isfinite(control.ax) && std::isfinite(control.ay) &&
         std::isfinite(control.az) && std::isfinite(control.yaw_accel);
}

[[nodiscard]] bool validFlightEnvelope(const FlightEnvelopeConfig& config) noexcept {
  return std::isfinite(config.minimum_target_z_m) &&
         std::isfinite(config.maximum_target_z_m) &&
         config.maximum_target_z_m > config.minimum_target_z_m;
}

[[nodiscard]] bool validDynamics(const MotionDynamicsConfig3D& config) noexcept {
  return std::isfinite(config.dt_s) && config.dt_s > 0.0F &&
         std::isfinite(config.linear_drag_1ps) && config.linear_drag_1ps >= 0.0F &&
         std::isfinite(config.maximum_horizontal_acceleration_mps2) &&
         config.maximum_horizontal_acceleration_mps2 > 0.0F &&
         std::isfinite(config.maximum_vertical_acceleration_mps2) &&
         config.maximum_vertical_acceleration_mps2 > 0.0F &&
         std::isfinite(config.maximum_horizontal_speed_mps) &&
         config.maximum_horizontal_speed_mps > 0.0F &&
         std::isfinite(config.maximum_vertical_speed_mps) &&
         config.maximum_vertical_speed_mps > 0.0F &&
         std::isfinite(config.maximum_translational_speed_mps) &&
         config.maximum_translational_speed_mps > 0.0F &&
         std::isfinite(config.maximum_yaw_acceleration_radps2) &&
         config.maximum_yaw_acceleration_radps2 > 0.0F &&
         std::isfinite(config.maximum_yaw_rate_radps) &&
         config.maximum_yaw_rate_radps > 0.0F &&
         std::isfinite(config.maximum_control_jerk_mps3) &&
         config.maximum_control_jerk_mps3 > 0.0F;
}

[[nodiscard]] bool
validAltitudeEnvelope(const MotionAltitudeEnvelopeConfig3D& config,
                      const MotionDynamicsConfig3D& dynamics) noexcept {
  return std::isfinite(config.minimum_z_m) && std::isfinite(config.maximum_z_m) &&
         config.maximum_z_m > config.minimum_z_m &&
         std::isfinite(config.guaranteed_vertical_deceleration_mps2) &&
         config.guaranteed_vertical_deceleration_mps2 > 0.0F &&
         config.guaranteed_vertical_deceleration_mps2 <=
             dynamics.maximum_vertical_acceleration_mps2 &&
         std::isfinite(config.reaction_latency_s) && config.reaction_latency_s >= 0.0F;
}

[[nodiscard]] bool validSweptFootprint(const SweptFootprintConfig& config) noexcept {
  return std::isfinite(config.radius_m) && config.radius_m >= 0.0 &&
         std::isfinite(config.body_radius_m) && config.body_radius_m >= 0.0 &&
         std::isfinite(config.body_lower_extent_m) &&
         config.body_lower_extent_m >= 0.0 &&
         std::isfinite(config.body_upper_extent_m) &&
         config.body_upper_extent_m >= 0.0 && std::isfinite(config.lower_extent_m) &&
         config.lower_extent_m >= 0.0 && std::isfinite(config.upper_extent_m) &&
         config.upper_extent_m >= 0.0 &&
         (config.radius_m == 0.0 ||
          (config.perimeter_samples > 0U && config.radial_rings > 0U)) &&
         config.axial_samples > 0U && std::isfinite(config.sweep_step_m) &&
         config.sweep_step_m > 0.0 &&
         std::isfinite(config.safe_clearance_threshold_m) &&
         config.safe_clearance_threshold_m >= 0.0;
}

[[nodiscard]] bool
validPolicy(const FlightEnvelopeConfig& flight_envelope,
            const MotionDynamicsConfig3D& dynamics,
            const MotionAltitudeEnvelopeConfig3D& altitude_envelope,
            const SweptFootprintConfig& swept_footprint,
            const double latest_lidar_maximum_age_ms,
            const double execution_input_maximum_pose_age_ms,
            const double execution_input_maximum_control_age_ms) noexcept {
  return validFlightEnvelope(flight_envelope) && validDynamics(dynamics) &&
         validAltitudeEnvelope(altitude_envelope, dynamics) &&
         validSweptFootprint(swept_footprint) &&
         std::isfinite(latest_lidar_maximum_age_ms) &&
         latest_lidar_maximum_age_ms > 0.0 &&
         std::isfinite(execution_input_maximum_pose_age_ms) &&
         execution_input_maximum_pose_age_ms > 0.0 &&
         std::isfinite(execution_input_maximum_control_age_ms) &&
         execution_input_maximum_control_age_ms > 0.0;
}

void hashFlightEnvelope(std::uint64_t& hash,
                        const FlightEnvelopeConfig& config) noexcept {
  hashValue(hash, canonicalDoubleBits(config.minimum_target_z_m));
  hashValue(hash, canonicalDoubleBits(config.maximum_target_z_m));
}

void hashDynamics(std::uint64_t& hash, const MotionDynamicsConfig3D& config) noexcept {
  hashValue(hash, canonicalFloatBits(config.dt_s));
  hashValue(hash, canonicalFloatBits(config.linear_drag_1ps));
  hashValue(hash, canonicalFloatBits(config.maximum_horizontal_acceleration_mps2));
  hashValue(hash, canonicalFloatBits(config.maximum_vertical_acceleration_mps2));
  hashValue(hash, canonicalFloatBits(config.maximum_horizontal_speed_mps));
  hashValue(hash, canonicalFloatBits(config.maximum_vertical_speed_mps));
  hashValue(hash, canonicalFloatBits(config.maximum_translational_speed_mps));
  hashValue(hash, canonicalFloatBits(config.maximum_yaw_acceleration_radps2));
  hashValue(hash, canonicalFloatBits(config.maximum_yaw_rate_radps));
  hashValue(hash, canonicalFloatBits(config.maximum_control_jerk_mps3));
}

void hashAltitudeEnvelope(std::uint64_t& hash,
                          const MotionAltitudeEnvelopeConfig3D& config) noexcept {
  hashValue(hash, canonicalFloatBits(config.minimum_z_m));
  hashValue(hash, canonicalFloatBits(config.maximum_z_m));
  hashValue(hash, canonicalFloatBits(config.guaranteed_vertical_deceleration_mps2));
  hashValue(hash, canonicalFloatBits(config.reaction_latency_s));
}

void hashSweptFootprint(std::uint64_t& hash,
                        const SweptFootprintConfig& config) noexcept {
  hashValue(hash, canonicalDoubleBits(config.radius_m));
  hashValue(hash, canonicalDoubleBits(config.body_radius_m));
  hashValue(hash, canonicalDoubleBits(config.body_lower_extent_m));
  hashValue(hash, canonicalDoubleBits(config.body_upper_extent_m));
  hashValue(hash, canonicalDoubleBits(config.lower_extent_m));
  hashValue(hash, canonicalDoubleBits(config.upper_extent_m));
  hashValue(hash, static_cast<std::uint64_t>(config.perimeter_samples));
  hashValue(hash, static_cast<std::uint64_t>(config.radial_rings));
  hashValue(hash, static_cast<std::uint64_t>(config.axial_samples));
  hashValue(hash, canonicalDoubleBits(config.sweep_step_m));
  hashValue(hash, canonicalDoubleBits(config.safe_clearance_threshold_m));
}

[[nodiscard]] std::uint64_t
policyFingerprint(const FlightEnvelopeConfig& flight_envelope,
                  const MotionDynamicsConfig3D& dynamics,
                  const MotionAltitudeEnvelopeConfig3D& altitude_envelope,
                  const SweptFootprintConfig& swept_footprint,
                  const double latest_lidar_maximum_age_ms,
                  const double execution_input_maximum_pose_age_ms,
                  const double execution_input_maximum_control_age_ms,
                  const bool route_cross_track_constraints_enabled,
                  const bool latest_lidar_freshness_required,
                  const bool route_tracking_tube_constraints_enabled,
                  const double route_station_credit_slack_m) noexcept {
  if (!validPolicy(flight_envelope, dynamics, altitude_envelope, swept_footprint,
                   latest_lidar_maximum_age_ms, execution_input_maximum_pose_age_ms,
                   execution_input_maximum_control_age_ms) ||
      !std::isfinite(route_station_credit_slack_m) ||
      route_station_credit_slack_m < 0.0) {
    return 0U;
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kValidationPolicyDomain);
  hashFlightEnvelope(hash, flight_envelope);
  hashDynamics(hash, dynamics);
  hashAltitudeEnvelope(hash, altitude_envelope);
  hashSweptFootprint(hash, swept_footprint);
  hashValue(hash, canonicalDoubleBits(latest_lidar_maximum_age_ms));
  hashValue(hash, canonicalDoubleBits(execution_input_maximum_pose_age_ms));
  hashValue(hash, canonicalDoubleBits(execution_input_maximum_control_age_ms));
  hashValue(hash, route_cross_track_constraints_enabled ? 1U : 0U);
  hashValue(hash, latest_lidar_freshness_required ? 1U : 0U);
  hashValue(hash, route_tracking_tube_constraints_enabled ? 1U : 0U);
  hashValue(hash, canonicalDoubleBits(route_station_credit_slack_m));
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] bool
knownStateFieldProvenance(const ExecutionStateFieldProvenance3D provenance) noexcept {
  switch (provenance) {
    case ExecutionStateFieldProvenance3D::kUnknown:
    case ExecutionStateFieldProvenance3D::kSourceSample:
    case ExecutionStateFieldProvenance3D::kEffectiveTimePrediction:
    case ExecutionStateFieldProvenance3D::kAssumedZero:
      return true;
  }
  return false;
}

[[nodiscard]] bool authoritativeStateFieldProvenance(
    const ExecutionStateFieldProvenance3D provenance) noexcept {
  return provenance == ExecutionStateFieldProvenance3D::kSourceSample ||
         provenance == ExecutionStateFieldProvenance3D::kEffectiveTimePrediction;
}

template<typename Predicate>
[[nodiscard]] bool everyStateField(const ExecutionStateProvenance3D& provenance,
                                   Predicate predicate) noexcept {
  return predicate(provenance.x) && predicate(provenance.y) &&
         predicate(provenance.z) && predicate(provenance.vx) &&
         predicate(provenance.vy) && predicate(provenance.vz) &&
         predicate(provenance.yaw) && predicate(provenance.yaw_rate);
}

[[nodiscard]] bool validPreviousControlSource(
    const ExecutionPreviousControlEvidenceSource3D source) noexcept {
  switch (source) {
    case ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback:
    case ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration:
      return true;
    case ExecutionPreviousControlEvidenceSource3D::kUnknown:
    case ExecutionPreviousControlEvidenceSource3D::kAssumedZero:
    case ExecutionPreviousControlEvidenceSource3D::kEngineFallback:
      return false;
  }
  return false;
}

[[nodiscard]] bool exactZeroControl(const MotionControl3D& control) noexcept {
  return control.ax == 0.0F && control.ay == 0.0F && control.az == 0.0F &&
         control.yaw_accel == 0.0F;
}

[[nodiscard]] bool
validPreviousControlEvidence(const ExecutionInputCapture3D& capture) noexcept {
  switch (capture.purpose) {
    case ExecutionInputPurpose3D::kGeneralExecution:
      return validPreviousControlSource(capture.previous_control_source) &&
             ((capture.previous_control_source ==
               ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback) ==
              (capture.previous_control_source_producer_instance_id != 0U));
    case ExecutionInputPurpose3D::kStationaryCaptureRearm:
      return capture.full_state_authoritative &&
             everyStateField(capture.state_provenance,
                             [](const ExecutionStateFieldProvenance3D provenance) {
                               return provenance ==
                                      ExecutionStateFieldProvenance3D::kSourceSample;
                             }) &&
             capture.previous_control_source ==
                 ExecutionPreviousControlEvidenceSource3D::kAssumedZero &&
             capture.previous_control_source_producer_instance_id == 0U &&
             capture.previous_control_source_sequence == capture.capture_sequence &&
             capture.previous_control_source_stamp_ns == capture.effective_stamp_ns &&
             capture.previous_control_receive_stamp_ns == capture.effective_stamp_ns &&
             exactZeroControl(capture.previous_control);
  }
  return false;
}

[[nodiscard]] bool
validExecutionInputCapture(const ExecutionInputCapture3D& capture) noexcept {
  return capture.capture_sequence != 0U && capture.pose_revision != 0U &&
         capture.pose_source_timestamp_us != 0U && capture.pose_receive_stamp_ns > 0 &&
         capture.effective_stamp_ns > 0 &&
         capture.pose_receive_stamp_ns <= capture.effective_stamp_ns &&
         finiteState(capture.state) && capture.state_provenance.valid() &&
         (!capture.full_state_authoritative ||
          capture.state_provenance.fullyAuthoritative()) &&
         finiteControl(capture.previous_control) &&
         validPreviousControlEvidence(capture) &&
         capture.previous_control_source_sequence != 0U &&
         capture.previous_control_source_stamp_ns > 0 &&
         capture.previous_control_receive_stamp_ns > 0 &&
         capture.previous_control_source_stamp_ns <=
             capture.previous_control_receive_stamp_ns &&
         capture.previous_control_receive_stamp_ns <= capture.effective_stamp_ns;
}

void hashState(std::uint64_t& hash, const MotionState3D& state) noexcept {
  hashValue(hash, canonicalFloatBits(state.x));
  hashValue(hash, canonicalFloatBits(state.y));
  hashValue(hash, canonicalFloatBits(state.z));
  hashValue(hash, canonicalFloatBits(state.vx));
  hashValue(hash, canonicalFloatBits(state.vy));
  hashValue(hash, canonicalFloatBits(state.vz));
  hashValue(hash, canonicalFloatBits(state.yaw));
  hashValue(hash, canonicalFloatBits(state.yaw_rate));
}

void hashStateProvenance(std::uint64_t& hash,
                         const ExecutionStateProvenance3D& provenance) noexcept {
  hashValue(hash, static_cast<std::uint64_t>(provenance.x));
  hashValue(hash, static_cast<std::uint64_t>(provenance.y));
  hashValue(hash, static_cast<std::uint64_t>(provenance.z));
  hashValue(hash, static_cast<std::uint64_t>(provenance.vx));
  hashValue(hash, static_cast<std::uint64_t>(provenance.vy));
  hashValue(hash, static_cast<std::uint64_t>(provenance.vz));
  hashValue(hash, static_cast<std::uint64_t>(provenance.yaw));
  hashValue(hash, static_cast<std::uint64_t>(provenance.yaw_rate));
}

void hashControl(std::uint64_t& hash, const MotionControl3D& control) noexcept {
  hashValue(hash, canonicalFloatBits(control.ax));
  hashValue(hash, canonicalFloatBits(control.ay));
  hashValue(hash, canonicalFloatBits(control.az));
  hashValue(hash, canonicalFloatBits(control.yaw_accel));
}

[[nodiscard]] std::uint64_t
executionInputFingerprint(const ExecutionInputCapture3D& capture) noexcept {
  if (!validExecutionInputCapture(capture)) {
    return 0U;
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kExecutionInputDomain);
  hashValue(hash, capture.capture_sequence);
  hashValue(hash, capture.pose_revision);
  hashValue(hash, capture.pose_source_timestamp_us);
  hashValue(hash, static_cast<std::uint64_t>(capture.pose_receive_stamp_ns));
  hashValue(hash, static_cast<std::uint64_t>(capture.effective_stamp_ns));
  hashState(hash, capture.state);
  hashValue(hash, capture.full_state_authoritative ? 1U : 0U);
  hashStateProvenance(hash, capture.state_provenance);
  hashValue(hash, static_cast<std::uint64_t>(capture.purpose));
  hashControl(hash, capture.previous_control);
  hashValue(hash, static_cast<std::uint64_t>(capture.previous_control_source));
  hashValue(hash, capture.previous_control_source_producer_instance_id);
  hashValue(hash, capture.previous_control_source_sequence);
  hashValue(hash, static_cast<std::uint64_t>(capture.previous_control_source_stamp_ns));
  hashValue(hash,
            static_cast<std::uint64_t>(capture.previous_control_receive_stamp_ns));
  return hash == 0U ? 1U : hash;
}

} // namespace

VersionedExecutionValidationPolicy3D::VersionedExecutionValidationPolicy3D(
    CaptureToken, FlightEnvelopeConfig flight_envelope, MotionDynamicsConfig3D dynamics,
    MotionAltitudeEnvelopeConfig3D altitude_envelope,
    SweptFootprintConfig swept_footprint, const double latest_lidar_maximum_age_ms,
    const double execution_input_maximum_pose_age_ms,
    const double execution_input_maximum_control_age_ms,
    const bool route_cross_track_constraints_enabled,
    const bool latest_lidar_freshness_required,
    const bool route_tracking_tube_constraints_enabled,
    const double route_station_credit_slack_m)
    : flight_envelope_{flight_envelope},
      dynamics_{dynamics},
      altitude_envelope_{altitude_envelope},
      swept_footprint_{swept_footprint},
      latest_lidar_maximum_age_ms_{latest_lidar_maximum_age_ms},
      execution_input_maximum_pose_age_ms_{execution_input_maximum_pose_age_ms},
      execution_input_maximum_control_age_ms_{execution_input_maximum_control_age_ms},
      route_cross_track_constraints_enabled_{route_cross_track_constraints_enabled},
      latest_lidar_freshness_required_{latest_lidar_freshness_required},
      route_tracking_tube_constraints_enabled_{route_tracking_tube_constraints_enabled},
      route_station_credit_slack_m_{route_station_credit_slack_m},
      content_fingerprint_{policyFingerprint(
          flight_envelope_, dynamics_, altitude_envelope_, swept_footprint_,
          latest_lidar_maximum_age_ms_, execution_input_maximum_pose_age_ms_,
          execution_input_maximum_control_age_ms_,
          route_cross_track_constraints_enabled_, latest_lidar_freshness_required_,
          route_tracking_tube_constraints_enabled_, route_station_credit_slack_m_)} {
}

std::shared_ptr<const VersionedExecutionValidationPolicy3D>
VersionedExecutionValidationPolicy3D::capture(
    FlightEnvelopeConfig flight_envelope, MotionDynamicsConfig3D dynamics,
    MotionAltitudeEnvelopeConfig3D altitude_envelope,
    SweptFootprintConfig swept_footprint, const double latest_lidar_maximum_age_ms,
    const double execution_input_maximum_pose_age_ms,
    const double execution_input_maximum_control_age_ms,
    const bool route_cross_track_constraints_enabled,
    const bool latest_lidar_freshness_required,
    const bool route_tracking_tube_constraints_enabled,
    const double route_station_credit_slack_m) {
  if (!validPolicy(flight_envelope, dynamics, altitude_envelope, swept_footprint,
                   latest_lidar_maximum_age_ms, execution_input_maximum_pose_age_ms,
                   execution_input_maximum_control_age_ms) ||
      !std::isfinite(route_station_credit_slack_m) ||
      route_station_credit_slack_m < 0.0) {
    return nullptr;
  }
  return std::make_shared<const VersionedExecutionValidationPolicy3D>(
      CaptureToken{}, flight_envelope, dynamics, altitude_envelope, swept_footprint,
      latest_lidar_maximum_age_ms, execution_input_maximum_pose_age_ms,
      execution_input_maximum_control_age_ms, route_cross_track_constraints_enabled,
      latest_lidar_freshness_required, route_tracking_tube_constraints_enabled,
      route_station_credit_slack_m);
}

double VersionedExecutionValidationPolicy3D::routeStationCreditSlackM() const noexcept {
  return route_station_credit_slack_m_;
}

const FlightEnvelopeConfig&
VersionedExecutionValidationPolicy3D::flightEnvelope() const noexcept {
  return flight_envelope_;
}

const MotionDynamicsConfig3D&
VersionedExecutionValidationPolicy3D::dynamics() const noexcept {
  return dynamics_;
}

const MotionAltitudeEnvelopeConfig3D&
VersionedExecutionValidationPolicy3D::altitudeEnvelope() const noexcept {
  return altitude_envelope_;
}

const SweptFootprintConfig&
VersionedExecutionValidationPolicy3D::sweptFootprint() const noexcept {
  return swept_footprint_;
}

double VersionedExecutionValidationPolicy3D::latestLidarMaximumAgeMs() const noexcept {
  return latest_lidar_maximum_age_ms_;
}

double
VersionedExecutionValidationPolicy3D::executionInputMaximumPoseAgeMs() const noexcept {
  return execution_input_maximum_pose_age_ms_;
}

double VersionedExecutionValidationPolicy3D::executionInputMaximumControlAgeMs()
    const noexcept {
  return execution_input_maximum_control_age_ms_;
}

bool VersionedExecutionValidationPolicy3D::routeCrossTrackConstraintsEnabled()
    const noexcept {
  return route_cross_track_constraints_enabled_;
}

bool VersionedExecutionValidationPolicy3D::latestLidarFreshnessRequired()
    const noexcept {
  return latest_lidar_freshness_required_;
}

bool VersionedExecutionValidationPolicy3D::routeTrackingTubeConstraintsEnabled()
    const noexcept {
  return route_tracking_tube_constraints_enabled_;
}

ExecutionValidationPolicyId3D
VersionedExecutionValidationPolicy3D::policyId() const noexcept {
  return ExecutionValidationPolicyId3D{.value = content_fingerprint_};
}

std::uint64_t
VersionedExecutionValidationPolicy3D::contentFingerprint() const noexcept {
  return content_fingerprint_;
}

bool VersionedExecutionValidationPolicy3D::valid() const noexcept {
  return content_fingerprint_ != 0U &&
         validPolicy(flight_envelope_, dynamics_, altitude_envelope_, swept_footprint_,
                     latest_lidar_maximum_age_ms_, execution_input_maximum_pose_age_ms_,
                     execution_input_maximum_control_age_ms_);
}

bool ExecutionStateProvenance3D::valid() const noexcept {
  return everyStateField(*this, knownStateFieldProvenance);
}

bool ExecutionStateProvenance3D::fullyAuthoritative() const noexcept {
  return everyStateField(*this, authoritativeStateFieldProvenance);
}

bool ExecutionStateProvenance3D::yawAuthoritative() const noexcept {
  return authoritativeStateFieldProvenance(yaw) &&
         authoritativeStateFieldProvenance(yaw_rate);
}

VersionedExecutionInput3D::VersionedExecutionInput3D(CaptureToken,
                                                     ExecutionInputCapture3D capture)
    : capture_{std::move(capture)},
      content_fingerprint_{executionInputFingerprint(capture_)} {
}

std::shared_ptr<const VersionedExecutionInput3D>
VersionedExecutionInput3D::capture(const ExecutionInputCapture3D& capture) {
  if (!validExecutionInputCapture(capture)) {
    return nullptr;
  }
  return std::make_shared<const VersionedExecutionInput3D>(CaptureToken{}, capture);
}

std::uint64_t VersionedExecutionInput3D::captureSequence() const noexcept {
  return capture_.capture_sequence;
}

std::uint64_t VersionedExecutionInput3D::poseRevision() const noexcept {
  return capture_.pose_revision;
}

std::uint64_t VersionedExecutionInput3D::poseSourceTimestampUs() const noexcept {
  return capture_.pose_source_timestamp_us;
}

std::int64_t VersionedExecutionInput3D::poseReceiveStampNs() const noexcept {
  return capture_.pose_receive_stamp_ns;
}

std::int64_t VersionedExecutionInput3D::effectiveStampNs() const noexcept {
  return capture_.effective_stamp_ns;
}

const MotionState3D& VersionedExecutionInput3D::state() const noexcept {
  return capture_.state;
}

bool VersionedExecutionInput3D::fullStateAuthoritative() const noexcept {
  return capture_.full_state_authoritative;
}

const ExecutionStateProvenance3D&
VersionedExecutionInput3D::stateProvenance() const noexcept {
  return capture_.state_provenance;
}

bool VersionedExecutionInput3D::nominalStateAuthoritative() const noexcept {
  return capture_.purpose == ExecutionInputPurpose3D::kGeneralExecution &&
         capture_.full_state_authoritative &&
         capture_.state_provenance.fullyAuthoritative();
}

bool VersionedExecutionInput3D::stationaryCaptureStateAuthoritative() const noexcept {
  return capture_.purpose == ExecutionInputPurpose3D::kStationaryCaptureRearm &&
         capture_.full_state_authoritative &&
         capture_.state_provenance.fullyAuthoritative();
}

ExecutionInputPurpose3D VersionedExecutionInput3D::purpose() const noexcept {
  return capture_.purpose;
}

const MotionControl3D& VersionedExecutionInput3D::previousControl() const noexcept {
  return capture_.previous_control;
}

ExecutionPreviousControlEvidenceSource3D
VersionedExecutionInput3D::previousControlSource() const noexcept {
  return capture_.previous_control_source;
}

std::uint64_t
VersionedExecutionInput3D::previousControlSourceProducerInstanceId() const noexcept {
  return capture_.previous_control_source_producer_instance_id;
}

std::uint64_t
VersionedExecutionInput3D::previousControlSourceSequence() const noexcept {
  return capture_.previous_control_source_sequence;
}

std::int64_t VersionedExecutionInput3D::previousControlSourceStampNs() const noexcept {
  return capture_.previous_control_source_stamp_ns;
}

std::int64_t VersionedExecutionInput3D::previousControlReceiveStampNs() const noexcept {
  return capture_.previous_control_receive_stamp_ns;
}

std::uint64_t VersionedExecutionInput3D::contentFingerprint() const noexcept {
  return content_fingerprint_;
}

bool VersionedExecutionInput3D::valid() const noexcept {
  return content_fingerprint_ != 0U && validExecutionInputCapture(capture_);
}

[[nodiscard]] static bool executionInputFreshForMaximumAges(
    const VersionedExecutionInput3D& input, const std::int64_t validation_stamp_ns,
    const double maximum_pose_age_ms, const double maximum_control_age_ms) noexcept {
  constexpr double kNanosecondsPerMillisecond{1.0e6};
  if (!input.valid() || validation_stamp_ns <= 0 ||
      !std::isfinite(maximum_pose_age_ms) || maximum_pose_age_ms < 0.0 ||
      !std::isfinite(maximum_control_age_ms) || maximum_control_age_ms < 0.0 ||
      input.poseReceiveStampNs() > validation_stamp_ns ||
      input.previousControlReceiveStampNs() > validation_stamp_ns) {
    return false;
  }
  const std::int64_t pose_age_ns = validation_stamp_ns - input.poseReceiveStampNs();
  const std::int64_t control_age_ns =
      validation_stamp_ns - input.previousControlReceiveStampNs();
  return static_cast<double>(pose_age_ns) <=
             maximum_pose_age_ms * kNanosecondsPerMillisecond &&
         static_cast<double>(control_age_ns) <=
             maximum_control_age_ms * kNanosecondsPerMillisecond;
}

bool executionInputFreshAt(const VersionedExecutionInput3D& input,
                           const VersionedExecutionValidationPolicy3D& policy,
                           const std::int64_t validation_stamp_ns) noexcept {
  return policy.valid() &&
         executionInputFreshForMaximumAges(input, validation_stamp_ns,
                                           policy.executionInputMaximumPoseAgeMs(),
                                           policy.executionInputMaximumControlAgeMs());
}

LatestLidarEvidenceUpdateStatus3D assessLatestLidarEvidenceUpdate3D(
    const VersionedLatestLidarEvidence3D* const current,
    const VersionedLatestLidarEvidence3D& candidate) noexcept {
  if (!candidate.valid()) {
    return LatestLidarEvidenceUpdateStatus3D::kRejectedInvalid;
  }
  if (current == nullptr) {
    return LatestLidarEvidenceUpdateStatus3D::kAcceptedInitial;
  }
  if (!current->valid()) {
    return LatestLidarEvidenceUpdateStatus3D::kRejectedInvalid;
  }
  if (candidate.producerInstanceId() != current->producerInstanceId()) {
    // Producer epochs are authority boundaries, not ordering fields. The
    // stateful admission function owns authenticated handoff.
    return LatestLidarEvidenceUpdateStatus3D::kRejectedUnauthenticatedProducer;
  }
  if (candidate.sequence() == current->sequence()) {
    return candidate.sourceContentFingerprint() == current->sourceContentFingerprint()
               ? LatestLidarEvidenceUpdateStatus3D::kIdempotentDuplicate
               : LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict;
  }
  if (candidate.sequence() <= current->sequence() ||
      candidate.poseGeneration() < current->poseGeneration() ||
      candidate.receiveStampNs() <= current->receiveStampNs()) {
    return LatestLidarEvidenceUpdateStatus3D::kRejectedRegression;
  }
  // Sequence is the producer-local order. Acquisition time is source content,
  // but is not an ordering high-water: a corrupt future value must not poison
  // all later observations and a producer epoch may restart its source clock.
  return LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer;
}

LatestLidarEvidenceClaimResult3D
claimLatestLidarEvidenceIdentity3D(const LatestLidarEvidenceAdmissionState3D& state,
                                   const VersionedLatestLidarEvidence3D* const current,
                                   LatestLidarEvidenceIdentityClaim3D claim) noexcept {
  LatestLidarEvidenceClaimResult3D result{.next_state = state, .claim = claim};
  if (!validIdentityClaim(claim) || !validAdmissionState(state, current)) {
    return result;
  }
  if (state.prospective_claim_capacity_exhausted) {
    result.status = LatestLidarEvidenceClaimStatus3D::kRejectedCapacity;
    return result;
  }

  if (current != nullptr &&
      claim.producer_instance_id == current->producerInstanceId()) {
    if (claim.sequence < current->sequence()) {
      result.status = LatestLidarEvidenceClaimStatus3D::kRejectedRegression;
      return result;
    }
    if (claim.sequence == current->sequence()) {
      result.claim.first_receive_stamp_ns = state.current_first_receive_stamp_ns;
      if (state.current_identity_conflicted ||
          claim.raw_wire_fingerprint != state.current_raw_wire_fingerprint) {
        result.authority_quarantine_opened = !state.current_identity_conflicted;
        result.next_state.current_identity_conflicted = true;
        result.status = LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict;
      } else {
        result.status = LatestLidarEvidenceClaimStatus3D::kInstalledReplay;
      }
      return result;
    }
  }

  const std::size_t index = prospectiveClaimIndex(state, claim.producer_instance_id);
  if (index != state.prospective_claim_count) {
    LatestLidarEvidenceProspectiveClaim3D& resident =
        result.next_state.prospective_claims[index];
    if (claim.sequence < resident.identity.sequence) {
      result.status = LatestLidarEvidenceClaimStatus3D::kRejectedRegression;
      return result;
    }
    if (claim.sequence == resident.identity.sequence) {
      result.claim = resident.identity;
      if (claim.raw_wire_fingerprint != resident.identity.raw_wire_fingerprint) {
        resident.conflicted = true;
        if (state.pending_producer_instance_id == claim.producer_instance_id &&
            state.pending_sequence == claim.sequence) {
          result.next_state.pending_identity_conflicted = true;
        }
        result.status = LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict;
      } else if (resident.conflicted) {
        result.status = LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict;
      } else if (state.pending_producer_instance_id == claim.producer_instance_id &&
                 state.pending_sequence == claim.sequence) {
        result.status = LatestLidarEvidenceClaimStatus3D::kPendingReplay;
      } else {
        result.status = LatestLidarEvidenceClaimStatus3D::kRejectedReplay;
      }
      return result;
    }
    resident = LatestLidarEvidenceProspectiveClaim3D{.identity = claim};
  } else {
    if (state.prospective_claim_count == state.prospective_claims.size()) {
      result.next_state.prospective_claim_capacity_exhausted = true;
      result.status = LatestLidarEvidenceClaimStatus3D::kRejectedCapacity;
      result.authority_quarantine_opened = true;
      return result;
    }
    result.next_state.prospective_claims[state.prospective_claim_count] =
        LatestLidarEvidenceProspectiveClaim3D{.identity = claim};
    ++result.next_state.prospective_claim_count;
  }
  result.status = LatestLidarEvidenceClaimStatus3D::kClaimed;
  result.assess_candidate = true;
  return result;
}

LatestLidarEvidenceAdmissionResult3D
admitClaimedLatestLidarEvidence3D(const LatestLidarEvidenceAdmissionState3D& state,
                                  const VersionedLatestLidarEvidence3D* const current,
                                  const VersionedLatestLidarEvidence3D& candidate,
                                  const LatestLidarEvidenceIdentityClaim3D& claim,
                                  const std::int64_t now_ns,
                                  const double maximum_age_ms) noexcept {
  LatestLidarEvidenceAdmissionResult3D result{.next_state = state};
  if (!candidate.valid() || !validIdentityClaim(claim) ||
      !validAdmissionState(state, current) ||
      state.prospective_claim_capacity_exhausted ||
      candidate.producerInstanceId() != claim.producer_instance_id ||
      candidate.sequence() != claim.sequence ||
      candidate.receiveStampNs() != claim.first_receive_stamp_ns) {
    if (state.prospective_claim_capacity_exhausted) {
      result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedClaimCapacity;
    }
    return result;
  }
  const std::size_t claim_index =
      prospectiveClaimIndex(state, claim.producer_instance_id);
  if (claim_index == state.prospective_claim_count ||
      state.prospective_claims[claim_index].conflicted ||
      !sameIdentityClaim(state.prospective_claims[claim_index].identity, claim)) {
    return result;
  }
  const LatestLidarEvidenceFreshness3D candidate_freshness =
      assessLatestLidarEvidenceFreshness3D(candidate, now_ns, maximum_age_ms);
  if (!candidate_freshness.fresh) {
    result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedStaleCandidate;
    return result;
  }
  if (producerRetired(state, candidate.producerInstanceId())) {
    result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedRetiredProducer;
    return result;
  }

  if (current == nullptr) {
    promoteCurrentClaim(result.next_state, claim);
    result.status = LatestLidarEvidenceUpdateStatus3D::kAcceptedInitial;
    result.install_candidate = true;
    return result;
  }

  if (candidate.producerInstanceId() == current->producerInstanceId()) {
    result.status = assessLatestLidarEvidenceUpdate3D(current, candidate);
    if (result.status == LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer) {
      if (candidate.acquisitionStampNs() <= current->acquisitionStampNs()) {
        result.status =
            LatestLidarEvidenceUpdateStatus3D::kAcceptedAcquisitionEpochReset;
        result.acquisition_epoch_reset = true;
      }
      promoteCurrentClaim(result.next_state, claim);
      result.install_candidate = true;
    }
    return result;
  }

  if (state.retired_producer_count == state.retired_producer_instance_ids.size()) {
    result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedHandoffCapacity;
    return result;
  }
  if (assessLatestLidarEvidenceFreshness3D(*current, now_ns, maximum_age_ms).fresh) {
    // A live owner cancels handoff probation, but the immutable prospective
    // claim remains recorded and cannot be replayed with a newer receipt.
    clearPendingProducerHandoff(result.next_state);
    result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedUnauthenticatedProducer;
    return result;
  }

  if (state.pending_producer_instance_id != candidate.producerInstanceId()) {
    beginPendingProducerHandoff(result.next_state, candidate);
    result.status = LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff;
    return result;
  }
  if (state.pending_identity_conflicted) {
    if (candidate.sequence() <= state.pending_sequence) {
      result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict;
      return result;
    }
    beginPendingProducerHandoff(result.next_state, candidate);
    result.status = LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff;
    return result;
  }
  if (candidate.sequence() <= state.pending_sequence ||
      candidate.poseGeneration() < state.pending_pose_generation ||
      candidate.receiveStampNs() <= state.pending_receive_stamp_ns) {
    result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedRegression;
    return result;
  }
  const double confirmation_interval_ms =
      static_cast<double>(candidate.receiveStampNs() - state.pending_receive_stamp_ns) *
      1.0e-6;
  if (!std::isfinite(confirmation_interval_ms) ||
      confirmation_interval_ms > maximum_age_ms) {
    // A lone observation must not remain an authentication credential after it
    // has aged out. Restart probation from the current fresh observation.
    beginPendingProducerHandoff(result.next_state, candidate);
    result.status = LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff;
    return result;
  }

  result.next_state
      .retired_producer_instance_ids[result.next_state.retired_producer_count] =
      current->producerInstanceId();
  ++result.next_state.retired_producer_count;
  promoteCurrentClaim(result.next_state, claim);
  result.status = LatestLidarEvidenceUpdateStatus3D::kAcceptedProducerHandoff;
  result.install_candidate = true;
  result.producer_handoff = true;
  return result;
}

LatestLidarEvidenceAdmissionResult3D
admitLatestLidarEvidence3D(const LatestLidarEvidenceAdmissionState3D& state,
                           const VersionedLatestLidarEvidence3D* const current,
                           const VersionedLatestLidarEvidence3D& candidate,
                           const std::int64_t now_ns,
                           const double maximum_age_ms) noexcept {
  const LatestLidarEvidenceClaimResult3D claimed = claimLatestLidarEvidenceIdentity3D(
      state, current,
      LatestLidarEvidenceIdentityClaim3D{
          .producer_instance_id = candidate.producerInstanceId(),
          .sequence = candidate.sequence(),
          .raw_wire_fingerprint = candidate.sourceContentFingerprint(),
          .first_receive_stamp_ns = candidate.receiveStampNs(),
      });
  LatestLidarEvidenceAdmissionResult3D result{.next_state = claimed.next_state};
  result.current_identity_conflict =
      claimed.authority_quarantine_opened &&
      claimed.status == LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict;
  switch (claimed.status) {
    case LatestLidarEvidenceClaimStatus3D::kClaimed:
      result = admitClaimedLatestLidarEvidence3D(claimed.next_state, current, candidate,
                                                 claimed.claim, now_ns, maximum_age_ms);
      result.current_identity_conflict = false;
      return result;
    case LatestLidarEvidenceClaimStatus3D::kInstalledReplay:
      result.status = LatestLidarEvidenceUpdateStatus3D::kIdempotentDuplicate;
      break;
    case LatestLidarEvidenceClaimStatus3D::kPendingReplay:
      result.status = LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff;
      break;
    case LatestLidarEvidenceClaimStatus3D::kRejectedReplay:
      result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedClaimReplay;
      break;
    case LatestLidarEvidenceClaimStatus3D::kRejectedRegression:
      result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedRegression;
      break;
    case LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict:
      result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict;
      break;
    case LatestLidarEvidenceClaimStatus3D::kRejectedCapacity:
      result.status = LatestLidarEvidenceUpdateStatus3D::kRejectedClaimCapacity;
      break;
    case LatestLidarEvidenceClaimStatus3D::kRejectedInvalid:
      break;
  }
  return result;
}

std::string_view latestLidarEvidenceUpdateStatus3DName(
    const LatestLidarEvidenceUpdateStatus3D status) noexcept {
  switch (status) {
    case LatestLidarEvidenceUpdateStatus3D::kAcceptedInitial:
      return "accepted_initial";
    case LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer:
      return "accepted_newer";
    case LatestLidarEvidenceUpdateStatus3D::kAcceptedAcquisitionEpochReset:
      return "accepted_acquisition_epoch_reset";
    case LatestLidarEvidenceUpdateStatus3D::kAcceptedProducerHandoff:
      return "accepted_producer_handoff";
    case LatestLidarEvidenceUpdateStatus3D::kIdempotentDuplicate:
      return "idempotent_duplicate";
    case LatestLidarEvidenceUpdateStatus3D::kPendingProducerHandoff:
      return "pending_producer_handoff";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedInvalid:
      return "invalid";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedStaleCandidate:
      return "stale_candidate";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedRegression:
      return "regression";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedIdentityConflict:
      return "identity_conflict";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedUnauthenticatedProducer:
      return "unauthenticated_producer";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedRetiredProducer:
      return "retired_producer";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedHandoffCapacity:
      return "handoff_capacity";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedClaimReplay:
      return "claim_replay";
    case LatestLidarEvidenceUpdateStatus3D::kRejectedClaimCapacity:
      return "claim_capacity";
  }
  return "unknown";
}

std::string_view latestLidarEvidenceClaimStatus3DName(
    const LatestLidarEvidenceClaimStatus3D status) noexcept {
  switch (status) {
    case LatestLidarEvidenceClaimStatus3D::kClaimed:
      return "claimed";
    case LatestLidarEvidenceClaimStatus3D::kInstalledReplay:
      return "installed_replay";
    case LatestLidarEvidenceClaimStatus3D::kPendingReplay:
      return "pending_replay";
    case LatestLidarEvidenceClaimStatus3D::kRejectedReplay:
      return "rejected_replay";
    case LatestLidarEvidenceClaimStatus3D::kRejectedRegression:
      return "regression";
    case LatestLidarEvidenceClaimStatus3D::kRejectedIdentityConflict:
      return "identity_conflict";
    case LatestLidarEvidenceClaimStatus3D::kRejectedCapacity:
      return "claim_capacity";
    case LatestLidarEvidenceClaimStatus3D::kRejectedInvalid:
      return "invalid";
  }
  return "unknown";
}

bool latestLidarEvidenceAuthorityQuarantined3D(
    const LatestLidarEvidenceAdmissionState3D& state) noexcept {
  return state.current_identity_conflicted ||
         state.prospective_claim_capacity_exhausted;
}

LatestLidarEvidenceFreshness3D
assessLatestLidarEvidenceFreshness3D(const VersionedLatestLidarEvidence3D& evidence,
                                     const std::int64_t now_ns,
                                     const double maximum_age_ms) noexcept {
  LatestLidarEvidenceFreshness3D result;
  constexpr double kMaximumRepresentableAgeMs =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1.0e6;
  if (!evidence.valid() || now_ns <= 0 || !std::isfinite(maximum_age_ms) ||
      maximum_age_ms <= 0.0 || maximum_age_ms > kMaximumRepresentableAgeMs) {
    return result;
  }

  const auto maximum_age_ns =
      static_cast<std::int64_t>(std::llround(maximum_age_ms * 1.0e6));
  if (maximum_age_ns <= 0) {
    return result;
  }
  const std::int64_t acquisition_age_ns = now_ns - evidence.acquisitionStampNs();
  const std::int64_t receive_age_ns = now_ns - evidence.receiveStampNs();
  result.receive_time_fallback = acquisition_age_ns < 0;
  result.age_ms = static_cast<double>(
                      std::max({std::int64_t{0}, acquisition_age_ns, receive_age_ns})) *
                  1.0e-6;
  result.fresh = receive_age_ns >= 0 && receive_age_ns <= maximum_age_ns &&
                 (acquisition_age_ns < 0 || acquisition_age_ns <= maximum_age_ns);
  return result;
}

} // namespace drone_city_nav
