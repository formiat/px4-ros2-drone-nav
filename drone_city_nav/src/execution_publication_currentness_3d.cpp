#include "drone_city_nav/execution_publication_currentness_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameRawVersion(const RawMapVersion& expected,
                                  const RawMapVersion& current) noexcept {
  return expected.producer_instance_id == current.producer_instance_id &&
         expected.base_snapshot_revision == current.base_snapshot_revision &&
         expected.revision == current.revision;
}

[[nodiscard]] bool
sameLidarIdentity(const VersionedLatestLidarEvidence3D& expected,
                  const VersionedLatestLidarEvidence3D& current) noexcept {
  return expected.producerInstanceId() == current.producerInstanceId() &&
         expected.sequence() == current.sequence() &&
         expected.poseGeneration() == current.poseGeneration() &&
         expected.acquisitionStampNs() == current.acquisitionStampNs();
}

} // namespace

ExecutionPublicationCurrentnessStatus3D assessExecutionPublicationCurrentness3D(
    const ExecutionPublicationCurrentnessCheck3D& check) noexcept {
  switch (check.raw_requirement) {
    case ExecutionPublicationRawRequirement3D::kOptional:
    case ExecutionPublicationRawRequirement3D::kRequired:
      break;
    default:
      return ExecutionPublicationCurrentnessStatus3D::kInvalidRawRequirement;
  }

  if (check.expected_snapshot == nullptr || check.current_snapshot == nullptr) {
    return ExecutionPublicationCurrentnessStatus3D::kSnapshotMissing;
  }
  if (!check.expected_snapshot->valid() || !check.current_snapshot->valid()) {
    return ExecutionPublicationCurrentnessStatus3D::kSnapshotInvalid;
  }
  if (check.expected_snapshot != check.current_snapshot) {
    return ExecutionPublicationCurrentnessStatus3D::kSnapshotOwnerChanged;
  }

  const bool expected_raw_present = check.expected_raw_world != nullptr;
  const bool current_raw_present = check.current_raw_world != nullptr;
  if (check.raw_requirement == ExecutionPublicationRawRequirement3D::kRequired &&
      (!expected_raw_present || !current_raw_present)) {
    return ExecutionPublicationCurrentnessStatus3D::kRequiredRawEvidenceMissing;
  }
  if (expected_raw_present != current_raw_present) {
    return ExecutionPublicationCurrentnessStatus3D::kRawEvidencePresenceChanged;
  }
  if (expected_raw_present) {
    if (!check.expected_raw_world->valid() || !check.current_raw_world->valid()) {
      return ExecutionPublicationCurrentnessStatus3D::kRawEvidenceInvalid;
    }
    if (!sameRawVersion(check.expected_raw_world->version(),
                        check.current_raw_world->version())) {
      return ExecutionPublicationCurrentnessStatus3D::kRawVersionChanged;
    }
    if (!check.expected_raw_world->sharesObservationOwner(*check.current_raw_world)) {
      return ExecutionPublicationCurrentnessStatus3D::kRawObservationOwnerChanged;
    }
  }

  if (check.expected_lidar_evidence == nullptr ||
      check.current_lidar_evidence == nullptr) {
    return ExecutionPublicationCurrentnessStatus3D::kLidarEvidenceMissing;
  }
  if (!check.expected_lidar_evidence->valid() ||
      !check.current_lidar_evidence->valid()) {
    return ExecutionPublicationCurrentnessStatus3D::kLidarEvidenceInvalid;
  }
  if (!sameLidarIdentity(*check.expected_lidar_evidence,
                         *check.current_lidar_evidence)) {
    return ExecutionPublicationCurrentnessStatus3D::kLidarIdentityChanged;
  }
  if (check.expected_lidar_evidence->contentFingerprint() !=
      check.current_lidar_evidence->contentFingerprint()) {
    return ExecutionPublicationCurrentnessStatus3D::kLidarContentChanged;
  }
  if (check.expected_lidar_evidence != check.current_lidar_evidence) {
    return ExecutionPublicationCurrentnessStatus3D::kLidarOwnerChanged;
  }

  const LatestLidarEvidenceFreshness3D freshness = assessLatestLidarEvidenceFreshness3D(
      *check.current_lidar_evidence, check.publication_now_ns,
      check.maximum_lidar_age_ms);
  if (freshness.age_ms < 0.0) {
    return ExecutionPublicationCurrentnessStatus3D::kInvalidPublicationTime;
  }
  return freshness.fresh ? ExecutionPublicationCurrentnessStatus3D::kCurrent
                         : ExecutionPublicationCurrentnessStatus3D::kLidarNotFresh;
}

std::string_view executionPublicationCurrentnessStatus3DName(
    const ExecutionPublicationCurrentnessStatus3D status) noexcept {
  switch (status) {
    case ExecutionPublicationCurrentnessStatus3D::kCurrent:
      return "current";
    case ExecutionPublicationCurrentnessStatus3D::kInvalidRawRequirement:
      return "invalid_raw_requirement";
    case ExecutionPublicationCurrentnessStatus3D::kSnapshotMissing:
      return "snapshot_missing";
    case ExecutionPublicationCurrentnessStatus3D::kSnapshotInvalid:
      return "snapshot_invalid";
    case ExecutionPublicationCurrentnessStatus3D::kSnapshotOwnerChanged:
      return "snapshot_owner_changed";
    case ExecutionPublicationCurrentnessStatus3D::kRequiredRawEvidenceMissing:
      return "required_raw_evidence_missing";
    case ExecutionPublicationCurrentnessStatus3D::kRawEvidencePresenceChanged:
      return "raw_evidence_presence_changed";
    case ExecutionPublicationCurrentnessStatus3D::kRawEvidenceInvalid:
      return "raw_evidence_invalid";
    case ExecutionPublicationCurrentnessStatus3D::kRawVersionChanged:
      return "raw_version_changed";
    case ExecutionPublicationCurrentnessStatus3D::kRawObservationOwnerChanged:
      return "raw_observation_owner_changed";
    case ExecutionPublicationCurrentnessStatus3D::kLidarEvidenceMissing:
      return "lidar_evidence_missing";
    case ExecutionPublicationCurrentnessStatus3D::kLidarEvidenceInvalid:
      return "lidar_evidence_invalid";
    case ExecutionPublicationCurrentnessStatus3D::kLidarIdentityChanged:
      return "lidar_identity_changed";
    case ExecutionPublicationCurrentnessStatus3D::kLidarContentChanged:
      return "lidar_content_changed";
    case ExecutionPublicationCurrentnessStatus3D::kLidarOwnerChanged:
      return "lidar_owner_changed";
    case ExecutionPublicationCurrentnessStatus3D::kInvalidPublicationTime:
      return "invalid_publication_time";
    case ExecutionPublicationCurrentnessStatus3D::kLidarNotFresh:
      return "lidar_not_fresh";
  }
  return "unknown";
}

} // namespace drone_city_nav
