#pragma once

#include "drone_city_nav/execution_plan_3d.hpp"

#include <cstdint>
#include <memory>
#include <string_view>

namespace drone_city_nav {

enum class ExecutionPublicationRawRequirement3D : std::uint8_t {
  kOptional,
  kRequired,
};

enum class ExecutionPublicationCurrentnessStatus3D : std::uint8_t {
  kCurrent,
  // The execution owner has not changed, but continuous raw/lidar evidence
  // advanced on the same lineage. Publication may proceed only after the
  // candidate swept horizon is validated against that latest evidence.
  kRevalidationRequired,
  kInvalidRawRequirement,
  kSnapshotMissing,
  kSnapshotInvalid,
  kSnapshotOwnerChanged,
  kRequiredRawEvidenceMissing,
  kRawEvidencePresenceChanged,
  kRawEvidenceInvalid,
  kRawVersionChanged,
  kRawObservationOwnerChanged,
  kLidarEvidenceMissing,
  kLidarEvidenceInvalid,
  kLidarIdentityChanged,
  kLidarContentChanged,
  kInvalidPublicationTime,
  kLidarNotFresh,
};

struct ExecutionPublicationCurrentnessCheck3D {
  std::shared_ptr<const ExecutionPlan3D> expected_snapshot;
  std::shared_ptr<const ExecutionPlan3D> current_snapshot;
  ExecutionPublicationRawRequirement3D raw_requirement{
      ExecutionPublicationRawRequirement3D::kRequired};
  std::shared_ptr<const VersionedObservedRawWorld3D> expected_raw_world;
  std::shared_ptr<const VersionedObservedRawWorld3D> current_raw_world;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> expected_lidar_evidence;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar_evidence;
  std::int64_t publication_now_ns{0};
  double maximum_lidar_age_ms{0.0};
  bool lidar_freshness_required{true};
};

[[nodiscard]] ExecutionPublicationCurrentnessStatus3D
assessExecutionPublicationCurrentness3D(
    const ExecutionPublicationCurrentnessCheck3D& check) noexcept;

[[nodiscard]] std::string_view executionPublicationCurrentnessStatus3DName(
    ExecutionPublicationCurrentnessStatus3D status) noexcept;

} // namespace drone_city_nav
