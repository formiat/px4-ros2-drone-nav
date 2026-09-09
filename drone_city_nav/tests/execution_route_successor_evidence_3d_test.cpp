#include "drone_city_nav/execution_route_certificates_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "execution_route_snapshot_3d_internal.hpp"
#include "execution_route_snapshot_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

using execution_route_snapshot_3d_internal::successorEvidenceRegression;
using execution_route_snapshot_3d_internal::successorRouteEvidenceRegression;

constexpr std::uint64_t kProducer{9U};
constexpr std::uint64_t kRevision{18U};
constexpr std::uint64_t kOccupancy{500U};

struct SuccessorEvidenceFixture {
  SnapshotFixture3D snapshot;

  [[nodiscard]] CertifiedRouteSuffix3D
  route(const std::uint64_t world_content_fingerprint,
        const std::uint64_t occupancy_content_fingerprint) const {
    CertifiedRouteSuffix3D suffix;
    suffix.validation_policy = snapshot.validation_policy;
    suffix.certificate = ObservedRawRouteCertificate3D{
        .producer_instance_id = kProducer,
        .validated_through_revision = kRevision,
        .observed_world_content_fingerprint = world_content_fingerprint,
        .geometry_derivation_occupancy_content_fingerprint =
            occupancy_content_fingerprint,
    };
    return suffix;
  }

  [[nodiscard]] static FiniteExecutionState3D
  execution(const std::uint64_t world_content_fingerprint,
            const std::uint64_t occupancy_content_fingerprint) {
    FiniteExecutionState3D state;
    state.validation_proof.lineage = ObservedRawFiniteExecutionValidationLineage3D{
        .producer_instance_id = kProducer,
        .validated_through_raw_revision = kRevision,
        .observed_world_content_fingerprint = world_content_fingerprint,
        .observed_occupancy_content_fingerprint = occupancy_content_fingerprint,
    };
    return state;
  }
};

} // namespace

TEST(ExecutionRouteSuccessorEvidence3DTest,
     ASeedThatMovedWithTheVehicleIsNotADifferentWorld) {
  // The resident route and its successor were both validated on raw revision
  // 18. The successor's observed world differs only in its transient part:
  // the free-space seed moved while the vehicle hovered. The persistent
  // occupancy is the same, so the successor is as current as the resident.
  const SuccessorEvidenceFixture fixture;
  const CertifiedRouteSuffix3D resident = fixture.route(1001U, kOccupancy);
  const FiniteExecutionState3D resident_execution =
      SuccessorEvidenceFixture::execution(1001U, kOccupancy);
  const CertifiedRouteSuffix3D successor = fixture.route(1002U, kOccupancy);
  const FiniteExecutionState3D successor_execution =
      SuccessorEvidenceFixture::execution(1002U, kOccupancy);

  EXPECT_EQ(successorRouteEvidenceRegression(resident, successor, successor_execution),
            ExecutionRouteTransitionDetail3D::kNone);
  EXPECT_EQ(successorEvidenceRegression(resident, resident_execution, successor,
                                        successor_execution),
            ExecutionRouteTransitionDetail3D::kNone);
}

TEST(ExecutionRouteSuccessorEvidence3DTest,
     ADifferentOccupancyAtTheSameRevisionIsStillAMismatch) {
  // Same revision, different persistent occupancy: neither certificate can
  // be ordered before the other, and the successor is refused as before.
  const SuccessorEvidenceFixture fixture;
  const CertifiedRouteSuffix3D resident = fixture.route(1001U, kOccupancy);
  const FiniteExecutionState3D resident_execution =
      SuccessorEvidenceFixture::execution(1001U, kOccupancy);
  const CertifiedRouteSuffix3D successor = fixture.route(1001U, kOccupancy + 1U);
  const FiniteExecutionState3D successor_execution =
      SuccessorEvidenceFixture::execution(1001U, kOccupancy + 1U);

  EXPECT_EQ(successorRouteEvidenceRegression(resident, successor, successor_execution),
            ExecutionRouteTransitionDetail3D::kSuccessorWorldContentMismatch);
  EXPECT_EQ(successorEvidenceRegression(resident, resident_execution, successor,
                                        successor_execution),
            ExecutionRouteTransitionDetail3D::kSuccessorWorldContentMismatch);
}

TEST(ExecutionRouteSuccessorEvidence3DTest,
     AnExecutionValidatedOnAnotherOccupancyAtTheSameRevisionIsAMismatch) {
  // The resident execution's lineage carries its own persistent occupancy
  // identity; a successor certified on a different occupancy at that revision
  // cannot be ordered against it either.
  const SuccessorEvidenceFixture fixture;
  const CertifiedRouteSuffix3D resident = fixture.route(1001U, kOccupancy);
  const FiniteExecutionState3D resident_execution =
      SuccessorEvidenceFixture::execution(1001U, kOccupancy + 1U);
  const CertifiedRouteSuffix3D successor = fixture.route(1003U, kOccupancy);
  const FiniteExecutionState3D successor_execution =
      SuccessorEvidenceFixture::execution(1003U, kOccupancy);

  EXPECT_EQ(successorEvidenceRegression(resident, resident_execution, successor,
                                        successor_execution),
            ExecutionRouteTransitionDetail3D::kSuccessorWorldContentMismatch);
}

} // namespace drone_city_nav
