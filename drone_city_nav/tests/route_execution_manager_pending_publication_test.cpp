#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <utility>

#include "execution_route_snapshot_3d_plan_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(RouteExecutionManagerPendingPublicationTest,
     AssignsSequenceOnlyAfterAtomicBaseValidation) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  RouteExecutionManager3D manager;
  const std::shared_ptr<const ExecutionPlan3D> expected_base = manager.plan();
  ASSERT_NE(expected_base, nullptr);

  const auto draft = [&] {
    return PendingCertifiedRoute3D{
        .publication_sequence = 0U,
        .base_execution_owner_epoch = expected_base->execution_owner_epoch,
        .base_kind = PendingExecutionBaseKind3D::kEmpty,
        .base_route_generation = 0U,
        .base_geometry_revision = 0U,
        .base_continuity_id = 0U,
        .base_direct_tracking_identity = std::nullopt,
        .route_splice = std::nullopt,
        .route = certified,
    };
  };

  const PendingRoutePublicationResult3D first =
      manager.publishPendingForCurrentBase(expected_base, draft());
  ASSERT_TRUE(first.published());
  ASSERT_NE(first.pending, nullptr);
  EXPECT_EQ(first.pending, manager.pending());
  EXPECT_EQ(first.pending->publication_sequence, 1U);

  const PendingRoutePublicationResult3D occupied =
      manager.publishPendingForCurrentBase(expected_base, draft());
  EXPECT_EQ(occupied.status, PendingRoutePublicationStatus3D::kPendingOccupied);
  EXPECT_EQ(occupied.pending, nullptr);
  ASSERT_TRUE(manager.acknowledgePendingIfSame(first.pending));

  const PendingRoutePublicationResult3D second =
      manager.publishPendingForCurrentBase(expected_base, draft());
  ASSERT_TRUE(second.published());
  ASSERT_NE(second.pending, nullptr);
  EXPECT_EQ(second.pending->publication_sequence, 2U);
  ASSERT_TRUE(manager.acknowledgePendingIfSame(second.pending));

  PendingCertifiedRoute3D externally_numbered = draft();
  externally_numbered.publication_sequence = 9U;
  EXPECT_EQ(
      manager
          .publishPendingForCurrentBase(expected_base, std::move(externally_numbered))
          .status,
      PendingRoutePublicationStatus3D::kInvalidCandidate);

  const std::shared_ptr<const ExecutionPlan3D> stale_base = fixture.activeSnapshot();
  ASSERT_NE(stale_base, nullptr);
  EXPECT_EQ(manager.publishPendingForCurrentBase(stale_base, draft()).status,
            PendingRoutePublicationStatus3D::kStaleExecutionBase);
  EXPECT_EQ(manager.pending(), nullptr);
}

TEST(RouteExecutionManagerPendingPublicationTest,
     ReplacesOnlyTheExactPendingAndPreservesMonotonicSequence) {
  SnapshotFixture3D fixture;
  const std::optional<CertifiedRouteSuffix3D> suffix = fixture.certify();
  ASSERT_TRUE(suffix.has_value());
  const CertifiedRouteSuffix3D& certified =
      suffix.value(); // NOLINT(bugprone-unchecked-optional-access)
  RouteExecutionManager3D manager;
  const std::shared_ptr<const ExecutionPlan3D> expected_base = manager.plan();
  ASSERT_NE(expected_base, nullptr);
  const auto draft = [&] {
    return PendingCertifiedRoute3D{
        .publication_sequence = 0U,
        .base_execution_owner_epoch = expected_base->execution_owner_epoch,
        .base_kind = PendingExecutionBaseKind3D::kEmpty,
        .base_route_generation = 0U,
        .base_geometry_revision = 0U,
        .base_continuity_id = 0U,
        .base_direct_tracking_identity = std::nullopt,
        .route_splice = std::nullopt,
        .route = certified,
    };
  };
  const PendingRoutePublicationResult3D first =
      manager.publishPendingForCurrentBase(expected_base, draft());
  ASSERT_TRUE(first.published());

  const PendingRoutePublicationResult3D replaced =
      manager.replacePendingForCurrentBase(expected_base, first.pending, draft());
  ASSERT_TRUE(replaced.published());
  EXPECT_EQ(replaced.status, PendingRoutePublicationStatus3D::kReplaced);
  ASSERT_NE(replaced.pending, nullptr);
  EXPECT_EQ(replaced.pending->publication_sequence, 2U);
  EXPECT_EQ(manager.pending(), replaced.pending);
  EXPECT_FALSE(manager.acknowledgePendingIfSame(first.pending));

  const PendingRoutePublicationResult3D stale =
      manager.replacePendingForCurrentBase(expected_base, first.pending, draft());
  EXPECT_EQ(stale.status, PendingRoutePublicationStatus3D::kPendingChanged);
  EXPECT_EQ(manager.pending(), replaced.pending);
}

TEST(RouteExecutionManagerPendingPublicationTest,
     SemanticBaseIgnoresSnapshotVersionButRejectsOwnerEpochChange) {
  SnapshotFixture3D fixture;
  const std::shared_ptr<const ExecutionPlan3D> active = fixture.activeSnapshot();
  ASSERT_NE(active, nullptr);

  auto refreshed = std::make_shared<ExecutionPlan3D>(*active);
  ++refreshed->version;
  ASSERT_TRUE(refreshed->valid());
  EXPECT_TRUE(sameExecutionRouteBase3D(active, refreshed));

  auto different_owner = std::make_shared<ExecutionPlan3D>(*refreshed);
  ++different_owner->execution_owner_epoch;
  ASSERT_TRUE(different_owner->valid());
  EXPECT_FALSE(sameExecutionRouteBase3D(active, different_owner));
}

} // namespace
} // namespace drone_city_nav
