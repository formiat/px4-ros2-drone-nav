#include "drone_city_nav/execution_publication_currentness_3d.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <barrier>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>
#include <thread>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kAcquisitionStampNs{1'000'000'000LL};
constexpr std::int64_t kReceiveStampNs{1'001'000'000LL};
constexpr double kMaximumLidarAgeMs{20.0};

[[nodiscard]] LatestLidarEvidenceCapture3D
lidarCapture(const std::uint64_t sequence = 11U) {
  return LatestLidarEvidenceCapture3D{
      .producer_instance_id = 7U,
      .sequence = sequence,
      .pose_generation = 5U,
      .acquisition_stamp_ns =
          kAcquisitionStampNs + static_cast<std::int64_t>(sequence - 11U) * 1'000'000LL,
      .receive_stamp_ns =
          kReceiveStampNs + static_cast<std::int64_t>(sequence - 11U) * 1'000'000LL,
      .source_beam_count = 2U,
      .invalid_beam_count = 1U,
      .hit_points_map_m = {{1.0, 2.0, 3.0}},
  };
}

[[nodiscard]] std::shared_ptr<const VersionedLatestLidarEvidence3D>
lidarEvidence(const std::uint64_t sequence = 11U) {
  return VersionedLatestLidarEvidence3D::capture(lidarCapture(sequence));
}

[[nodiscard]] std::shared_ptr<const ObservedOccupancyGrid3D>
rawObservation(const bool occupied = false) {
  auto observation = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{-5.0, -5.0, 0.0, 1.0, 20, 10, 10});
  if (occupied) {
    static_cast<void>(observation->setState({10, 5, 5}, ObservedVoxelState::kOccupied));
  }
  return observation;
}

[[nodiscard]] RawMapVersion rawVersion(const std::uint64_t revision = 4U) {
  return RawMapVersion{
      .producer_instance_id = 13U,
      .base_snapshot_revision = 2U,
      .revision = revision,
  };
}

[[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D>
rawWorld(std::shared_ptr<const ObservedOccupancyGrid3D> observation,
         const RawMapVersion version = rawVersion()) {
  return VersionedObservedRawWorld3D::captureOwned(version, std::move(observation),
                                                   std::nullopt, std::nullopt);
}

[[nodiscard]] ExecutionPublicationCurrentnessCheck3D
currentCheck(std::shared_ptr<const ExecutionRouteSnapshot3D> snapshot,
             std::shared_ptr<const VersionedLatestLidarEvidence3D> lidar) {
  return ExecutionPublicationCurrentnessCheck3D{
      .expected_snapshot = snapshot,
      .current_snapshot = std::move(snapshot),
      .raw_requirement = ExecutionPublicationRawRequirement3D::kOptional,
      .expected_raw_world = nullptr,
      .current_raw_world = nullptr,
      .expected_lidar_evidence = lidar,
      .current_lidar_evidence = std::move(lidar),
      .publication_now_ns = kAcquisitionStampNs + 20'000'000LL,
      .maximum_lidar_age_ms = kMaximumLidarAgeMs,
  };
}

TEST(ExecutionPublicationCurrentness3DTest,
     AcceptsExactSnapshotAndLidarOwnersWhenFresh) {
  const auto snapshot = makeInitialExecutionRouteSnapshot3D();
  const auto lidar = lidarEvidence();
  ASSERT_NE(snapshot, nullptr);
  ASSERT_NE(lidar, nullptr);

  const ExecutionPublicationCurrentnessStatus3D status =
      assessExecutionPublicationCurrentness3D(currentCheck(snapshot, lidar));

  EXPECT_EQ(status, ExecutionPublicationCurrentnessStatus3D::kCurrent);
  EXPECT_EQ(executionPublicationCurrentnessStatus3DName(status), "current");
  EXPECT_EQ(executionPublicationCurrentnessStatus3DName(
                static_cast<ExecutionPublicationCurrentnessStatus3D>(255U)),
            "unknown");

  ExecutionPublicationCurrentnessCheck3D invalid_requirement =
      currentCheck(snapshot, lidar);
  invalid_requirement.raw_requirement =
      static_cast<ExecutionPublicationRawRequirement3D>(255U);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(invalid_requirement),
            ExecutionPublicationCurrentnessStatus3D::kInvalidRawRequirement);
}

TEST(ExecutionPublicationCurrentness3DTest,
     RejectsMissingInvalidAndDistinctSnapshotOwners) {
  const auto snapshot = makeInitialExecutionRouteSnapshot3D();
  const auto lidar = lidarEvidence();
  ASSERT_NE(snapshot, nullptr);
  ASSERT_NE(lidar, nullptr);

  ExecutionPublicationCurrentnessCheck3D check = currentCheck(snapshot, lidar);
  check.current_snapshot.reset();
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kSnapshotMissing);

  auto invalid_snapshot = std::make_shared<ExecutionRouteSnapshot3D>(*snapshot);
  invalid_snapshot->version = 0U;
  check = currentCheck(snapshot, lidar);
  check.current_snapshot = std::move(invalid_snapshot);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kSnapshotInvalid);

  check = currentCheck(snapshot, lidar);
  check.current_snapshot = makeInitialExecutionRouteSnapshot3D();
  ASSERT_NE(check.current_snapshot, nullptr);
  ASSERT_TRUE(check.current_snapshot->valid());
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kSnapshotOwnerChanged);
}

TEST(ExecutionPublicationCurrentness3DTest,
     RawRequirementRejectsMissingPresenceAndLineageChanges) {
  const auto snapshot = makeInitialExecutionRouteSnapshot3D();
  const auto lidar = lidarEvidence();
  const auto observation = rawObservation();
  const auto expected_raw = rawWorld(observation);
  ASSERT_NE(snapshot, nullptr);
  ASSERT_NE(lidar, nullptr);
  ASSERT_NE(expected_raw, nullptr);

  ExecutionPublicationCurrentnessCheck3D check = currentCheck(snapshot, lidar);
  check.raw_requirement = ExecutionPublicationRawRequirement3D::kRequired;
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRequiredRawEvidenceMissing);

  check.raw_requirement = ExecutionPublicationRawRequirement3D::kOptional;
  check.expected_raw_world = expected_raw;
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRawEvidencePresenceChanged);

  check.raw_requirement = ExecutionPublicationRawRequirement3D::kRequired;
  check.current_raw_world = rawWorld(observation, rawVersion(5U));
  ASSERT_NE(check.current_raw_world, nullptr);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRevalidationRequired);

  RawMapVersion rebased_successor_version = rawVersion(5U);
  rebased_successor_version.base_snapshot_revision = 5U;
  check.current_raw_world = rawWorld(observation, rebased_successor_version);
  ASSERT_NE(check.current_raw_world, nullptr);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRevalidationRequired);

  RawMapVersion changed_base_version = rawVersion();
  ++changed_base_version.base_snapshot_revision;
  check.current_raw_world = rawWorld(observation, changed_base_version);
  ASSERT_NE(check.current_raw_world, nullptr);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRawVersionChanged);

  RawMapVersion changed_producer_version = rawVersion();
  ++changed_producer_version.producer_instance_id;
  check.current_raw_world = rawWorld(observation, changed_producer_version);
  ASSERT_NE(check.current_raw_world, nullptr);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRawVersionChanged);

  check.current_raw_world = rawWorld(rawObservation());
  ASSERT_NE(check.current_raw_world, nullptr);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRawObservationOwnerChanged);
}

TEST(ExecutionPublicationCurrentness3DTest,
     AcceptsDerivedRawEvidenceSharingTheExactObservationAndVersion) {
  const auto snapshot = makeInitialExecutionRouteSnapshot3D();
  const auto lidar = lidarEvidence();
  const auto base_raw = rawWorld(rawObservation());
  ASSERT_NE(snapshot, nullptr);
  ASSERT_NE(lidar, nullptr);
  ASSERT_NE(base_raw, nullptr);

  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = {2.0, 0.0, 5.0},
      .body_axis = {},
      .footprint =
          SweptFootprintConfig{
              .radius_m = 0.0,
              .perimeter_samples = 0U,
              .radial_rings = 0U,
              .axial_samples = 1U,
              .sweep_step_m = 0.25,
          },
  };
  const auto derived_raw = base_raw->deriveRouteEvidence(seed, std::nullopt);
  ASSERT_NE(derived_raw, nullptr);
  ASSERT_NE(derived_raw, base_raw);
  ASSERT_NE(derived_raw->contentFingerprint(), base_raw->contentFingerprint());

  ExecutionPublicationCurrentnessCheck3D check = currentCheck(snapshot, lidar);
  check.raw_requirement = ExecutionPublicationRawRequirement3D::kRequired;
  check.expected_raw_world = derived_raw;
  check.current_raw_world = base_raw;

  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kCurrent);
}

TEST(ExecutionPublicationCurrentness3DTest,
     RequiresRevalidationForAdvancedLidarAndAcceptsCopiesOfTheSameEvidence) {
  const auto snapshot = makeInitialExecutionRouteSnapshot3D();
  const auto expected_lidar = lidarEvidence();
  ASSERT_NE(snapshot, nullptr);
  ASSERT_NE(expected_lidar, nullptr);

  ExecutionPublicationCurrentnessCheck3D check = currentCheck(snapshot, expected_lidar);
  check.current_lidar_evidence.reset();
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kLidarEvidenceMissing);

  check.current_lidar_evidence = lidarEvidence(12U);
  ASSERT_NE(check.current_lidar_evidence, nullptr);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kRevalidationRequired);

  LatestLidarEvidenceCapture3D changed_content = lidarCapture();
  changed_content.hit_points_map_m.front().x += 1.0;
  check.current_lidar_evidence =
      VersionedLatestLidarEvidence3D::capture(std::move(changed_content));
  ASSERT_NE(check.current_lidar_evidence, nullptr);
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kLidarContentChanged);

  check.current_lidar_evidence = lidarEvidence();
  ASSERT_NE(check.current_lidar_evidence, nullptr);
  ASSERT_NE(check.current_lidar_evidence, expected_lidar);
  ASSERT_EQ(check.current_lidar_evidence->contentFingerprint(),
            expected_lidar->contentFingerprint());
  ASSERT_EQ(check.current_lidar_evidence->evidenceId(), expected_lidar->evidenceId());
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kCurrent);
}

TEST(ExecutionPublicationCurrentness3DTest,
     RechecksLidarFreshnessAtTheExactPublicationBoundary) {
  const auto snapshot = makeInitialExecutionRouteSnapshot3D();
  const auto lidar = lidarEvidence();
  ASSERT_NE(snapshot, nullptr);
  ASSERT_NE(lidar, nullptr);

  ExecutionPublicationCurrentnessCheck3D check = currentCheck(snapshot, lidar);
  check.publication_now_ns = kAcquisitionStampNs + 20'000'000LL;
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kCurrent);

  ++check.publication_now_ns;
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kLidarNotFresh);

  check.publication_now_ns = 0;
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kInvalidPublicationTime);

  check.publication_now_ns = kAcquisitionStampNs + 20'000'000LL;
  check.maximum_lidar_age_ms = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(assessExecutionPublicationCurrentness3D(check),
            ExecutionPublicationCurrentnessStatus3D::kInvalidPublicationTime);
}

TEST(ExecutionPublicationCurrentness3DTest,
     ConcurrentNewerLidarEvidenceRequiresPublicationRevalidation) {
  const auto snapshot = makeInitialExecutionRouteSnapshot3D();
  const auto initial_lidar = lidarEvidence();
  const auto newer_lidar = lidarEvidence(12U);
  ASSERT_NE(snapshot, nullptr);
  ASSERT_NE(initial_lidar, nullptr);
  ASSERT_NE(newer_lidar, nullptr);

  std::atomic<std::shared_ptr<const VersionedLatestLidarEvidence3D>> latest_lidar{
      initial_lidar};
  std::barrier evidence_captured{2};
  std::barrier evidence_updated{2};
  ExecutionPublicationCurrentnessStatus3D status{
      ExecutionPublicationCurrentnessStatus3D::kCurrent};

  std::thread publisher{[&] {
    const auto expected_lidar = latest_lidar.load(std::memory_order_acquire);
    evidence_captured.arrive_and_wait();
    evidence_updated.arrive_and_wait();
    ExecutionPublicationCurrentnessCheck3D check =
        currentCheck(snapshot, expected_lidar);
    check.current_lidar_evidence = latest_lidar.load(std::memory_order_acquire);
    status = assessExecutionPublicationCurrentness3D(check);
  }};
  std::thread updater{[&] {
    evidence_captured.arrive_and_wait();
    latest_lidar.store(newer_lidar, std::memory_order_release);
    evidence_updated.arrive_and_wait();
  }};

  publisher.join();
  updater.join();

  EXPECT_EQ(status, ExecutionPublicationCurrentnessStatus3D::kRevalidationRequired);
}

} // namespace
} // namespace drone_city_nav
