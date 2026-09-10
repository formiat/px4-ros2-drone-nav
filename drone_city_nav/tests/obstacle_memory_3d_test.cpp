#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

constexpr GridBounds3D kBounds{0.0, 0.0, 0.0, 1.0, 32, 32, 16};

TEST(ObservedOccupancyGrid3D, PreservesUnknownFreeAndOccupiedStates) {
  ObservedOccupancyGrid3D grid{kBounds};
  EXPECT_EQ(grid.state({1, 2, 3}), ObservedVoxelState::kUnknown);
  EXPECT_TRUE(grid.setState({1, 2, 3}, ObservedVoxelState::kFree));
  EXPECT_TRUE(grid.setState({17, 18, 4}, ObservedVoxelState::kOccupied));

  EXPECT_TRUE(grid.isKnownFree({1, 2, 3}));
  EXPECT_TRUE(grid.isOccupied({17, 18, 4}));
  EXPECT_FALSE(grid.isKnown({2, 2, 3}));
  EXPECT_EQ(grid.knownVoxelCount(), 2U);
  EXPECT_EQ(grid.freeVoxelCount(), 1U);
  EXPECT_EQ(grid.occupiedVoxelCount(), 1U);
  EXPECT_EQ(grid.chunks().size(), 2U);
}

TEST(ObservedOccupancyGrid3D, CopiesRemainIndependentAfterChunkMutations) {
  ObservedOccupancyGrid3D original{kBounds};
  constexpr GridIndex3D first{1, 2, 3};
  constexpr GridIndex3D second{17, 18, 4};
  ASSERT_TRUE(original.setState(first, ObservedVoxelState::kFree));
  ASSERT_TRUE(original.setState(second, ObservedVoxelState::kOccupied));

  ObservedOccupancyGrid3D copy = original;
  ASSERT_TRUE(copy.setState(first, ObservedVoxelState::kOccupied));

  EXPECT_TRUE(original.isKnownFree(first));
  EXPECT_TRUE(original.isOccupied(second));
  EXPECT_EQ(original.freeVoxelCount(), 1U);
  EXPECT_EQ(original.occupiedVoxelCount(), 1U);
  EXPECT_TRUE(copy.isOccupied(first));
  EXPECT_TRUE(copy.isOccupied(second));
  EXPECT_EQ(copy.freeVoxelCount(), 0U);
  EXPECT_EQ(copy.occupiedVoxelCount(), 2U);

  ASSERT_TRUE(original.setState(second, ObservedVoxelState::kUnknown));
  EXPECT_EQ(original.state(second), ObservedVoxelState::kUnknown);
  EXPECT_TRUE(copy.isOccupied(second));
}

TEST(ObservedOccupancyGrid3D, CropDoesNotTurnUnknownIntoFree) {
  ObservedOccupancyGrid3D grid{kBounds};
  static_cast<void>(grid.setState({8, 8, 4}, ObservedVoxelState::kFree));
  static_cast<void>(grid.setState({9, 8, 4}, ObservedVoxelState::kOccupied));

  const ObservedOccupancyGrid3D cropped =
      grid.crop(GridBounds3D{4.0, 4.0, 2.0, 1.0, 12, 12, 8});
  EXPECT_TRUE(cropped.isKnownFree({4, 4, 2}));
  EXPECT_TRUE(cropped.isOccupied({5, 4, 2}));
  EXPECT_EQ(cropped.state({3, 4, 2}), ObservedVoxelState::kUnknown);
}

TEST(ObstacleMemory3D, SurfaceOnlyBeamMarksItsEndpointWithoutCarvingFreeSpace) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array beams{LidarBeam3D{.direction_map = {1.0, 0.0, 0.0},
                                     .range_m = 8.0,
                                     .hit = true,
                                     .valid = true,
                                     .surface_only = true}};

  const ObstacleMemory3DStats stats = memory.integrateScan(
      LidarScan3DView{.origin_map = {0.5, 0.5, 0.5}, .beams = beams});

  EXPECT_EQ(stats.surface_beams, 1U);
  EXPECT_EQ(stats.hit_beams, 1U);
  EXPECT_EQ(stats.free_voxel_updates, 0U);
  EXPECT_TRUE(memory.grid().isOccupied({8, 0, 0}));
  EXPECT_EQ(memory.grid().state({4, 0, 0}), ObservedVoxelState::kUnknown);
  EXPECT_EQ(memory.grid().state({0, 0, 0}), ObservedVoxelState::kUnknown);
}

TEST(ObstacleMemory3D, IntegratesHitAndMissEvidenceAlongFullRay) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array beams{
      LidarBeam3D{
          .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true},
      LidarBeam3D{.direction_map = {0.0, 1.0, 0.0},
                  .range_m = 10.0,
                  .hit = false,
                  .valid = true}};
  const ObstacleMemory3DStats stats =
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = beams});

  EXPECT_EQ(stats.hit_beams, 1U);
  EXPECT_EQ(stats.miss_beams, 1U);
  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  EXPECT_TRUE(memory.grid().isKnownFree({5, 1, 2}));
  EXPECT_TRUE(memory.grid().isKnownFree({1, 8, 2}));
  EXPECT_EQ(memory.grid().state({12, 12, 2}), ObservedVoxelState::kUnknown);
  EXPECT_GT(memory.revision(), 0U);
}

TEST(ObstacleMemory3D, DoesNotAmplifyOneScanAcrossAnAcquisitionGap) {
  ObstacleMemory3D memory{kBounds,
                          ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                 .minimum_range_m = 0.1,
                                                 .hit_weight = 4,
                                                 .miss_weight = 1,
                                                 .occupied_score = 3,
                                                 .free_score = -1,
                                                 .nominal_evidence_interval_s = 0.1,
                                                 .maximum_evidence_interval_s = 0.5}};
  const std::array hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  const std::array miss{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 12.0, .hit = false, .valid = true}};

  static_cast<void>(memory.integrateScan({.origin_map = {1.5, 1.5, 2.5},
                                          .beams = hit,
                                          .acquisition_stamp_ns = 1'000'000'000}));
  ASSERT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  const ObstacleMemory3DStats stats =
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5},
                            .beams = miss,
                            .acquisition_stamp_ns = 1'500'000'000});

  EXPECT_NEAR(stats.evidence_interval_s, 0.5, 1.0e-12);
  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
}

TEST(ObstacleMemory3D, RejectsReplayedAcquisitionWithoutChangingEvidence) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  const std::array miss{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 12.0, .hit = false, .valid = true}};

  static_cast<void>(memory.integrateScan({.origin_map = {1.5, 1.5, 2.5},
                                          .beams = hit,
                                          .acquisition_stamp_ns = 1'000'000'000}));
  const ObstacleMemory3DStats stats =
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5},
                            .beams = miss,
                            .acquisition_stamp_ns = 1'000'000'000});

  EXPECT_TRUE(stats.stale_acquisition);
  EXPECT_EQ(stats.processed_beams, 0U);
  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
}

TEST(ObstacleMemory3D, NoReturnMarksFreeSpaceThroughMaximumSensorRange) {
  constexpr GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 48, 8, 8};
  ObstacleMemory3D memory{
      bounds, ObstacleMemory3DConfig{.maximum_range_m = 35.0, .minimum_range_m = 0.1}};
  const std::array beams{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 35.0, .hit = false, .valid = true}};

  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 1.5}, .beams = beams}));

  EXPECT_TRUE(memory.grid().isKnownFree({1, 1, 1}));
  EXPECT_TRUE(memory.grid().isKnownFree({35, 1, 1}));
  EXPECT_TRUE(memory.grid().isKnownFree({36, 1, 1}));
  EXPECT_EQ(memory.grid().state({37, 1, 1}), ObservedVoxelState::kUnknown);
}

TEST(ObstacleMemory3D, TraversesEveryVoxelAlongAnObliqueRay) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array beams{LidarBeam3D{
      .direction_map = {1.0, 0.5, 0.25}, .range_m = 10.0, .hit = false, .valid = true}};

  const ObstacleMemory3DStats stats =
      memory.integrateScan({.origin_map = {1.5, 1.5, 1.5}, .beams = beams});

  EXPECT_GT(stats.free_voxel_updates, 8U);
  EXPECT_TRUE(memory.grid().isKnownFree({1, 1, 1}));
  EXPECT_TRUE(memory.grid().isKnownFree({5, 3, 2}));
  EXPECT_TRUE(memory.grid().isKnownFree({9, 5, 3}));
}

TEST(ObstacleMemory3D, ClipsRaysThatEnterTheGridFromOutside) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array beams{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 11.5, .hit = false, .valid = true}};

  static_cast<void>(
      memory.integrateScan({.origin_map = {-2.0, 3.5, 2.5}, .beams = beams}));

  EXPECT_TRUE(memory.grid().isKnownFree({0, 3, 2}));
  EXPECT_TRUE(memory.grid().isKnownFree({9, 3, 2}));
  EXPECT_EQ(memory.grid().state({10, 3, 2}), ObservedVoxelState::kUnknown);
}

TEST(ObstacleMemory3D, DoesNotCreateAnOccupiedVoxelAtTruncatedMaximumRange) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 5.0, .minimum_range_m = 0.1}};
  const std::array beams{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};

  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = beams}));

  EXPECT_TRUE(memory.grid().isKnownFree({6, 1, 2}));
  EXPECT_FALSE(memory.grid().isOccupied({6, 1, 2}));
  EXPECT_EQ(memory.grid().occupiedVoxelCount(), 0U);
}

TEST(ObstacleMemory3D, ConflictingEvidenceCanClearStaleOccupiedVoxel) {
  ObstacleMemory3D memory{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                          .minimum_range_m = 0.1,
                                                          .hit_weight = 4,
                                                          .miss_weight = 2,
                                                          .minimum_score = -8,
                                                          .maximum_score = 12,
                                                          .occupied_score = 3,
                                                          .free_score = -1}};
  const std::array hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = hit}));
  ASSERT_TRUE(memory.grid().isOccupied({9, 1, 2}));

  const std::array miss{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 12.0, .hit = false, .valid = true}};
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));

  EXPECT_TRUE(memory.grid().isKnownFree({9, 1, 2}));
}

TEST(ObstacleMemory3D, AFarHitOccupiesAVoxelThatTheFirstNearLookFrees) {
  ObstacleMemory3D memory{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                          .minimum_range_m = 0.1,
                                                          .near_hit_range_m = 5.0,
                                                          .hit_weight = 4,
                                                          .miss_weight = 1,
                                                          .minimum_score = -8,
                                                          .maximum_score = 12,
                                                          .occupied_score = 3,
                                                          .free_score = -1}};
  const std::array far_hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  const std::array miss{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 12.0, .hit = false, .valid = true}};
  // Three far hits leave the voxel at the occupied threshold, not saturated.
  for (int scan = 0; scan < 3; ++scan) {
    static_cast<void>(
        memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = far_hit}));
  }
  ASSERT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  for (int scan = 0; scan < 3; ++scan) {
    static_cast<void>(
        memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));
    EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2})) << "scan " << scan;
  }
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));
  EXPECT_TRUE(memory.grid().isKnownFree({9, 1, 2}));

  // A near hit saturates the voxel: the same four misses leave it occupied.
  const std::array near_hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 4.0, .hit = true, .valid = true}};
  for (int scan = 0; scan < 3; ++scan) {
    static_cast<void>(
        memory.integrateScan({.origin_map = {5.5, 1.5, 2.5}, .beams = near_hit}));
  }
  ASSERT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  for (int scan = 0; scan < 4; ++scan) {
    static_cast<void>(
        memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));
  }
  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
}

TEST(ObstacleMemory3D, HitEndpointDominatesCrossingMissesWithinOneScan) {
  ObstacleMemory3D memory{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                          .minimum_range_m = 0.1,
                                                          .hit_weight = 4,
                                                          .miss_weight = 1,
                                                          .minimum_score = -8,
                                                          .maximum_score = 12,
                                                          .occupied_score = 3,
                                                          .free_score = -1}};
  const LidarBeam3D hit{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true};
  const LidarBeam3D miss{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 12.0, .hit = false, .valid = true};
  const std::array beams{hit, miss, miss, miss, miss, miss};

  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = beams}));

  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
}

TEST(ObstacleMemory3D, DuplicateMissesContributeOncePerScan) {
  ObstacleMemory3D memory{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                          .minimum_range_m = 0.1,
                                                          .hit_weight = 4,
                                                          .miss_weight = 2,
                                                          .minimum_score = -8,
                                                          .maximum_score = 12,
                                                          .occupied_score = 3,
                                                          .free_score = -1}};
  const std::array hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  const LidarBeam3D miss{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 12.0, .hit = false, .valid = true};
  const std::array duplicate_misses{miss, miss, miss};
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = hit}));

  // One hit scores 4. Each miss scan subtracts the miss weight once, and the
  // occupied state is retained while the score stays above free_score: 2 and
  // 0 keep the voxel occupied, -2 frees it. Counting the duplicates three
  // times would free the voxel after the first scan.
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = duplicate_misses}));
  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = duplicate_misses}));
  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = duplicate_misses}));
  EXPECT_TRUE(memory.grid().isKnownFree({9, 1, 2}));
}

TEST(ObstacleMemory3D, StateHysteresisKeepsASurfaceVoxelOccupiedAcrossGrazingMisses) {
  ObstacleMemory3D memory{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                          .minimum_range_m = 0.1,
                                                          .hit_weight = 4,
                                                          .miss_weight = 1,
                                                          .minimum_score = -8,
                                                          .maximum_score = 12,
                                                          .occupied_score = 3,
                                                          .free_score = -1}};
  const std::array hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  const std::array miss{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 12.0, .hit = false, .valid = true}};
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = hit}));
  ASSERT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  // Scores 3, 2, 1, 0 all keep the occupied state; -1 releases it.
  for (int scan = 0; scan < 4; ++scan) {
    static_cast<void>(
        memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));
    EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2})) << scan;
  }
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = miss}));
  EXPECT_TRUE(memory.grid().isKnownFree({9, 1, 2}));
  // A free voxel likewise keeps its state until the score reaches
  // occupied_score again.
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = hit}));
  EXPECT_TRUE(memory.grid().isOccupied({9, 1, 2}));
}

TEST(ObstacleMemory3D, ExposesRevisionedDirtyChunks) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  EXPECT_TRUE(memory.takeChanges().full_reset);
  const std::array beams{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 20.0, .hit = true, .valid = true}};
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = beams}));
  const ObstacleMemory3DChanges changes = memory.takeChanges();
  EXPECT_GT(changes.revision, 0U);
  EXPECT_GE(changes.dirty_chunks.size(), 2U);
  EXPECT_TRUE(memory.takeChanges().dirty_chunks.empty());
}

TEST(ObstacleMemory3D, ForgetsOnlyVoxelsOverlappedByDynamicVolumes) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array dynamic_hit{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  const std::array static_hit{LidarBeam3D{
      .direction_map = {0.0, 1.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = dynamic_hit}));
  static_cast<void>(
      memory.integrateScan({.origin_map = {1.5, 1.5, 2.5}, .beams = static_hit}));
  static_cast<void>(memory.takeChanges());
  ASSERT_TRUE(memory.grid().isOccupied({9, 1, 2}));
  ASSERT_TRUE(memory.grid().isOccupied({1, 9, 2}));
  const std::uint64_t revision_before = memory.revision();

  const std::array volumes{DynamicAgentLidarVolume{
      .position = {9.5, 1.5, 2.5},
      .radius_m = 0.6,
      .lower_extent_m = 0.6,
      .upper_extent_m = 0.6,
  }};
  EXPECT_GT(memory.forgetDynamicVolumes(volumes), 0U);

  EXPECT_EQ(memory.grid().state({9, 1, 2}), ObservedVoxelState::kUnknown);
  EXPECT_TRUE(memory.grid().isOccupied({1, 9, 2}));
  EXPECT_GT(memory.revision(), revision_before);
  EXPECT_FALSE(memory.takeChanges().dirty_chunks.empty());
}

} // namespace
} // namespace drone_city_nav
