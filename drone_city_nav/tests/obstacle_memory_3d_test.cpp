#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

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

TEST(ObstacleMemory3D, DarknessIsUnobservableUntilAMeasuredReturnHitsIt) {
  // Specification K14: the dark a sensor looked into is occupied without a
  // measured hit, told apart from a surface for RViz; a measured return
  // makes it a surface, free rays through it make it free and no longer
  // dark.
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array dark{LidarBeam3D{.direction_map = {1.0, 0.0, 0.0},
                                    .range_m = 8.0,
                                    .hit = true,
                                    .valid = true,
                                    .surface_only = true}};
  static_cast<void>(memory.integrateScan(
      LidarScan3DView{.origin_map = {0.5, 0.5, 0.5}, .beams = dark}));
  EXPECT_TRUE(memory.grid().isOccupied({8, 0, 0}));
  EXPECT_TRUE(memory.grid().isUnobservable({8, 0, 0}));

  const std::array measured{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  static_cast<void>(memory.integrateScan(
      LidarScan3DView{.origin_map = {0.5, 0.5, 0.5}, .beams = measured}));
  EXPECT_TRUE(memory.grid().isOccupied({8, 0, 0}));
  EXPECT_FALSE(memory.grid().isUnobservable({8, 0, 0}));
  // A surface the dark confirms stays a surface.
  static_cast<void>(memory.integrateScan(
      LidarScan3DView{.origin_map = {0.5, 0.5, 0.5}, .beams = dark}));
  EXPECT_FALSE(memory.grid().isUnobservable({8, 0, 0}));

  const std::array through{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 15.0, .hit = false, .valid = true}};
  ObstacleMemory3D dark_memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  static_cast<void>(dark_memory.integrateScan(
      LidarScan3DView{.origin_map = {0.5, 0.5, 0.5}, .beams = dark}));
  for (int scan = 0; scan < 20; ++scan) {
    static_cast<void>(dark_memory.integrateScan(
        LidarScan3DView{.origin_map = {0.5, 0.5, 0.5}, .beams = through}));
  }
  EXPECT_TRUE(dark_memory.grid().isKnownFree({8, 0, 0}));
  EXPECT_FALSE(dark_memory.grid().isUnobservable({8, 0, 0}));
}

TEST(ObservedOccupancyGrid3D, OnlyAnOccupiedVoxelIsUnobservable) {
  ObservedOccupancyGrid3D grid{kBounds};
  EXPECT_FALSE(grid.setUnobservable({1, 2, 3}, true));
  EXPECT_TRUE(grid.setState({1, 2, 3}, ObservedVoxelState::kOccupied));
  EXPECT_TRUE(grid.setUnobservable({1, 2, 3}, true));
  EXPECT_FALSE(grid.setUnobservable({1, 2, 3}, true));
  EXPECT_TRUE(grid.isUnobservable({1, 2, 3}));
  EXPECT_TRUE(grid.setState({1, 2, 3}, ObservedVoxelState::kFree));
  EXPECT_FALSE(grid.isUnobservable({1, 2, 3}));
  // A replaced chunk's darkness is a subset of its occupancy.
  ObservedOccupancyGrid3D::Chunk chunk{};
  chunk.observed[0] = 0b11U;
  chunk.occupied[0] = 0b01U;
  chunk.unobservable[0] = 0b11U;
  EXPECT_TRUE(grid.replaceChunk({0, 0, 0}, chunk));
  EXPECT_TRUE(grid.isUnobservable({0, 0, 0}));
  EXPECT_FALSE(grid.isUnobservable({1, 0, 0}));
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

TEST(ObstacleMemory3D, ARayGrazingAFloorCarvesNoVoxelOfIt) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  // A floor patch at z = 1, x 8 to 12 and y 0 to 2, hit from above.
  for (int x = 8; x <= 12; ++x) {
    for (int y = 0; y <= 2; ++y) {
      const std::array hit{LidarBeam3D{.direction_map = {0.0, 0.0, -1.0},
                                       .range_m = 2.5 - 1.5,
                                       .hit = true,
                                       .valid = true}};
      static_cast<void>(
          memory.integrateScan({.origin_map = {x + 0.5, y + 0.5, 2.5}, .beams = hit}));
    }
  }
  const GridIndex3D floor{10, 1, 1};
  ASSERT_TRUE(memory.grid().isOccupied(floor));
  // Rays grazing the floor at 5 degrees from the west, passing through that
  // voxel and ending 15 m on: the floor stays.
  const double sine = std::sin(5.0 * std::numbers::pi / 180.0);
  const double cosine = std::cos(5.0 * std::numbers::pi / 180.0);
  for (int scan = 0; scan < 30; ++scan) {
    const std::array grazing{LidarBeam3D{.direction_map = {cosine, 0.0, -sine},
                                         .range_m = 18.0,
                                         .hit = true,
                                         .valid = true}};
    static_cast<void>(memory.integrateScan(
        {.origin_map = {0.5, 1.5, 1.9 + 10.0 * sine}, .beams = grazing}));
  }
  EXPECT_TRUE(memory.grid().isOccupied(floor));
  // Rays through it at 45 degrees carve it as before.
  for (int scan = 0; scan < 30; ++scan) {
    const std::array steep{LidarBeam3D{.direction_map = {0.7071, 0.0, -0.7071},
                                       .range_m = 4.0,
                                       .hit = false,
                                       .valid = true}};
    static_cast<void>(
        memory.integrateScan({.origin_map = {9.5, 1.5, 2.9}, .beams = steep}));
  }
  EXPECT_FALSE(memory.grid().isOccupied(floor));
}

TEST(ObstacleMemory3D, AnOccupancyNotConfirmedDecaysByHowOftenItWasSeen) {
  // Roadmap item 17 stage 8: a voxel seen occupied once and never again
  // returns to unknown a second later; one seen five times outlasts it by
  // five seconds; a decayed voxel seen again is occupied again.
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                      .minimum_range_m = 0.1,
                                      .hit_weight = 4,
                                      .miss_weight = 1,
                                      .occupied_score = 3,
                                      .free_score = -1,
                                      .decay_seconds_per_confirmation = 1.0}};
  const std::array once{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  const std::array often{LidarBeam3D{
      .direction_map = {0.0, 1.0, 0.0}, .range_m = 8.0, .hit = true, .valid = true}};
  constexpr GridIndex3D kSeenOnce{9, 1, 2};
  constexpr GridIndex3D kSeenOften{1, 9, 2};
  const Point3 origin{1.5, 1.5, 2.5};
  std::int64_t stamp_ns = 1'000'000'000;
  static_cast<void>(memory.integrateScan(
      {.origin_map = origin, .beams = once, .acquisition_stamp_ns = stamp_ns}));
  for (int scan = 0; scan < 5; ++scan) {
    stamp_ns += 100'000'000;
    static_cast<void>(memory.integrateScan(
        {.origin_map = origin, .beams = often, .acquisition_stamp_ns = stamp_ns}));
  }
  ASSERT_TRUE(memory.grid().isOccupied(kSeenOnce));
  stamp_ns += 1'000'000'000;
  const ObstacleMemory3DStats later = memory.integrateScan(
      {.origin_map = origin, .beams = often, .acquisition_stamp_ns = stamp_ns});
  EXPECT_EQ(later.decayed_voxels, 1U);
  EXPECT_EQ(memory.grid().state(kSeenOnce), ObservedVoxelState::kUnknown);
  EXPECT_TRUE(memory.grid().isOccupied(kSeenOften));
  EXPECT_EQ(memory.decayedVoxelTotal(), 1U);
  EXPECT_FALSE(memory.takeChanges().dirty_chunks.empty());

  stamp_ns += 100'000'000;
  static_cast<void>(memory.integrateScan(
      {.origin_map = origin, .beams = once, .acquisition_stamp_ns = stamp_ns}));
  EXPECT_TRUE(memory.grid().isOccupied(kSeenOnce));
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

// Roadmap item 21, specification K65: the moving layer.
constexpr std::int64_t kSecondNs{1'000'000'000};

[[nodiscard]] LidarBeam3D hitAlongX(const double range_m,
                                    const bool warm_body = false) {
  return LidarBeam3D{.direction_map = {1.0, 0.0, 0.0},
                     .range_m = range_m,
                     .hit = true,
                     .valid = true,
                     .warm_body = warm_body};
}

[[nodiscard]] LidarBeam3D missAlongX(const double range_m) {
  return LidarBeam3D{.direction_map = {1.0, 0.0, 0.0},
                     .range_m = range_m,
                     .hit = false,
                     .valid = true};
}

TEST(ObstacleMemory3D, AWarmBodysHitIsOccupiedAtOnceAndGoneWithTheFirstFreeRay) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  // The hall seen free to the wall at 12 m, three scans: the way is -8 and known free.
  for (int scan = 0; scan < 3; ++scan) {
    const std::array wall{hitAlongX(12.0)};
    static_cast<void>(
        memory.integrateScan({.origin_map = origin,
                              .beams = wall,
                              .acquisition_stamp_ns = (scan + 1) * kSecondNs}));
  }
  ASSERT_TRUE(memory.grid().isKnownFree({6, 1, 2}));
  // A warm body at 5 m: occupied at once, held as moving.
  const std::array body{hitAlongX(5.0, true)};
  ObstacleMemory3DStats stats = memory.integrateScan(
      {.origin_map = origin, .beams = body, .acquisition_stamp_ns = 4 * kSecondNs});
  EXPECT_EQ(stats.moving_hits, 1U);
  EXPECT_EQ(stats.moving_voxels, 1U);
  EXPECT_TRUE(memory.grid().isOccupied({6, 1, 2}));
  // The body walked on: the first free ray through its voxel clears it.
  const std::array gone{missAlongX(12.0)};
  stats = memory.integrateScan(
      {.origin_map = origin, .beams = gone, .acquisition_stamp_ns = 5 * kSecondNs});
  EXPECT_EQ(stats.moving_cleared, 1U);
  EXPECT_EQ(stats.moving_voxels, 0U);
  EXPECT_EQ(memory.grid().state({6, 1, 2}), ObservedVoxelState::kUnknown);
}

TEST(ObstacleMemory3D, AMovingVoxelUnobservedExpiresAndOneThatStoodBecomesAWall) {
  ObstacleMemory3D memory{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                          .minimum_range_m = 0.1,
                                                          .moving_life_s = 2.0,
                                                          .moving_stand_s = 3.0}};
  const Point3 origin{1.5, 1.5, 2.5};
  const std::array body{hitAlongX(5.0, true)};
  static_cast<void>(memory.integrateScan(
      {.origin_map = origin, .beams = body, .acquisition_stamp_ns = kSecondNs}));
  ASSERT_TRUE(memory.grid().isOccupied({6, 1, 2}));
  // Looked away: a scan elsewhere 2.5 s later, past the layer's life.
  const std::array elsewhere{LidarBeam3D{
      .direction_map = {0.0, 1.0, 0.0}, .range_m = 10.0, .hit = true, .valid = true}};
  ObstacleMemory3DStats stats =
      memory.integrateScan({.origin_map = origin,
                            .beams = elsewhere,
                            .acquisition_stamp_ns = 3'500'000'000});
  EXPECT_EQ(stats.moving_expired, 1U);
  EXPECT_EQ(memory.grid().state({6, 1, 2}), ObservedVoxelState::kUnknown);
  // A body that stands under the sensor's eye for the stand's time is a wall from
  // then: a free ray no longer clears it at once.
  for (int scan = 0; scan <= 30; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = body,
         .acquisition_stamp_ns =
             10 * kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  EXPECT_EQ(stats.moving_promoted, 1U);
  EXPECT_EQ(stats.moving_voxels, 0U);
  const std::array gone{missAlongX(12.0)};
  stats = memory.integrateScan(
      {.origin_map = origin, .beams = gone, .acquisition_stamp_ns = 14 * kSecondNs});
  EXPECT_EQ(stats.moving_cleared, 0U);
  EXPECT_TRUE(memory.grid().isOccupied({6, 1, 2}));
}

TEST(ObstacleMemory3D, AGrazingRayKeepsAHeldVoxelOfASurfaceAsAnyOther) {
  // K42 keeps a surface seen edge-on from the rays grazing it, a held voxel
  // among them (r3061: wall voxels the layer had taken were carved away by
  // the rays along the wall, and the vehicle met the wall); the layer's life
  // takes a body's trail the grazing rays alone see.
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  std::vector<LidarBeam3D> column;
  for (const double dz : {-1.0, 0.0, 1.0}) {
    const double norm = std::sqrt(25.0 + dz * dz);
    column.push_back(LidarBeam3D{.direction_map = {5.0 / norm, 0.0, dz / norm},
                                 .range_m = norm,
                                 .hit = true,
                                 .valid = true,
                                 .warm_body = true});
  }
  static_cast<void>(memory.integrateScan(
      {.origin_map = origin, .beams = column, .acquisition_stamp_ns = kSecondNs}));
  ASSERT_TRUE(memory.grid().isOccupied({6, 1, 2}));
  const std::array gone{missAlongX(12.0)};
  ObstacleMemory3DStats stats = memory.integrateScan(
      {.origin_map = origin, .beams = gone, .acquisition_stamp_ns = 2 * kSecondNs});
  EXPECT_EQ(stats.moving_cleared, 0U);
  EXPECT_TRUE(memory.grid().isOccupied({6, 1, 2}));
  // Unobserved by anything but grazing rays, the trail expires with the life.
  stats = memory.integrateScan(
      {.origin_map = origin, .beams = gone, .acquisition_stamp_ns = 5 * kSecondNs});
  EXPECT_GE(stats.moving_expired, 3U);
  EXPECT_EQ(memory.grid().state({6, 1, 2}), ObservedVoxelState::kUnknown);
}

TEST(ObstacleMemory3D, AHitInObservedFreeSpaceIsMovingEvidenceUnlessBesideAWall) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  // The hall seen free to the wall along two rays, three scans.
  const std::array hall{hitAlongX(12.0), LidarBeam3D{.direction_map = {1.0, 0.2, 0.0},
                                                     .range_m = 12.06,
                                                     .hit = true,
                                                     .valid = true}};
  for (int scan = 0; scan < 3; ++scan) {
    static_cast<void>(
        memory.integrateScan({.origin_map = origin,
                              .beams = hall,
                              .acquisition_stamp_ns = (scan + 1) * kSecondNs}));
  }
  ASSERT_TRUE(memory.grid().isKnownFree({6, 1, 2}));
  ASSERT_TRUE(memory.grid().isKnownFree({6, 2, 2}));
  // A cold body at 5 m, in the free hall: moving by its appearance where free
  // space had been observed.
  const std::array cold{hitAlongX(5.0)};
  ObstacleMemory3DStats stats = memory.integrateScan(
      {.origin_map = origin, .beams = cold, .acquisition_stamp_ns = 4 * kSecondNs});
  EXPECT_EQ(stats.moving_hits, 1U);
  EXPECT_TRUE(memory.grid().isOccupied({6, 1, 2}));
  // A voxel beside the held body is the body's, beside a wall or not.
  const std::array beside{LidarBeam3D{
      .direction_map = {1.0, 0.2, 0.0}, .range_m = 5.02, .hit = true, .valid = true}};
  stats = memory.integrateScan(
      {.origin_map = origin, .beams = beside, .acquisition_stamp_ns = 4'100'000'000});
  EXPECT_EQ(stats.moving_hits, 1U);
  EXPECT_TRUE(memory.grid().isOccupied({6, 2, 2}));
  // A return in the free voxel before the wall is the wall's own noise: ordinary.
  const std::array noise{hitAlongX(11.0)};
  stats = memory.integrateScan(
      {.origin_map = origin, .beams = noise, .acquisition_stamp_ns = 5 * kSecondNs});
  EXPECT_EQ(stats.moving_hits, 0U);
  // With the layer off every hit is an ordinary one.
  ObstacleMemory3D plain{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                         .minimum_range_m = 0.1,
                                                         .moving_life_s = 0.0}};
  const std::array body{hitAlongX(5.0, true)};
  EXPECT_EQ(
      plain
          .integrateScan(
              {.origin_map = origin, .beams = body, .acquisition_stamp_ns = kSecondNs})
          .moving_hits,
      0U);
}

// A hit on the cell whose centre is `end`, from `origin`.
[[nodiscard]] LidarBeam3D hitAt(const Point3& origin, const Point3& end,
                                const bool warm_body = false) {
  const Vec3 along{end.x - origin.x, end.y - origin.y, end.z - origin.z};
  const double norm =
      std::sqrt(along.x * along.x + along.y * along.y + along.z * along.z);
  return LidarBeam3D{.direction_map = {along.x / norm, along.y / norm, along.z / norm},
                     .range_m = norm,
                     .hit = true,
                     .valid = true,
                     .warm_body = warm_body};
}

// A cold body of four voxels, two wide and two tall, its front face at x.
[[nodiscard]] std::vector<LidarBeam3D> coldBlockAt(const Point3& origin,
                                                   const double x) {
  std::vector<LidarBeam3D> block;
  for (const double y : {1.5, 2.5}) {
    for (const double z : {1.5, 2.5}) {
      block.push_back(hitAt(origin, {x, y, z}));
    }
  }
  return block;
}

TEST(ObstacleMemory3D, ABodyFirstSeenInUnknownSpaceJoinsTheLayerOnceItIsSeenToMove) {
  // r3018 to r3023: a body first seen by the lidar where nothing had been
  // observed left the ends of its walk as ordinary occupancies for ever. Its
  // first returns are nothing yet; once its cluster is seen to have moved by
  // a cell, every voxel it has had is the layer's.
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan <= 5; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = coldBlockAt(origin, 6.5 + 0.12 * scan),
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
    if (scan < 5) {
      EXPECT_EQ(stats.moving_hits, 0U) << scan;
      EXPECT_EQ(stats.moving_joined, 0U) << scan;
    }
  }
  // The fifth scan put the block a cell on: seen to move, held from now.
  EXPECT_GE(stats.moving_joined, 4U);
  EXPECT_EQ(stats.followed_bodies, 1U);
  ASSERT_TRUE(memory.grid().isOccupied({7, 1, 2}));
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = coldBlockAt(origin, 7.1),
                                .acquisition_stamp_ns = kSecondNs + 600'000'000});
  EXPECT_EQ(stats.moving_hits, 4U);
  const std::array gone{missAlongX(12.0)};
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = gone,
                                .acquisition_stamp_ns = kSecondNs + 700'000'000});
  EXPECT_GE(stats.moving_cleared, 1U);
  EXPECT_EQ(memory.grid().state({7, 1, 2}), ObservedVoxelState::kUnknown);
}

TEST(ObstacleMemory3D, AnOldWallBesideANoiseVoxelOrAWarmBodyStaysOrdinary) {
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  const std::array wall{hitAlongX(11.0)};
  for (int scan = 0; scan < 20; ++scan) {
    static_cast<void>(memory.integrateScan(
        {.origin_map = origin,
         .beams = wall,
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000}));
  }
  // The range's noise: a return a voxel before the wall for three scans (the
  // voxel was seen free twenty times), then gone.
  const std::array noise{hitAlongX(10.0)};
  for (int scan = 0; scan < 3; ++scan) {
    static_cast<void>(memory.integrateScan(
        {.origin_map = origin,
         .beams = noise,
         .acquisition_stamp_ns =
             4 * kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000}));
  }
  ASSERT_TRUE(memory.grid().isOccupied({11, 1, 2}));
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan < 6; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = wall,
         .acquisition_stamp_ns =
             5 * kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  EXPECT_TRUE(memory.grid().isKnownFree({11, 1, 2}));
  EXPECT_EQ(stats.moving_joined, 0U);
  EXPECT_EQ(stats.moving_voxels, 0U);
  EXPECT_TRUE(memory.grid().isOccupied({12, 1, 2}));
  // A warm body beside the wall does not take the wall with it either: its
  // hit is the body's, the wall's own voxel stays ordinary.
  const std::array beside{hitAt(origin, {11.5, 2.5, 2.5}, true)};
  stats = memory.integrateScan(
      {.origin_map = origin, .beams = beside, .acquisition_stamp_ns = 6 * kSecondNs});
  EXPECT_EQ(stats.moving_hits, 1U);
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = wall,
                                .acquisition_stamp_ns = 6 * kSecondNs + 100'000'000});
  EXPECT_EQ(stats.moving_hits, 0U);
}

TEST(ObstacleMemory3D, AClusterThatWalksAlongAnOldWallIsTheWallsSkinNotABody) {
  // r3035: the range's noise on a wall's near side, held, walked along the
  // wall with the vehicle and was read as a body at 0.5 to 3 m/s. A cluster
  // hugging an old surface along more than half its voxels is no body.
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  // The wall at x 12, seen for two seconds over four rows.
  std::vector<LidarBeam3D> wall;
  for (const double y : {1.5, 2.5, 3.5, 4.5}) {
    for (const double z : {1.5, 2.5}) {
      wall.push_back(hitAt(origin, {12.5, y, z}));
    }
  }
  for (int scan = 0; scan < 20; ++scan) {
    static_cast<void>(memory.integrateScan(
        {.origin_map = origin,
         .beams = wall,
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000}));
  }
  // Its skin: four returns a voxel before it, warm, walking a row along it.
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan < 6; ++scan) {
    const double y_low = scan < 3 ? 1.5 : 2.5;
    std::vector<LidarBeam3D> skin;
    for (const double y : {y_low, y_low + 1.0}) {
      for (const double z : {1.5, 2.5}) {
        skin.push_back(hitAt(origin, {11.5, y, z}, true));
      }
    }
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = skin,
         .acquisition_stamp_ns =
             4 * kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  EXPECT_EQ(stats.followed_bodies, 0U);
  EXPECT_EQ(stats.swept_voxels, 0U);
}

TEST(ObstacleMemory3D, AnUnknownVoxelBesideAFollowedBodyIsTheBodysOnItsHit) {
  // Inside a body's envelope, a body followed moving, a voxel beside a held
  // one joins it whatever it was; beyond every envelope only a free one does
  // (r3025), and a standing cluster is no body (r3033).
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  const auto block = [&](const double x) {
    std::vector<LidarBeam3D> beams;
    for (const double y : {1.625, 1.875}) {
      for (const double z : {2.125, 2.375, 2.625, 2.875, 3.125}) {
        beams.push_back(hitAt(origin, {x, y, z}, true));
      }
    }
    return beams;
  };
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan <= 6; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = block(6.625 + 0.12 * scan),
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  ASSERT_EQ(stats.followed_bodies, 1U);
  // A cold return beside it, in a voxel nothing had observed, within its
  // reach (between scans the sweep holds it, by no measurement).
  const GridIndex3D beside{29, 8, 10};
  ASSERT_TRUE(memory.grid().isUnobservable(beside) ||
              memory.grid().state(beside) == ObservedVoxelState::kUnknown);
  const std::array cold{hitAt(origin, {7.375, 2.125, 2.625})};
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = cold,
                                .acquisition_stamp_ns = kSecondNs + 700'000'000});
  EXPECT_EQ(stats.moving_hits, 1U);
  EXPECT_TRUE(memory.grid().isOccupied(beside));
  // A lone warm voxel is no followed body: its unknown neighbour stays ordinary.
  ObstacleMemory3D lone{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array one{hitAlongX(5.0, true)};
  static_cast<void>(lone.integrateScan(
      {.origin_map = origin, .beams = one, .acquisition_stamp_ns = kSecondNs}));
  const std::array next{hitAt(origin, {6.5, 2.5, 2.5})};
  EXPECT_EQ(lone.integrateScan({.origin_map = origin,
                                .beams = next,
                                .acquisition_stamp_ns = kSecondNs + 100'000'000})
                .moving_hits,
            0U);
}

TEST(ObstacleMemory3D, AWalkingBodySweepsTheCellsAheadOfItBetweenScans) {
  // K66: a body's column walks along x at 1.2 m/s over a grid of 0.25 m; once
  // followed, the cells it will sweep in the next 1.5 s are occupied and
  // unobservable between scans, lifted before the next scan reads them.
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  ObstacleMemory3DStats stats{};
  const auto column = [&](const double x) {
    std::vector<LidarBeam3D> beams;
    for (const double y : {1.625, 1.875}) {
      for (const double z : {2.125, 2.375, 2.625, 2.875, 3.125}) {
        beams.push_back(hitAt(origin, {x, y, z}, true));
      }
    }
    return beams;
  };
  for (int scan = 0; scan <= 20; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = column(6.5 + 0.12 * scan),
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  ASSERT_EQ(stats.followed_bodies, 1U);
  const MovingBody3D& body = memory.movingBodies().front();
  EXPECT_NEAR(body.velocity_map_mps.x, 1.2, 0.5);
  EXPECT_GT(stats.swept_voxels, 0U);
  // Ahead of the body, within 1.5 s of its walk: occupied, by no measurement.
  const GridIndex3D ahead{38, 6, 10};
  EXPECT_TRUE(memory.grid().isOccupied(ahead));
  EXPECT_TRUE(memory.grid().isUnobservable(ahead));
  // The next scan reads the cell as it was: free rays through the body clear
  // it, the body is gone, and the sweep with it.
  std::vector<LidarBeam3D> gone = column(6.5 + 0.12 * 20);
  for (LidarBeam3D& beam : gone) {
    beam.hit = false;
    beam.range_m = 12.0;
    beam.warm_body = false;
  }
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = gone,
                                .acquisition_stamp_ns = kSecondNs + 2'100'000'000});
  EXPECT_EQ(stats.followed_bodies, 0U);
  EXPECT_TRUE(memory.grid().isKnownFree(ahead));
  EXPECT_EQ(stats.swept_voxels, 0U);
  // With no sweep configured the bodies are still followed and nothing is laid.
  ObstacleMemory3D plain{kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0,
                                                         .minimum_range_m = 0.1,
                                                         .moving_sweep_s = 0.0}};
  const std::array still{hitAlongX(5.0, true)};
  EXPECT_EQ(
      plain
          .integrateScan(
              {.origin_map = origin, .beams = still, .acquisition_stamp_ns = kSecondNs})
          .swept_voxels,
      0U);
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

TEST(ObstacleMemory3D, AScanThatChangesNothingIsStillARevision) {
  // The published revision carries the scan's stamp; two stamps under one
  // revision are an identity conflict to the consumer.
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const std::array beams{LidarBeam3D{
      .direction_map = {1.0, 0.0, 0.0}, .range_m = 4.0, .hit = true, .valid = true}};
  const auto scan = [&](const std::int64_t stamp_ns) {
    return memory.integrateScan(LidarScan3DView{.origin_map = Point3{2.0, 2.0, 2.0},
                                                .beams = beams,
                                                .acquisition_stamp_ns = stamp_ns});
  };
  for (std::int64_t index = 1; index <= 20; ++index) {
    scan(index * 100'000'000LL);
  }
  const std::uint64_t settled = memory.revision();
  const ObstacleMemory3DStats unchanged = scan(2'100'000'000LL);
  EXPECT_EQ(unchanged.state_transitions, 0U);
  EXPECT_EQ(memory.revision(), settled + 1U);
}

} // namespace drone_city_nav
