#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "obstacle_memory_3d_test_support.hpp"

// Roadmap item 21, specification K65 and K66: the obstacle memory's moving
// layer, the bodies it follows and the places they keep.

namespace drone_city_nav {
namespace {

using memory_test::coldBlockAt;
using memory_test::hitAlongX;
using memory_test::hitAt;
using memory_test::kBounds;
using memory_test::kSecondNs;
using memory_test::missAlongX;

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
  // The fifth scan put the block a cell on: seen to move, held from now, and
  // without the core's class no body the planner reads (K66).
  EXPECT_GE(stats.moving_joined, 4U);
  EXPECT_EQ(stats.followed_bodies, 0U);
  EXPECT_EQ(stats.swept_voxels, 0U);
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

TEST(ObstacleMemory3D, AColumnsFootJoinsTheLayerInTheScanThatFirstShowsIt) {
  // K65: a column appears in free air over an old floor. Its lowest voxels
  // stand beside the floor's and are ordinary by their own state; under a
  // voxel the scan holds as moving they are the column's foot (r3521).
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  std::vector<Point3> column;
  for (const double y : {1.625, 1.875}) {
    for (const double z : {1.375, 1.625, 1.875, 2.125}) {
      column.push_back({6.5, y, z});
    }
  }
  // Twenty scans of the floor under it and of a wall behind it: the floor is
  // old, the column's cells are observed free.
  std::vector<LidarBeam3D> before{hitAt(origin, {6.5, 1.625, 1.125}),
                                  hitAt(origin, {6.5, 1.875, 1.125})};
  for (const Point3& point : column) {
    before.push_back(hitAt(origin, {origin.x + 2.2 * (point.x - origin.x),
                                    origin.y + 2.2 * (point.y - origin.y),
                                    origin.z + 2.2 * (point.z - origin.z)}));
  }
  for (int scan = 0; scan < 20; ++scan) {
    static_cast<void>(memory.integrateScan(
        {.origin_map = origin,
         .beams = before,
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000}));
  }
  std::vector<LidarBeam3D> appeared;
  appeared.reserve(column.size());
  for (const Point3& point : column) {
    appeared.push_back(hitAt(origin, point));
  }
  const ObstacleMemory3DStats stats =
      memory.integrateScan({.origin_map = origin,
                            .beams = appeared,
                            .acquisition_stamp_ns = kSecondNs + 2'000'000'000});
  EXPECT_EQ(stats.moving_hits, column.size());
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
  // First a scan that shows no cluster of a body's height and none of the
  // core's marks, its rays through the column's upper rows alone: the body is
  // read on unseen (r3475, r3476, r3503), and no scan having looked at its
  // legs, where it stood is kept (r3582).
  std::vector<LidarBeam3D> upper;
  for (const LidarBeam3D& beam : gone) {
    if (origin.z + beam.direction_map.z * 5.0 > origin.z) {
      upper.push_back(beam);
    }
  }
  ASSERT_EQ(upper.size(), 6U);
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = upper,
                                .acquisition_stamp_ns = kSecondNs + 2'050'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  EXPECT_GT(memory.movingBodies().front().unseen_s, 0.0);
  EXPECT_TRUE(memory.movingBodies().front().warm);
  EXPECT_GT(stats.swept_voxels, 0U);
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = {},
                                .acquisition_stamp_ns = kSecondNs + 2'650'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  EXPECT_GT(memory.movingBodies().front().unseen_s, 0.5);
  EXPECT_GT(stats.swept_voxels, 0U);
  // The whole column looked at and empty, it is unseen still, wherever it
  // has walked to (r3573), its place kept (r3645), until the stand's time
  // from its last mark is out (the vehicle stands seven metres off, farther
  // than a walk: its sensors could look, and the time ran all along, r3649).
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = gone,
                                .acquisition_stamp_ns = kSecondNs + 2'700'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  EXPECT_GT(memory.movingBodies().front().unseen_s, 0.5);
  EXPECT_TRUE(memory.grid().isOccupied(ahead));
  EXPECT_GT(stats.swept_voxels, 0U);
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = gone,
                                .acquisition_stamp_ns = kSecondNs + 7'200'000'000});
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

TEST(ObstacleMemory3D, ABodyReadBeforeStandsWhereTheCoreMarksIt) {
  // K66 (r3563): a scan whose marked returns make no cluster of a body's
  // size still shows where the body is, its walk not measured.
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan <= 20; ++scan) {
    std::vector<LidarBeam3D> column;
    for (const double y : {1.625, 1.875}) {
      for (const double z : {2.125, 2.375, 2.625, 2.875, 3.125}) {
        column.push_back(hitAt(origin, {6.5 + 0.12 * scan, y, z}, true));
      }
    }
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = column,
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  ASSERT_EQ(stats.followed_bodies, 1U);
  ASSERT_TRUE(memory.movingBodies().front().moving());
  // Its head alone, half a metre on.
  const std::array head{hitAt(origin, {9.375, 1.625, 3.125}, true),
                        hitAt(origin, {9.375, 1.875, 3.125}, true)};
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = head,
                                .acquisition_stamp_ns = kSecondNs + 2'100'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  const MovingBody3D& body = memory.movingBodies().front();
  EXPECT_NEAR(body.centre_map.x, 9.375, 1.0e-9);
  EXPECT_NEAR(body.centre_map.y, 1.75, 1.0e-9);
  EXPECT_NEAR(body.top_m, 3.25, 1.0e-9);
  EXPECT_DOUBLE_EQ(body.unseen_s, 0.0);
  EXPECT_FALSE(body.moving());
  // Its walk not measured, the place is kept a person's walk wide on every
  // side: two metres beside it, where the walk along x swept nothing.
  EXPECT_TRUE(memory.grid().isOccupied({37, 15, 10}));
  EXPECT_TRUE(memory.grid().isUnobservable({37, 15, 10}));
}

TEST(ObstacleMemory3D, TheCoresMarksThatMakeNoClusterAreABodyAtOnce) {
  // K66 (r3568): four marked voxels in a row, a cell tall, are no cluster of a
  // body's height; they are the body the core sighted all the same. Three of
  // them are a return.
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  std::vector<LidarBeam3D> row;
  for (const double y : {1.375, 1.625, 1.875}) {
    row.push_back(hitAt(origin, {6.125, y, 2.625}, true));
  }
  ObstacleMemory3DStats stats = memory.integrateScan(
      {.origin_map = origin, .beams = row, .acquisition_stamp_ns = kSecondNs});
  EXPECT_EQ(stats.followed_bodies, 0U);
  row.push_back(hitAt(origin, {6.125, 2.125, 2.625}, true));
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = row,
                                .acquisition_stamp_ns = kSecondNs + 100'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  const MovingBody3D& body = memory.movingBodies().front();
  EXPECT_NEAR(body.centre_map.x, 6.125, 1.0e-9);
  EXPECT_NEAR(body.centre_map.y, 1.75, 1.0e-9);
  EXPECT_NEAR(body.bottom_m, 2.5, 1.0e-9);
  EXPECT_NEAR(body.top_m, 2.75, 1.0e-9);
  EXPECT_TRUE(body.warm);
  EXPECT_FALSE(body.moving());
  // A cold return a cell beside the row, in a voxel nothing had observed: in
  // the envelope of a body read by its marks it is the body's (r3599).
  const std::array cold{hitAt(origin, {6.375, 1.625, 2.625})};
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = cold,
                                .acquisition_stamp_ns = kSecondNs + 200'000'000});
  EXPECT_EQ(stats.moving_hits, 1U);
}

TEST(ObstacleMemory3D, TheWidePlaceOfABodyLeavesTheVehicleItsWayOut) {
  // K66: a warm column 2.1 m before the vehicle, read at once, its walk not
  // measured. The place kept for it reaches the vehicle; the vehicle's own
  // metre and what lies behind a plane 1.5 m before the vehicle as the body
  // sees it stay as they were (r3663).
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{4.125, 4.125, 2.625};
  std::vector<LidarBeam3D> column;
  for (const double y : {4.125, 4.375}) {
    for (const double z : {2.125, 2.375, 2.625, 2.875, 3.125}) {
      column.push_back(hitAt(origin, {6.125, y, z}, true));
    }
  }
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan < 3; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = column,
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  ASSERT_EQ(stats.followed_bodies, 1U);
  ASSERT_FALSE(memory.movingBodies().front().moving());
  // Beyond the body, and beside it past the plane: kept.
  EXPECT_TRUE(memory.grid().isOccupied({32, 16, 10}));
  EXPECT_TRUE(memory.grid().isOccupied({23, 20, 10}));
  // The vehicle's metre, the way out before it and beside it, and behind it.
  EXPECT_FALSE(memory.grid().isOccupied({19, 16, 10}));
  EXPECT_FALSE(memory.grid().isOccupied({21, 16, 10}));
  EXPECT_FALSE(memory.grid().isOccupied({18, 22, 10}));
  EXPECT_FALSE(memory.grid().isOccupied({10, 16, 10}));
  EXPECT_FALSE(memory.grid().isOccupied({14, 22, 10}));
}

TEST(ObstacleMemory3D, AWarmBodyLaidByItsSizeIsReadAndIsNoOccupancy) {
  // K64, K66 (r3673, r3679): the surface samples of a warm body the core's
  // reading of its size puts 4 m off. A body is read at once, for the law; no
  // voxel is occupied by it and it keeps no place. With a range sensor's
  // returns on it, it is the layer's and keeps its place.
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{2.125, 4.125, 2.625};
  std::vector<LidarBeam3D> column;
  for (const double y : {4.125, 4.375}) {
    for (const double z : {2.125, 2.375, 2.625, 2.875, 3.125}) {
      column.push_back(hitAt(origin, {6.125, y, z}, true));
      column.back().surface_only = true;
    }
  }
  ObstacleMemory3DStats stats = memory.integrateScan(
      {.origin_map = origin, .beams = column, .acquisition_stamp_ns = kSecondNs});
  ASSERT_EQ(stats.followed_bodies, 1U);
  EXPECT_TRUE(memory.movingBodies().front().warm);
  EXPECT_TRUE(memory.movingBodies().front().sized);
  EXPECT_NEAR(memory.movingBodies().front().top_m, 3.25, 1.0e-9);
  EXPECT_EQ(stats.moving_voxels, 0U);
  EXPECT_EQ(stats.swept_voxels, 0U);
  EXPECT_FALSE(memory.grid().isOccupied({24, 16, 10}));
  for (LidarBeam3D& beam : column) {
    beam.surface_only = false;
  }
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = column,
                                .acquisition_stamp_ns = kSecondNs + 100'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  EXPECT_FALSE(memory.movingBodies().front().sized);
  EXPECT_GT(stats.moving_voxels, 0U);
  EXPECT_GT(stats.swept_voxels, 0U);
  EXPECT_TRUE(memory.grid().isOccupied({24, 16, 10}));
}

TEST(ObstacleMemory3D, ABodysPlaceStandsStillWhileItsReadingStaysWithinIt) {
  // K66 (r3679): a warm column a cell wide, read at once, its walk not
  // measured: its place reaches 2.875 m from it. Read a quarter metre on, the
  // place stands where it was laid; read a metre on, it is laid anew.
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{2.125, 4.125, 2.625};
  const auto read_at = [&](const double x, const int scan) {
    std::vector<LidarBeam3D> column;
    for (const double z : {2.125, 2.375, 2.625, 2.875, 3.125}) {
      column.push_back(hitAt(origin, {x, 4.125, z}, true));
    }
    return memory.integrateScan(
        {.origin_map = origin,
         .beams = column,
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  };
  ASSERT_EQ(read_at(6.125, 0).followed_bodies, 1U);
  EXPECT_TRUE(memory.grid().isOccupied({35, 16, 10}));
  EXPECT_FALSE(memory.grid().isOccupied({36, 16, 10}));
  ASSERT_EQ(read_at(6.375, 1).followed_bodies, 1U);
  EXPECT_NEAR(memory.movingBodies().front().centre_map.x, 6.375, 0.13);
  EXPECT_FALSE(memory.grid().isOccupied({36, 16, 10}));
  ASSERT_EQ(read_at(7.125, 2).followed_bodies, 1U);
  EXPECT_TRUE(memory.grid().isOccupied({36, 16, 10}));
}

TEST(ObstacleMemory3D, AColdClusterFartherThanAWalkDoesNotContinueAReadBody) {
  // K66 (r3665): a warm column read at once; the next scan shows a cold
  // column 1.25 m beyond it and nothing where it stood. A walk of a scan's
  // time is 0.15 m: the body is read on unseen where it stood.
  constexpr GridBounds3D kFine{0.0, 0.0, 0.0, 0.25, 80, 80, 40};
  ObstacleMemory3D memory{
      kFine, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{2.125, 4.125, 2.625};
  const auto column = [&origin](const double x, const bool warm) {
    std::vector<LidarBeam3D> beams;
    for (const double y : {4.125, 4.375}) {
      for (const double z : {2.125, 2.375, 2.625, 2.875, 3.125}) {
        beams.push_back(hitAt(origin, {x, y, z}, warm));
      }
    }
    return beams;
  };
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan < 4; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = column(scan < 3 ? 6.125 : 7.375, scan < 3),
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  ASSERT_EQ(stats.followed_bodies, 1U);
  EXPECT_NEAR(memory.movingBodies().front().centre_map.x, 6.125, 0.3);
  EXPECT_GT(memory.movingBodies().front().unseen_s, 0.0);
  // Unseen 1.5 s, the body may have walked 2 m; a cold column that far, new
  // to the scan and not seen to move, is no body all the same (r3681).
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = {},
                                .acquisition_stamp_ns = kSecondNs + 1'800'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  stats = memory.integrateScan({.origin_map = origin,
                                .beams = column(8.125, false),
                                .acquisition_stamp_ns = kSecondNs + 1'900'000'000});
  ASSERT_EQ(stats.followed_bodies, 1U);
  EXPECT_NEAR(memory.movingBodies().front().centre_map.x, 6.125, 0.3);
  EXPECT_GT(memory.movingBodies().front().unseen_s, 1.0);
}

TEST(ObstacleMemory3D, AClusterSeenToStepNearTheVehicleIsSweptAndHandsNoBody) {
  // Roadmap item 21, motion without the class (r3561): a cold block steps
  // along x two to three metres from the vehicle. Once seen to move its way is
  // swept; the planner reads no body. The same block five metres off
  // (ABodyFirstSeenInUnknownSpaceJoinsTheLayerOnceItIsSeenToMove) sweeps none.
  ObstacleMemory3D memory{
      kBounds, ObstacleMemory3DConfig{.maximum_range_m = 20.0, .minimum_range_m = 0.1}};
  const Point3 origin{1.5, 1.5, 2.5};
  ObstacleMemory3DStats stats{};
  for (int scan = 0; scan <= 5; ++scan) {
    stats = memory.integrateScan(
        {.origin_map = origin,
         .beams = coldBlockAt(origin, 3.5 + 0.12 * scan),
         .acquisition_stamp_ns =
             kSecondNs + static_cast<std::int64_t>(scan) * 100'000'000});
  }
  EXPECT_GE(stats.moving_joined, 4U);
  EXPECT_EQ(stats.followed_bodies, 0U);
  EXPECT_GT(stats.swept_voxels, 0U);
}

} // namespace
} // namespace drone_city_nav
