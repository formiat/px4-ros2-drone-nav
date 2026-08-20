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

} // namespace
} // namespace drone_city_nav
