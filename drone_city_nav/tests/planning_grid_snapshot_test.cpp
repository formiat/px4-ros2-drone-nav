#include "drone_city_nav/planning_grid_snapshot.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] ObstacleFieldBuildResult readyBuild() {
  ObstacleFieldBuildResult result{};
  result.status = PlanningGridStatus::kReady;
  result.raw_occupancy.emplace(GridBounds{0.0, 0.0, 1.0, 8, 8});
  result.raw_occupancy->reset(CellState::kFree);
  result.raw_occupancy->setOccupied(GridIndex{3, 3});
  result.risk_policy = {
      .critical_distance_m = 1.0,
      .preferred_distance_m = 4.0,
  };
  result.evaluation_bounds = result.raw_occupancy->bounds();
  result.applied_memory_producer_instance_id = 17U;
  result.applied_memory_sequence = 42U;
  result.applied_lidar_update_ns = 123456;
  return result;
}

} // namespace

TEST(ObstacleRiskSnapshot, FailedBuildDoesNotConsumeRevision) {
  ObstacleRiskSnapshotBuilder builder;
  ObstacleFieldBuildResult failed{};
  failed.status = PlanningGridStatus::kNoReadySourceData;

  EXPECT_FALSE(builder.prepare(ObstacleRiskPreparationInput{.build_result = &failed})
                   .has_value());
  EXPECT_EQ(builder.nextRevision(), 1U);

  ObstacleFieldBuildResult ready = readyBuild();
  const auto prepared =
      builder.prepare(ObstacleRiskPreparationInput{.build_result = &ready});
  ASSERT_TRUE(prepared.has_value());
  EXPECT_EQ(prepared.value().version.build_revision, 1U);
  EXPECT_EQ(builder.nextRevision(), 2U);
}

TEST(ObstacleRiskSnapshot, OwnsRawGridDistanceFieldAndIdentity) {
  ObstacleRiskSnapshotBuilder builder;
  ObstacleFieldBuildResult ready = readyBuild();

  const auto prepared = builder.prepare(ObstacleRiskPreparationInput{
      .build_result = &ready,
      .config_fingerprint = 99U,
  });

  ASSERT_TRUE(prepared.has_value());
  const PreparedObstacleRiskSnapshot& snapshot = prepared.value();
  EXPECT_TRUE(snapshot.raw_occupancy.isOccupied(GridIndex{3, 3}));
  EXPECT_EQ(snapshot.risk_field.tierAt(GridIndex{4, 3}),
            ObstacleRiskTier::kPlanningBand);
  EXPECT_EQ(&snapshot.rawClearance(), &snapshot.risk_field.occupiedClearance());
  EXPECT_DOUBLE_EQ(snapshot.rawClearance().distanceAt(GridIndex{4, 3}),
                   snapshot.risk_field.occupiedClearance().distanceAt(GridIndex{4, 3}));
  EXPECT_EQ(snapshot.version.memory_producer_instance_id, 17U);
  EXPECT_EQ(snapshot.version.memory_sequence, 42U);
  EXPECT_EQ(snapshot.version.lidar_update_ns, 123456);
  EXPECT_EQ(snapshot.version.config_fingerprint, 99U);
  EXPECT_NE(snapshot.version.risk_policy_fingerprint, 0U);
  EXPECT_EQ(snapshot.version.raw_occupancy.cells_hash,
            snapshot.raw_occupancy.rawFingerprint().cells_hash);
}

TEST(ObstacleRiskSnapshot, PolicyParticipatesInVersionIdentity) {
  ObstacleRiskSnapshotBuilder builder;
  ObstacleFieldBuildResult first_build = readyBuild();
  ObstacleFieldBuildResult second_build = readyBuild();
  second_build.risk_policy.preferred_distance_m = 6.0;

  const auto first =
      builder.prepare(ObstacleRiskPreparationInput{.build_result = &first_build});
  const auto second =
      builder.prepare(ObstacleRiskPreparationInput{.build_result = &second_build});

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_NE(first.value().version.risk_policy_fingerprint,
            second.value().version.risk_policy_fingerprint);
  EXPECT_FALSE(
      obstacleRiskVersionsEqual(first.value().version, second.value().version));
}

} // namespace drone_city_nav
