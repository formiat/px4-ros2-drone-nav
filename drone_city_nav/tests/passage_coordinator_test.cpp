#include "drone_city_nav/passage_coordinator.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageOpening testOpening() {
  return PassageOpening{
      .id = "test_opening",
      .structure_id = "test_structure",
      .center = Point3{10.0, 0.0, 5.0},
      .normal_xy = Point2{1.0, 0.0},
      .width_m = 6.0,
      .height_m = 7.0,
      .depth_m = 2.0,
      .min_z_m = 1.5,
      .max_z_m = 8.5,
      .approach_distance_m = 10.0,
      .exit_distance_m = 10.0,
  };
}

[[nodiscard]] PassageCoordinatorInput inputAt(const PassageOpening* opening,
                                              const float x, const float z,
                                              const float vz = 0.0F) {
  return PassageCoordinatorInput{
      .state =
          mppi::State{
              .x = x,
              .y = 0.0F,
              .z = z,
              .vx = 5.0F,
              .vy = 0.0F,
              .vz = vz,
          },
      .selected_opening = opening,
      .approach_speed_mps = 5.0,
      .passage_speed_limit_mps = 5.0,
  };
}

TEST(PassageCoordinatorTest, RemainsInactiveWithoutSelectedPassage) {
  PassageCoordinator coordinator;

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(nullptr, 0.0F, 15.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kInactive);
  EXPECT_FALSE(result.active);
  EXPECT_FALSE(result.constraint.has_value());
}

TEST(PassageCoordinatorTest, ApproachesWhileVerticalCaptureRemainsFeasible) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(&opening, -20.0F, 15.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kApproach);
  EXPECT_TRUE(result.active);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_GT(result.distance_to_entry_m, result.required_alignment_distance_m);
}

TEST(PassageCoordinatorTest, LatchesStationaryAlignmentWhenCaptureIsTooLate) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();

  const PassageCoordinatorResult first =
      coordinator.update(inputAt(&opening, 0.0F, 15.0F));
  const PassageCoordinatorResult second =
      coordinator.update(inputAt(&opening, 0.5F, 12.0F));

  ASSERT_EQ(first.phase, PassageCoordinatorPhase::kStationaryVerticalAlignment);
  EXPECT_TRUE(first.hold_xy);
  EXPECT_DOUBLE_EQ(first.hold_position.x, 0.0);
  EXPECT_EQ(second.phase, PassageCoordinatorPhase::kStationaryVerticalAlignment);
  EXPECT_TRUE(second.hold_xy);
  EXPECT_DOUBLE_EQ(second.hold_position.x, 0.0);
  ASSERT_TRUE(second.constraint.has_value());
  const mppi::PassageConstraint constraint =
      second.constraint.value_or(mppi::PassageConstraint{});
  EXPECT_EQ(constraint.phase, mppi::PassagePhase::kStationaryVerticalAlignment);
}

TEST(PassageCoordinatorTest, IncludesHorizontalStoppingDistanceInHoldTrigger) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();
  PassageCoordinatorInput input = inputAt(&opening, -40.0F, 7.4F);
  input.state.vx = 20.0F;

  const PassageCoordinatorResult result = coordinator.update(input);

  EXPECT_GT(result.required_stopping_distance_m, result.distance_to_entry_m);
  EXPECT_DOUBLE_EQ(result.required_alignment_distance_m,
                   result.required_stopping_distance_m);
  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kStationaryVerticalAlignment);
  EXPECT_TRUE(result.hold_xy);
}

TEST(PassageCoordinatorTest, RequiresLowVerticalSpeedBeforeTraversalRelease) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();
  static_cast<void>(coordinator.update(inputAt(&opening, 0.0F, 15.0F)));

  const PassageCoordinatorResult moving =
      coordinator.update(inputAt(&opening, 0.0F, 5.0F, -1.0F));
  const PassageCoordinatorResult captured =
      coordinator.update(inputAt(&opening, 0.0F, 5.0F, -0.1F));

  EXPECT_EQ(moving.phase, PassageCoordinatorPhase::kStationaryVerticalAlignment);
  EXPECT_TRUE(moving.hold_xy);
  EXPECT_EQ(captured.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_FALSE(captured.hold_xy);
  EXPECT_TRUE(captured.vertical_ready);
}

TEST(PassageCoordinatorTest, RetainsTraversalAcrossCaptureHysteresis) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();
  static_cast<void>(coordinator.update(inputAt(&opening, 0.0F, 15.0F)));
  static_cast<void>(coordinator.update(inputAt(&opening, 0.0F, 5.0F)));

  const PassageCoordinatorResult retained =
      coordinator.update(inputAt(&opening, 2.0F, 7.4F));
  const PassageCoordinatorResult lost =
      coordinator.update(inputAt(&opening, 2.0F, 7.6F));

  EXPECT_EQ(retained.phase, PassageCoordinatorPhase::kTraversal);
  EXPECT_TRUE(retained.vertical_ready);
  EXPECT_EQ(lost.phase, PassageCoordinatorPhase::kStationaryVerticalAlignment);
  EXPECT_TRUE(lost.hold_xy);
}

TEST(PassageCoordinatorTest, PreservesAltitudeWhenStartingInsideOpening) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(&opening, 10.0F, 6.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kPartialFromInside);
  EXPECT_FALSE(result.hold_xy);
  EXPECT_TRUE(result.vertical_ready);
  EXPECT_DOUBLE_EQ(result.preferred_z_m, 6.0);
  ASSERT_TRUE(result.constraint.has_value());
  const mppi::PassageConstraint constraint =
      result.constraint.value_or(mppi::PassageConstraint{});
  EXPECT_EQ(constraint.phase, mppi::PassagePhase::kPartialFromInside);
}

TEST(PassageCoordinatorTest, CompletesAfterLeavingPassageExit) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();
  static_cast<void>(coordinator.update(inputAt(&opening, 10.0F, 5.0F)));

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(nullptr, 22.0F, 5.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kInactive);
  EXPECT_FALSE(result.active);
}

TEST(PassageCoordinatorTest, PartialFromInsideCompletesThroughEitherExit) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();
  static_cast<void>(coordinator.update(inputAt(&opening, 10.0F, 5.0F)));

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(nullptr, -2.0F, 5.0F));

  EXPECT_EQ(result.phase, PassageCoordinatorPhase::kInactive);
  EXPECT_FALSE(result.active);
}

TEST(PassageCoordinatorTest, OrientsConstraintForReverseTraversal) {
  PassageCoordinator coordinator;
  const PassageOpening opening = testOpening();

  const PassageCoordinatorResult result =
      coordinator.update(inputAt(&opening, 30.0F, 5.0F));

  ASSERT_TRUE(result.constraint.has_value());
  const mppi::PassageConstraint constraint =
      result.constraint.value_or(mppi::PassageConstraint{});
  EXPECT_FLOAT_EQ(constraint.normal_x, -1.0F);
  EXPECT_FLOAT_EQ(constraint.normal_y, 0.0F);
}

} // namespace
} // namespace drone_city_nav
