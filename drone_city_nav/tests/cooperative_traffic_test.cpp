#include "drone_city_nav/cooperative_passage_coordination.hpp"
#include "drone_city_nav/cooperative_traffic.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kNowNs{1'000'000'000LL};

[[nodiscard]] CooperativeFlightIntentData
linearIntent(const std::string& vehicle_id, const Point3& first, const Point3& second,
             const Vec3& velocity, const CooperativePassageUse& passage = {}) {
  return CooperativeFlightIntentData{
      .vehicle_id = vehicle_id,
      .frame_id = "map",
      .stamp_ns = kNowNs,
      .intent_generation = 1U,
      .valid_from_ns = kNowNs,
      .valid_until_ns = kNowNs + 5'000'000'000LL,
      .footprint_radius_m = 0.82,
      .footprint_lower_extent_m = 0.35,
      .footprint_upper_extent_m = 0.35,
      .current_position = first,
      .current_velocity = velocity,
      .maneuver_state = CooperativeManeuver::kKeep,
      .conflict_generation = 0U,
      .conflicting_vehicle_ids = {},
      .passage = passage,
      .trajectory =
          {
              CooperativeTrajectorySample{
                  .time_ns = kNowNs, .position = first, .velocity = velocity},
              CooperativeTrajectorySample{.time_ns = kNowNs + 2'000'000'000LL,
                                          .position = second,
                                          .velocity = velocity},
              CooperativeTrajectorySample{
                  .time_ns = kNowNs + 5'000'000'000LL,
                  .position = Point3{second.x + 1.5 * (second.x - first.x),
                                     second.y + 1.5 * (second.y - first.y),
                                     second.z + 1.5 * (second.z - first.z)},
                  .velocity = velocity},
          },
  };
}

[[nodiscard]] CooperativePassageUse
passageUse(const std::string& passage_traversal_id, const std::string& resource_id,
           const double lateral_offset_m, const double minimum_offset_m,
           const double maximum_offset_m, const int direction_sign,
           const CooperativePassagePhase phase = CooperativePassagePhase::kApproach) {
  return CooperativePassageUse{
      .passage_traversal_id = passage_traversal_id,
      .route_generation = 1U,
      .phase = phase,
      .lateral_offset_m = lateral_offset_m,
      .minimum_lateral_offset_m = minimum_offset_m,
      .maximum_lateral_offset_m = maximum_offset_m,
      .desired_center_separation_m = 5.0,
      .direction_sign = direction_sign,
      .predicted_entry_ns = kNowNs + 1'000'000'000LL,
      .predicted_exit_ns = kNowNs + 4'000'000'000LL,
      .conflict_resources = {CooperativeConflictResourceUse{
          .conflict_resource_id = resource_id,
          .begin_station_m = 10.0,
          .end_station_m = 40.0,
          .predicted_entry_ns = kNowNs + 1'000'000'000LL,
          .predicted_exit_ns = kNowNs + 4'000'000'000LL,
      }},
  };
}

[[nodiscard]] CooperativePassageUse
withResources(CooperativePassageUse passage,
              const std::vector<std::string>& resource_ids) {
  passage.conflict_resources.clear();
  for (const std::string& resource_id : resource_ids) {
    passage.conflict_resources.push_back(CooperativeConflictResourceUse{
        .conflict_resource_id = resource_id,
        .begin_station_m = 10.0,
        .end_station_m = 40.0,
        .predicted_entry_ns = kNowNs + 1'000'000'000LL,
        .predicted_exit_ns = kNowNs + 4'000'000'000LL,
    });
  }
  return passage;
}

TEST(CooperativePeerStore, RejectsStaleAndOutOfOrderIntents) {
  CooperativePeerStore store{"civilian_0"};
  CooperativeFlightIntentData peer =
      linearIntent("civilian_1", Point3{0.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
                   Vec3{5.0, 0.0, 0.0});

  EXPECT_EQ(store.update(peer, kNowNs), CooperativePeerUpdateStatus::kAccepted);
  EXPECT_EQ(store.update(peer, kNowNs + 1), CooperativePeerUpdateStatus::kOutOfOrder);
  peer.intent_generation = 2U;
  peer.stamp_ns = kNowNs + 100'000'000LL;
  EXPECT_EQ(store.update(peer, kNowNs + 100'000'000LL),
            CooperativePeerUpdateStatus::kAccepted);
  EXPECT_EQ(store.activeIntents(kNowNs + 700'000'000LL).size(), 0U);
  EXPECT_EQ(store.update(peer, kNowNs + 700'000'000LL),
            CooperativePeerUpdateStatus::kStale);
}

TEST(CooperativeConflictPrediction, FindsExactCrossingBetweenPublishedSamples) {
  const CooperativeFlightIntentData first =
      linearIntent("civilian_0", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
                   Vec3{10.0, 0.0, 0.0});
  const CooperativeFlightIntentData second =
      linearIntent("civilian_1", Point3{0.0, -10.0, 10.0}, Point3{0.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0});

  const CooperativeConflictPrediction prediction =
      predictCooperativeConflict(first, second, kNowNs, {});

  ASSERT_TRUE(prediction.valid);
  EXPECT_TRUE(prediction.conflict_predicted);
  EXPECT_NEAR(prediction.minimum_separation_m, 0.0, 1.0e-9);
  EXPECT_NEAR(prediction.time_to_minimum_s, 1.0, 1.0e-9);
  EXPECT_NEAR(prediction.current_separation_m, std::sqrt(200.0), 1.0e-9);
}

TEST(CooperativeConflictLifecycle, ProducesOpposingDeterministicPreferences) {
  const CooperativeFlightIntentData first =
      linearIntent("civilian_0", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
                   Vec3{10.0, 0.0, 0.0});
  const CooperativeFlightIntentData second =
      linearIntent("civilian_1", Point3{0.0, -10.0, 10.0}, Point3{0.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0});
  CooperativeConflictLifecycle first_lifecycle;
  CooperativeConflictLifecycle second_lifecycle;

  const CooperativeAvoidanceDecision first_decision =
      first_lifecycle.update(kNowNs, first, std::vector{second});
  const CooperativeAvoidanceDecision second_decision =
      second_lifecycle.update(kNowNs, second, std::vector{first});

  ASSERT_TRUE(first_decision.active);
  ASSERT_TRUE(second_decision.active);
  EXPECT_NE(first_decision.preferred_maneuver, CooperativeManeuver::kKeep);
  EXPECT_NE(second_decision.preferred_maneuver, CooperativeManeuver::kKeep);
  EXPECT_NEAR(first_decision.preferred_acceleration_direction.x +
                  second_decision.preferred_acceleration_direction.x,
              0.0, 1.0e-9);
  EXPECT_NEAR(first_decision.preferred_acceleration_direction.y +
                  second_decision.preferred_acceleration_direction.y,
              0.0, 1.0e-9);
  EXPECT_NEAR(first_decision.preferred_acceleration_direction.z +
                  second_decision.preferred_acceleration_direction.z,
              0.0, 1.0e-9);
}

TEST(CooperativeConflictLifecycle, ReleasesOnlyAfterConfirmedSafeSeparation) {
  const CooperativeFlightIntentData crossing =
      linearIntent("civilian_0", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
                   Vec3{10.0, 0.0, 0.0});
  const CooperativeFlightIntentData peer_crossing =
      linearIntent("civilian_1", Point3{0.0, -10.0, 10.0}, Point3{0.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0});
  CooperativeConflictLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.update(kNowNs, crossing, std::vector{peer_crossing}).active);

  CooperativeFlightIntentData separated =
      linearIntent("civilian_0", Point3{0.0, 0.0, 10.0}, Point3{-10.0, 0.0, 10.0},
                   Vec3{-5.0, 0.0, 0.0});
  CooperativeFlightIntentData peer_separated =
      linearIntent("civilian_1", Point3{10.0, 0.0, 10.0}, Point3{20.0, 0.0, 10.0},
                   Vec3{5.0, 0.0, 0.0});
  separated.stamp_ns = kNowNs + 1'000'000'000LL;
  peer_separated.stamp_ns = separated.stamp_ns;
  ASSERT_TRUE(
      lifecycle.update(kNowNs + 1'000'000'000LL, separated, std::vector{peer_separated})
          .active);
  EXPECT_TRUE(
      lifecycle.update(kNowNs + 1'400'000'000LL, separated, std::vector{peer_separated})
          .active);
  EXPECT_FALSE(
      lifecycle.update(kNowNs + 1'500'000'000LL, separated, std::vector{peer_separated})
          .active);
}

TEST(CooperativeConflictLifecycle, LatchesPrimaryPeerDuringThreeVehicleConflict) {
  const CooperativeFlightIntentData ownship =
      linearIntent("civilian_0", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
                   Vec3{10.0, 0.0, 0.0});
  const CooperativeFlightIntentData first_primary =
      linearIntent("civilian_1", Point3{0.0, -10.0, 10.0}, Point3{0.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0});
  const CooperativeFlightIntentData second_peer =
      linearIntent("civilian_2", Point3{4.0, -10.0, 10.0}, Point3{4.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0});
  CooperativeConflictLifecycle lifecycle;

  const CooperativeAvoidanceDecision initial =
      lifecycle.update(kNowNs, ownship, std::vector{second_peer, first_primary});

  ASSERT_TRUE(initial.active);
  ASSERT_EQ(initial.peers.size(), 2U);
  EXPECT_EQ(initial.primary_peer_id, "civilian_1");
  const CooperativeManeuver initial_maneuver = initial.preferred_maneuver;

  const CooperativeFlightIntentData still_conflicting_primary =
      linearIntent("civilian_1", Point3{4.0, -10.0, 10.0}, Point3{4.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0});
  const CooperativeFlightIntentData now_closest_peer =
      linearIntent("civilian_2", Point3{0.0, -10.0, 10.0}, Point3{0.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0});
  const CooperativeAvoidanceDecision updated =
      lifecycle.update(kNowNs + 100'000'000LL, ownship,
                       std::vector{still_conflicting_primary, now_closest_peer});

  ASSERT_TRUE(updated.active);
  ASSERT_EQ(updated.peers.size(), 2U);
  EXPECT_LT(updated.peers.front().prediction.minimum_separation_m,
            updated.peers.back().prediction.minimum_separation_m);
  EXPECT_EQ(updated.peers.front().intent.vehicle_id, "civilian_2");
  EXPECT_EQ(updated.primary_peer_id, "civilian_1");
  EXPECT_EQ(updated.preferred_maneuver, initial_maneuver);
}

TEST(CooperativePassageCoordination, MeasuresContinuousLateralSeparation) {
  const CooperativePassageUse first =
      passageUse("passage", "passage", -3.0, -6.0, 6.0, 1);
  const CooperativePassageUse second =
      passageUse("passage", "passage", 3.0, -6.0, 6.0, -1);

  EXPECT_DOUBLE_EQ(passageLateralSeparationM(first, second), 6.0);
}

TEST(CooperativePassageCoordination, AllowsSeparatedOppositeContinuousOffsets) {
  const CooperativeFlightIntentData ownship = linearIntent(
      "civilian_1", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
      Vec3{10.0, 0.0, 0.0}, passageUse("passage", "passage", -3.0, -6.0, 6.0, 1));
  const CooperativeFlightIntentData peer = linearIntent(
      "civilian_0", Point3{10.0, 0.0, 10.0}, Point3{-10.0, 0.0, 10.0},
      Vec3{-10.0, 0.0, 0.0}, passageUse("passage", "passage", 3.0, -6.0, 6.0, -1));

  const CooperativePassageDecision decision =
      coordinateCooperativePassage(ownship, std::vector{peer});

  EXPECT_TRUE(decision.active);
  EXPECT_DOUBLE_EQ(decision.lateral_offset_m, -3.0);
  EXPECT_FALSE(decision.yield_before_entry);
}

TEST(CooperativePassageCoordination, OrdersAnExclusiveNarrowCorridorInTime) {
  const CooperativeFlightIntentData ownship = linearIntent(
      "civilian_1", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
      Vec3{10.0, 0.0, 0.0}, passageUse("passage", "passage", -2.0, -2.0, 2.0, 1));
  const CooperativeFlightIntentData peer = linearIntent(
      "civilian_0", Point3{10.0, 0.0, 10.0}, Point3{-10.0, 0.0, 10.0},
      Vec3{-10.0, 0.0, 0.0}, passageUse("passage", "passage", 2.0, -2.0, 2.0, -1));

  const CooperativePassageDecision decision =
      coordinateCooperativePassage(ownship, std::vector{peer});

  EXPECT_TRUE(decision.yield_before_entry);
  EXPECT_EQ(decision.yield_to_vehicle_id, "civilian_0");
  EXPECT_FALSE(decision.conflict_zone_only);
  EXPECT_EQ(decision.entry_not_before_ns, kNowNs + 4'500'000'000LL);
}

TEST(CooperativePassageCoordination, ReservesOnlyAConflictingJunctionMovement) {
  const CooperativeFlightIntentData ownship =
      linearIntent("civilian_1", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
                   Vec3{10.0, 0.0, 0.0},
                   withResources(passageUse("junction:west_east", "segment:west", -3.0,
                                            -6.0, 6.0, 1),
                                 {"segment:west", "segment:center", "segment:east"}));
  const CooperativeFlightIntentData peer =
      linearIntent("civilian_0", Point3{0.0, -10.0, 10.0}, Point3{0.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0},
                   withResources(passageUse("junction:south_north", "segment:south",
                                            -3.0, -6.0, 6.0, 1),
                                 {"segment:south", "segment:center", "segment:north"}));

  const CooperativePassageDecision decision =
      coordinateCooperativePassage(ownship, std::vector{peer});

  EXPECT_TRUE(decision.yield_before_entry);
  EXPECT_TRUE(decision.conflict_zone_only);
  EXPECT_EQ(decision.yield_to_vehicle_id, "civilian_0");
  EXPECT_EQ(decision.conflict_resource_id, "segment:center");
}

TEST(CooperativePassageCoordination, DoesNotReserveDisjointJunctionArms) {
  const CooperativeFlightIntentData ownship =
      linearIntent("civilian_1", Point3{-10.0, 0.0, 10.0}, Point3{10.0, 0.0, 10.0},
                   Vec3{10.0, 0.0, 0.0},
                   withResources(passageUse("junction:west_north", "segment:west", -3.0,
                                            -6.0, 6.0, 1),
                                 {"segment:west", "segment:north"}));
  const CooperativeFlightIntentData peer =
      linearIntent("civilian_0", Point3{0.0, -10.0, 10.0}, Point3{0.0, 10.0, 10.0},
                   Vec3{0.0, 10.0, 0.0},
                   withResources(passageUse("junction:south_east", "segment:south",
                                            -3.0, -6.0, 6.0, 1),
                                 {"segment:south", "segment:east"}));

  const CooperativePassageDecision decision =
      coordinateCooperativePassage(ownship, std::vector{peer});

  EXPECT_TRUE(decision.active);
  EXPECT_FALSE(decision.yield_before_entry);
}

TEST(CooperativePassageCoordination, SamePathNeedsNoDelayWhenSpacingIsSafe) {
  CooperativeFlightIntentData ownship = linearIntent(
      "civilian_1", Point3{-20.0, 0.0, 10.0}, Point3{-10.0, 0.0, 10.0},
      Vec3{5.0, 0.0, 0.0}, passageUse("passage", "passage", -3.0, -6.0, 6.0, 1));
  CooperativeFlightIntentData peer = linearIntent(
      "civilian_0", Point3{10.0, 0.0, 10.0}, Point3{20.0, 0.0, 10.0},
      Vec3{5.0, 0.0, 0.0}, passageUse("passage", "passage", -3.0, -6.0, 6.0, 1));
  ownship.passage.predicted_entry_ns = kNowNs + 2'000'000'000LL;
  ownship.passage.conflict_resources.front().predicted_entry_ns =
      kNowNs + 2'000'000'000LL;
  peer.passage.predicted_entry_ns = kNowNs + 1'000'000'000LL;

  const CooperativePassageDecision decision =
      coordinateCooperativePassage(ownship, std::vector{peer});

  EXPECT_FALSE(decision.yield_before_entry);
  EXPECT_EQ(decision.entry_not_before_ns, 0);
}

} // namespace
} // namespace drone_city_nav
