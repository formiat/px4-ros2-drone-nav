#include "drone_city_nav/cooperative_space_time.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kStartNs{1'000'000'000LL};

[[nodiscard]] CooperativeFlightIntentData
makeLinearIntent(std::string vehicle_id, const Point3& start, const Vec3& velocity,
                 const double duration_s = 5.0) {
  CooperativeFlightIntentData result{
      .vehicle_id = std::move(vehicle_id),
      .frame_id = "map",
      .stamp_ns = kStartNs,
      .intent_generation = 1U,
      .valid_from_ns = kStartNs,
      .valid_until_ns = kStartNs + static_cast<std::int64_t>(duration_s * 1.0e9),
      .footprint_radius_m = 0.82,
      .current_position = start,
      .current_velocity = velocity,
      .maneuver_state = CooperativeManeuver::kKeep,
      .conflicting_vehicle_ids = {},
      .channel = {},
      .trajectory = {},
  };
  for (std::size_t index = 0U; index <= static_cast<std::size_t>(duration_s * 10.0);
       ++index) {
    const double elapsed_s = 0.1 * static_cast<double>(index);
    result.trajectory.push_back(CooperativeTrajectorySample{
        .time_ns = kStartNs + static_cast<std::int64_t>(elapsed_s * 1.0e9),
        .position =
            Point3{start.x + velocity.x * elapsed_s, start.y + velocity.y * elapsed_s,
                   start.z + velocity.z * elapsed_s},
        .velocity = velocity,
    });
  }
  return result;
}

[[nodiscard]] CooperativeConflictPeer
makeConflictPeer(const CooperativeFlightIntentData& ownship,
                 const CooperativeFlightIntentData& peer) {
  return CooperativeConflictPeer{
      .intent = peer,
      .prediction = predictCooperativeConflict(ownship, peer, kStartNs,
                                               CooperativeConflictConfig{}),
  };
}

TEST(CooperativeSpaceTimeTest, HeadOnAgentsSelectDeterministicImprovingPlans) {
  const CooperativeFlightIntentData first =
      makeLinearIntent("civilian_0", Point3{-5.0, 0.0, 10.0}, Vec3{2.0, 0.0, 0.0});
  const CooperativeFlightIntentData second =
      makeLinearIntent("civilian_1", Point3{5.0, 0.0, 10.0}, Vec3{-2.0, 0.0, 0.0});
  const CooperativeConflictPeer second_peer = makeConflictPeer(first, second);
  const CooperativeConflictPeer first_peer = makeConflictPeer(second, first);
  ASSERT_TRUE(second_peer.prediction.conflict_predicted);
  ASSERT_TRUE(first_peer.prediction.conflict_predicted);

  const CooperativeSpaceTimeDecision first_decision =
      optimizeCooperativeSpaceTime(first, std::span{&second_peer, 1U}, kStartNs);
  const CooperativeSpaceTimeDecision second_decision =
      optimizeCooperativeSpaceTime(second, std::span{&first_peer, 1U}, kStartNs);

  ASSERT_TRUE(first_decision.valid);
  ASSERT_TRUE(second_decision.valid);
  EXPECT_TRUE(first_decision.active);
  EXPECT_TRUE(second_decision.active);
  EXPECT_GT(first_decision.predicted_minimum_separation_m,
            second_peer.prediction.minimum_separation_m);
  EXPECT_GT(second_decision.predicted_minimum_separation_m,
            first_peer.prediction.minimum_separation_m);
  EXPECT_GE(first_decision.evaluated_candidate_count, 5U);
  EXPECT_GE(second_decision.evaluated_candidate_count, 6U);
}

TEST(CooperativeSpaceTimeTest, MultiPeerResultDoesNotDependOnMessageOrder) {
  const CooperativeFlightIntentData ownship =
      makeLinearIntent("civilian_1", Point3{-5.0, 0.0, 10.0}, Vec3{2.0, 0.0, 0.0});
  const CooperativeFlightIntentData first =
      makeLinearIntent("civilian_0", Point3{5.0, 0.0, 10.0}, Vec3{-2.0, 0.0, 0.0});
  const CooperativeFlightIntentData second =
      makeLinearIntent("civilian_2", Point3{0.0, -5.0, 10.0}, Vec3{0.0, 2.0, 0.0});
  std::array peers{makeConflictPeer(ownship, first), makeConflictPeer(ownship, second)};

  const CooperativeSpaceTimeDecision ordered =
      optimizeCooperativeSpaceTime(ownship, peers, kStartNs);
  std::ranges::reverse(peers);
  const CooperativeSpaceTimeDecision reversed =
      optimizeCooperativeSpaceTime(ownship, peers, kStartNs);

  ASSERT_TRUE(ordered.valid);
  ASSERT_TRUE(reversed.valid);
  EXPECT_EQ(ordered.maneuver, reversed.maneuver);
  EXPECT_DOUBLE_EQ(ordered.lateral_offset_m, reversed.lateral_offset_m);
  EXPECT_DOUBLE_EQ(ordered.vertical_offset_m, reversed.vertical_offset_m);
  EXPECT_DOUBLE_EQ(ordered.time_shift_s, reversed.time_shift_s);
}

TEST(CooperativeSpaceTimeTest, LowerPriorityAgentCanSelectTemporalSeparation) {
  const CooperativeFlightIntentData ownship =
      makeLinearIntent("civilian_1", Point3{-10.0, 0.0, 10.0}, Vec3{4.0, 0.0, 0.0});
  const CooperativeFlightIntentData peer =
      makeLinearIntent("civilian_0", Point3{0.0, -10.0, 10.0}, Vec3{0.0, 4.0, 0.0});
  const CooperativeConflictPeer conflict = makeConflictPeer(ownship, peer);
  CooperativeSpaceTimeConfig config;
  config.minimum_spatial_offset_m = 0.1;
  config.maximum_spatial_offset_m = 0.1;
  config.minimum_time_shift_s = 2.0;
  config.maximum_time_shift_s = 2.0;

  const CooperativeSpaceTimeDecision decision =
      optimizeCooperativeSpaceTime(ownship, std::span{&conflict, 1U}, kStartNs, config);

  ASSERT_TRUE(decision.valid);
  EXPECT_EQ(decision.maneuver, CooperativeManeuver::kSlow);
  EXPECT_DOUBLE_EQ(decision.time_shift_s, 2.0);
  EXPECT_GT(decision.predicted_minimum_separation_m,
            conflict.prediction.minimum_separation_m);
}

TEST(CooperativeSpaceTimeTest, IncumbentPlanIsStableWithinHysteresis) {
  const CooperativeFlightIntentData ownship =
      makeLinearIntent("civilian_0", Point3{-5.0, 0.0, 10.0}, Vec3{2.0, 0.0, 0.0});
  const CooperativeFlightIntentData peer =
      makeLinearIntent("civilian_1", Point3{5.0, 0.0, 10.0}, Vec3{-2.0, 0.0, 0.0});
  const CooperativeConflictPeer conflict = makeConflictPeer(ownship, peer);
  const CooperativeSpaceTimeDecision initial =
      optimizeCooperativeSpaceTime(ownship, std::span{&conflict, 1U}, kStartNs);
  ASSERT_TRUE(initial.valid);

  const CooperativeSpaceTimeDecision retained =
      optimizeCooperativeSpaceTime(ownship, std::span{&conflict, 1U}, kStartNs,
                                   CooperativeSpaceTimeConfig{}, initial.maneuver);

  EXPECT_EQ(retained.maneuver, initial.maneuver);
  EXPECT_FALSE(retained.changed);
}

} // namespace
} // namespace drone_city_nav
