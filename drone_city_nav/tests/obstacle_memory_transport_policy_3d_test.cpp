#include "drone_city_nav/obstacle_memory_transport_policy_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecondNs{1'000'000'000LL};

TEST(ObstacleMemoryTransportPolicy3DTest, RequiresInitialSnapshot) {
  ObstacleMemoryTransportPolicy3D policy;

  EXPECT_EQ(policy.decide(ObstacleMemoryTransportPolicy3DInput{
                .now_steady_ns = kSecondNs,
                .revision = 1U,
                .current_chunk_count = 10U,
                .dirty_chunk_count = 10U,
                .full_reset = true,
            }),
            ObstacleMemoryTransportDecision3D::kSnapshot);
}

TEST(ObstacleMemoryTransportPolicy3DTest, UsesCumulativeDeltaBeforeRebaseWindow) {
  ObstacleMemoryTransportPolicy3D policy{
      ObstacleMemoryTransportPolicy3DConfig{10.0, 30.0, 0.75}};
  policy.recordSnapshot(1U, kSecondNs);

  EXPECT_EQ(policy.decide(ObstacleMemoryTransportPolicy3DInput{
                .now_steady_ns = 2 * kSecondNs,
                .revision = 2U,
                .current_chunk_count = 100U,
                .dirty_chunk_count = 100U,
            }),
            ObstacleMemoryTransportDecision3D::kDelta);
}

TEST(ObstacleMemoryTransportPolicy3DTest, RebasesWhenDeltaApproachesSnapshotSize) {
  ObstacleMemoryTransportPolicy3D policy{
      ObstacleMemoryTransportPolicy3DConfig{10.0, 30.0, 0.75}};
  policy.recordSnapshot(1U, kSecondNs);

  EXPECT_EQ(policy.decide(ObstacleMemoryTransportPolicy3DInput{
                .now_steady_ns = 11 * kSecondNs,
                .revision = 2U,
                .current_chunk_count = 100U,
                .dirty_chunk_count = 75U,
            }),
            ObstacleMemoryTransportDecision3D::kSnapshot);
}

TEST(ObstacleMemoryTransportPolicy3DTest, MaximumAgeEventuallyRebasesSmallDelta) {
  ObstacleMemoryTransportPolicy3D policy{
      ObstacleMemoryTransportPolicy3DConfig{10.0, 30.0, 0.75}};
  policy.recordSnapshot(1U, kSecondNs);

  EXPECT_EQ(policy.decide(ObstacleMemoryTransportPolicy3DInput{
                .now_steady_ns = 31 * kSecondNs,
                .revision = 2U,
                .current_chunk_count = 100U,
                .dirty_chunk_count = 1U,
            }),
            ObstacleMemoryTransportDecision3D::kSnapshot);
}

TEST(ObstacleMemoryTransportPolicy3DTest, RejectsContradictoryPeriods) {
  EXPECT_THROW(static_cast<void>(ObstacleMemoryTransportPolicy3D{
                   ObstacleMemoryTransportPolicy3DConfig{30.0, 10.0, 0.75}}),
               std::invalid_argument);
}

} // namespace
} // namespace drone_city_nav
