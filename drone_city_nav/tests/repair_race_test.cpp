#include "drone_city_nav/repair_race.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PlanningGridVersion version(const std::uint64_t revision = 1U) {
  return PlanningGridVersion{
      .build_revision = revision,
      .memory_producer_instance_id = 7U,
      .memory_sequence = 19U,
      .lidar_update_ns = 1234,
      .config_fingerprint = 91U,
  };
}

[[nodiscard]] RepairSnapshot snapshot() {
  RepairSnapshot value{};
  value.generation = 5U;
  value.blocked_path_id = 11U;
  value.temporary_prefix_fingerprint = 17U;
  value.grid_version = version();
  return value;
}

[[nodiscard]] RepairResult result(const bool valid = true) {
  RepairResult value{};
  value.generation = 5U;
  value.blocked_path_id = 11U;
  value.temporary_prefix_fingerprint = 17U;
  value.source_grid_version = version();
  value.valid = valid;
  return value;
}

TEST(RepairRaceArbiter, InvalidCompletionDoesNotCloseRace) {
  const RepairSnapshot input = snapshot();
  RepairRaceArbiter arbiter{input};

  EXPECT_FALSE(arbiter.consider(result(false)));
  EXPECT_FALSE(arbiter.winnerSelected());
  EXPECT_TRUE(arbiter.consider(result()));
  EXPECT_TRUE(arbiter.winnerSelected());
}

TEST(RepairRaceArbiter, RejectsStaleIdentityAndAcceptsExactVersion) {
  const RepairSnapshot input = snapshot();
  RepairRaceArbiter arbiter{input};

  RepairResult stale_generation = result();
  stale_generation.generation = 6U;
  EXPECT_FALSE(arbiter.consider(stale_generation));

  RepairResult stale_path = result();
  stale_path.blocked_path_id = 12U;
  EXPECT_FALSE(arbiter.consider(stale_path));

  RepairResult stale_prefix = result();
  stale_prefix.temporary_prefix_fingerprint = 18U;
  EXPECT_FALSE(arbiter.consider(stale_prefix));

  RepairResult stale_grid = result();
  stale_grid.source_grid_version.build_revision = 2U;
  EXPECT_FALSE(arbiter.consider(stale_grid));
  EXPECT_TRUE(arbiter.consider(result()));
}

TEST(RepairRaceArbiter, ConcurrentCompletionsSelectExactlyOneWinner) {
  const RepairSnapshot input = snapshot();
  RepairRaceArbiter arbiter{input};
  std::atomic_size_t winners{0U};
  std::vector<std::jthread> threads;
  threads.reserve(16U);
  for (std::size_t index = 0U; index < 16U; ++index) {
    threads.emplace_back([&arbiter, &winners]() {
      if (arbiter.consider(result())) {
        winners.fetch_add(1U);
      }
    });
  }
  threads.clear();

  EXPECT_EQ(winners.load(), 1U);
  EXPECT_TRUE(arbiter.winnerSelected());
}

} // namespace
} // namespace drone_city_nav
