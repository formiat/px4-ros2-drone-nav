#include "drone_city_nav/repair_race.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <future>
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

[[nodiscard]] std::shared_ptr<const RepairSnapshot> sharedSnapshot() {
  return std::make_shared<const RepairSnapshot>(snapshot());
}

[[nodiscard]] std::shared_ptr<const RepairSnapshot> productionSnapshot() {
  OccupancyGrid2D grid{GridBounds{-1.0, -10.0, 1.0, 50, 20}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  RepairSnapshot value = snapshot();
  value.grids.push_back(RepairGridSnapshot{
      .name = "runtime_prohibited",
      .grid = grid,
      .clearance = ClearanceField2D::build(grid, 10.0, ClearanceSource::kProhibited),
  });
  value.old_trajectory = ExecutableTrajectoryArtifact{
      .path_id = value.blocked_path_id,
      .mission_goal = Point2{30.0, 0.0},
      .samples = trajectoryPointSamplesFromPoints(
          std::vector<Point2>{{0.0, 0.0}, {30.0, 0.0}}),
      .current_s_m = 0.0,
  };
  value.anchor = trajectorySampleAtS(value.old_trajectory.samples, 2.0);
  value.truncation_s_m = 2.0;
  value.blocked_span = BlockedSpan{
      .first_blocked_s_m = 5.0,
      .last_blocked_s_m = 10.0,
  };
  return std::make_shared<const RepairSnapshot>(std::move(value));
}

[[nodiscard]] RepairRaceConfig productionConfig() {
  RepairRaceConfig config{};
  config.moving_astar.initial_heading_bias_enabled = false;
  config.trajectory.corridor.max_radius_m = 5.0;
  config.trajectory.corridor.sample_step_m = 1.0;
  config.trajectory.trajectory_optimizer.max_iterations = 2U;
  config.trajectory.trajectory_optimizer.parallel_workers = 1U;
  config.trajectory.vertical_profile.enabled = false;
  config.trajectory.known_passage_validation.enabled = false;
  config.trajectory.passage_insertion.enabled = false;
  config.trajectory.speed_profile.cruise_speed_mps = 10.0;
  config.trajectory.speed_profile.min_turn_speed_mps = 2.0;
  config.trajectory.speed_profile.speed_profile_decel_mps2 = 4.0;
  config.trajectory.speed_profile.speed_profile_sample_step_m = 1.0;
  config.reconnect_margins_m = {5.0};
  return config;
}

[[nodiscard]] RepairResult trajectoryResult() {
  RepairResult candidate = result();
  candidate.trajectory.valid = true;
  candidate.trajectory.samples =
      trajectoryPointSamplesFromPoints(std::vector<Point2>{{0.0, 0.0}, {10.0, 0.0}});
  return candidate;
}

[[nodiscard]] OccupancyGrid2D validationGrid() {
  OccupancyGrid2D grid{GridBounds{-1.0, -5.0, 1.0, 20, 10}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
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

TEST(RepairRaceCoordinator, InvalidFirstCompletionDoesNotPreventNextWinner) {
  std::promise<void> invalid_completed;
  const std::shared_future<void> invalid_completed_future =
      invalid_completed.get_future().share();
  const std::vector<RepairJob> jobs{
      [&invalid_completed](std::stop_token) {
        RepairResult invalid = result(false);
        invalid.reason = "invalid";
        invalid_completed.set_value();
        return invalid;
      },
      [invalid_completed_future](std::stop_token) {
        invalid_completed_future.wait();
        return result();
      },
  };
  std::atomic_size_t handoffs{0U};

  const RepairRaceOutcome outcome =
      runRepairJobs(sharedSnapshot(), jobs, {},
                    [&handoffs](const RepairResult&, std::size_t, std::size_t) {
                      handoffs.fetch_add(1U);
                    });

  ASSERT_TRUE(outcome.winner.has_value());
  EXPECT_EQ(handoffs.load(), 1U);
  EXPECT_EQ(outcome.summary.invalid_results, 1U);
  EXPECT_EQ(outcome.summary.completions, 2U);
}

TEST(RepairRaceCoordinator, HandsOffWinnerBeforeSlowLoserCompletes) {
  std::promise<void> slow_started;
  std::promise<void> release_slow;
  std::promise<void> winner_handed_off;
  const std::shared_future<void> slow_started_future =
      slow_started.get_future().share();
  const std::shared_future<void> release_slow_future =
      release_slow.get_future().share();
  const std::vector<RepairJob> jobs{
      [&slow_started, release_slow_future](std::stop_token) {
        slow_started.set_value();
        release_slow_future.wait();
        RepairResult slow = result(false);
        slow.reason = "slow_loser";
        return slow;
      },
      [slow_started_future](std::stop_token) {
        slow_started_future.wait();
        return result();
      },
  };

  auto race = std::async(std::launch::async, [&jobs, &winner_handed_off]() {
    return runRepairJobs(
        sharedSnapshot(), jobs, {},
        [&winner_handed_off](const RepairResult&, std::size_t, std::size_t) {
          winner_handed_off.set_value();
        });
  });

  const std::future_status handoff_status =
      winner_handed_off.get_future().wait_for(std::chrono::seconds{1});
  const std::future_status race_status = race.wait_for(std::chrono::milliseconds{0});
  release_slow.set_value();
  const RepairRaceOutcome outcome = race.get();

  EXPECT_EQ(handoff_status, std::future_status::ready);
  EXPECT_EQ(race_status, std::future_status::timeout);
  EXPECT_TRUE(outcome.winner.has_value());
}

TEST(RepairRaceCoordinator, WinnerRequestsCooperativeCancellation) {
  std::promise<void> loser_started;
  const std::shared_future<void> loser_started_future =
      loser_started.get_future().share();
  std::atomic_bool cancellation_observed{false};
  const std::vector<RepairJob> jobs{
      [&loser_started, &cancellation_observed](const std::stop_token stop_token) {
        loser_started.set_value();
        while (!stop_token.stop_requested()) {
          std::this_thread::yield();
        }
        cancellation_observed.store(true);
        RepairResult canceled = result(false);
        canceled.canceled = true;
        canceled.reason = "canceled";
        return canceled;
      },
      [loser_started_future](std::stop_token) {
        loser_started_future.wait();
        return result();
      },
  };

  const RepairRaceOutcome outcome = runRepairJobs(sharedSnapshot(), jobs);

  EXPECT_TRUE(outcome.winner.has_value());
  EXPECT_TRUE(cancellation_observed.load());
  EXPECT_EQ(outcome.summary.canceled_results, 1U);
}

TEST(RepairRaceCoordinator, FreshRejectedCandidateDoesNotCloseRace) {
  std::promise<void> first_completed;
  const std::shared_future<void> first_completed_future =
      first_completed.get_future().share();
  const std::vector<RepairJob> jobs{
      [&first_completed](std::stop_token) {
        RepairResult first = result();
        first.reconnect_margin_m = 10.0;
        first_completed.set_value();
        return first;
      },
      [first_completed_future](std::stop_token) {
        first_completed_future.wait();
        RepairResult second = result();
        second.reconnect_margin_m = 20.0;
        return second;
      },
  };

  const RepairRaceOutcome outcome =
      runRepairJobs(sharedSnapshot(), jobs, [](const RepairResult& candidate) {
        return candidate.reconnect_margin_m > 10.0;
      });

  ASSERT_TRUE(outcome.winner.has_value());
  EXPECT_DOUBLE_EQ(outcome.winner->reconnect_margin_m, 20.0);
  EXPECT_EQ(outcome.summary.invalid_results, 1U);
}

TEST(RepairRaceCoordinator, FullJobWinsAfterPartialFailures) {
  std::promise<void> partial_completed;
  const std::shared_future<void> partial_completed_future =
      partial_completed.get_future().share();
  const std::vector<RepairJob> jobs{
      [&partial_completed](std::stop_token) {
        RepairResult partial = result(false);
        partial.kind = RepairJobKind::kPartial;
        partial.reason = "partial_failed";
        partial_completed.set_value();
        return partial;
      },
      [partial_completed_future](std::stop_token) {
        partial_completed_future.wait();
        RepairResult full = result();
        full.kind = RepairJobKind::kFull;
        return full;
      },
  };

  const RepairRaceOutcome outcome = runRepairJobs(sharedSnapshot(), jobs);

  ASSERT_TRUE(outcome.winner.has_value());
  EXPECT_EQ(outcome.winner->kind, RepairJobKind::kFull);
}

TEST(RepairRaceCoordinator, AllFailuresProduceAggregateWithoutHandoff) {
  const std::vector<RepairJob> jobs{
      [](std::stop_token) { return result(false); },
      [](std::stop_token) {
        RepairResult failed = result(false);
        failed.kind = RepairJobKind::kFull;
        return failed;
      },
  };
  std::atomic_size_t handoffs{0U};

  const RepairRaceOutcome outcome =
      runRepairJobs(sharedSnapshot(), jobs, {},
                    [&handoffs](const RepairResult&, std::size_t, std::size_t) {
                      handoffs.fetch_add(1U);
                    });

  EXPECT_FALSE(outcome.winner.has_value());
  EXPECT_EQ(handoffs.load(), 0U);
  EXPECT_EQ(outcome.summary.jobs_started, 2U);
  EXPECT_EQ(outcome.summary.invalid_results, 2U);
  EXPECT_EQ(outcome.summary.completions, 2U);
}

TEST(RepairRaceCoordinator, ProductionRaceHandsOffOneValidTrajectory) {
  std::atomic_size_t handoffs{0U};

  const RepairRaceOutcome outcome =
      runRepairRace(productionSnapshot(), productionConfig(), {},
                    [&handoffs](const RepairResult& winner, std::size_t, std::size_t) {
                      EXPECT_TRUE(winner.trajectory.valid);
                      handoffs.fetch_add(1U);
                    });

  ASSERT_TRUE(outcome.winner.has_value());
  EXPECT_TRUE(outcome.winner->trajectory.valid);
  EXPECT_EQ(handoffs.load(), 1U);
  EXPECT_EQ(outcome.summary.completions, outcome.summary.jobs_started);
}

TEST(RepairFreshValidation, NewRevisionWithSameGridAcceptsCandidate) {
  const RepairResult candidate = trajectoryResult();
  PlanningGridVersion fresh_version = candidate.source_grid_version;
  fresh_version.build_revision += 1U;
  const OccupancyGrid2D grid = validationGrid();
  const std::array prefix{Point2{0.0, 0.0}, Point2{1.0, 0.0}};

  const RepairFreshValidationResult validation =
      validateRepairResultOnFreshGrid(RepairFreshValidationInput{
          .candidate = &candidate,
          .fresh_grid_version = &fresh_version,
          .fresh_runtime_grid = &grid,
          .remaining_prefix = prefix,
      });

  EXPECT_TRUE(validation.valid);
  EXPECT_EQ(validation.reason, RepairFreshValidationReason::kAccepted);
}

TEST(RepairFreshValidation, NewLidarBlockerRejectsOnlyAffectedCandidate) {
  const RepairResult candidate = trajectoryResult();
  PlanningGridVersion fresh_version = candidate.source_grid_version;
  fresh_version.build_revision += 1U;
  fresh_version.lidar_update_ns += 100;
  OccupancyGrid2D blocked_grid = validationGrid();
  const auto blocker = blocked_grid.worldToCell(Point2{5.0, 0.0});
  ASSERT_TRUE(blocker.has_value());
  blocked_grid.setOccupied(*blocker);
  const std::array prefix{Point2{0.0, 0.0}, Point2{1.0, 0.0}};

  const RepairFreshValidationResult affected =
      validateRepairResultOnFreshGrid(RepairFreshValidationInput{
          .candidate = &candidate,
          .fresh_grid_version = &fresh_version,
          .fresh_runtime_grid = &blocked_grid,
          .remaining_prefix = prefix,
      });
  const OccupancyGrid2D clear_grid = validationGrid();
  const RepairFreshValidationResult unaffected =
      validateRepairResultOnFreshGrid(RepairFreshValidationInput{
          .candidate = &candidate,
          .fresh_grid_version = &fresh_version,
          .fresh_runtime_grid = &clear_grid,
          .remaining_prefix = prefix,
      });

  EXPECT_FALSE(affected.valid);
  EXPECT_EQ(affected.reason, RepairFreshValidationReason::kCandidateBlocked);
  EXPECT_TRUE(unaffected.valid);
}

} // namespace
} // namespace drone_city_nav
