#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct TrajectoryRefinementJob {
  std::uint64_t generation{0U};
  std::uint64_t baseline_path_id{0U};

  [[nodiscard]] friend bool operator==(const TrajectoryRefinementJob& lhs,
                                       const TrajectoryRefinementJob& rhs) noexcept {
    return lhs.generation == rhs.generation &&
           lhs.baseline_path_id == rhs.baseline_path_id;
  }
};

enum class TrajectoryRefinementScheduleAction {
  kDisabled,
  kStartNow,
  kQueuedLatest,
  kReplacedQueuedLatest,
};

struct TrajectoryRefinementScheduleDecision {
  TrajectoryRefinementScheduleAction action{
      TrajectoryRefinementScheduleAction::kDisabled};
  std::optional<TrajectoryRefinementJob> active_job;
  std::optional<TrajectoryRefinementJob> queued_job;
};

class TrajectoryRefinementScheduler {
public:
  void configure(std::size_t worker_count) noexcept;

  [[nodiscard]] std::size_t workerCount() const noexcept;
  [[nodiscard]] bool enabled() const noexcept;
  [[nodiscard]] std::optional<TrajectoryRefinementJob> activeJob() const noexcept;
  [[nodiscard]] std::optional<TrajectoryRefinementJob> queuedJob() const noexcept;

  [[nodiscard]] TrajectoryRefinementScheduleDecision
  submit(TrajectoryRefinementJob job) noexcept;

  [[nodiscard]] std::optional<TrajectoryRefinementJob>
  completeActive(TrajectoryRefinementJob job) noexcept;

private:
  std::size_t worker_count_{1U};
  std::optional<TrajectoryRefinementJob> active_job_;
  std::optional<TrajectoryRefinementJob> queued_job_;
};

} // namespace drone_city_nav
