#include "drone_city_nav/trajectory_refinement_scheduler.hpp"

#include <algorithm>

namespace drone_city_nav {

void TrajectoryRefinementScheduler::configure(const std::size_t worker_count) noexcept {
  worker_count_ = std::min<std::size_t>(worker_count, 1U);
  if (worker_count_ == 0U) {
    active_job_.reset();
    queued_job_.reset();
  }
}

std::size_t TrajectoryRefinementScheduler::workerCount() const noexcept {
  return worker_count_;
}

bool TrajectoryRefinementScheduler::enabled() const noexcept {
  return worker_count_ > 0U;
}

std::optional<TrajectoryRefinementJob>
TrajectoryRefinementScheduler::activeJob() const noexcept {
  return active_job_;
}

std::optional<TrajectoryRefinementJob>
TrajectoryRefinementScheduler::queuedJob() const noexcept {
  return queued_job_;
}

TrajectoryRefinementScheduleDecision
TrajectoryRefinementScheduler::submit(const TrajectoryRefinementJob job) noexcept {
  if (!enabled()) {
    return TrajectoryRefinementScheduleDecision{
        .action = TrajectoryRefinementScheduleAction::kDisabled,
        .active_job = active_job_,
        .queued_job = queued_job_,
    };
  }

  if (!active_job_.has_value()) {
    active_job_ = job;
    return TrajectoryRefinementScheduleDecision{
        .action = TrajectoryRefinementScheduleAction::kStartNow,
        .active_job = active_job_,
        .queued_job = queued_job_,
    };
  }

  const bool replaced_existing_queue = queued_job_.has_value();
  queued_job_ = job;
  return TrajectoryRefinementScheduleDecision{
      .action = replaced_existing_queue
                    ? TrajectoryRefinementScheduleAction::kReplacedQueuedLatest
                    : TrajectoryRefinementScheduleAction::kQueuedLatest,
      .active_job = active_job_,
      .queued_job = queued_job_,
  };
}

std::optional<TrajectoryRefinementJob> TrajectoryRefinementScheduler::completeActive(
    const TrajectoryRefinementJob job) noexcept {
  if (!active_job_.has_value() || *active_job_ != job) {
    return std::nullopt;
  }

  active_job_.reset();
  if (!queued_job_.has_value()) {
    return std::nullopt;
  }

  active_job_ = *queued_job_;
  queued_job_.reset();
  return active_job_;
}

} // namespace drone_city_nav
