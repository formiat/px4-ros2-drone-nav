#pragma once

#include <mutex>
#include <optional>
#include <utility>

namespace drone_city_nav {

// A single-consumer queue for evidence that supersedes itself: at most one
// value waits to be processed, and a newer submission replaces it. A value the
// processor has taken but cannot finish yet, because it waits for evidence
// that arrives on another path, is deferred instead of dropped: it keeps its
// place ahead of whatever is submitted meanwhile, and the processor is
// released until that evidence calls tryAcquireProcessor(). Letting each newer
// submission supersede the deferred value starved a 3D lidar whose scans each
// waited for a pose bracket almost a scan period long: every scan was replaced
// by its successor just before its own bracket arrived, and the successor then
// waited for a bracket a period further on.
template<typename T> class LatestProcessingQueue final {
public:
  struct Submission {
    bool replaced_pending{false};
    bool processor_acquired{false};
  };

  [[nodiscard]] Submission submit(T value) {
    const std::scoped_lock lock{mutex_};
    const bool replaced_pending = pending_.has_value();
    pending_ = std::move(value);
    const bool processor_acquired = !processor_active_;
    processor_active_ = true;
    return Submission{replaced_pending, processor_acquired};
  }

  [[nodiscard]] bool tryAcquireProcessor() {
    const std::scoped_lock lock{mutex_};
    if (processor_active_ || (!deferred_.has_value() && !pending_.has_value())) {
      return false;
    }
    processor_active_ = true;
    return true;
  }

  // The deferred value first, then the latest submission. Releases the
  // processor when nothing is left.
  [[nodiscard]] std::optional<T> take() {
    const std::scoped_lock lock{mutex_};
    if (!processor_active_) {
      return std::nullopt;
    }
    if (deferred_.has_value()) {
      std::optional<T> value = std::move(deferred_);
      deferred_.reset();
      return value;
    }
    if (pending_.has_value()) {
      std::optional<T> value = std::move(pending_);
      pending_.reset();
      return value;
    }
    processor_active_ = false;
    return std::nullopt;
  }

  // The processor could not finish this value yet. It waits, ahead of any
  // newer submission, for the evidence that lets it finish; the processor is
  // released until tryAcquireProcessor() or a submission reacquires it.
  void defer(T value) {
    const std::scoped_lock lock{mutex_};
    deferred_ = std::move(value);
    processor_active_ = false;
  }

private:
  std::mutex mutex_;
  std::optional<T> deferred_;
  std::optional<T> pending_;
  bool processor_active_{false};
};

} // namespace drone_city_nav
