#pragma once

#include <mutex>
#include <optional>
#include <utility>

namespace drone_city_nav {

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
    if (processor_active_ || !pending_.has_value()) {
      return false;
    }
    processor_active_ = true;
    return true;
  }

  [[nodiscard]] std::optional<T> take() {
    const std::scoped_lock lock{mutex_};
    if (!processor_active_) {
      return std::nullopt;
    }
    if (!pending_.has_value()) {
      processor_active_ = false;
      return std::nullopt;
    }
    std::optional<T> value = std::move(pending_);
    pending_.reset();
    return value;
  }

  // Returns true when a newer value is already pending and the caller keeps
  // processor ownership. Otherwise the supplied value is deferred and the
  // processor is released until new evidence calls tryAcquireProcessor().
  [[nodiscard]] bool deferOrContinue(T value) {
    const std::scoped_lock lock{mutex_};
    if (!processor_active_) {
      return false;
    }
    if (pending_.has_value()) {
      return true;
    }
    pending_ = std::move(value);
    processor_active_ = false;
    return false;
  }

private:
  std::mutex mutex_;
  std::optional<T> pending_;
  bool processor_active_{false};
};

} // namespace drone_city_nav
