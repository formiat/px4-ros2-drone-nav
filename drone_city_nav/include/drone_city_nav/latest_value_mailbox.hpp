#pragma once

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <stop_token>
#include <utility>

namespace drone_city_nav {

template<typename T> class LatestValueMailbox final {
public:
  [[nodiscard]] bool push(T value) {
    const std::scoped_lock lock{mutex_};
    const bool replaced = pending_.has_value();
    pending_ = std::move(value);
    condition_.notify_one();
    return replaced;
  }

  [[nodiscard]] std::optional<T> waitPop(const std::stop_token stop_token) {
    std::unique_lock lock{mutex_};
    condition_.wait(lock, stop_token,
                    [this]() noexcept { return pending_.has_value(); });
    if (!pending_.has_value()) {
      return std::nullopt;
    }
    std::optional<T> value = std::move(pending_);
    pending_.reset();
    return value;
  }

  [[nodiscard]] std::optional<T> tryPop() {
    const std::scoped_lock lock{mutex_};
    if (!pending_.has_value()) {
      return std::nullopt;
    }
    std::optional<T> value = std::move(pending_);
    pending_.reset();
    return value;
  }

  void notifyAll() noexcept {
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable_any condition_;
  std::optional<T> pending_;
};

template<typename T> class BoundedFifoMailbox final {
public:
  explicit BoundedFifoMailbox(const std::size_t capacity) noexcept
      : capacity_{std::max<std::size_t>(1U, capacity)} {
  }

  // Keeps the freshest bounded sequence under overload, but never represents
  // dropped physical observations as repeated evidence from a retained sample.
  [[nodiscard]] bool push(T value) {
    const std::scoped_lock lock{mutex_};
    const bool dropped = pending_.size() == capacity_;
    if (dropped) {
      pending_.pop_front();
    }
    pending_.push_back(std::move(value));
    condition_.notify_one();
    return dropped;
  }

  [[nodiscard]] std::optional<T> waitPop(const std::stop_token stop_token) {
    std::unique_lock lock{mutex_};
    condition_.wait(lock, stop_token, [this]() noexcept { return !pending_.empty(); });
    if (pending_.empty()) {
      return std::nullopt;
    }
    T value = std::move(pending_.front());
    pending_.pop_front();
    return value;
  }

  [[nodiscard]] std::optional<T> tryPop() {
    const std::scoped_lock lock{mutex_};
    if (pending_.empty()) {
      return std::nullopt;
    }
    T value = std::move(pending_.front());
    pending_.pop_front();
    return value;
  }

  void notifyAll() noexcept {
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable_any condition_;
  std::deque<T> pending_;
  std::size_t capacity_{1U};
};

} // namespace drone_city_nav
