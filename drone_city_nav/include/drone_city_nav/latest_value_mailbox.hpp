#pragma once

#include <condition_variable>
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

} // namespace drone_city_nav
