#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <string_view>
#include <utility>

namespace drone_city_nav {

class ScopedOffboardCallbackDuration final {
public:
  ScopedOffboardCallbackDuration(rclcpp::Logger logger,
                                 const std::string_view callback_name,
                                 const std::size_t payload_size)
      : logger_{std::move(logger)},
        callback_name_{callback_name},
        payload_size_{payload_size},
        started_at_{std::chrono::steady_clock::now()} {
  }

  ScopedOffboardCallbackDuration(const ScopedOffboardCallbackDuration&) = delete;
  ScopedOffboardCallbackDuration&
  operator=(const ScopedOffboardCallbackDuration&) = delete;
  ScopedOffboardCallbackDuration(ScopedOffboardCallbackDuration&&) = delete;
  ScopedOffboardCallbackDuration& operator=(ScopedOffboardCallbackDuration&&) = delete;

  ~ScopedOffboardCallbackDuration() {
    constexpr auto kSlowCallbackThreshold = std::chrono::milliseconds{100};
    const double duration_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - started_at_)
                                   .count();
    if (duration_ms < static_cast<double>(kSlowCallbackThreshold.count())) {
      return;
    }
    RCLCPP_WARN(logger_,
                "Slow offboard callback: callback=%.*s duration_ms=%.1f "
                "payload_size=%zu outcome=%.*s planner_path_id=%" PRIu64
                " path_stamp_ns=%" PRIu64,
                static_cast<int>(callback_name_.size()), callback_name_.data(),
                duration_ms, payload_size_, static_cast<int>(outcome_.size()),
                outcome_.data(), planner_path_id_, path_stamp_ns_);
  }

  void setOutcome(const std::string_view outcome) noexcept {
    outcome_ = outcome;
  }

  void setTrajectoryIdentity(const std::uint64_t planner_path_id,
                             const std::uint64_t path_stamp_ns) noexcept {
    planner_path_id_ = planner_path_id;
    path_stamp_ns_ = path_stamp_ns;
  }

private:
  rclcpp::Logger logger_;
  std::string_view callback_name_;
  std::size_t payload_size_{0U};
  std::chrono::steady_clock::time_point started_at_;
  std::string_view outcome_{"completed"};
  std::uint64_t planner_path_id_{0U};
  std::uint64_t path_stamp_ns_{0U};
};

} // namespace drone_city_nav
