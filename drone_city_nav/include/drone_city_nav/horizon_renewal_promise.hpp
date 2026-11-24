#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

// The promise a planned horizon carries (specification K89): within this long
// of the horizon the planner publishes the next. The speed laws charge what
// the promise takes over its floor as reaction latency, and the offboard
// brakes a horizon the promise ran out on. It is measured: one and a half
// times the longest of the last intervals between one planned horizon and the
// planned horizon published next after it, so that a planner that renews
// slowly promises late, flies slower for it and, past the limit, rests.
//
// The floor is what the flown planner keeps: its intervals stand under 0.2 s but
// for a few of a flight (2 to 13 of 5 000 to 15 000 in motion, r4165 to
// r4180), and those few are what the brake is for.
inline constexpr std::int64_t kHorizonRenewalPromiseFloorNs{200'000'000};
// A promise past this is none: the offboard reads the planner dead at the
// same age of its heartbeat (K24).
inline constexpr std::int64_t kHorizonRenewalPromiseLimitNs{1'000'000'000};
// The intervals the promise is measured over: half a second of ticks at their
// nominal rate, so that it returns as soon as the renewals have.
inline constexpr std::size_t kHorizonRenewalIntervalCount{25U};

class HorizonRenewalPromise final {
public:
  void recordInterval(const std::int64_t interval_ns) noexcept {
    intervals_ns_[next_] = std::max<std::int64_t>(0, interval_ns);
    next_ = (next_ + 1U) % intervals_ns_.size();
  }

  [[nodiscard]] std::int64_t promiseNs() const noexcept {
    const std::int64_t longest_ns = *std::ranges::max_element(intervals_ns_);
    return std::max(kHorizonRenewalPromiseFloorNs, longest_ns + longest_ns / 2);
  }

private:
  std::array<std::int64_t, kHorizonRenewalIntervalCount> intervals_ns_{};
  std::size_t next_{0U};
};

} // namespace drone_city_nav
