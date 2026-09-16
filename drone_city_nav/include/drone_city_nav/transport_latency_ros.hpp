#pragma once

#include <rclcpp/message_info.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

// What a hop of the ROS transport costs: the delivery latency of each
// received message, and the latencies of one hop over a run.

namespace drone_city_nav {

// The delivery latency of one received message: the middleware's receive
// timestamp against the source timestamp the publisher's middleware set,
// both on the host clock the two processes share. NaN when the middleware
// set neither, so a missing measurement never reads as a fast one.
[[nodiscard]] inline double
transportDeliveryLatencyMs(const rclcpp::MessageInfo& info) noexcept {
  const rmw_message_info_t& rmw = info.get_rmw_message_info();
  if (rmw.source_timestamp <= 0 || rmw.received_timestamp <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 1.0e-6 * static_cast<double>(rmw.received_timestamp - rmw.source_timestamp);
}

// The delivery latencies of one hop over a run: the last one for the
// periodic diagnostics, the percentiles for the summary. Receipt and
// reporting happen on different threads.
class TransportLatencySamples {
public:
  void add(const double latency_ms) {
    if (!std::isfinite(latency_ms)) {
      return;
    }
    const std::scoped_lock lock{mutex_};
    samples_.push_back(latency_ms);
    last_ms_ = latency_ms;
  }

  [[nodiscard]] double last() const {
    const std::scoped_lock lock{mutex_};
    return last_ms_;
  }

  [[nodiscard]] std::size_t count() const {
    const std::scoped_lock lock{mutex_};
    return samples_.size();
  }

  // The nearest-rank percentile at `share` in [0, 1]; NaN without samples.
  [[nodiscard]] double percentile(const double share) const {
    std::vector<double> sorted;
    {
      const std::scoped_lock lock{mutex_};
      sorted = samples_;
    }
    if (sorted.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(sorted.begin(), sorted.end());
    const double clamped = std::clamp(share, 0.0, 1.0);
    const std::size_t rank = static_cast<std::size_t>(
        std::ceil(clamped * static_cast<double>(sorted.size())));
    return sorted[rank == 0U ? 0U : rank - 1U];
  }

private:
  mutable std::mutex mutex_;
  std::vector<double> samples_;
  double last_ms_{std::numeric_limits<double>::quiet_NaN()};
};

// The fields every hop reports in its periodic diagnostic line.
[[nodiscard]] inline std::string
transportLatencyFields(const TransportLatencySamples& samples) {
  char buffer[160];
  std::snprintf(buffer, sizeof buffer,
                "delivery_ms=%.3f delivery_p50_ms=%.3f delivery_p95_ms=%.3f "
                "delivery_max_ms=%.3f delivery_samples=%zu",
                samples.last(), samples.percentile(0.50), samples.percentile(0.95),
                samples.percentile(1.0), samples.count());
  return buffer;
}

} // namespace drone_city_nav
