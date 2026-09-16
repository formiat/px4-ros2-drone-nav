#include "drone_city_nav/transport_latency_ros.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] rclcpp::MessageInfo messageInfo(const std::int64_t source_ns,
                                              const std::int64_t received_ns) {
  rclcpp::MessageInfo info;
  info.get_rmw_message_info().source_timestamp = source_ns;
  info.get_rmw_message_info().received_timestamp = received_ns;
  return info;
}

// The latency is the receive timestamp against the source timestamp, and a
// middleware that set neither yields no measurement rather than a fast one.
TEST(TransportLatencyRosTest, DeliveryLatencyReadsTheMiddlewareStamps) {
  EXPECT_DOUBLE_EQ(
      transportDeliveryLatencyMs(messageInfo(1'000'000'000, 1'002'500'000)), 2.5);
  EXPECT_TRUE(std::isnan(transportDeliveryLatencyMs(messageInfo(0, 1'002'500'000))));
  EXPECT_TRUE(std::isnan(transportDeliveryLatencyMs(messageInfo(1'000'000'000, 0))));
}

// The samples keep the last latency for the periodic line and the
// nearest-rank percentiles for the summary; a missing measurement is not a
// sample.
TEST(TransportLatencyRosTest, SamplesReportLastAndPercentiles) {
  TransportLatencySamples samples;
  EXPECT_EQ(samples.count(), 0U);
  EXPECT_TRUE(std::isnan(samples.last()));
  EXPECT_TRUE(std::isnan(samples.percentile(0.5)));

  for (const double latency_ms : {5.0, 1.0, 3.0, 2.0, 4.0}) {
    samples.add(latency_ms);
  }
  samples.add(std::nan(""));

  EXPECT_EQ(samples.count(), 5U);
  EXPECT_DOUBLE_EQ(samples.last(), 4.0);
  EXPECT_DOUBLE_EQ(samples.percentile(0.5), 3.0);
  EXPECT_DOUBLE_EQ(samples.percentile(0.95), 5.0);
  EXPECT_DOUBLE_EQ(samples.percentile(1.0), 5.0);
  EXPECT_DOUBLE_EQ(samples.percentile(0.0), 1.0);
  EXPECT_EQ(transportLatencyFields(samples),
            "delivery_ms=4.000 delivery_p50_ms=3.000 delivery_p95_ms=5.000 "
            "delivery_max_ms=5.000 delivery_samples=5");
}

} // namespace
} // namespace drone_city_nav
