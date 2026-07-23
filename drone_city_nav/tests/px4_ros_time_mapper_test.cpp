#include "drone_city_nav/px4_ros_time_mapper.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace drone_city_nav {

TEST(Px4RosTimeMapperTest, RecoversPx4LocalTimestampFromDdsAdjustedTime) {
  Px4RosTimeMapper mapper;
  mapper.observeTimesync(1'700'000'900U, -1'700'000'000, 100U, 1'010'000'000);

  const auto local = mapper.recoverPx4LocalTimeNs(1'700'001'250U);

  EXPECT_EQ(local, std::optional<std::int64_t>{1'250'000});
}

TEST(Px4RosTimeMapperTest, FitsOffsetAndDriftUsingLowerLatencyEnvelope) {
  Px4RosTimeMapper mapper{Px4RosTimeMapperConfig{4U, 16U, 0.99, 1.01, 50'000'000}};
  constexpr std::int64_t estimated_offset_us{-1'700'000'000};
  for (std::int64_t i = 0; i < 8; ++i) {
    const std::uint64_t local_us = static_cast<std::uint64_t>(1'000'000 + i * 100'000);
    const std::uint64_t adjusted_us = static_cast<std::uint64_t>(
        static_cast<std::int64_t>(local_us) - estimated_offset_us);
    const std::int64_t transport_ns = (i == 3 ? 2'000'000 : 5'000'000);
    const std::int64_t ros_ns =
        2'000'000'000 +
        static_cast<std::int64_t>(1.0002 * static_cast<double>(local_us) * 1000.0) +
        transport_ns;
    mapper.observeTimesync(adjusted_us, estimated_offset_us, 500U, ros_ns);
  }

  ASSERT_TRUE(mapper.ready());
  const auto ros = mapper.px4LocalToRosTimeNs(1'350'000'000);
  ASSERT_TRUE(ros.has_value());
  const std::int64_t ros_stamp = ros.value_or(0);
  EXPECT_NEAR(static_cast<double>(ros_stamp), 3'352'270'000.0, 1'000'000.0);
  const auto round_trip = mapper.rosToPx4LocalTimeNs(ros_stamp);
  ASSERT_TRUE(round_trip.has_value());
  EXPECT_NEAR(static_cast<double>(round_trip.value_or(0)), 1'350'000'000.0, 2.0);
}

TEST(Px4RosTimeMapperTest, RejectsHighRttSamplesAndUnsafeArithmetic) {
  Px4RosTimeMapper mapper;
  mapper.observeTimesync(10U, 0, 100'000U, 1'000'000'000);

  EXPECT_FALSE(mapper.ready());
  EXPECT_EQ(mapper.diagnostics().sample_count, 0U);
  EXPECT_EQ(mapper.diagnostics().rejected_sample_count, 1U);
  EXPECT_FALSE(mapper.recoverPx4LocalTimeNs(0U).has_value());
}

TEST(Px4RosTimeMapperTest, RejectedOffsetDiscontinuityDoesNotPoisonRecovery) {
  Px4RosTimeMapper mapper{
      Px4RosTimeMapperConfig{2U, 16U, 0.999, 1.001, 50'000'000, 500'000'000}};
  constexpr std::int64_t estimated_offset_us{-1'700'000'000};
  mapper.observeTimesync(1'701'000'000U, estimated_offset_us, 100U, 1'000'000'000);
  mapper.observeTimesync(1'701'100'000U, estimated_offset_us, 100U, 1'100'000'000);
  ASSERT_TRUE(mapper.ready());

  mapper.observeTimesync(1'701'200'000U, 0, 100U, 1'200'000'000);

  const auto recovered = mapper.recoverPx4LocalTimeNs(1'701'250'000U);
  ASSERT_TRUE(recovered.has_value());
  EXPECT_EQ(recovered.value_or(0), 1'250'000'000);
  const Px4RosTimeMappingDiagnostics diagnostics = mapper.diagnostics();
  EXPECT_EQ(diagnostics.sample_count, 2U);
  EXPECT_EQ(diagnostics.rejected_sample_count, 1U);
  EXPECT_EQ(diagnostics.clock_discontinuity_count, 1U);
  EXPECT_EQ(diagnostics.latest_estimated_offset_ns, estimated_offset_us * 1000);
}

TEST(Px4RosTimeMapperTest, RejectedHighRttSampleDoesNotReplaceAcceptedOffset) {
  Px4RosTimeMapper mapper;
  mapper.observeTimesync(1'701'000'000U, -1'700'000'000, 100U, 1'000'000'000);

  mapper.observeTimesync(1'701'100'000U, 0, 100'000U, 1'100'000'000);

  EXPECT_EQ(mapper.recoverPx4LocalTimeNs(1'701'200'000U),
            std::optional<std::int64_t>{1'200'000'000});
  EXPECT_EQ(mapper.diagnostics().latest_estimated_offset_ns, -1'700'000'000'000LL);
}

} // namespace drone_city_nav
