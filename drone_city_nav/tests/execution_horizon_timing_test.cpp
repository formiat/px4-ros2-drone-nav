#include "drone_city_nav/execution_horizon_timing.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kControlIntervalNs{50'000'000};
constexpr std::size_t kPointCount{16U};

void expectBracket(const std::optional<ExecutionHorizonBracket>& actual,
                   const std::size_t expected_lower, const std::size_t expected_upper,
                   const std::int64_t expected_numerator_ns) {
  ASSERT_TRUE(actual.has_value());
  EXPECT_TRUE(actual->valid());
  EXPECT_EQ(actual->lower_index, expected_lower);
  EXPECT_EQ(actual->upper_index, expected_upper);
  EXPECT_EQ(actual->numerator_ns, expected_numerator_ns);
  EXPECT_EQ(actual->denominator_ns, kControlIntervalNs);
}

TEST(ExecutionHorizonTimingTest, RejectsInvalidGridShapeAndInterval) {
  EXPECT_FALSE(executionHorizonTerminalOffsetNs(0U, kControlIntervalNs));
  EXPECT_FALSE(executionHorizonTerminalOffsetNs(1U, kControlIntervalNs));
  EXPECT_FALSE(executionHorizonTerminalOffsetNs(2U, 0));
  EXPECT_FALSE(executionHorizonTerminalOffsetNs(2U, -1));

  EXPECT_FALSE(executionHorizonBracketAt(0U, kControlIntervalNs, 0));
  EXPECT_FALSE(executionHorizonBracketAt(1U, kControlIntervalNs, 0));
  EXPECT_FALSE(executionHorizonBracketAt(2U, 0, 0));
  EXPECT_FALSE(executionHorizonBracketAt(2U, -1, 0));
}

TEST(ExecutionHorizonTimingTest, RejectsTerminalOffsetOverflow) {
  constexpr std::int64_t overflowing_interval_ns =
      std::numeric_limits<std::int64_t>::max() / 2 + 1;

  EXPECT_FALSE(executionHorizonTerminalOffsetNs(3U, overflowing_interval_ns));
  EXPECT_FALSE(executionHorizonBracketAt(3U, overflowing_interval_ns, 0));
}

TEST(ExecutionHorizonTimingTest, AcceptsLargestRepresentableTerminalOffset) {
  const std::optional<std::int64_t> terminal_offset_ns =
      executionHorizonTerminalOffsetNs(2U, std::numeric_limits<std::int64_t>::max());

  ASSERT_TRUE(terminal_offset_ns.has_value());
  EXPECT_EQ(*terminal_offset_ns, std::numeric_limits<std::int64_t>::max());
  const std::optional<ExecutionHorizonBracket> terminal = executionHorizonBracketAt(
      2U, *terminal_offset_ns, std::numeric_limits<std::int64_t>::max());
  ASSERT_TRUE(terminal.has_value());
  EXPECT_TRUE(terminal->valid());
  EXPECT_EQ(terminal->lower_index, 0U);
  EXPECT_EQ(terminal->upper_index, 1U);
  EXPECT_EQ(terminal->numerator_ns, *terminal_offset_ns);
  EXPECT_EQ(terminal->denominator_ns, *terminal_offset_ns);
  EXPECT_DOUBLE_EQ(terminal->ratio(), 1.0);
}

TEST(ExecutionHorizonTimingTest, ClampsPointZeroAndEarlierElapsedTime) {
  for (const std::int64_t elapsed_ns :
       std::array<std::int64_t, 3U>{std::numeric_limits<std::int64_t>::min(), -1, 0}) {
    const std::optional<ExecutionHorizonBracket> bracket =
        executionHorizonBracketAt(kPointCount, kControlIntervalNs, elapsed_ns);
    expectBracket(bracket, 0U, 0U, 0);
    ASSERT_TRUE(bracket.has_value());
    EXPECT_DOUBLE_EQ(bracket->ratio(), 0.0);
  }
}

TEST(ExecutionHorizonTimingTest, ExactPositiveKnotsUsePreviousInterval) {
  for (const std::size_t interval_count : std::array{1U, 7U, 14U}) {
    SCOPED_TRACE(interval_count);
    const std::int64_t elapsed_ns =
        static_cast<std::int64_t>(interval_count) * kControlIntervalNs;
    const std::optional<ExecutionHorizonBracket> bracket =
        executionHorizonBracketAt(kPointCount, kControlIntervalNs, elapsed_ns);

    expectBracket(bracket, interval_count - 1U, interval_count, kControlIntervalNs);
    ASSERT_TRUE(bracket.has_value());
    EXPECT_DOUBLE_EQ(bracket->ratio(), 1.0);
  }
}

TEST(ExecutionHorizonTimingTest, OneNanosecondAfterKnotsUsesNextInterval) {
  for (const std::size_t knot_interval_count : std::array{1U, 7U, 14U}) {
    SCOPED_TRACE(knot_interval_count);
    const std::int64_t elapsed_ns =
        static_cast<std::int64_t>(knot_interval_count) * kControlIntervalNs + 1;
    const std::optional<ExecutionHorizonBracket> bracket =
        executionHorizonBracketAt(kPointCount, kControlIntervalNs, elapsed_ns);

    expectBracket(bracket, knot_interval_count, knot_interval_count + 1U, 1);
    ASSERT_TRUE(bracket.has_value());
    EXPECT_DOUBLE_EQ(bracket->ratio(), 1.0 / 50'000'000.0);
  }
}

TEST(ExecutionHorizonTimingTest, PreservesExactInteriorRemainder) {
  constexpr std::int64_t remainder_ns{12'345'678};
  constexpr std::int64_t elapsed_ns = 7 * kControlIntervalNs + remainder_ns;

  const std::optional<ExecutionHorizonBracket> bracket =
      executionHorizonBracketAt(kPointCount, kControlIntervalNs, elapsed_ns);

  expectBracket(bracket, 7U, 8U, remainder_ns);
  ASSERT_TRUE(bracket.has_value());
  EXPECT_DOUBLE_EQ(bracket->ratio(), static_cast<double>(remainder_ns) /
                                         static_cast<double>(kControlIntervalNs));
}

TEST(ExecutionHorizonTimingTest, TerminalAndLaterUseFinalIntervalAtRatioOne) {
  const std::optional<std::int64_t> terminal_offset_ns =
      executionHorizonTerminalOffsetNs(kPointCount, kControlIntervalNs);
  ASSERT_TRUE(terminal_offset_ns.has_value());
  EXPECT_EQ(*terminal_offset_ns, 15 * kControlIntervalNs);

  for (const std::int64_t elapsed_ns :
       std::array{*terminal_offset_ns, *terminal_offset_ns + 1,
                  std::numeric_limits<std::int64_t>::max()}) {
    const std::optional<ExecutionHorizonBracket> bracket =
        executionHorizonBracketAt(kPointCount, kControlIntervalNs, elapsed_ns);
    expectBracket(bracket, 14U, 15U, kControlIntervalNs);
    ASSERT_TRUE(bracket.has_value());
    EXPECT_DOUBLE_EQ(bracket->ratio(), 1.0);
  }
}

} // namespace
} // namespace drone_city_nav
