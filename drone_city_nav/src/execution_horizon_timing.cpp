#include "drone_city_nav/execution_horizon_timing.hpp"

#include <cstdint>
#include <limits>
#include <utility>

namespace drone_city_nav {

bool ExecutionHorizonBracket::valid() const noexcept {
  if (denominator_ns <= 0 || numerator_ns < 0 || numerator_ns > denominator_ns ||
      lower_index > upper_index) {
    return false;
  }
  if (lower_index == upper_index) {
    return lower_index == 0U && numerator_ns == 0;
  }
  return upper_index == lower_index + 1U && numerator_ns > 0;
}

double ExecutionHorizonBracket::ratio() const noexcept {
  return valid()
             ? static_cast<double>(numerator_ns) / static_cast<double>(denominator_ns)
             : 0.0;
}

std::optional<std::int64_t>
executionHorizonTerminalOffsetNs(const std::size_t point_count,
                                 const std::int64_t interval_ns) noexcept {
  if (point_count < 2U || interval_ns <= 0) {
    return std::nullopt;
  }

  const std::size_t last_point_index = point_count - 1U;
  const std::uintmax_t maximum_interval_count = static_cast<std::uintmax_t>(
      std::numeric_limits<std::int64_t>::max() / interval_ns);
  if (std::cmp_greater(last_point_index, maximum_interval_count)) {
    return std::nullopt;
  }

  return static_cast<std::int64_t>(last_point_index) * interval_ns;
}

std::optional<ExecutionHorizonBracket>
executionHorizonBracketAt(const std::size_t point_count, const std::int64_t interval_ns,
                          const std::int64_t elapsed_ns) noexcept {
  const std::optional<std::int64_t> terminal_offset_ns =
      executionHorizonTerminalOffsetNs(point_count, interval_ns);
  if (!terminal_offset_ns) {
    return std::nullopt;
  }

  if (elapsed_ns <= 0) {
    return ExecutionHorizonBracket{
        .lower_index = 0U,
        .upper_index = 0U,
        .numerator_ns = 0,
        .denominator_ns = interval_ns,
    };
  }
  if (elapsed_ns >= *terminal_offset_ns) {
    return ExecutionHorizonBracket{
        .lower_index = point_count - 2U,
        .upper_index = point_count - 1U,
        .numerator_ns = interval_ns,
        .denominator_ns = interval_ns,
    };
  }

  const std::int64_t completed_intervals = elapsed_ns / interval_ns;
  const std::int64_t remainder_ns = elapsed_ns % interval_ns;
  const std::size_t completed_index = static_cast<std::size_t>(completed_intervals);
  if (remainder_ns == 0) {
    return ExecutionHorizonBracket{
        .lower_index = completed_index - 1U,
        .upper_index = completed_index,
        .numerator_ns = interval_ns,
        .denominator_ns = interval_ns,
    };
  }

  return ExecutionHorizonBracket{
      .lower_index = completed_index,
      .upper_index = completed_index + 1U,
      .numerator_ns = remainder_ns,
      .denominator_ns = interval_ns,
  };
}

} // namespace drone_city_nav
