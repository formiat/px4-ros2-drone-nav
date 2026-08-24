#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct ExecutionHorizonBracket {
  std::size_t lower_index{0U};
  std::size_t upper_index{0U};
  std::int64_t numerator_ns{0};
  std::int64_t denominator_ns{0};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double ratio() const noexcept;
};

// Returns the exact offset of the final point. Invalid grid dimensions and
// offsets that cannot be represented in signed nanoseconds fail closed.
[[nodiscard]] std::optional<std::int64_t>
executionHorizonTerminalOffsetNs(std::size_t point_count,
                                 std::int64_t interval_ns) noexcept;

// Locates elapsed time on an exact, uniform nanosecond grid. Exact positive
// knots belong to the preceding control interval so the control that produced
// the knot remains authoritative at that instant.
[[nodiscard]] std::optional<ExecutionHorizonBracket>
executionHorizonBracketAt(std::size_t point_count, std::int64_t interval_ns,
                          std::int64_t elapsed_ns) noexcept;

} // namespace drone_city_nav
