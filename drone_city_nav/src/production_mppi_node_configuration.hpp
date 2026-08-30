#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t durationNanoseconds(const double seconds,
                                               const char* const parameter_name,
                                               const bool allow_zero = false) {
  const long double nanoseconds = static_cast<long double>(seconds) * 1'000'000'000.0L;
  const long double first_unrepresentable_rounding_input =
      static_cast<long double>(std::numeric_limits<std::int64_t>::max()) + 0.5L;
  if (!std::isfinite(seconds) || (allow_zero ? seconds < 0.0 : !(seconds > 0.0)) ||
      nanoseconds >= first_unrepresentable_rounding_input) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " must be finite, non-negative, and representable"};
  }
  const std::int64_t duration_ns = static_cast<std::int64_t>(std::llround(nanoseconds));
  if (!allow_zero && duration_ns <= 0) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " rounds to a non-positive duration"};
  }
  return duration_ns;
}

[[nodiscard]] std::int64_t checkedDurationSum(const std::int64_t first_ns,
                                              const std::int64_t second_ns,
                                              const std::int64_t third_ns,
                                              const char* const name) {
  if (first_ns < 0 || second_ns < 0 || third_ns < 0 ||
      first_ns > std::numeric_limits<std::int64_t>::max() - second_ns ||
      first_ns + second_ns > std::numeric_limits<std::int64_t>::max() - third_ns) {
    throw std::invalid_argument{std::string{name} + " is not representable"};
  }
  return first_ns + second_ns + third_ns;
}

} // namespace
} // namespace drone_city_nav
