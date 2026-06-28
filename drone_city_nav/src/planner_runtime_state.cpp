#include "drone_city_nav/planner_runtime_state.hpp"

namespace drone_city_nav {

[[nodiscard]] double ageSecondsFromStamp(const std::int64_t stamp_ns,
                                         const std::int64_t now_ns) noexcept {
  if (stamp_ns <= 0 || now_ns <= stamp_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - stamp_ns) / 1.0e9;
}

} // namespace drone_city_nav
