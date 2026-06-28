#pragma once

#include <cstdint>
#include <limits>

namespace drone_city_nav {

[[nodiscard]] double ageSecondsFromStamp(std::int64_t stamp_ns,
                                         std::int64_t now_ns) noexcept;

} // namespace drone_city_nav
