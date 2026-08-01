#pragma once

namespace drone_city_nav {

[[nodiscard]] constexpr bool
semanticPassagesEnabled(const bool configured_enabled,
                        const bool use_static_map) noexcept {
  return configured_enabled && use_static_map;
}

} // namespace drone_city_nav
