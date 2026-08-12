#pragma once

#include <string>

namespace drone_city_nav {

struct ProductionMppiCooperativeUpdate;

namespace mppi {
struct MppiTickResult;
}

namespace detail {

[[nodiscard]] std::string
cooperativeInfoFields(const ProductionMppiCooperativeUpdate& cooperative,
                      const mppi::MppiTickResult& result);

[[nodiscard]] std::string
cooperativeJsonFields(const ProductionMppiCooperativeUpdate& cooperative,
                      const mppi::MppiTickResult& result);

} // namespace detail
} // namespace drone_city_nav
