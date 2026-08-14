#pragma once

#include <string>

namespace drone_city_nav {

struct ProductionMppiNonCooperativeUpdate;

namespace mppi {
struct MppiTickResult;
}

namespace detail {

[[nodiscard]] std::string
nonCooperativeInfoFields(const ProductionMppiNonCooperativeUpdate& noncooperative,
                         const mppi::MppiTickResult& result);

[[nodiscard]] std::string
nonCooperativeJsonFields(const ProductionMppiNonCooperativeUpdate& noncooperative,
                         const mppi::MppiTickResult& result);

} // namespace detail
} // namespace drone_city_nav
