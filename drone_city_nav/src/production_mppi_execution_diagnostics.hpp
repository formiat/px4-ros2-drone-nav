#pragma once

#include <string>

namespace drone_city_nav {

struct ProductionMppiExecutionPublication;

namespace detail {

[[nodiscard]] std::string
executionInfoFields(const ProductionMppiExecutionPublication& execution);

[[nodiscard]] std::string
executionJsonFields(const ProductionMppiExecutionPublication& execution);

} // namespace detail
} // namespace drone_city_nav
