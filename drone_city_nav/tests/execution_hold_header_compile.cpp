#include "drone_city_nav/execution_hold_3d.hpp"

#include <type_traits>

static_assert(std::is_default_constructible_v<drone_city_nav::ExecutionHoldRequest3D>);
static_assert(
    std::is_default_constructible_v<drone_city_nav::ExecutionHoldPreparation3D>);
