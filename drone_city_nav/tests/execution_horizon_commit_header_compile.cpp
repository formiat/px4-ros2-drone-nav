#include "drone_city_nav/execution_horizon_commit_3d.hpp"

#include <type_traits>

static_assert(
    std::is_default_constructible_v<drone_city_nav::ExecutionHorizonCommitRequest3D>);
static_assert(
    std::is_default_constructible_v<drone_city_nav::ExecutionHorizonCommitResult3D>);
