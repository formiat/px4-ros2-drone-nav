#pragma once

#include "drone_city_nav/risk_aware_lattice.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <string>

namespace drone_city_nav::detail {

[[nodiscard]] std::string
successorProfilingJsonFields(const LatticeSuccessorProfiling& lattice_2d,
                             const Lattice3DSuccessorProfiling& lattice_3d);

} // namespace drone_city_nav::detail
