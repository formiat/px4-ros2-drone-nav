#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"

namespace drone_city_nav {

[[nodiscard]] MotionState3D
integrateMotionState3D(MotionState3D state, MotionControl3D control,
                       const MotionDynamicsConfig3D& config) noexcept;

} // namespace drone_city_nav
