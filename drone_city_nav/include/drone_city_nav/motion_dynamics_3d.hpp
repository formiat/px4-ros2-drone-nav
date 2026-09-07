#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"

#include <cstdint>

namespace drone_city_nav {

[[nodiscard]] MotionState3D
integrateMotionState3D(MotionState3D state, MotionControl3D control,
                       const MotionDynamicsConfig3D& config) noexcept;

// Why a step of a finite horizon is not one this vehicle can carry out. Every
// stage that judges a horizon's dynamics — the physical path validator, the
// execution certificate and the tests — reports through this one verdict, so a
// horizon accepted by one stage cannot be refused by another, and a refusal
// names the law it broke instead of collapsing into "invalid".
enum class MotionDynamicsConsistency3D : std::uint8_t {
  kConsistent,
  kNotFinite,
  kAccelerationLimitExceeded,
  kJerkLimitExceeded,
  kStateMismatch,
};

[[nodiscard]] const char*
motionDynamicsConsistency3DName(MotionDynamicsConsistency3D consistency) noexcept;

// The single admissibility law for one horizon step: the control lies inside
// the acceleration envelope, no further from `previous_control` than one
// interval of jerk allows, and `to` is what integrating `from` under it
// produces.
[[nodiscard]] MotionDynamicsConsistency3D motionStepDynamicallyConsistent3D(
    const MotionState3D& from, const MotionControl3D& previous_control,
    const MotionControl3D& control, const MotionState3D& to,
    const MotionDynamicsConfig3D& dynamics) noexcept;

// The whole horizon under the same law, from the control applied before its
// first step: the first reason a step breaks, or kConsistent. This is the one
// dynamics verdict on a finite horizon — the builder checks what it emits with
// it, and the execution certificate admits a horizon by it, so neither can
// disagree with the other.
[[nodiscard]] MotionDynamicsConsistency3D finiteMotionHorizonDynamicsConsistency3D(
    const FiniteMotionHorizon3D& horizon,
    const MotionControl3D& previous_applied_control,
    const MotionDynamicsConfig3D& dynamics) noexcept;

// True when `dynamics` carries the finite, positive limits the consistency law
// needs to mean anything.
[[nodiscard]] bool
motionDynamicsConfigIsValid3D(const MotionDynamicsConfig3D& dynamics) noexcept;

} // namespace drone_city_nav
