#pragma once

#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>
#include <utility>

namespace drone_city_nav {

template<typename FiniteTrajectory> class ExecutionArbiter3D {
public:
  void activate(const std::uint64_t route_generation, FiniteTrajectory trajectory) {
    if (route_generation == 0U) {
      return;
    }
    active_trajectory_ = std::move(trajectory);
    active_route_generation_ = route_generation;
    trajectory_revalidation_required_ = false;
    no_executable_hold_position_.reset();
  }

  [[nodiscard]] bool observe(const RouteLifecycleEvent3D& event) noexcept {
    if (!active_trajectory_.has_value() || event.generation == 0U ||
        event.generation != active_route_generation_) {
      return false;
    }
    if (event.kind != RouteLifecycleEventKind3D::kControlCandidateRejected) {
      trajectory_revalidation_required_ = true;
    }
    return true;
  }

  [[nodiscard]] FiniteTrajectory* activeTrajectory() noexcept {
    return active_trajectory_ ? &*active_trajectory_ : nullptr;
  }

  [[nodiscard]] const FiniteTrajectory* activeTrajectory() const noexcept {
    return active_trajectory_ ? &*active_trajectory_ : nullptr;
  }

  [[nodiscard]] bool trajectoryRevalidationRequired() const noexcept {
    return trajectory_revalidation_required_;
  }

  void confirmRetainedTrajectory() noexcept {
    trajectory_revalidation_required_ = false;
  }

  void rejectTrajectory() noexcept {
    active_trajectory_.reset();
    active_route_generation_ = 0U;
    trajectory_revalidation_required_ = false;
  }

  [[nodiscard]] std::uint64_t activeRouteGeneration() const noexcept {
    return active_route_generation_;
  }

  [[nodiscard]] const std::optional<Point3>& noExecutableHoldPosition() const noexcept {
    return no_executable_hold_position_;
  }

  void enterNoExecutableHold(const Point3& position) noexcept {
    no_executable_hold_position_ = position;
  }

  void leaveNoExecutableHold() noexcept {
    no_executable_hold_position_.reset();
  }

private:
  std::optional<FiniteTrajectory> active_trajectory_;
  std::optional<Point3> no_executable_hold_position_;
  std::uint64_t active_route_generation_{0U};
  bool trajectory_revalidation_required_{false};
};

} // namespace drone_city_nav
