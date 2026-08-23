#pragma once

#include "drone_city_nav/incremental_topological_planner_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct StrategicRouteCursor3D {
  double station_m{0.0};
  std::size_t segment_index{0U};
};

struct StrategicRoutePlan3D {
  std::uint64_t plan_id{0U};
  IncrementalTopologicalPlan3D plan{};
  StrategicRouteCursor3D cursor{};
};

struct StrategicRouteCommit3D {
  std::uint64_t plan_id{0U};
  bool accepted{false};
  bool cursor_preserved{false};
};

[[nodiscard]] bool
sameStrategicRoutePlan3D(const IncrementalTopologicalPlan3D& first,
                         const IncrementalTopologicalPlan3D& second) noexcept;

class StrategicRouteManager3D {
public:
  [[nodiscard]] std::uint64_t
  previewPlanId(const IncrementalTopologicalPlan3D& plan) const noexcept;
  [[nodiscard]] StrategicRouteCommit3D
  accept(const IncrementalTopologicalPlan3D& plan) noexcept;
  [[nodiscard]] bool advance(const IncrementalTopologicalPlan3D& plan, double station_m,
                             std::size_t segment_index) noexcept;
  [[nodiscard]] bool invalidate(const IncrementalTopologicalPlan3D& plan) noexcept;
  [[nodiscard]] bool supersede() noexcept;
  void reset() noexcept;

  [[nodiscard]] const StrategicRoutePlan3D* active() const noexcept;
  [[nodiscard]] std::uint64_t lastAllocatedPlanId() const noexcept;

private:
  std::optional<StrategicRoutePlan3D> active_;
  std::uint64_t last_allocated_plan_id_{0U};
};

} // namespace drone_city_nav
