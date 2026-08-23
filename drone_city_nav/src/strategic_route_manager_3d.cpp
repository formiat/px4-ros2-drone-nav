#include "drone_city_nav/strategic_route_manager_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kMissionTargetIdentityToleranceM{1.0e-6};

} // namespace

bool sameStrategicRoutePlan3D(const IncrementalTopologicalPlan3D& first,
                              const IncrementalTopologicalPlan3D& second) noexcept {
  if (first.status != second.status || first.purpose != second.purpose ||
      first.planned_on_revision != second.planned_on_revision ||
      first.target_node != second.target_node ||
      distance3D(first.mission_target, second.mission_target) >
          kMissionTargetIdentityToleranceM ||
      first.route_nodes != second.route_nodes ||
      first.guidance_points.size() != second.guidance_points.size()) {
    return false;
  }
  if (first.selected_frontier.has_value() != second.selected_frontier.has_value() ||
      (first.selected_frontier.has_value() &&
       first.selected_frontier->id != second.selected_frontier->id)) {
    return false;
  }
  return std::ranges::equal(first.guidance_points, second.guidance_points,
                            [](const Point3& lhs, const Point3& rhs) {
                              return distance3D(lhs, rhs) <=
                                     kMissionTargetIdentityToleranceM;
                            });
}

std::uint64_t StrategicRouteManager3D::previewPlanId(
    const IncrementalTopologicalPlan3D& plan) const noexcept {
  if (!plan.executableTargetSelected()) {
    return 0U;
  }
  if (active_ && sameStrategicRoutePlan3D(active_->plan, plan)) {
    return active_->plan_id;
  }
  return last_allocated_plan_id_ == std::numeric_limits<std::uint64_t>::max()
             ? 0U
             : last_allocated_plan_id_ + 1U;
}

StrategicRouteCommit3D
StrategicRouteManager3D::accept(const IncrementalTopologicalPlan3D& plan) noexcept {
  StrategicRouteCommit3D result;
  if (!plan.executableTargetSelected()) {
    return result;
  }
  if (active_ && sameStrategicRoutePlan3D(active_->plan, plan)) {
    active_->plan = plan;
    active_->plan.strategic_plan_id = active_->plan_id;
    result.plan_id = active_->plan_id;
    result.accepted = true;
    result.cursor_preserved = true;
    return result;
  }
  if (last_allocated_plan_id_ == std::numeric_limits<std::uint64_t>::max()) {
    return result;
  }
  ++last_allocated_plan_id_;
  IncrementalTopologicalPlan3D accepted_plan = plan;
  accepted_plan.strategic_plan_id = last_allocated_plan_id_;
  active_ = StrategicRoutePlan3D{
      .plan_id = last_allocated_plan_id_,
      .plan = std::move(accepted_plan),
      .cursor = {},
  };
  result.plan_id = last_allocated_plan_id_;
  result.accepted = true;
  return result;
}

bool StrategicRouteManager3D::advance(const IncrementalTopologicalPlan3D& plan,
                                      const double station_m,
                                      const std::size_t segment_index) noexcept {
  if (!active_ || !sameStrategicRoutePlan3D(active_->plan, plan) ||
      !std::isfinite(station_m) || station_m < 0.0) {
    return false;
  }
  if (station_m >= active_->cursor.station_m) {
    active_->cursor.station_m = station_m;
    active_->cursor.segment_index =
        std::max(active_->cursor.segment_index, segment_index);
  }
  return true;
}

bool StrategicRouteManager3D::invalidate(
    const IncrementalTopologicalPlan3D& plan) noexcept {
  if (!active_ || !sameStrategicRoutePlan3D(active_->plan, plan)) {
    return false;
  }
  active_.reset();
  return true;
}

bool StrategicRouteManager3D::supersede() noexcept {
  const bool superseded = active_.has_value();
  active_.reset();
  return superseded;
}

void StrategicRouteManager3D::reset() noexcept {
  active_.reset();
}

const StrategicRoutePlan3D* StrategicRouteManager3D::active() const noexcept {
  return active_ ? std::addressof(*active_) : nullptr;
}

std::uint64_t StrategicRouteManager3D::lastAllocatedPlanId() const noexcept {
  return last_allocated_plan_id_;
}

} // namespace drone_city_nav
