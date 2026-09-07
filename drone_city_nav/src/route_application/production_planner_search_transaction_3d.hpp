#pragma once

#include "drone_city_nav/execution_route_certificates_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <memory>
#include <optional>
#include <utility>

namespace drone_city_nav {

// Immutable execution evidence used only by a certified successor search.
// It deliberately captures route geometry and progress without retaining an
// execution plan, pending slot, diagnostics payload, or resident world owner.
struct PlannerSearchContinuityBase3D {
  std::shared_ptr<const CertifiedRouteSuffix3D> route;
  RouteProgressProjection3D request_projection{};

  [[nodiscard]] bool
  validFor(const StaticRouteSearchRequestIdentity& request) const noexcept {
    return request.kind == StaticRouteSearchRequestKind::kExtension &&
           request.base_route_generation != 0U && route != nullptr && route->valid() &&
           route->route_instance_id.valid() &&
           route->identity.generation == request.base_route_generation;
  }
};

// One immutable planner request. Continuations retain this exact object; a
// resident world update or route activation can enqueue a newer transaction,
// but cannot mutate the request already owned by the running search session.
struct PlannerSearchTransaction3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const PersistentPlannerWorld3D> planner_world;
  StaticRouteObjective objective{};
  StaticRouteSearchRequestIdentity request{};
  std::optional<PlannerSearchContinuityBase3D> continuity_base;
  RouteReleaseReason3D release_reason{RouteReleaseReason3D::kNone};
  // When the route was asked for. A search continues across several planner
  // updates, so the lead time a caller has to plan around is measured from
  // here, not from the update that happens to deliver the result. Zero means
  // the request carries no stamp and its latency is not measured.
  std::int64_t requested_stamp_ns{0};

  [[nodiscard]] bool valid() const noexcept {
    if (world == nullptr || planner_world == nullptr || !planner_world->valid() ||
        !objective.available || objective.mission_epoch == 0U || !request.valid()) {
      return false;
    }
    const bool zero_generation_request =
        request.kind == StaticRouteSearchRequestKind::kInitial ||
        request.kind == StaticRouteSearchRequestKind::kInitialRetry;
    if (zero_generation_request != (request.base_route_generation == 0U) ||
        replacement() != (release_reason != RouteReleaseReason3D::kNone)) {
      return false;
    }
    if (request.kind == StaticRouteSearchRequestKind::kExtension) {
      if (!continuity_base.has_value() || !continuity_base->validFor(request) ||
          release_reason != RouteReleaseReason3D::kNone) {
        return false;
      }
    } else if (continuity_base.has_value()) {
      return false;
    }

    if (world->observed_occupancy != nullptr) {
      const bool resident_input =
          planner_world->revision == world->source_raw_revision &&
          planner_world->observed_occupancy == world->observed_occupancy &&
          planner_world->occupied_fingerprint == world->raw_occupied_fingerprint;
      const bool newer_full_overlay =
          planner_world->revision > world->source_raw_revision &&
          planner_world->full_reset;
      return planner_world->static_occupancy == nullptr &&
             planner_world->producer_instance_id == world->producer_instance_id &&
             (resident_input || newer_full_overlay);
    }
    return world->static_occupancy != nullptr &&
           planner_world->observed_occupancy == nullptr &&
           planner_world->static_occupancy == world->static_occupancy &&
           planner_world->occupied_fingerprint == world->raw_occupied_fingerprint;
  }

  [[nodiscard]] bool extension() const noexcept {
    return request.kind == StaticRouteSearchRequestKind::kExtension;
  }

  [[nodiscard]] bool replacement() const noexcept {
    return request.kind == StaticRouteSearchRequestKind::kInitialRetry ||
           request.kind == StaticRouteSearchRequestKind::kReplan;
  }

  [[nodiscard]] bool initial() const noexcept {
    return request.kind == StaticRouteSearchRequestKind::kInitial;
  }
};

[[nodiscard]] inline std::shared_ptr<const PlannerSearchTransaction3D>
makePlannerSearchTransaction3D(
    std::shared_ptr<const WorldSnapshot3D> world,
    std::shared_ptr<const PersistentPlannerWorld3D> planner_world,
    const StaticRouteObjective& objective,
    const StaticRouteSearchRequestIdentity request,
    std::optional<PlannerSearchContinuityBase3D> continuity_base = std::nullopt,
    const RouteReleaseReason3D release_reason = RouteReleaseReason3D::kNone,
    const std::int64_t requested_stamp_ns = 0) {
  auto transaction =
      std::make_shared<const PlannerSearchTransaction3D>(PlannerSearchTransaction3D{
          .world = std::move(world),
          .planner_world = std::move(planner_world),
          .objective = objective,
          .request = request,
          .continuity_base = std::move(continuity_base),
          .release_reason = release_reason,
          .requested_stamp_ns = requested_stamp_ns,
      });
  return transaction->valid() ? std::move(transaction) : nullptr;
}

} // namespace drone_city_nav
