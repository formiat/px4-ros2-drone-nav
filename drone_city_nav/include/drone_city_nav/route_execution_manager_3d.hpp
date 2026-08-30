#pragma once

#include "drone_city_nav/pending_certified_route_3d.hpp"

#include <cstdint>
#include <memory>
#include <mutex>

namespace drone_city_nav {

enum class ExecutionRoutePublicationStatus3D : std::uint8_t {
  kPublished,
  kInvalidCandidate,
  kStaleSnapshotVersion,
};

struct RouteExecutionManagerSnapshot3D {
  std::shared_ptr<const ExecutionPlan3D> plan;
  std::shared_ptr<const PendingCertifiedRoute3D> pending;

  [[nodiscard]] bool valid() const noexcept;
};

// Sole owner of resident and pending execution plans. All reads and writes are
// linearized by one mutex, including the pending-to-resident transition. The
// reducer certifies semantic state changes; this manager only owns publication.
class RouteExecutionManager3D final {
public:
  RouteExecutionManager3D();

  [[nodiscard]] RouteExecutionManagerSnapshot3D snapshot() const;
  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> plan() const;
  [[nodiscard]] std::shared_ptr<const PendingCertifiedRoute3D> pending() const;

  [[nodiscard]] ExecutionRoutePublicationStatus3D
  publishPlan(const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
              const ExecutionRouteTransitionResult3D& transition);

  [[nodiscard]] bool
  publishPending(std::shared_ptr<const PendingCertifiedRoute3D> candidate);

  [[nodiscard]] bool acknowledgePendingIfSame(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending);

  [[nodiscard]] bool commitPendingTransitionIfSame(
      const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
      const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
      const ExecutionRouteTransitionResult3D& transition);

private:
  [[nodiscard]] ExecutionRoutePublicationStatus3D
  publishPlanLocked(const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
                    const ExecutionRouteTransitionResult3D& transition);

  mutable std::mutex mutex_;
  std::shared_ptr<const ExecutionPlan3D> plan_;
  std::shared_ptr<const PendingCertifiedRoute3D> pending_;
  std::uint64_t last_accepted_pending_sequence_{0U};
};

} // namespace drone_city_nav
