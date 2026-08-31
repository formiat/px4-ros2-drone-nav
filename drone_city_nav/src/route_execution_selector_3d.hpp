#pragma once

#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "production_mppi_execution_control.hpp"
#include "production_mppi_node_execution_types.hpp"
#include "production_mppi_node_types.hpp"
#include "production_mppi_raw_world.hpp"

namespace drone_city_nav {

struct RouteExecutionSelectorConfig3D {
  SweptFootprintConfig physical_footprint{};
  FlightEnvelopeConfig flight_envelope{};
  RouteTrackingPolicy3D route_tracking{};
  bool route_cross_track_constraints_enabled{false};
  bool route_tracking_tube_constraints_enabled{false};
};

struct RouteExecutionSelectorRequest3D {
  const WorldSnapshot3D* world{nullptr};
  const ProductionNavigationObjective* objective{nullptr};
  ProductionMppiNavigation navigation{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::int64_t validation_stamp_ns{0};
  std::uint64_t minimum_tracking_sample_sequence{0U};
  std::uint64_t physically_invalidated_through_generation{0U};
  std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity;
  bool observed_3d_world{false};

  [[nodiscard]] bool valid() const noexcept {
    return world != nullptr;
  }
};

enum class RouteExecutionSelectorEffectKind3D : std::uint8_t {
  kRequestRouteRelease,
  kHandlePhysicalTrajectoryCollision,
};

struct RouteExecutionSelectorEffect3D {
  RouteExecutionSelectorEffectKind3D kind{
      RouteExecutionSelectorEffectKind3D::kRequestRouteRelease};
  RouteReleaseReason3D release_reason{RouteReleaseReason3D::kNoActiveRoute};
  ProductionMppiResidentObstacleDisposition obstacle_disposition{
      ProductionMppiResidentObstacleDisposition::kClear};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::uint64_t route_generation{0U};
};

struct RouteExecutionSelectorResult3D {
  ProductionRouteExecutionSelection3D selection{};
  std::vector<RouteExecutionSelectorEffect3D> effects;
};

// Selects and advances the exact execution route snapshot without depending on
// ROS or applying node-side publication/replan effects.
class RouteExecutionSelector3D final {
public:
  RouteExecutionSelector3D(ExecutionSupervisor3D& execution_supervisor,
                           const RouteExecutionSelectorConfig3D& config);

  [[nodiscard]] RouteExecutionSelectorResult3D
  select(const RouteExecutionSelectorRequest3D& request);

private:
  ExecutionSupervisor3D& execution_supervisor_;
  RouteExecutionSelectorConfig3D config_{};
};

} // namespace drone_city_nav
