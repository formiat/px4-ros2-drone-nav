#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace drone_city_nav {

// The complete hard spatial-safety input. Occupancy values are interpreted as
// occupied-only evidence: missing cells, unobserved cells, and cells outside a
// snapshot are traversable unknown space. Derived ESDF and topology resources
// intentionally cannot be supplied to this contract.
struct OccupiedCollisionWorld3D {
  const ObservedOccupancyGrid3D* observed_occupancy{nullptr};
  const OccupancyGrid3D* static_occupancy{nullptr};
  const OccupancyGrid2D* planar_occupancy{nullptr};
  std::span<const Point3> raw_point_cloud;
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  SweptFootprintConfig footprint{};
  std::optional<FlightEnvelopeConfig> flight_envelope;
};

enum class OccupiedCollisionStatus3D : std::uint8_t {
  kClear,
  kOutsideFlightEnvelope,
  kRawCollision,
  kInvalidInput,
};

enum class OccupiedCollisionSource3D : std::uint8_t {
  kNone,
  kFlightEnvelope,
  kObservedOccupancy,
  kStaticOccupancy,
  kPlanarOccupancy,
  kRawPointCloud,
};

struct OccupiedCollisionResult3D {
  OccupiedCollisionStatus3D status{OccupiedCollisionStatus3D::kInvalidInput};
  OccupiedCollisionSource3D source{OccupiedCollisionSource3D::kNone};
  Point3 failure_point{};

  [[nodiscard]] bool clear() const noexcept {
    return status == OccupiedCollisionStatus3D::kClear;
  }
};

// Single hard-collision authority shared by planning, route materialization,
// certification, and finite execution. It computes exactly:
//
//   blocked = outside flight envelope OR swept footprint intersects raw occupied
//
// Unknown and known-free cells therefore have identical traversability.
class OccupiedCollisionOracle3D final {
public:
  explicit OccupiedCollisionOracle3D(OccupiedCollisionWorld3D world) noexcept;

  [[nodiscard]] OccupiedCollisionResult3D
  validatePoint(const Point3& position,
                const FootprintBodyAxis& body_axis = {}) const noexcept;

  [[nodiscard]] OccupiedCollisionResult3D
  validateSegment(const Point3& first, const FootprintBodyAxis& first_body_axis,
                  const Point3& second,
                  const FootprintBodyAxis& second_body_axis) const noexcept;

  [[nodiscard]] const OccupiedCollisionWorld3D& world() const noexcept;

private:
  OccupiedCollisionWorld3D world_{};
  bool world_valid_{false};
};

[[nodiscard]] const char*
occupiedCollisionStatus3DName(OccupiedCollisionStatus3D status) noexcept;

} // namespace drone_city_nav
