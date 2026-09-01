#pragma once

#include "drone_city_nav/execution_evidence_3d.hpp"
#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace drone_city_nav {

struct RouteInstanceId3D {
  std::uint64_t value{0U};

  [[nodiscard]] bool valid() const noexcept {
    return value != 0U;
  }

  friend bool operator==(const RouteInstanceId3D&, const RouteInstanceId3D&) = default;
};

// Immutable progress evidence for one certified route revision. A route waiting
// for activation has no execution input; every activated route owns the exact
// input that established its current station and position.
struct CertifiedRouteProgress3D {
  std::uint64_t route_generation{0U};
  std::uint64_t geometry_revision{0U};
  double station_m{0.0};
  Point3 last_observed_position{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;

  [[nodiscard]] bool valid() const noexcept;
};

} // namespace drone_city_nav
